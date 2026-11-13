// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Original Last Modified: 2024.10.14
//
// ================================================================
// 这是一份“真机现场可读版 / 诊断版”。
// 基础结构来自老师给的 Original.wall_follower.cpp。
// 我没有改变 ROS2 的整体架构：仍然是
//   /scan -> scan_callback() -> scan_data_[]
//   /odom -> odom_callback() -> 起点判断
//   update_callback() -> 规则判断 -> /cmd_vel
//
// 为了真机 1 小时排故，加入了几个【真机建议修改】：
//   1) 收到第一帧 /scan 之前绝不移动，防止 scan_data_ 初始全 0 导致误转。
//   2) 过滤 NaN / inf / 小于 range_min 的 LiDAR 无效值。
//   3) 第一次收到 LiDAR 时打印 ranges.size / angle_min / angle_increment，
//      用来判断“数组下标=角度”这个老师原始假设在真机上是否成立。
//   4) 每个控制周期打印关键距离 + 当前触发规则，现场直接知道机器人为什么转。
//   5) 控制频率从原始 100 Hz (10 ms) 暂时改成 20 Hz (50 ms)，方便观察日志。
//   6) 把“正前方障碍”放到“左墙消失”之前，优先保证不撞墙。
//   7) 采用更保守的真机速度，减少一启动就猛烈原地旋转的风险。
//   8) 把“已经离开起点”和“回到起点”的距离分开，降低过早判定完成的风险。
//
// 【真机现场最重要的调参位置】请搜索下面这些关键词：
//   FRONT_BLOCK_DIST   正前方多近算堵住
//   LEFT_OPEN_DIST     左墙多远算“消失/门口”
//   FRONT_LEFT_CLOSE   左前方多近要右修
//   FRONT_RIGHT_CLOSE  右前方多近要左修
//   LINEAR_SPEED       正常前进速度
//   TURN_SPEED         转弯角速度
//
// 【现场读日志】
//   [WAIT]   还没收到 LiDAR，机器人不会动
//   [SCAN META] 第一次 LiDAR 的格式信息
//   [DATA]   当前四个关键方向距离
//   [RULE 1]...[RULE 6] 当前到底触发了哪条规则
// ================================================================

// 引入 WallFollower 类的声明。
// 这里面的 class WallFollower、Publisher、Subscriber、scan_data_[]、
// FRONT / FRONT_LEFT / LEFT_FRONT / FRONT_RIGHT 等通常都在 .hpp 中声明。
#include "wall_follower/wall_follower.hpp"

// std::make_shared 等智能指针功能需要 <memory>。
#include <memory>

// std::isfinite、std::fabs 等数学函数需要 <cmath>。
#include <cmath>

// fprintf(stderr, ...) 用来把现场调试信息打印到 Terminal。
#include <cstdio>

// 允许我们写 50ms，而不是 std::chrono::milliseconds(50)。
using namespace std::chrono_literals;

// ================================================================
// 【真机建议修改 1】全局状态：有没有收到过有效 /scan
// ================================================================
// static 表示这个变量只在当前 .cpp 文件里可见。
// 初始 false：程序刚启动时，还没收到 LiDAR。
static bool g_scan_received = false;

// 只打印一次 LiDAR 元信息，防止每帧刷屏。
static bool g_scan_meta_printed = false;

// 【新增日志】最近一次真正发出去的指令，只给节流日志用。
static double g_last_cmd_v = 0.0;
static double g_last_cmd_w = 0.0;

// ================================================================
// 【真机现场可改参数】
// ================================================================
namespace
{
    // 正前方距离小于 0.40 m：认为前面被堵住。
    // 如果机器人经常离墙很远就转：调小，比如 0.35。
    // 如果机器人总是快撞墙才转：调大，比如 0.45~0.50。
    constexpr double FRONT_BLOCK_DIST = 0.50;

    // 左侧/左前方距离大于 0.60 m：认为左墙“消失”，可能遇到门口/外角。
    // 如果机器人总是乱往左拐：把它调大，例如 0.70~0.90。
    // 如果机器人错过房间门口：把它调小，例如 0.50~0.55。
    constexpr double LEFT_OPEN_DIST = 0.60;

    // 左前方小于 0.25 m：太靠左墙，需要向右修。
    constexpr double FRONT_LEFT_CLOSE = 0.6;

    // 右前方小于 0.20 m：太靠右侧障碍，需要向左修。
    constexpr double FRONT_RIGHT_CLOSE = 0.6;

    // 左侧略偏远：0.50~0.60 m 之间轻微向左找墙。
    constexpr double LEFT_TOO_FAR_DIST = 0.50;

    // 正常直行速度，单位 m/s。
    // 真机第一次测试建议 0.10~0.15，不要一开始设 0.3。
    constexpr double LINEAR_SPEED = 0.15;

    // 转弯时仍然保留一点前进速度，单位 m/s。
    constexpr double TURN_LINEAR_SPEED = 0.13;

    // 普通转弯角速度，单位 rad/s。
    // 正数 = 左转，负数 = 右转（标准 ROS Twist 约定）。
    constexpr double TURN_SPEED = 0.60;

    // 较轻微的修正角速度。
    constexpr double GENTLE_TURN_SPEED = 0.30;
}

// ================================================================
// Constructor：创建 WallFollower 节点时自动执行一次
// ================================================================
WallFollower::WallFollower()
// 调用父类 rclcpp::Node 的构造函数，并把 ROS2 节点命名为 wall_follower_node。
: Node("wall_follower_node")
{
    // ------------------------------------------------------------
    // 初始化 12 个 LiDAR 扇区数据。
    // ------------------------------------------------------------
    // i 从 0 开始，每次 +1，一直到 11，所以一共循环 12 次。
    for (int i = 0; i < 12; i++)
    {
        // 程序刚启动时还没有 LiDAR 数据，所以先放 0。
        // 注意：0 本来会被误认为“障碍就在眼前”。
        // 所以本诊断版另外用了 g_scan_received 防止机器人提前动作。
        scan_data_[i] = 0.0;
    }

    // robot_pose_ 用来保存机器人当前 yaw（朝向角）。
    robot_pose_ = 0.0;

    // near_start=false 表示暂时没有判定机器人“回到起点”。
    near_start = false;

    // ------------------------------------------------------------
    // 设置 ROS2 QoS（消息队列策略）。
    // ------------------------------------------------------------
    // KeepLast(10) = 最多保留最近 10 条还没处理的消息。
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // ------------------------------------------------------------
    // Publisher：本节点向 /cmd_vel 发速度命令。
    // ------------------------------------------------------------
    // geometry_msgs::msg::Twist 是 ROS2 标准速度消息。
    // "cmd_vel" 是 Topic 名；通常会解析为 /cmd_vel。
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

    // ------------------------------------------------------------
    // Subscriber 1：订阅 /scan LiDAR。
    // ------------------------------------------------------------
    scan_sub_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            // 订阅的 Topic 名。
            "scan",

            // SensorDataQoS 专门适合高频传感器数据，宁可丢旧帧也不要积压。
            rclcpp::SensorDataQoS(),

            // 每收到一帧 /scan，就调用 scan_callback(msg)。
            std::bind(
                &WallFollower::scan_callback,
                this,
                std::placeholders::_1));

    // ------------------------------------------------------------
    // Subscriber 2：订阅 /odom 里程计。
    // ------------------------------------------------------------
    odom_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            // 订阅 Topic /odom。
            "odom",

            // 使用上面的普通 qos。
            qos,

            // 每收到一帧 /odom，就调用 odom_callback(msg)。
            std::bind(
                &WallFollower::odom_callback,
                this,
                std::placeholders::_1));

    // ------------------------------------------------------------
    // Timer：定期执行控制逻辑 update_callback()。
    // ------------------------------------------------------------
    // 【原始代码】10ms = 100 Hz。
    // 【真机建议修改】这里暂时改成 50ms = 20 Hz。
    // 对简单 wall follower 足够，而且 Terminal 日志不会瞬间刷几百行。
    update_timer_ =
        this->create_wall_timer(
            50ms,
            std::bind(&WallFollower::update_callback, this));

    // 打印一条 INFO，证明节点构造成功。
    RCLCPP_INFO(
        this->get_logger(),
        "Wall follower node has been initialised");
}

// ================================================================
// Destructor：节点被销毁/程序退出时执行
// ================================================================
WallFollower::~WallFollower()
{
    // 打印退出信息。
    RCLCPP_INFO(
        this->get_logger(),
        "Wall follower node has been terminated");
}

// ================================================================
// /odom 回调：处理机器人位置与朝向
// ================================================================

// 回到起点时，x/y 都在起点 ±0.20 m 范围内，认为“接近起点”。
#define START_RANGE 0.20

// 【真机建议修改】机器人先真正离开起点至少约 1 m，
// 之后才允许判断“回到起点”。
// 否则只移动 0.2 m 左右后稍微晃回来，就可能被提前判定完成。
#define LEAVE_RANGE 1.00

void WallFollower::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // static：函数第一次调用时初始化，以后每次调用保留上次的值。
    // first=true 表示还没记录起点。
    static bool first = true;

    // start_moving=true 表示机器人还处于“正在离开起点阶段”。
    static bool start_moving = true;

    // ------------------------------------------------------------
    // 读取四元数姿态 quaternion。
    // ------------------------------------------------------------
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // 把 quaternion 放进 3x3 旋转矩阵，方便转成 roll/pitch/yaw。
    tf2::Matrix3x3 m(q);

    // 声明三个 double 变量保存欧拉角。
    double roll, pitch, yaw;

    // 把四元数转换为 roll / pitch / yaw。
    m.getRPY(roll, pitch, yaw);

    // 保存 yaw（机器人平面上的朝向）。
    robot_pose_ = yaw;

    // ------------------------------------------------------------
    // 读取当前位置 x / y。
    // ------------------------------------------------------------
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // ------------------------------------------------------------
    // 第一次收到 /odom：把当前位置记为起点。
    // ------------------------------------------------------------
    if (first)
    {
        // 保存起点 x。
        start_x = current_x;

        // 保存起点 y。
        start_y = current_y;

        // 起点已经记过了，之后不再重复记录。
        first = false;

        // 打印起点，真机现场方便确认 odom 确实在更新。
        fprintf(
            stderr,
            "[ODOM] Start position: x=%.3f y=%.3f\n",
            start_x,
            start_y);
    }

    // ------------------------------------------------------------
    // 起点记好后，先等机器人真正离开起点区域。
    // ------------------------------------------------------------
    else if (start_moving)
    {
        // 只要 x 或 y 与起点相差超过 LEAVE_RANGE，就认为已经离开。
        // 注意：这是简单的“轴向距离”判断，不是严格欧氏距离。
        if (
            std::fabs(current_x - start_x) > LEAVE_RANGE ||
            std::fabs(current_y - start_y) > LEAVE_RANGE)
        {
            // 以后开始允许判断“是否回到起点”。
            start_moving = false;

            fprintf(stderr, "[ODOM] Robot has left the starting area\n");
        }
    }

    // ------------------------------------------------------------
    // 已经离开过起点后，再判断是否重新回到起点附近。
    // ------------------------------------------------------------
    else if (
        std::fabs(current_x - start_x) < START_RANGE &&
        std::fabs(current_y - start_y) < START_RANGE)
    {
        // 告诉 Terminal：机器人回到起点附近。
        fprintf(stderr, "[ODOM] Near start!!\n");

        // update_callback() 下一次看到 near_start=true 就会停车退出。
        near_start = true;

        // 下面两行只是把状态复位；如果以后重新启动逻辑可以重新记录。
        first = true;
        start_moving = true;
    }
}

// ================================================================
// /scan 回调：把一整圈 LiDAR 压缩成 12 个方向
// ================================================================

// 每个方向取中心角 ±15°，所以一个扇区总宽约 30°。
#define BEAM_WIDTH 15

void WallFollower::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 12 个扇区中心角：0,30,60,...,330 度。
    // 老师原始代码把 msg->ranges 的数组下标直接当作“度数”。
    uint16_t scan_angle[12] =
    {
        0, 30, 60, 90, 120, 150,
        180, 210, 240, 270, 300, 330
    };

    // ------------------------------------------------------------
    // 【真机诊断】第一次收到 /scan 时打印消息格式。
    // ------------------------------------------------------------
    if (!g_scan_meta_printed)
    {
        fprintf(
            stderr,
            "[SCAN META] ranges.size=%zu range_min=%.3f range_max=%.3f "
            "angle_min=%.3f angle_max=%.3f angle_increment=%.6f\n",
            msg->ranges.size(),
            msg->range_min,
            msg->range_max,
            msg->angle_min,
            msg->angle_max,
            msg->angle_increment);

        fprintf(
            stderr,
            "[SCAN META] IMPORTANT: this code assumes indexes 0..359 roughly map to 0..359 degrees.\n");

        g_scan_meta_printed = true;
    }

    // 如果一帧 LiDAR 连 360 个数据都没有，老师原始的 at(359) 会越界。
    // 这种情况下直接停止使用这帧数据，先不要让机器人运动。
    if (msg->ranges.size() < 360)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "LaserScan has only %zu ranges; this code expects at least 360",
            msg->ranges.size());
        return;
    }
    // ------------------------------------------------------------
    // 扇区 0：FRONT，中心在 0°。
    // 因为 0° 正好跨数组首尾，所以要取 345~359° 和 0~14° 两段。
    // ------------------------------------------------------------

    // closest 先设成 LiDAR 最大测距，相当于“暂时没找到更近障碍”。
    double closest = msg->range_max;

    // 检查 345°~359°。
    for (int angle = 360 - BEAM_WIDTH; angle < 360; angle++)
    {
        // 读取这一束激光的距离。
        const float r = msg->ranges.at(angle);

        // 【真机建议修改】只接受有限、且在传感器合法范围内的值。
        // NaN / inf / 太小的异常值都跳过。
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            continue;

        // 如果这一束比目前 closest 更近，就更新最近障碍。
        if (r < closest)
            closest = r;
    }

    // 检查 0°~14°。
    for (int angle = 0; angle < BEAM_WIDTH; angle++)
    {
        // 读取距离。
        const float r = msg->ranges.at(angle);

        // 跳过无效值。
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            continue;

        // 保存更近的障碍。
        if (r < closest)
            closest = r;
    }

    // 把 FRONT 扇区的最近距离放入 scan_data_[0]。
    scan_data_[0] = closest;

    // ------------------------------------------------------------
    // 处理剩余 11 个扇区：30°~330°。
    // ------------------------------------------------------------
    for (int i = 1; i < 12; i++)
    {
        // 每个新扇区都重新从 range_max 开始找最近障碍。
        closest = msg->range_max;

        // 从中心角 -15° 扫到中心角 +14°。
        for (
            int angle = scan_angle[i] - BEAM_WIDTH;
            angle < scan_angle[i] + BEAM_WIDTH;
            angle++)
        {
            // 当前激光束距离。
            const float r = msg->ranges.at(angle);

            // 跳过 NaN / inf / 超出合法测距范围的值。
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
                continue;

            // 找这个 30° 扇区里的最近障碍。
            if (r < closest)
                closest = r;
        }

        // 保存这个方向的最近距离。
        scan_data_[i] = closest;
    }

    // 到这里说明至少已经成功处理过一帧 /scan。
    // update_callback() 从下一次开始才允许机器人移动。
    g_scan_received = true;
}

// ================================================================
// 把 linear / angular 发布到 /cmd_vel
// ================================================================
void WallFollower::update_cmd_vel(
    double linear,
    double angular)
{
    // 创建一条 Twist 消息。
    geometry_msgs::msg::Twist cmd_vel;

    // linear.x：机器人前后方向速度。
    // >0 前进，<0 后退，单位 m/s。
    cmd_vel.linear.x = linear;

    // angular.z：绕竖直 z 轴旋转速度。
    // >0 左转，<0 右转，单位 rad/s。
    cmd_vel.angular.z = angular;

    // 把速度命令发到 /cmd_vel。
    cmd_vel_pub_->publish(cmd_vel);

    // 【新增日志】记录真正发出去的值。
    // 这样节流日志里 cmd=(v,w) 显示的就是实际发布的内容，不会与日志不一致。
    g_last_cmd_v = linear;
    g_last_cmd_w = angular;
}

// ================================================================
// 每 50ms 执行一次：根据 LiDAR 选择动作
// ================================================================
void WallFollower::update_callback()
{
    // ------------------------------------------------------------
    // 【真机建议修改 1】没收到 LiDAR 前绝不移动。
    // ------------------------------------------------------------
    if (!g_scan_received)
    {
        // 发布 0 速度，保证机器人停住。
        update_cmd_vel(0.0, 0.0);

        // 只打印一次 WAIT，避免刷屏。
        static bool waiting_message_printed = false;
        if (!waiting_message_printed)
        {
            fprintf(stderr, "[WAIT] No valid /scan received yet. Robot stays stopped.\n");
            waiting_message_printed = true;
        }

        // return = 本次 update_callback 到这里结束，不执行下面任何转弯规则。
        return;
    }

    // ------------------------------------------------------------
    // 如果已经跑一圈回到起点：立即停车并结束程序。
    // ------------------------------------------------------------
    if (near_start)
    {
        // 发一次 0 速度。
        update_cmd_vel(0.0, 0.0);

        // 告诉现场人员为什么停。
        fprintf(stderr, "[STOP] Robot returned near start.\n");

        // 退出当前 wall_follower 进程。
        exit(0);
    }

    // ------------------------------------------------------------
    // 本拍命中的规则号（0 仅作哨兵，正常一定被赋成 1..6）。
    // ------------------------------------------------------------
    int rule = 0;

    // ------------------------------------------------------------
    // 【真机诊断】打印四个最关键的 LiDAR 扇区。
    // ------------------------------------------------------------
    // FRONT        通常对应 0°，机器人正前方。
    // FRONT_LEFT   通常对应 30°，机器人左前方。
    // LEFT_FRONT   通常对应 60°，更靠左的方向。
    // FRONT_RIGHT  通常对应 330°（即 -30°），机器人右前方。
    fprintf(
        stderr,
        "[DATA] FRONT=%.3f | FRONT_LEFT=%.3f | LEFT_FRONT=%.3f | FRONT_RIGHT=%.3f\n",
        scan_data_[FRONT],
        scan_data_[FRONT_LEFT],
        scan_data_[LEFT_FRONT],
        scan_data_[FRONT_RIGHT]);

    // ============================================================
    // Rule 1：正前方太近 -> 优先右转
    // ============================================================
    // 【真机建议修改】把安全避障放在最高优先级。
    // 原始代码先检查“左墙消失”，可能出现前面已经有墙却仍先左转的情况。
    if (scan_data_[FRONT] < FRONT_BLOCK_DIST)
    {
        // 记录规则号，供下面的节流日志使用。
        rule = 1;
        fprintf(
            stderr,
            "[RULE 1] FRONT < %.2f -> TURN RIGHT IN PLACE\n",
            FRONT_BLOCK_DIST);

        // linear=0：不再向前顶墙。
        // angular<0：向右转。
        update_cmd_vel(0.0, -TURN_SPEED);
    }

    // ============================================================
    // Rule 2：左墙突然很远 -> 左转寻找墙/进入左侧开口
    // ============================================================
    else if (scan_data_[LEFT_FRONT] > LEFT_OPEN_DIST)
    {
        // 记录规则号，供下面的节流日志使用。
        rule = 2;
        fprintf(
            stderr,
            "[RULE 2] LEFT_FRONT > %.2f -> LEFT WALL OPEN, TURN LEFT\n",
            LEFT_OPEN_DIST);

        // 一边慢慢向前，一边左转，形成弧线。
        update_cmd_vel(TURN_LINEAR_SPEED, TURN_SPEED);
    }

    // ============================================================
    // Rule 3：左前方太近 -> 向右修正
    // ============================================================
    else if (scan_data_[FRONT_LEFT] < FRONT_LEFT_CLOSE)
    {
        // 记录规则号，供下面的节流日志使用。
        rule = 3;
        fprintf(
            stderr,
            "[RULE 3] FRONT_LEFT < %.2f -> TOO CLOSE LEFT, STEER RIGHT\n",
            FRONT_LEFT_CLOSE);

        // 保持前进，但 angular 为负，向右偏。
        update_cmd_vel(LINEAR_SPEED, -0.40);
    }

    // ============================================================
    // 新增：左前方 60° 近障碍保护
    // ============================================================
    // Marker 在右转过程中可能从 0° -> 30° -> 60°。
    // 如果已经进入 LEFT_FRONT(约 60°) 且距离小于 0.30 m，
    // 继续向右避让，避免过早恢复前进。
    else if (scan_data_[LEFT_FRONT] < 0.30)
    {
        fprintf(
            stderr,
            "[LEFT_FRONT CLOSE] LEFT_FRONT < 0.30 -> KEEP STEERING RIGHT\n");

        update_cmd_vel(TURN_LINEAR_SPEED, -0.40);
    }

    // ============================================================
    // Rule 4：右前方太近 -> 向左修正
    // ============================================================
    else if (scan_data_[FRONT_RIGHT] < FRONT_RIGHT_CLOSE)
    {
        // 记录规则号，供下面的节流日志使用。
        rule = 4;
        fprintf(
            stderr,
            "[RULE 4] FRONT_RIGHT < %.2f -> TOO CLOSE RIGHT, STEER LEFT\n",
            FRONT_RIGHT_CLOSE);

        // 保持前进，但 angular 为正，向左偏。
        update_cmd_vel(LINEAR_SPEED, 0.40);
    }

    // ============================================================
    // Rule 5：左墙有点远，但还没完全消失 -> 轻微左修
    // ============================================================
    else if (scan_data_[LEFT_FRONT] > LEFT_TOO_FAR_DIST)
    {
        // 记录规则号，供下面的节流日志使用。
        rule = 5;
        fprintf(
            stderr,
            "[RULE 5] LEFT_FRONT > %.2f -> SLIGHTLY FAR FROM LEFT, STEER LEFT\n",
            LEFT_TOO_FAR_DIST);

        // 比 Rule 2 转得轻，避免来回摆动太大。
        update_cmd_vel(LINEAR_SPEED, GENTLE_TURN_SPEED);
    }

    // ============================================================
    // Rule 6：都没有触发 -> 直行
    // ============================================================
    else
    {
        // 记录规则号，供下面的节流日志使用。
        rule = 6;

        fprintf(stderr, "[RULE 6] Normal corridor -> MOVE FORWARD\n");

        // 正常直行。
        update_cmd_vel(LINEAR_SPEED, 0.0);
    }

    // ============================================================
    // 【新增日志】每 0.5 秒一行：命中规则 + 实际发出的指令 + 五个扇区
    //
    // 控制循环是 20 Hz，所以这是故意的欠采样：它回答“这段时间都在哪些
    // 规则里、各扇区大概什么值”，不能用来数精确的切换次数。
    //
    // F=0°(±15°)、FL=30°、LF=60°、FR=330° 是六条规则的判据；
    // RF=300° 不参与任何规则，它是 LF 的镜像，用来看左右不对称。
    //
    // 现场判读：
    //   cmd=(0.000,-0.600) 连续出现 -> 原地右转，看起来就像“不动”
    //   cmd=(0.150,0.000) 但机器人不动 -> 命令没被执行：
    //       另开终端对比 ros2 topic echo /cmd_vel 与 /odom 的 twist
    // ============================================================
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "rule=%d cmd=(%.3f,%.3f) F=%.3f FL=%.3f LF=%.3f RF=%.3f FR=%.3f",
        rule,
        g_last_cmd_v,
        g_last_cmd_w,
        scan_data_[FRONT],
        scan_data_[FRONT_LEFT],
        scan_data_[LEFT_FRONT],
        scan_data_[RIGHT_FRONT],
        scan_data_[FRONT_RIGHT]);

    // ============================================================
    // 【现场怎么调】
    // ============================================================
    // 现象 A：机器人一启动一直原地右转
    //   先看 [DATA] FRONT。
    //   - 如果 FRONT 一直 0 / 很小：LiDAR 映射或无效值处理有问题，不要先改速度。
    //   - 如果 FRONT 合理但仍 RULE 1：FRONT_BLOCK_DIST 可能太大。
    //
    // 现象 B：机器人一直往左绕圈
    //   看 LEFT_FRONT 是否长期 > LEFT_OPEN_DIST。
    //   - 如果明明左边有墙却一直很大：方向映射可能错。
    //   - 如果左边确实很空：LEFT_OPEN_DIST 调大。
    //
    // 现象 C：机器人完全不动
    //   - 如果一直 [WAIT]：/scan 没收到。
    //   - 如果有 [RULE x] 但不动：另开终端 ros2 topic echo /cmd_vel，
    //     再用手动 /cmd_vel 测底盘，排除网络/bringup/电机链路。
    //
    // 现象 D：机器人左右疯狂抖
    //   - 把 TURN_SPEED / 0.40 / GENTLE_TURN_SPEED 调小。
    //   - 或把阈值之间的“死区”拉大，避免临界值反复切换规则。
}

// ================================================================
// main：整个 ROS2 程序的入口
// ================================================================
int main(int argc, char ** argv)
{
    // 初始化 ROS2；argc/argv 是命令行参数。
    rclcpp::init(argc, argv);

    // 创建一个 WallFollower 节点，并让 ROS2 不断处理回调：
    // /scan -> scan_callback
    // /odom -> odom_callback
    // timer -> update_callback
    rclcpp::spin(std::make_shared<WallFollower>());

    // spin 结束后，关闭 ROS2。
    rclcpp::shutdown();

    // main 返回 0，表示程序正常结束。
    return 0;
}
