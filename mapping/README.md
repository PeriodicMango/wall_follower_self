# TurtleBot3 .103 · 真机循墙建图 2.3（2026-11-11）
给接手这份 ZIP 的队友和 AI：目标是用实体 Waffle Pi 的激光循墙、压缩相机识别双色标、Cartographer 建图，并把地图和真实观测写到同一轮目录。统一入口是 `真机巡墙建图.command`。不要开 Gazebo，不要把 `use_sim_time` 设为 true。此包的交付范围是建图和地标记录；不含 Nav2 自动导航任务。
 
## 先弄清楚哪个版本成功过
- 基线：2026-11-11 15:27:32 的 Real_103 真机运行，当时控制器是 **wall_follower_3**。它保存了 88 × 87 格、0.05 m/格的地图，和黄/粉、蓝/粉、粉/绿三个地标；没有第四个。样例在 `sample/Real_103_2026-11-11_152732/`，不可冒充 2.3 的实车结果。
- 新控制器基于包内的 `reference/wall_follower 2.3.cpp`。运行的适配版在 `local_variants/wall_follower_2_3/src/`：保持 2.3 的雷达决策顺序，补上建图器需要的起点与结束消息、参数及安全交接。**看见 Marker 仍会记录和显示，但不会触发 3 版专有的 Marker 转向。** 编译和独立回环行为测试已通过；本次未用 2.3 重跑实体迷宫。
- `upstream/` 原样来自队友 [wall_follower_self](https://github.com/PeriodicMango/wall_follower_self/tree/aa52fb5294769d0cfefede48f022e21ec98b90ff) 的已固定版本。不要另找同名 see_marker.py。原件哈希、版本来源见 `PROVENANCE.json`；解压后可运行 `shasum -a 256 -c MANIFEST.sha256`。
 
## 队友在 Mac 上照做
前提：Apple Silicon Mac，课程的 mac-native-gazebo-11.10.2 包及其已装好的 Pixi ROS 2 Humble 环境、Xcode Command Line Tools。Pi 端需要 ROS 2 Humble、TurtleBot3 bringup、CycloneDDS、Python OpenCV 和 NumPy、正常的 OpenCR、LDS 及相机。课程工具链在本机使用 macOS 26.5 SDK；若系统默认 27.x 出现 libSystem.tbd / arm64e.x1 报错，安装兼容 SDK，并在编译前 export TB3_MACOS_SDK=/实际/SDK路径。包不包含 ROS 安装或密码。Mac 与目标 Pi 在同一可互通 Wi‑Fi，时钟同步。此流程不修改防火墙。
 
1. 把 ZIP 完整解压到可写目录，终端 `cd` 到能看到本 README 的那一层。终端执行：
   ```bash
   cp settings.example.sh settings.sh
   open -e settings.sh
   ```
   填写 `TB3_ROBOT_IP` 和已核实的 Wi‑Fi MAC `TB3_ROBOT_MAC`；本轮原车为 192.168.1.103。勿凭共享的 turtlebot-test.local 名称选车。现场可在树莓派本地终端查 `cat /sys/class/net/wlan0/address`，对照实物。课程目录不在默认位置就填 `TB3_COURSE`；Pi 的用户名、SSH 端口和 ROS/workspace setup 路径也可配置。
2. 首次连这台车，请与现场可信的主机指纹核对后，在普通 SSH 下完成 known_hosts 登记并退出。示例：`ssh pi@192.168.1.103`。用自己的 SSH 登录/密钥，不要编辑脚本以跳过主机认证。
3. 准备本机程序和专用通信配置（不驱动车轮）：
   ```bash
   # 自定义了课程目录时，在此 shell 载入你填写的配置
   set -a; source ./settings.sh; set +a
   bash "./重新编译巡墙.command"
   bash ./setup_robot.sh
   ```
   setup 配对两端的 IP/独立 DDS 标签，把 guard、相机 JPEG 转换和配对环境传到 Pi 的独立目录；不安装系统包，不启动车轮。换 Mac 或 Wi‑Fi 后重新执行该步。若检测到另一队的 guard/bringup/速度发布器，先与操作者交接；不要强杀对方进程。
4. 放好实体迷宫和双色标，小车放在地面起点。**执行下一条会在数据检查通过后实际开始行驶**：
   ```bash
   ./真机巡墙建图.command
   ```
   此终端编排 Pi 驱动、压缩相机、Cartographer、识别、记录、2.3 和 RViz；无需另开仿真。相机窗口若未弹出，可在另一个终端从包根运行 `./打开真机相机.command`。
5. 在**建图终端按回车**：停轮，保存地图与地标；再按一次回车关闭结果窗口。完整回到起点也会触发保存。若需单独请求停车，从包根运行 `bash scripts/pi_mapping_session.sh disarm`。不要用关闭 RViz 代替停车。
 
## 数据流和必须保持的接口
| 输入 → 输出 | 说明 |
|---|---|
| Pi 原始相机 /camera/image_raw → /turtlebot_test/camera/compressed | JPEG；默认最多 8 Hz、质量 80，保留拍摄时间戳。 |
| 压缩图像 → see_marker → /marker_position | 真机适配固定前向后拉为 **0.00 m**，不减 0.7 m。识别使用队友的测距和色对；新相机若安装角度或标定不同，需另行实测。 |
| /marker_position + TF → point_transformer → /visualization_marker_array | 变换到 map 坐标系，生成双色圆柱并刷新。刷新显示不增加测量次数。 |
| /scan、/odom → wall_follower_node → /turtlebot_test/wall_cmd_vel → Pi guard → /cmd_vel | 只有 guard 可以直接向底盘发布速度。建图限速为 0.06 m/s、0.30 rad/s；输入过时保持零速。不存在早期“全方向 25 cm 即停车”规则，2.3 自己的前方避障仍有效。 |
| /wall_follower/start_pose、/wall_follower/finished → 记录器 | 起点来自真实里程计并保留给晚订阅者；离开起点区域后再返回触发结束，也可以手动回车保存。 |
 
ROS Domain 3，ROS_LOCALHOST_ONLY=0，使用实际时钟。RViz 的 Fixed Frame 用 map；建图时墙体订阅 /map（Reliable / Transient Local），彩标用 /visualization_marker_array。每轮新建 `runs/Real_103_日期_时间/`。其中 `maps/maze_Real_*.yaml` **配同名 PGM 一起使用**；`maps/markers_Real_*.txt` 的首行是起点的 odom x,y,yaw，后续行是地标的 map x,y,上色/下色；`*.start_map.txt` 是起点的地图位姿，不等于停车位姿。另有 landmarks.csv、实际使用的 YAML、日志和本轮图像证据。缺哪个地标就如实保存几个，不补造第四个。
 
## 建图失败时给 AI 的检查顺序
- 无墙：先看 `logs/cartographer.log`、`logs/preflight.log`、`mapping_preflight.json`，核实 /scan、/odom、/map、map→base_footprint 的来源和时效。只看到 topic 名字不等于收到数据。不要用仿真地图代替。
- 无地标：看 `logs/detector.log` 与 `vision/camera_latest.jpg`、`vision/mask_latest.jpg` 是否更新；检查压缩图像和相机坐标到 map 的 TF，再看 recorder.log。地图与地标保存是否成功分开判定。
- 有传感器但车不动：看 `bash scripts/pi_mapping_session.sh status`、preflight 和 follower.log；检查是否只有 guard 发布 /cmd_vel，不要绕过它直接发车速。
- 调雷达距离或速度：改 `local_variants/wall_follower_2_3/wall_follower_2_3.yaml` 或执行 `./真机巡墙建图.command 参数`，停止后重启即可。修改 C++ 则需重新 `./重新编译巡墙.command`。2.3 假设约 360 个、按角度顺序排列的雷达采样；更换雷达先核对 angle_min、angle_increment 和样本数。
- 一轮验收：确认本轮二进制为 wall_follower_2_3；看到新鲜的 JPEG/激光/里程计和非空 /map；运行后保存新的 YAML+PGM 及真实色标；确认终端报告停车。样例哈希可以证明文件一致，不能证明另一台车已实测通过。
