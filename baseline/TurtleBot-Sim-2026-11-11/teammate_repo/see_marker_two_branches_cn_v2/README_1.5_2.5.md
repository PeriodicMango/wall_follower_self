# SeeMarker 1.5 / 2.5 固定补偿版

这两个版本按照之前的两分支命名框架补充。

---

## 1.5_原几何_固定后拉40cm

基于：

```text
1_原始保底版
```

保留原几何：

```python
x = distance * cos(angle)
y = distance * sin(angle)
```

然后在发布 Marker 之前加入：

```python
marker_x -= 0.40
```

也就是：

```text
前向 X：往后修正 40 cm
横向 Y：不变
```

### 适用目的

如果你们认为：
- 原几何静态测试基本合理；
- 但 RViz / map 中所有 Marker 都整体向前偏约 40 cm；

优先测试 1.5。

---

## 2.5_新几何_固定后拉40cm

基于：

```text
2_新几何_不限制正对
```

先使用实验新几何：

```python
x = distance
y = distance * tan(angle)
```

再加入：

```python
marker_x -= 0.40
```

用于比较：

> 新几何 + 同样 40 cm 固定补偿

---

# 为什么只修 X

ROS 坐标：

```text
+x = 前
+y = 左
```

如果算法得到：

```text
x = 1.20
y = 0.35
```

而你们观察真实应该约：

```text
x = 0.80
y = 0.35
```

最直接的补偿就是：

```python
x -= 0.40
```

而不是：

```python
distance -= 0.40
```

因为斜视时直接减 distance 会连 Y 一起改变。

---

# 调参

两个文件顶部都有：

```python
FORWARD_OFFSET_CORRECTION = 0.40
```

现场建议：

```text
0.30 m
0.35 m
0.40 m  当前默认
0.45 m
```

如果补偿后 Marker 跑到真实位置后面：
- 0.40 -> 0.35 -> 0.30

如果补偿后仍然在真实位置前面：
- 0.40 -> 0.45

---

# 注意

这两个版本都不改：

- point_transformer.py
- landmark.py
- 老师其他 Python 文件
- 距离拟合公式

距离仍然是：

```python
distance = 39.2644 * pow(h, -0.9413)
```

只替换：

```text
src/wall_follower/scripts/see_marker.py
```

---

# 当前优先级建议

根据你们刚才静态测试：
- 正前 distance ≈ 0.76 m
- 约 30° 斜视 distance ≈ 0.90 m

这组数据目前更支持原几何。

因此如果现在只能先试一个：

```text
优先 1.5
```

2.5 保留作对照。
