# bsp_motioncontrol_node 调试与测试指南

## 目录

- [快速启动](#快速启动)
- [Mock 测试（无硬件）](#mock-测试无硬件)
- [实机调试流程](#实机调试流程)
- [在线调参](#在线调参)
- [数据记录与分析](#数据记录与分析)
- [常见问题排查](#常见问题排查)
- [参考命令速查](#参考命令速查)
- [参数完整列表](#参数完整列表)

---

## 快速启动

```bash
cd ~/UVMS_WS
source install/setup.bash
ros2 run bsp bsp_motioncontrol_node --ros-args \
  --params-file install/bsp/share/bsp/config/bsp_motioncontrol.yaml
```

---

## Mock 测试（无硬件）

在 PC 上模拟传感器和指令，验证控制器逻辑。

**终端 1** — 启动控制节点：
```bash
cd ~/UVMS_WS
source install/setup.bash
ros2 run bsp bsp_motioncontrol_node --ros-args \
  --params-file install/bsp/share/bsp/config/bsp_motioncontrol.yaml
```

**终端 2** — 启动 mock 传感器 + 目标指令：
```bash
cd ~/UVMS_WS
source install/setup.bash
python3 uvms3_bsp/test/manual_mock_test.py
```

**终端 3** — 观察推进器输出：
```bash
source install/setup.bash
ros2 topic echo /hal/thruster/cmd
```

预期行为：
- 主推（data[0]）正值 → 前进推力
- 垂推（data[1], data[2]）正值 → 下潜推力
- 侧推（data[3], data[4]）根据 yaw 指令微调

---

## 实机调试流程

### 阶段 1：启动前检查

**传感器在线检查：**

```bash
# 确认全部传感器话题有数据 (用 ros2 topic hz 测量发布频率, 正常应 > 0)
ros2 topic hz /hal/inertialnavi
ros2 topic hz /hal/dvl
ros2 topic hz /hal/depthsensor

# 确认传感器硬件连接正常 (connection_status = 1)
ros2 topic echo /hal/inertialnavi --once | grep connection_status
ros2 topic echo /hal/depthsensor --once | grep connection_status
```

**推进器在线检查：**

```bash
# 确认推进器话题有数据 (hz 应为非零)
ros2 topic hz /hal/mainthruster
ros2 topic hz /hal/auxithruster

# 确认无故障 (fault_status = 0)
ros2 topic echo /hal/mainthruster --once | grep fault_status
ros2 topic echo /hal/auxithruster --once | grep fault_status
```

> **注意**: 如果任一推进器 `fault_status != 0`，控制节点会立即零推力停机并打印 `[MC] 主推/辅推故障`。

**全部检查通过清单：**

| 检查项 | 命令 | 正常值 |
|--------|------|--------|
| IMU 在线 | `ros2 topic hz /hal/inertialnavi` | ~50Hz |
| DVL 在线 | `ros2 topic hz /hal/dvl` | ~10Hz |
| 深度传感器在线 | `ros2 topic hz /hal/depthsensor` | ~10Hz |
| 主推在线 | `ros2 topic hz /hal/mainthruster` | ~10Hz |
| 辅推在线 | `ros2 topic hz /hal/auxithruster` | ~10Hz |
| IMU connection_status | grep | 1 |
| 深度 connection_status | grep | 1 |
| 主推 fault_status | grep | 0 |
| 辅推 5路 fault_status | grep | [0,0,0,0,0] |

### 阶段 2：水面系留 — 纯 PID，无 DOB

先在安全环境下验证基础稳定性：

```bash
ros2 run bsp bsp_motioncontrol_node --ros-args \
  --params-file install/bsp/share/bsp/config/bsp_motioncontrol.yaml \
  -p controller_mode:=1
```

然后发悬停指令：
```bash
ros2 topic pub -1 /app/motioncontrol std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0]"
```

观察推进器是否有异常振荡。无异常后，逐步发小幅机动指令：

```bash
# 前进 0.2 m/s
ros2 topic pub -1 /app/motioncontrol std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.2, 0.0, 2.0, 0.0, 0.0, 0.0]"

# 旋转 0.3 rad
ros2 topic pub -1 /app/motioncontrol std_msgs/msg/Float64MultiArray \
  "data: [0.0, 0.0, 0.0, 2.0, 0.3, 0.0, 0.0]"
```

### 阶段 3：开启前馈 — mode 2

```bash
# 热切换模式(无需重启)
ros2 param set /bsp_motioncontrol_node controller_mode 2
```

### 阶段 4：逐步开启 DOB — mode 3

```bash
ros2 param set /bsp_motioncontrol_node controller_mode 3
# 从小带宽开始, 逐个通道开启
ros2 param set /bsp_motioncontrol_node surge.dob_bandwidth 1.0
ros2 param set /bsp_motioncontrol_node sway.dob_bandwidth 1.0
ros2 param set /bsp_motioncontrol_node yaw.dob_bandwidth 0.5
```

每个通道开一个，确认稳定后再开下一个。**带宽越大响应越快但也越容易振荡。**

---

## 在线调参

所有参数都可以在线修改，无需重启节点：

```bash
# === PID 增益 ===
ros2 param set /bsp_motioncontrol_node surge.kp 250.0    # 比例: 增大 → 响应快但可能振荡
ros2 param set /bsp_motioncontrol_node surge.ki 30.0     # 积分: 消除稳态误差
ros2 param set /bsp_motioncontrol_node surge.kd 15.0     # 微分: 增加阻尼

# === 前馈拖拽系数 ===
ros2 param set /bsp_motioncontrol_node surge.ff_linear 60.0
ros2 param set /bsp_motioncontrol_node surge.ff_quadratic 150.0

# === DOB 参数 ===
ros2 param set /bsp_motioncontrol_node surge.dob_bandwidth 2.0    # 0 = 关闭
ros2 param set /bsp_motioncontrol_node surge.mass_eff 280.0        # 等效质量
ros2 param set /bsp_motioncontrol_node surge.damp_eff 55.0         # 等效阻尼

# === 全局 ===
ros2 param set /bsp_motioncontrol_node controller_mode 3    # 1/2/3
ros2 param set /bsp_motioncontrol_node deadzone_pct 5.0     # 死区, 值越大越不灵敏
ros2 param set /bsp_motioncontrol_node cmd_timeout_s 2.0    # 看门狗超时

# === 通道开关 ===
ros2 param set /bsp_motioncontrol_node enable_pitch false   # 关闭纵倾控制
ros2 param set /bsp_motioncontrol_node enable_roll false    # 关闭横摇控制
```

---

## 数据记录与分析

### 记录推进器指令

```bash
# 记录到文件
ros2 topic echo /hal/thruster/cmd --csv > thruster_log.csv

# 用 ros2 bag 记录全部话题
ros2 bag record -o test_session \
  /hal/thruster/cmd \
  /hal/inertialnavi \
  /hal/dvl \
  /hal/depthsensor \
  /app/motioncontrol
```

### 回放分析

```bash
ros2 bag play test_session
# 另一个终端实时看
ros2 topic echo /hal/thruster/cmd
```

### 查看节点当前全部参数

```bash
ros2 param dump /bsp_motioncontrol_node
```

---

## 常见问题排查

### 1. 节点启动后立即零推力停机

检查传感器连接状态：
```bash
ros2 topic echo /hal/inertialnavi --once | grep connection_status
ros2 topic echo /hal/depthsensor --once | grep connection_status
```
两者都必须为 1。如果为 0，说明对应传感器 HAL 节点未启动或硬件断连。

检查推进器故障状态：
```bash
ros2 topic echo /hal/mainthruster --once | grep fault_status
ros2 topic echo /hal/auxithruster --once | grep fault_status
```
两者都必须为 0。如果非零，说明推进器自检报错，需要排查推进器硬件或 CAN 总线通信。

### 2. 指令超时触发看门狗

`/app/motioncontrol` 必须持续发布（频率至少 > 1/cmd_timeout_s），否则节点自动停机。确认上层应用节点在运行，或用 `ros2 topic pub -r 10` 持续发布。

### 3. 推进器持续振荡

可能原因及对策：

| 原因 | 对策 |
|------|------|
| kp 过大 | 降低对应通道 kp (如 `surge.kp`) |
| kd 太小 | 增大 kd 增加阻尼 |
| DOB 带宽过大 | 降低 dob_bandwidth 或临时关闭 (=0) |
| 死区过小 | 增大 deadzone_pct |

### 4. AUV 无法达到目标速度

| 原因 | 对策 |
|------|------|
| ki 太小 | 增大积分增益, 消除稳态误差 |
| max_out 太小 | 增大输出限幅 |
| 前馈不准 | 微调 ff_linear / ff_quadratic |

### 5. DOB 开启后不稳定

```
dob_bandwidth 从小到大: 0.5 → 1.0 → 2.0 → 3.0
每调大一档, 观察 30 秒, 确认无振荡再继续
```

---

## 参考命令速查

```bash
# 节点列表
ros2 node list

# 查看节点所有参数
ros2 param list /bsp_motioncontrol_node

# 查看单个参数
ros2 param get /bsp_motioncontrol_node surge.kp

# 设置参数
ros2 param set /bsp_motioncontrol_node surge.kp 250.0

# 保存当前参数到文件
ros2 param dump /bsp_motioncontrol_node --output-dir ~/

# 话题速率
ros2 topic hz /hal/inertialnavi

# 话题内容 (只看最新一条)
ros2 topic echo /hal/thruster/cmd --once

# 持续话题内容
ros2 topic echo /hal/thruster/cmd

# 关机节点
ros2 lifecycle set /bsp_motioncontrol_node shutdown
# (注意: 本节点是普通 Node, 直接 ctrl-c 即可)
```

---

## 参数完整列表

### 运行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `control_rate_hz` | 50.0 | 控制循环频率 |
| `cmd_timeout_s` | 1.0 | 指令超时看门狗 (s) |
| `controller_mode` | 3 | 1=PID, 2=FF+PID, 3=FF+PID+DOB |
| `deadzone_pct` | 3.0 | 推进器死区 (%) |

### 通道使能

| 参数 | 默认值 |
|------|--------|
| `enable_surge` | true |
| `enable_sway` | true |
| `enable_depth` | true |
| `enable_yaw` | true |
| `enable_pitch` | true |
| `enable_roll` | true |

### 逐自由度参数 (以 surge 为例, sway/depth/yaw/pitch/roll 同理)

| 参数 | 示例值 | 说明 |
|------|--------|------|
| `surge.kp` | 200.0 | 比例增益 |
| `surge.ki` | 20.0 | 积分增益 |
| `surge.kd` | 10.0 | 微分增益 |
| `surge.max_i` | 50.0 | 积分限幅 (N) |
| `surge.max_out` | 300.0 | 输出限幅 (N) |
| `surge.ff_linear` | 50.0 | 线性阻尼系数 |
| `surge.ff_quadratic` | 120.0 | 二次阻尼系数 |
| `surge.dob_bandwidth` | 3.0 | DOB 带宽 (rad/s, 0=关闭) |
| `surge.mass_eff` | 275.0 | DOB 等效质量 (kg) |
| `surge.damp_eff` | 50.0 | DOB 等效阻尼 (N·s/m) |

depth 通道额外参数: `depth.buoyancy_trim` (默认 0.0, 浮力配平 N)

### 推力分配

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `alloc_matrix` | 36 元素列表 | 6×6 推力分配矩阵 (行优先) |
| `thrust_limits` | [441, 69, 69, 69, 69, 69] | 6 推进器推力上限 (N) |
