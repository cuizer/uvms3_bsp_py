# uvms3_bsp_py

UVMS 三代 BSP (Board Support Package) 层 Python 功能包。

## 概述

本包为 UVMS 三代硬件平台提供 Python 层的板级支持，作为 ROS 2 节点与底层硬件驱动之间的桥梁。

## 结构

当前包含 AD10 侧主从部署接收节点：

- `teleop_receiver`
  - 通过 TCP 接收本地电脑发来的 44-byte 主从目标帧
  - 发布 `/slave/joint_states`

## 依赖

- `rclpy`
- `sensor_msgs`
- `std_msgs`

## 构建

```bash
cd <workspace_root>
colcon build --packages-select bsp
```

## 运行

```bash
source install/setup.bash
ros2 run bsp teleop_receiver
```

或使用 launch：

```bash
ros2 launch bsp ad10_slave_joint_receiver.launch.py server_host:=192.168.1.100
```
