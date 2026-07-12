#!/usr/bin/env python3
"""
运动控制 mock 集成测试 —— 无需真实传感器, 在 PC 上直接验证控制逻辑.

启动方式:
  # 终端1: 启动运动控制节点
  ros2 run bsp bsp_motioncontrol_node --ros-args \
    --params-file install/bsp/share/bsp/config/bsp_motioncontrol.yaml

  # 终端2: 运行本脚本 (发布模拟传感器 + 目标指令)
  python3 test/manual_mock_test.py

本脚本发布:
  - /hal/inertialnavi   : yaw=0, pitch=0, roll=0 (水平)
  - /hal/dvl            : vx=0, vy=0, vz=0 (静止)
  - /hal/depthsensor    : depth=2.0 (当前深度 2m)
  - /app/motioncontrol  : target vx=0.5, vy=0, depth=5.0, yaw=0.3

然后观察 /hal/thruster/cmd 的输出, 验证控制器的响应是否合理.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# hal 消息
try:
    from hal.msg import (
        HalAuxithruster,
        HalDepthsensor,
        HalDvl,
        HalInertialnavi,
        HalMainthruster,
        HalTailservo,
        HalWingservo,
    )
except ImportError:
    print("hal 消息未安装. 请先 source install/setup.bash", file=sys.stderr)
    sys.exit(1)


class MockSensorPublisher(Node):
    """周期发布模拟传感器数据 + 目标运动指令."""

    def __init__(self) -> None:
        super().__init__("mock_sensor_publisher")
        self._t0 = time.time()

        # 传感器发布
        self._imu_pub = self.create_publisher(HalInertialnavi, "/hal/inertialnavi", 10)
        self._dvl_pub = self.create_publisher(HalDvl, "/hal/dvl", 10)
        self._depth_pub = self.create_publisher(HalDepthsensor, "/hal/depthsensor", 10)
        self._main_pub = self.create_publisher(HalMainthruster, "/hal/mainthruster", 10)
        self._aux_pub = self.create_publisher(HalAuxithruster, "/hal/auxithruster", 10)
        self._tail_pub = self.create_publisher(HalTailservo, "/hal/tailservo", 10)
        self._wing_pub = self.create_publisher(HalWingservo, "/hal/wingservo", 10)

        # 指令发布
        self._cmd_pub = self.create_publisher(Float64MultiArray, "/app/motioncontrol", 10)

        # 结果订阅 (观察控制器输出)
        self._thr_sub = self.create_subscription(
            Float64MultiArray, "/hal/thruster/cmd", self._thr_cb, 10,
        )

        self._timer = self.create_timer(0.05, self._publish_all)

    def _publish_all(self) -> None:
        t = time.time() - self._t0

        # -- 模拟传感器 --
        imu = HalInertialnavi()
        imu.yaw = 0.0
        imu.pitch = 0.0
        imu.roll = 0.0
        imu.connection_status = 1
        self._imu_pub.publish(imu)

        dvl = HalDvl()
        dvl.velocity_x = 0.0
        dvl.velocity_y = 0.0
        dvl.velocity_z = 0.0
        dvl.connection_status = 1
        self._dvl_pub.publish(dvl)

        depth = HalDepthsensor()
        depth.depth_avg = 2.0
        depth.connection_status = 1
        self._depth_pub.publish(depth)

        main = HalMainthruster()
        main.fault_status = 0
        self._main_pub.publish(main)

        aux = HalAuxithruster()
        aux.fault_status = [0] * 5
        self._aux_pub.publish(aux)

        tail = HalTailservo()
        self._tail_pub.publish(tail)

        wing = HalWingservo()
        self._wing_pub.publish(wing)

        # -- 目标指令 --
        cmd = Float64MultiArray()
        cmd.data = [
            0.0,  # data[0] 保留
            0.5,  # vx = 0.5 m/s
            0.0,  # vy = 0.0
            5.0,  # depth = 5.0 m (从2m下潜到5m)
            0.3,  # yaw = 0.3 rad
        ]
        self._cmd_pub.publish(cmd)

    def _thr_cb(self, msg: Float64MultiArray) -> None:
        """打印推进器指令."""
        if len(msg.data) >= 6:
            vals = ", ".join(f"{v:+6.1f}%" for v in msg.data[:6])
            print(f"[推力%] {vals}", end="\r")


def main() -> None:
    rclpy.init()
    node = MockSensorPublisher()
    print("=" * 60)
    print("Mock 传感器已启动, 观察 /hal/thruster/cmd 输出")
    print("期望: 主推正转(前进), 垂推正转(下潜), 侧推微调(偏航)")
    print("=" * 60)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n测试结束.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
