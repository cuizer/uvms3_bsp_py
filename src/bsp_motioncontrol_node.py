#!/usr/bin/env python3
"""
BSP 层运动控制节点 —— 6-DOF 逐自由度 FF + PID + DOB 控制.

## 输入 (Subscriptions)
  - /app/motioncontrol  (Float64MultiArray): data[1]=vx, data[2]=vy,
                            data[3]=depth, data[4]=yaw
  - /hal/inertialnavi    (HalInertialnavi):    yaw / pitch / roll
  - /hal/dvl             (HalDvl):             体坐标系速度 vx / vy / vz
  - /hal/depthsensor     (HalDepthsensor):     深度 depth_avg
  - /hal/mainthruster    (HalMainthruster):    主推状态反馈
  - /hal/auxithruster    (HalAuxithruster):    辅推状态反馈
  - /hal/tailservo       (HalTailservo):       尾舵状态反馈
  - /hal/wingservo       (HalWingservo):       翼舵状态反馈

## 输出 (Publishers)
  - /hal/thruster/cmd    (Float64MultiArray):  data[0]=主推%, data[1..5]=5路辅推%
  - /hal/servo/tail_cmd  (Float64MultiArray):  尾舵指令 (当前为零位)
  - /hal/servo/wing_cmd  (Float64MultiArray):  翼舵指令 (当前为零位)

## 控制算法
  6 个独立 DofController 实例, 各自执行:
    τ_i = τ_FF_i + τ_PID_i - d̂_i
  推力分配采用逐次截断伪逆算法将 τ → u[6].

参考:
  - MATLAB FF_PID_SIMULATION / AUV_Controller.m (DOB 概念)
  - uvms3_test/src/bsp_motioncontrol_node.cpp (ROS 接口)
"""

from __future__ import annotations

import math
import threading
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# hal 消息由 uvms3_test CMake 包生成, 安装后在 Python path 中
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
    HalInertialnavi = None  # type: ignore[assignment]
    HalDvl = None  # type: ignore[assignment]
    HalDepthsensor = None  # type: ignore[assignment]
    HalMainthruster = None  # type: ignore[assignment]
    HalAuxithruster = None  # type: ignore[assignment]
    HalTailservo = None  # type: ignore[assignment]
    HalWingservo = None  # type: ignore[assignment]

from dof_controller import DofController, DofControllerConfig
from thrust_alloc import thrust_allocate

# ============================================================================
# 工具
# ============================================================================

def _wrap_angle(a: float) -> float:
    """归一化到 [-π, π]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _load_dof_configs(node: Node) -> Dict[str, DofControllerConfig]:
    """从 ROS 参数中加载 6 个自由度的控制器配置."""
    dof_names = ["surge", "sway", "depth", "yaw", "pitch", "roll"]
    configs: Dict[str, DofControllerConfig] = {}

    for name in dof_names:
        # 声明参数 (YAML 中已提供默认值, 这里作为 fallback)
        prefix = f"{name}."
        for key, default in [
            ("kp", 0.0), ("ki", 0.0), ("kd", 0.0),
            ("max_i", 0.0), ("max_out", 0.0), ("alpha_deriv", 0.7),
            ("ff_linear", 0.0), ("ff_quadratic", 0.0),
            ("dob_bandwidth", 0.0), ("mass_eff", 1.0), ("damp_eff", 0.0),
        ]:
            node.declare_parameter(prefix + key, default)

        # depth 通道特有参数
        if name == "depth":
            node.declare_parameter("depth.buoyancy_trim", 0.0)

        # 读取到 dict 再构造成 config
        raw = {
            k: node.get_parameter(prefix + k).value
            for k in [
                "kp", "ki", "kd", "max_i", "max_out", "alpha_deriv",
                "ff_linear", "ff_quadratic",
                "dob_bandwidth", "mass_eff", "damp_eff",
            ]
        }
        configs[name] = DofControllerConfig.from_dict(raw)

    return configs


# ============================================================================
# 节点
# ============================================================================

class BspMotionControlNode(Node):
    """BSP 层运动控制节点.

    与 C++ 版本 `bsp_motioncontrol_node` 的 ROS 接口完全兼容,
    内部控制算法升级为逐自由度 FF + PID + DOB.
    """

    N_THRUSTERS = 6

    def __init__(self) -> None:
        super().__init__("bsp_motioncontrol_node")

        # -- 运行参数 --
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_s", 1.0)
        self.declare_parameter("controller_mode", 3)
        self.declare_parameter("deadzone_pct", 3.0)

        # -- 通道使能 --
        self.declare_parameter("enable_surge", True)
        self.declare_parameter("enable_sway", True)
        self.declare_parameter("enable_depth", True)
        self.declare_parameter("enable_yaw", True)
        self.declare_parameter("enable_pitch", True)
        self.declare_parameter("enable_roll", True)

        # -- 分配矩阵 & 推力限幅 --
        self.declare_parameter(
            "alloc_matrix",
            [1.0, 0.0, 0.0, 0.0, 0.5, 0.5,
             0.0, 0.0, 0.0, 1.0, 1.0, 0.0,
             0.0, 1.0, 1.0, 0.0, 0.0, 0.0,
             0.0, 0.1, -0.1, 0.0, 0.0, 0.0,
             0.0, 0.3, 0.3, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.2, -0.2, 0.1],
        )
        self.declare_parameter(
            "thrust_limits",
            [441.0, 69.0, 69.0, 69.0, 69.0, 69.0],
        )

        # -- 加载控制器 --
        self._dof_cfgs = _load_dof_configs(self)
        self._dof_ctrls: Dict[str, DofController] = {
            name: DofController(cfg)
            for name, cfg in self._dof_cfgs.items()
        }

        # -- 传感器状态缓存 --
        self._state_lock = threading.Lock()
        self._yaw = 0.0
        self._pitch = 0.0
        self._roll = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._depth = 0.0
        self._imu_valid = False
        self._dvl_valid = False
        self._depth_valid = False

        # 推进器故障状态
        self._main_fault = False
        self._aux_faults = [False] * 5

        # 上一拍姿态 (用于估计角速率)
        self._prev_yaw: Optional[float] = None
        self._prev_pitch: Optional[float] = None
        self._prev_roll: Optional[float] = None
        self._prev_att_time: Optional[rclpy.time.Time] = None

        # 上一拍体坐标系速度 (用于 DOB 有限差分)
        self._prev_vx: Optional[float] = None
        self._prev_vy: Optional[float] = None
        self._prev_vz: Optional[float] = None

        # 上一拍实际力/力矩 (用于 DOB)
        self._prev_tau_real = np.zeros(6)

        # -- 目标指令缓存 --
        self._cmd_lock = threading.Lock()
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_depth = 0.0
        self._target_yaw = 0.0
        self._target_valid = False
        self._last_cmd_time = self.get_clock().now()

        # -- 订阅 --
        qos_sensor = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )
        qos_cmd = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )

        self._imu_sub = self.create_subscription(
            HalInertialnavi, "/hal/inertialnavi", self._imu_cb, qos_sensor,
        )
        self._dvl_sub = self.create_subscription(
            HalDvl, "/hal/dvl", self._dvl_cb, qos_sensor,
        )
        self._depth_sub = self.create_subscription(
            HalDepthsensor, "/hal/depthsensor", self._depth_cb, qos_sensor,
        )
        self._main_thr_sub = self.create_subscription(
            HalMainthruster, "/hal/mainthruster", self._main_thr_cb, qos_sensor,
        )
        self._aux_thr_sub = self.create_subscription(
            HalAuxithruster, "/hal/auxithruster", self._aux_thr_cb, qos_sensor,
        )
        self._tail_sub = self.create_subscription(
            HalTailservo, "/hal/tailservo", self._tail_servo_cb, qos_sensor,
        )
        self._wing_sub = self.create_subscription(
            HalWingservo, "/hal/wingservo", self._wing_servo_cb, qos_sensor,
        )
        self._cmd_sub = self.create_subscription(
            Float64MultiArray, "/app/motioncontrol", self._cmd_cb, qos_cmd,
        )

        # -- 发布 --
        self._thruster_pub = self.create_publisher(
            Float64MultiArray, "/hal/thruster/cmd", 10,
        )
        self._tail_pub = self.create_publisher(
            Float64MultiArray, "/hal/servo/tail_cmd", 10,
        )
        self._wing_pub = self.create_publisher(
            Float64MultiArray, "/hal/servo/wing_cmd", 10,
        )

        # -- 控制定时器 --
        rate = self.get_parameter("control_rate_hz").value
        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"[MC] bsp_motioncontrol_node 就绪, 控制频率 {rate:.1f} Hz"
        )

    # ==================================================================
    # 传感器回调 (仅写缓存)
    # ==================================================================

    def _imu_cb(self, msg: HalInertialnavi) -> None:
        with self._state_lock:
            self._yaw = float(msg.yaw)
            self._pitch = float(msg.pitch)
            self._roll = float(msg.roll)
            self._imu_valid = (msg.connection_status == 1)

    def _dvl_cb(self, msg: HalDvl) -> None:
        with self._state_lock:
            self._vx = float(msg.velocity_x)
            self._vy = float(msg.velocity_y)
            self._vz = float(msg.velocity_z)
            self._dvl_valid = (msg.connection_status == 1)

    def _depth_cb(self, msg: HalDepthsensor) -> None:
        with self._state_lock:
            self._depth = float(msg.depth_avg)
            self._depth_valid = (msg.connection_status == 1)

    def _main_thr_cb(self, msg: HalMainthruster) -> None:
        with self._state_lock:
            self._main_fault = (msg.fault_status != 0)

    def _aux_thr_cb(self, msg: HalAuxithruster) -> None:
        with self._state_lock:
            for i in range(5):
                self._aux_faults[i] = (msg.fault_status[i] != 0)

    def _tail_servo_cb(self, msg: HalTailservo) -> None:
        pass

    def _wing_servo_cb(self, msg: HalWingservo) -> None:
        pass

    def _cmd_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 5:
            self.get_logger().warn(
                f"[MC] 指令数据不足 (需 >=5, 实际={len(msg.data)})",
                throttle_duration_sec=5.0,
            )
            return
        with self._cmd_lock:
            self._target_vx = float(msg.data[1])
            self._target_vy = float(msg.data[2])
            self._target_depth = float(msg.data[3])
            self._target_yaw = float(msg.data[4])
            self._target_valid = True
            self._last_cmd_time = self.get_clock().now()

    # ==================================================================
    # 主控制循环
    # ==================================================================

    def _control_loop(self) -> None:
        # 1. 快照传感器状态
        with self._state_lock:
            yaw, pitch, roll = self._yaw, self._pitch, self._roll
            vx, vy, vz = self._vx, self._vy, self._vz
            depth = self._depth
            imu_ok = self._imu_valid
            dvl_ok = self._dvl_valid
            depth_ok = self._depth_valid
            main_fault = self._main_fault
            aux_any_fault = any(self._aux_faults)

        # 2. 快照目标指令
        with self._cmd_lock:
            target_vx = self._target_vx
            target_vy = self._target_vy
            target_depth = self._target_depth
            target_yaw = self._target_yaw
            target_valid = self._target_valid
            last_cmd_time = self._last_cmd_time

        # 3. 看门狗
        cmd_timeout = self.get_parameter("cmd_timeout_s").value
        dt_cmd = (self.get_clock().now() - last_cmd_time).nanoseconds * 1e-9
        if not target_valid or dt_cmd > cmd_timeout:
            if target_valid and dt_cmd > cmd_timeout:
                self.get_logger().warn(
                    f"[MC] 指令超时 ({dt_cmd:.1f}s), 零推力停机",
                    throttle_duration_sec=2.0,
                )
            self._send_zero_thrust()
            return

        # 4. 传感器 & 推进器健康检查
        if not imu_ok or not dvl_ok or not depth_ok:
            self.get_logger().warn(
                f"[MC] 传感器无效 (imu={imu_ok}, dvl={dvl_ok}, depth={depth_ok}), 零推力停机",
                throttle_duration_sec=2.0,
            )
            self._send_zero_thrust()
            return

        if main_fault:
            self.get_logger().warn(
                "[MC] 主推故障, 零推力停机",
                throttle_duration_sec=2.0,
            )
            self._send_zero_thrust()
            return

        if aux_any_fault:
            self.get_logger().warn(
                "[MC] 辅推故障, 零推力停机",
                throttle_duration_sec=2.0,
            )
            self._send_zero_thrust()
            return

        # 5. 估计角速率 p, q, r (体坐标系, 用于 DOB)
        now = self.get_clock().now()
        p, q, r = self._estimate_body_angular_rates(yaw, pitch, roll, now)

        # 6. 计算各 DOF 误差 & 调用控制器
        mode = self.get_parameter("controller_mode").value
        en = {
            "surge": self.get_parameter("enable_surge").value,
            "sway": self.get_parameter("enable_sway").value,
            "depth": self.get_parameter("enable_depth").value,
            "yaw": self.get_parameter("enable_yaw").value,
            "pitch": self.get_parameter("enable_pitch").value,
            "roll": self.get_parameter("enable_roll").value,
        }

        tau_des = np.zeros(6)
        d_hat_all = np.zeros(6)

        # -- surge --
        err_surge = target_vx - vx if en["surge"] else 0.0
        tau_des[0], d_hat_all[0] = self._dof_ctrls["surge"].step(
            self._dt, err_surge,
            nu_des=target_vx, nu_actual=vx,
            tau_prev=self._prev_tau_real[0], mode=mode,
        )

        # -- sway --
        err_sway = target_vy - vy if en["sway"] else 0.0
        tau_des[1], d_hat_all[1] = self._dof_ctrls["sway"].step(
            self._dt, err_sway,
            nu_des=target_vy, nu_actual=vy,
            tau_prev=self._prev_tau_real[1], mode=mode,
        )

        # -- depth (位置式, nu_actual = vz) --
        err_depth = target_depth - depth if en["depth"] else 0.0
        buoyancy = self.get_parameter("depth.buoyancy_trim").value
        tau_des[2], d_hat_all[2] = self._dof_ctrls["depth"].step(
            self._dt, err_depth,
            nu_des=0.0, nu_actual=vz,
            tau_prev=self._prev_tau_real[2], mode=mode,
            ff_override=buoyancy,
        )

        # -- yaw (角度 wrapping) --
        err_yaw = _wrap_angle(target_yaw - yaw) if en["yaw"] else 0.0
        tau_des[5], d_hat_all[5] = self._dof_ctrls["yaw"].step(
            self._dt, err_yaw,
            nu_des=0.0, nu_actual=r,
            tau_prev=self._prev_tau_real[5], mode=mode,
        )

        # -- pitch (锁定为 0) --
        err_pitch = _wrap_angle(0.0 - pitch) if en["pitch"] else 0.0
        tau_des[4], d_hat_all[4] = self._dof_ctrls["pitch"].step(
            self._dt, err_pitch,
            nu_des=0.0, nu_actual=q,
            tau_prev=self._prev_tau_real[4], mode=mode,
        )

        # -- roll (锁定为 0) --
        err_roll = _wrap_angle(0.0 - roll) if en["roll"] else 0.0
        tau_des[3], d_hat_all[3] = self._dof_ctrls["roll"].step(
            self._dt, err_roll,
            nu_des=0.0, nu_actual=p,
            tau_prev=self._prev_tau_real[3], mode=mode,
        )

        # 7. 推力分配
        T_alloc = self._build_alloc_matrix()
        u_max = np.array(self.get_parameter("thrust_limits").value, dtype=float)
        u_cmd = thrust_allocate(T_alloc, tau_des, u_max)

        # 8. 死区
        deadzone_pct = self.get_parameter("deadzone_pct").value / 100.0
        for i in range(self.N_THRUSTERS):
            if abs(u_cmd[i]) < deadzone_pct * u_max[i]:
                u_cmd[i] = 0.0

        # 9. 存 τ_real 供下周期 DOB 使用
        self._prev_tau_real = T_alloc @ u_cmd

        # 10. N → 百分比 → 发布
        thr_pct = np.clip(u_cmd / u_max * 100.0, -100.0, 100.0)
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(x) for x in thr_pct]
        self._thruster_pub.publish(cmd_msg)

        # 11. 舵机零位
        self._tail_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0]))
        self._wing_pub.publish(Float64MultiArray(data=[0.0, 0.0]))

    # ==================================================================
    # 角速率估计
    # ==================================================================

    def _estimate_body_angular_rates(
        self, yaw: float, pitch: float, roll: float,
        now: rclpy.time.Time,
    ) -> tuple:
        """从姿态角后向差分估计体坐标系角速率 p, q, r.

        用精确 J2_inv 变换将欧拉角速率映射到体坐标系:
          [p, q, r]^T = J2_inv(φ, θ) · [dφ/dt, dθ/dt, dψ/dt]^T
        """
        if self._prev_att_time is None:
            self._prev_yaw, self._prev_pitch, self._prev_roll = yaw, pitch, roll
            self._prev_att_time = now
            return 0.0, 0.0, 0.0

        dt_att = (now - self._prev_att_time).nanoseconds * 1e-9
        if dt_att <= 0.0:
            return 0.0, 0.0, 0.0

        # 欧拉角差分 (带 wrap)
        dphi = _wrap_angle(roll - self._prev_roll) / dt_att
        dtheta = _wrap_angle(pitch - self._prev_pitch) / dt_att
        dpsi = _wrap_angle(yaw - self._prev_yaw) / dt_att

        # J2_inv 精确变换
        st = math.sin(pitch)
        ct = math.cos(pitch)
        cp = math.cos(roll)
        sp = math.sin(roll)

        p = dphi - st * dpsi
        q = cp * dtheta + sp * ct * dpsi
        r = -sp * dtheta + cp * ct * dpsi

        self._prev_yaw, self._prev_pitch, self._prev_roll = yaw, pitch, roll
        self._prev_att_time = now
        return p, q, r

    # ==================================================================
    # 工具
    # ==================================================================

    def _build_alloc_matrix(self) -> np.ndarray:
        raw = self.get_parameter("alloc_matrix").value
        arr = np.array(raw, dtype=float)
        if arr.size != 36:
            self.get_logger().error(
                f"[MC] alloc_matrix 大小 {arr.size} != 36, 使用零矩阵",
            )
            return np.zeros((6, 6))
        return arr.reshape(6, 6)

    def _send_zero_thrust(self) -> None:
        self._thruster_pub.publish(
            Float64MultiArray(data=[0.0] * self.N_THRUSTERS))
        self._tail_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0]))
        self._wing_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        self._prev_tau_real = np.zeros(6)
        for ctrl in self._dof_ctrls.values():
            ctrl.reset()


# ============================================================================
# main
# ============================================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = BspMotionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
