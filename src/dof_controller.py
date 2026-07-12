#!/usr/bin/env python3
"""
逐自由度 FF+PID+DOB 控制器。

为每个运动自由度提供统一的控制接口，组合三种控制策略：

    τ = τ_FF + τ_PID - d̂

其中:
  τ_FF = k_lin·v_des + k_quad·v_des·|v_des|    稳态水动力前馈
  τ_PID = kp·e + ki·∫e dt + kd·ė               PID 反馈 (带抗饱和+D项滤波)
  d̂    = 一阶扰动观测器估算的集总扰动

DOB 离散化 (零阶保持):
  α = exp(-L·dt)
  raw_d = τ_prev - m_eff·(ν_k - ν_{k-1})/dt - d_eff·ν_k
  d̂_k = α·d̂_{k-1} + (1-α)·raw_d

参考:
  - AUV_Controller.m  MATLAB 全耦合 6-DOF 控制器
  - bsp_motioncontrol_node.cpp  现有 C++ 实现
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class DofControllerConfig:
    """单自由度控制器全部可调参数, 从 YAML / ROS param 加载."""

    # -- PID --
    kp: float = 0.0  # 比例增益
    ki: float = 0.0  # 积分增益
    kd: float = 0.0  # 微分增益
    max_i: float = 0.0  # 积分限幅 (绝对值, 0 = 不限幅)
    max_out: float = 0.0  # 输出限幅 (绝对值, 0 = 不限幅)
    alpha_deriv: float = 0.7  # D 项一阶低通滤波系数 [0,1)

    # -- 前馈 --
    ff_linear: float = 0.0  # 线性阻尼系数
    ff_quadratic: float = 0.0  # 二次阻尼系数

    # -- DOB --
    dob_bandwidth: float = 0.0  # 观测器带宽 L (rad/s, 0 = 禁用)
    mass_eff: float = 1.0  # 该自由度等效质量/惯量
    damp_eff: float = 0.0  # 该自由度等效线性阻尼

    @classmethod
    def from_dict(cls, d: dict) -> "DofControllerConfig":
        """从字典构造, 缺失键使用默认值."""
        return cls(
            kp=float(d.get("kp", 0.0)),
            ki=float(d.get("ki", 0.0)),
            kd=float(d.get("kd", 0.0)),
            max_i=float(d.get("max_i", 0.0)),
            max_out=float(d.get("max_out", 0.0)),
            alpha_deriv=float(d.get("alpha_deriv", 0.7)),
            ff_linear=float(d.get("ff_linear", 0.0)),
            ff_quadratic=float(d.get("ff_quadratic", 0.0)),
            dob_bandwidth=float(d.get("dob_bandwidth", 0.0)),
            mass_eff=float(d.get("mass_eff", 1.0)),
            damp_eff=float(d.get("damp_eff", 0.0)),
        )


class DofController:
    """单自由度 FF + PID + DOB 控制器.

    每个实例维护独立的 PID 积分、D 项滤波、DOB 估计状态。
    典型用法: 节点中创建 6 个实例, 分别对应 surge/sway/heave/roll/pitch/yaw.
    """

    def __init__(self, cfg: DofControllerConfig) -> None:
        self._cfg = cfg

        # PID 内部状态
        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._prev_filtered_deriv: float = 0.0

        # DOB 内部状态
        self._d_hat: float = 0.0  # 当前扰动估计
        self._prev_nu: Optional[float] = None  # 上一拍速度 (用于有限差分)
        self._dof_enabled: bool = cfg.dob_bandwidth > 0.0
        self._dob_alpha: float = 0.0  # 预计算 α

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def step(
        self,
        dt: float,
        error: float,
        nu_des: float,
        nu_actual: float,
        tau_prev: float,
        mode: int = 3,
        *,
        ff_override: Optional[float] = None,
    ) -> Tuple[float, float]:
        """执行一步控制更新.

        Args:
            dt:         控制周期 (s)
            error:      当前误差 (期望值 - 实际值), yaw 通道应预先 wrap
            nu_des:     期望速度 (用于前馈)
            nu_actual:  实际速度 (用于 DOB)
            tau_prev:   上一拍实际输出的力/力矩 (用于 DOB)
            mode:       1=纯PID, 2=FF+PID, 3=FF+PID+DOB (默认)
            ff_override: 显式覆盖前馈值, 用于浮力配平等静态前馈

        Returns:
            (tau_cmd, d_hat): 控制输出 + 当前扰动估计
        """
        # 1. 前馈
        if ff_override is not None:
            tau_ff = ff_override
        else:
            tau_ff = self._compute_feedforward(nu_des)

        # 2. PID
        tau_pid = self._compute_pid(error, dt)

        # 3. DOB
        d_hat = self._compute_dob(dt, nu_actual, tau_prev)

        # 4. 合成
        if mode == 1:
            tau_cmd = tau_pid
        elif mode == 2:
            tau_cmd = tau_ff + tau_pid
        else:  # mode == 3 (完全体)
            tau_cmd = tau_ff + tau_pid - d_hat

        return tau_cmd, d_hat

    def reset(self) -> None:
        """清零全部内部状态 (停机 / 模式切换时调用)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_filtered_deriv = 0.0
        self._d_hat = 0.0
        self._prev_nu = None

    # ------------------------------------------------------------------
    # 前馈
    # ------------------------------------------------------------------

    def _compute_feedforward(self, nu_des: float) -> float:
        """稳态水动力拖拽前馈.

        F_ff = k_lin · v  +  k_quad · v · |v|

        仅补偿稳态阻力, 不包含惯量项 (实际运行中 ν̇_d ≈ 0).
        """
        return (self._cfg.ff_linear * nu_des
                + self._cfg.ff_quadratic * nu_des * abs(nu_des))

    # ------------------------------------------------------------------
    # PID (梯形积分 + 抗饱和 + D 项一阶低通滤波)
    # ------------------------------------------------------------------

    def _compute_pid(self, error: float, dt: float) -> float:
        cfg = self._cfg
        if dt <= 0.0:
            return 0.0

        # P 项
        p_term = cfg.kp * error

        # I 项 (梯形积分 + 抗饱和钳位)
        i_term = 0.0
        if cfg.ki > 0.0 and cfg.max_i > 0.0:
            self._integral += cfg.ki * error * dt
            self._integral = _clamp(self._integral, -cfg.max_i, cfg.max_i)
            i_term = self._integral
        elif cfg.ki > 0.0:
            self._integral += cfg.ki * error * dt
            i_term = self._integral

        # D 项 (一阶低通滤波抑制高频噪声)
        d_term = 0.0
        if cfg.kd > 0.0:
            raw_deriv = (error - self._prev_error) / dt
            filtered = (cfg.alpha_deriv * self._prev_filtered_deriv
                        + (1.0 - cfg.alpha_deriv) * raw_deriv)
            self._prev_filtered_deriv = filtered
            d_term = cfg.kd * filtered

        self._prev_error = error

        # 合成 + 输出限幅
        out = p_term + i_term + d_term
        if cfg.max_out > 0.0:
            out = _clamp(out, -cfg.max_out, cfg.max_out)

        return out

    # ------------------------------------------------------------------
    # DOB (一阶扰动观测器, 逐自由度解耦)
    # ------------------------------------------------------------------

    def _compute_dob(self, dt: float, nu: float, tau_prev: float) -> float:
        """一阶扰动观测器.

        对 raw_d = τ_prev - m_eff·ν̇ - d_eff·ν 做低通滤波得到 d̂.

        ν̇ 通过后向有限差分估计: ν̇ ≈ (ν_k - ν_{k-1}) / dt
        """
        cfg = self._cfg
        if not self._dof_enabled or dt <= 0.0:
            return 0.0

        # 延迟一拍的 α (第一次进入时计算)
        if self._dob_alpha == 0.0:
            self._dob_alpha = math.exp(-cfg.dob_bandwidth * dt)

        # 估算加速度 (后向差分)
        if self._prev_nu is not None:
            nu_dot = (nu - self._prev_nu) / dt
        else:
            nu_dot = 0.0

        self._prev_nu = nu

        # 施加力 - 预期动力学响应 = 扰动
        raw_d = tau_prev - cfg.mass_eff * nu_dot - cfg.damp_eff * nu

        # 一阶低通: d̂_k = α·d̂_{k-1} + (1-α)·raw_d
        self._d_hat = (self._dob_alpha * self._d_hat
                       + (1.0 - self._dob_alpha) * raw_d)

        return self._d_hat


# ------------------------------------------------------------------
# 工具
# ------------------------------------------------------------------

def _clamp(value: float, lo: float, hi: float) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value
