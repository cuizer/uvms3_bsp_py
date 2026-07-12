"""dof_controller 单元测试 (纯数学, 无需 ROS)."""

import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from dof_controller import DofController, DofControllerConfig


def _make_cfg(**overrides) -> DofControllerConfig:
    defaults = {
        "kp": 200, "ki": 20, "kd": 10,
        "max_i": 50, "max_out": 300,
        "ff_linear": 50, "ff_quadratic": 120,
        "dob_bandwidth": 3.0, "mass_eff": 275, "damp_eff": 50,
    }
    defaults.update(overrides)
    return DofControllerConfig.from_dict(defaults)


# ------------------------------------------------------------------
# Mode 1: pure PID
# ------------------------------------------------------------------

def test_mode1_pure_pid_zero_error():
    """零误差时 PID 不输出."""
    cfg = _make_cfg(dob_bandwidth=0)
    ctrl = DofController(cfg)
    tau, d_hat = ctrl.step(0.02, 0.0, nu_des=0.5, nu_actual=0.5, tau_prev=0, mode=1)
    assert tau == 0.0
    assert d_hat == 0.0


def test_mode1_p_proportional():
    """P 项: tau ≈ kp * error."""
    cfg = _make_cfg(ki=0, kd=0, dob_bandwidth=0)
    ctrl = DofController(cfg)
    tau, _ = ctrl.step(0.02, 1.0, nu_des=0.0, nu_actual=0.0, tau_prev=0, mode=1)
    assert abs(tau - 200.0) < 1.0


# ------------------------------------------------------------------
# Mode 2: FF + PID
# ------------------------------------------------------------------

def test_mode2_feedforward():
    """前馈补偿稳态拖拽."""
    cfg = _make_cfg(ki=0, kd=0, dob_bandwidth=0)
    ctrl = DofController(cfg)
    tau, _ = ctrl.step(0.02, 0.0, nu_des=0.5, nu_actual=0.5, tau_prev=0, mode=2)
    expected_ff = 50 * 0.5 + 120 * 0.5 * 0.5  # = 55.0
    assert abs(tau - expected_ff) < 1.0


# ------------------------------------------------------------------
# Mode 3: FF + PID + DOB (完全体)
# ------------------------------------------------------------------

def _simulate_1dof(ctrl, dt, n_steps, nu_des, disturbance, mass, damp, ki_enabled=True):
    """模拟 1-DOF 闭环: 控制器 + 一阶惯性+阻尼被控对象.

    Plant:  mass * nu_dot + damp * nu = tau_cmd + disturbance
    """
    nu = 0.0
    tau_prev = 0.0
    for _ in range(n_steps):
        err = nu_des - nu
        tau, _ = ctrl.step(dt, err, nu_des=nu_des, nu_actual=nu,
                           tau_prev=tau_prev, mode=3)
        # 被控对象动力学 (欧拉积分)
        tau_total = tau + disturbance
        nu_dot = (tau_total - damp * nu) / mass
        nu += nu_dot * dt
        tau_prev = tau_total
    return nu, tau


def test_mode3_dob_disturbance_rejection():
    """DOB 有效抑制外部扰动: 模式3 稳态误差 < 模式2."""
    dt = 0.02
    disturbance = 50.0  # 强外部扰动力

    # 模式 2 (FF+PID, 无 DOB)
    cfg2 = _make_cfg(ff_linear=50, ff_quadratic=0, damp_eff=50, mass_eff=275, dob_bandwidth=0)
    ctrl2 = DofController(cfg2)
    nu2, _ = _simulate_1dof(ctrl2, dt, 1000, 0.5, disturbance, 275, 50)

    # 模式 3 (FF+PID+DOB)
    cfg3 = _make_cfg(ff_linear=50, ff_quadratic=0, damp_eff=50, mass_eff=275, dob_bandwidth=3.0)
    ctrl3 = DofController(cfg3)
    nu3, _ = _simulate_1dof(ctrl3, dt, 1000, 0.5, disturbance, 275, 50)

    # 模式3 有 DOB 补偿, 稳态速度应更接近目标
    err2 = abs(nu2 - 0.5)
    err3 = abs(nu3 - 0.5)
    assert err3 < err2, f"DOB 应改善抗扰: mode2 err={err2:.3f}, mode3 err={err3:.3f}"


# ------------------------------------------------------------------
# Anti-windup
# ------------------------------------------------------------------

def test_anti_windup():
    """积分项被钳位在 max_i."""
    cfg = _make_cfg(kd=0, dob_bandwidth=0, max_i=10.0)
    ctrl = DofController(cfg)
    dt = 0.02
    tau_prev = 0.0
    for _ in range(1000):  # 持续大误差
        tau, _ = ctrl.step(dt, 2.0, nu_des=0.0, nu_actual=0.0,
                           tau_prev=tau_prev, mode=1)
        tau_prev = tau
    # 积分项不应爆炸
    assert abs(ctrl._integral) <= 10.0 + 1e-6


# ------------------------------------------------------------------
# Reset
# ------------------------------------------------------------------

def test_reset():
    """reset() 清零所有内部状态."""
    cfg = _make_cfg()
    ctrl = DofController(cfg)
    ctrl.step(0.02, 1.0, nu_des=0.0, nu_actual=0.0, tau_prev=0, mode=3)
    ctrl.reset()
    assert ctrl._integral == 0.0
    assert ctrl._d_hat == 0.0
    assert ctrl._prev_nu is None


# ------------------------------------------------------------------
# DOB disabled
# ------------------------------------------------------------------

def test_dob_disabled():
    """dob_bandwidth=0 时 d_hat 始终为 0."""
    cfg = _make_cfg(dob_bandwidth=0)
    ctrl = DofController(cfg)
    _, d_hat = ctrl.step(0.02, 1.0, nu_des=0.0, nu_actual=0.0, tau_prev=100, mode=3)
    assert d_hat == 0.0
