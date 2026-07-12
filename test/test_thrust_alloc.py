"""thrust_alloc 单元测试 (纯数学, 无需 ROS)."""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import numpy as np
import pytest
from thrust_alloc import thrust_allocate


# 6×6 测试矩阵 (简化, 近似解耦)
T = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1],
], dtype=float)

u_max = np.array([100, 50, 50, 30, 30, 30], dtype=float)


def test_no_saturation():
    """无饱和时直接返回伪逆解."""
    tau = np.array([50, 20, 10, 5, 5, 5], dtype=float)
    u = thrust_allocate(T, tau, u_max)
    np.testing.assert_allclose(u, tau, rtol=1e-6)


def test_clip():
    """超限时最终输出被钳位."""
    tau = np.array([200, 0, 0, 0, 0, 0], dtype=float)
    u = thrust_allocate(T, tau, u_max)
    assert u[0] == pytest.approx(100.0)


def test_sequential_truncation():
    """当一个推进器饱和时, 其余推进器补偿残余."""
    # 构造耦合分配矩阵: 前两列都对 Fx 有贡献
    T_coupled = T.copy()
    T_coupled[0, 1] = 0.5  # 第2个推进器也贡献 Fx

    tau = np.array([120, 0, 0, 0, 0, 0], dtype=float)
    # 各推进器上限: [100,50,...], 第1个只能出力100, 剩余20由第2个出力
    # 第2个贡献0.5 per unit → 需出力40, 在其50限幅内
    u = thrust_allocate(T_coupled, tau, u_max)
    assert u[0] == pytest.approx(100.0)  # 饱和
    assert u[1] > 0.0  # 第2个补偿了残余


def test_pinv_fallback():
    """当 T 是方阵且满秩时, pinv 等价于 inv."""
    tau = np.array([30, 15, 10, 2, 2, 2], dtype=float)
    u = thrust_allocate(T, tau, u_max)
    np.testing.assert_allclose(u, tau, rtol=1e-6)
