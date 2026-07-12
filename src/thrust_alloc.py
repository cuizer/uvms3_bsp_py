#!/usr/bin/env python3
"""
逐次截断推力分配算法。

将 6-DOF 期望合力/力矩 τ 分配到 6 个推进器的推力指令 u。

算法 (移植自 MATLAB thrust_alloc.m):
  1. u = pinv(T) @ τ                          初解 (Moore-Penrose 伪逆)
  2. 检查饱和: 任一 |u_i| >= u_max_i?
  3. 若饱和: 将该推进器钉死在限幅值,
     计算残余 τ_rem = τ - T_sat @ u_sat,
     用剩余推进器列重新求解
  4. 最多迭代 n_thrusters 次
  5. 最终钳位: u = clip(u, -u_max, u_max)

与简单伪逆相比, 当某推进器饱和时不会把力分配给物理上无法补偿的推进器,
保证分配结果在推进器约束下的物理可行性。
"""

from __future__ import annotations

import numpy as np


def thrust_allocate(
    T: np.ndarray,
    tau: np.ndarray,
    u_max: np.ndarray,
    *,
    max_iters: int = 10,
) -> np.ndarray:
    """逐次截断推力分配.

    Args:
        T:          6×N 推力分配矩阵 (N = 推进器数量)
                    T[:, i] = [v_i; cross(r_i, v_i)]
        tau:        6×1 期望广义力/力矩 [Fx, Fy, Fz, Mx, My, Mz]
        u_max:      N×1 各推进器推力上限 (均视为对称限幅, 即 [-u_max_i, u_max_i])
        max_iters:  最大迭代次数 (默认 10, 实际很少超过推进器数量)

    Returns:
        u: N×1 分配的推力指令 (已限幅)
    """
    n_thr = T.shape[1]

    # 1. 初解
    u = np.linalg.pinv(T) @ tau
    saturated = np.abs(u) >= u_max

    if not np.any(saturated):
        return np.clip(u, -u_max, u_max)

    # 2. 迭代截断
    for _ in range(min(max_iters, n_thr)):
        # 饱和推进器钉死在限幅值 (保持符号)
        u[saturated] = np.sign(u[saturated]) * u_max[saturated]

        # 残余力/力矩
        tau_rem = tau - T[:, saturated] @ u[saturated]

        # 剩余推进器重新分配
        free_idx = np.where(~saturated)[0]
        if len(free_idx) == 0:
            break

        u[free_idx] = np.linalg.pinv(T[:, free_idx]) @ tau_rem
        saturated = np.abs(u) >= u_max

        if not np.any(saturated):
            break

    return np.clip(u, -u_max, u_max)
