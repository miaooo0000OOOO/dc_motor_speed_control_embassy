#!/usr/bin/env python3
"""
带约束的分段线性拟合：空载转速-电压特性

优化问题：
    - 3 段折线，段间相接（连续）
    - 最左侧斜率强制为 0（死区水平线）
    - x 轴为电压 V，y 轴为转速 rpm
    - 段 1: y = 0                    (0   ≤ V ≤ V₁)
    - 段 2: y = k₂(V - V₁)           (V₁ ≤ V ≤ V₂)
    - 段 3: y = k₂(V₂-V₁) + k₃(V-V₂) (V₂ ≤ V ≤ V_max)
    - 优化变量：V₁, V₂, k₂, k₃
    - 目标：最小化 MSE

用法：
    python scripts/fit_piecewise_linear.py [csv_path]
"""

import csv
import sys

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 使用微软雅黑
fm.fontManager.addfont("/usr/local/share/fonts/truetype/微软雅黑.ttf")
plt.rcParams['font.family'] = ['Microsoft YaHei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

import numpy as np


def load_data(path='speed_voltage.csv'):
    data = []
    with open(path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append(
                {
                    'duty': float(row['duty_percent']),
                    'voltage': float(row['voltage_V']),
                    'rpm': float(row['rpm_avg']),
                }
            )
    return data


def piecewise_model(params, x):
    """分段折线模型，x 为电压"""
    v1, v2, k2, k3 = params
    y = np.zeros_like(x, dtype=float)
    y[x <= v1] = 0.0
    mask2 = (x > v1) & (x <= v2)
    y[mask2] = k2 * (x[mask2] - v1)
    mask3 = x > v2
    y[mask3] = k2 * (v2 - v1) + k3 * (x[mask3] - v2)
    return y


def mse_loss(params, x, y):
    return np.mean((y - piecewise_model(params, x)) ** 2)


def fit_continuous(x, y):
    """scipy.optimize 连续优化"""
    try:
        from scipy.optimize import minimize
    except ImportError:
        return None

    # 从数据估计初始值
    idx_nonzero = np.where(y > 1.0)[0]
    if len(idx_nonzero) > 0:
        v1_0 = x[idx_nonzero[0]] - 0.2
    else:
        v1_0 = 1.5
    v1_0 = np.clip(v1_0, 0.1, 5.0)

    idx_mid = len(x) // 2
    v2_0 = x[idx_mid] if idx_mid < len(x) else 5.0
    v2_0 = max(v2_0, v1_0 + 0.5)

    mask2 = (x > v1_0) & (x <= v2_0)
    if np.any(mask2):
        k2_0 = np.mean(y[mask2]) / max(np.mean(x[mask2]) - v1_0, 0.1)
    else:
        k2_0 = 20.0

    mask3 = x > v2_0
    if np.any(mask3):
        y3_base = np.mean(y[mask2]) if np.any(mask2) else 0
        k3_0 = (np.mean(y[mask3]) - y3_base) / max(np.mean(x[mask3]) - v2_0, 0.1)
    else:
        k3_0 = 15.0

    k2_0 = max(k2_0, 0.0)
    k3_0 = max(k3_0, 0.0)

    x0 = [v1_0, v2_0, k2_0, k3_0]
    bounds = [(0.0, 12.0), (0.0, 12.0), (0.0, None), (0.0, None)]
    constraints = [
        {'type': 'ineq', 'fun': lambda p: p[0] - 0.05},
        {'type': 'ineq', 'fun': lambda p: p[1] - p[0] - 0.2},
        {'type': 'ineq', 'fun': lambda p: 11.5 - p[1]},
    ]

    result = minimize(
        mse_loss,
        x0,
        args=(x, y),
        method='SLSQP',
        bounds=bounds,
        constraints=constraints,
        options={'ftol': 1e-12, 'maxiter': 500},
    )
    return result.x


def fit_grid_search(x, y):
    """离散网格搜索 + 解析最小二乘"""
    voltages = np.unique(x)
    best_mse = float('inf')
    best_params = None

    for i, v1 in enumerate(voltages):
        for v2 in voltages[i + 1 :]:
            if v2 - v1 < 0.3:
                continue

            mask2 = (x > v1) & (x <= v2)
            mask3 = x > v2
            if not np.any(mask2) or not np.any(mask3):
                continue

            dx2 = x[mask2] - v1
            k2 = np.sum(y[mask2] * dx2) / np.sum(dx2 ** 2)
            k2 = max(k2, 0.0)

            y3_base = k2 * (v2 - v1)
            dy3 = y[mask3] - y3_base
            dx3 = x[mask3] - v2
            k3 = np.sum(dy3 * dx3) / np.sum(dx3 ** 2)
            k3 = max(k3, 0.0)

            params = (v1, v2, k2, k3)
            mse = mse_loss(params, x, y)
            if mse < best_mse:
                best_mse = mse
                best_params = params

    return best_params


def compute_r2(y_true, y_pred):
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0


def print_results(params, x, y):
    v1, v2, k2, k3 = params
    y_pred = piecewise_model(params, x)
    mse = np.mean((y - y_pred) ** 2)
    r2 = compute_r2(y, y_pred)

    print("=" * 60)
    print("带约束分段线性拟合结果（电压-转速）")
    print("=" * 60)
    print(f"\n最优分段点（电压）:")
    print(f"  V1 = {v1:.3f} V")
    print(f"  V2 = {v2:.3f} V")
    print(f"\n拟合方程（折线相接，左端斜率 = 0）:")
    print(f"  段 1 [ 0.00, {v1:6.3f}] V : rpm = 0")
    print(f"  段 2 [{v1:6.3f}, {v2:6.3f}] V : rpm = {k2:.4f} * (V - {v1:.3f})")
    print(f"  段 3 [{v2:6.3f}, {max(x):.2f}] V : rpm = {k2*(v2-v1):.2f} + {k3:.4f} * (V - {v2:.3f})")
    print(f"\n等价展开式:")
    print(f"  段 2 : rpm = {k2:.4f} * V - {k2*v1:.4f}")
    print(f"  段 3 : rpm = {k3:.4f} * V + {k2*(v2-v1) - k3*v2:.4f}")
    print(f"\n整体 MSE = {mse:.4f}")
    print(f"整体 R²  = {r2:.4f}")
    print("=" * 60)


def plot(params, data, save_path='fit_piecewise_linear.png'):
    v1, v2, k2, k3 = params
    fig, ax = plt.subplots(figsize=(10, 6))

    voltages = np.array([d['voltage'] for d in data])
    rpms = np.array([d['rpm'] for d in data])
    ax.scatter(voltages, rpms, c='blue', s=80, zorder=3, label='实测数据')

    xs_fine = np.linspace(0, max(voltages) * 1.02, 500)
    ys_fine = piecewise_model(params, xs_fine)
    ax.plot(xs_fine, ys_fine, 'r-', linewidth=2.5, label='拟合折线', zorder=2)

    ax.axvline(x=v1, color='gray', linestyle='--', alpha=0.7, label=f'V1={v1:.2f}V')
    ax.axvline(x=v2, color='gray', linestyle=':', alpha=0.7, label=f'V2={v2:.2f}V')

    ax.fill_between([0, v1], 0, ax.get_ylim()[1] if ax.get_ylim()[1] > 0 else 50,
                    alpha=0.1, color='red', label='死区')

    ax.set_xlabel('电压 (V)', fontsize=12)
    ax.set_ylabel('转速 (RPM)', fontsize=12)
    ax.set_title('带约束分段线性拟合：转速-电压特性\n(k1=0, 连续节点)', fontsize=14)
    ax.legend(loc='lower right')
    ax.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"\nSaved plot to {save_path}")
    plt.show()


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else 'speed_voltage.csv'
    print(f"Loading {csv_path} ...")
    data = load_data(csv_path)
    if not data:
        print("No data loaded. Exiting.")
        sys.exit(1)

    x = np.array([d['voltage'] for d in data])
    y = np.array([d['rpm'] for d in data])

    print("Running continuous optimization (scipy.optimize)...")
    params_cont = fit_continuous(x, y)

    print("Running grid search validation...")
    params_grid = fit_grid_search(x, y)

    if params_cont is not None:
        mse_cont = mse_loss(params_cont, x, y)
        mse_grid = mse_loss(params_grid, x, y)
        print(f"\nContinuous opt MSE: {mse_cont:.4f}")
        print(f"Grid search  MSE: {mse_grid:.4f}")
        if mse_cont <= mse_grid:
            params = params_cont
            print("=> Using continuous optimization result.\n")
        else:
            params = params_grid
            print("=> Using grid search result.\n")
    else:
        params = params_grid
        print("scipy not available, using grid search result.\n")

    print_results(params, x, y)
    plot(params, data)


if __name__ == '__main__':
    main()
