#!/usr/bin/env python3
"""
PI控制器参数整定分析与仿真

基于辨识出的一阶电机模型：
    G(s) = K / (τ·s + 1)  [rpm / % duty]

输出图片（适合报告插图）：
    - pi_tuning_step_response.png    阶跃响应对比
    - pi_tuning_bode.png             开环Bode图
    - pi_tuning_closed_loop.png      闭环特性对比
    - pi_tuning_disturbance.png      抗扰动性能对比

用法：
    python scripts/tune_pi_controller.py
"""

import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 配置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

from scipy import signal


# ═══════════════════════════════════════════════════════════════════
# 系统参数
# ═══════════════════════════════════════════════════════════════════
K_MOTOR = 23.4          # rpm/V  (电机空载稳态增益)
TAU_MOTOR = 0.017       # s      (电机机电时间常数)
V_MAX = 11.0            # V      (100% duty 对应电机端电压)
TS = 0.05               # s      (控制周期 50ms)

# 被控对象：duty(%) → rpm，扣除死区后
# G_p(s) = (V_max/100) * K_motor / (tau*s + 1)
K_PLANT = (V_MAX / 100.0) * K_MOTOR  # = 2.574 rpm/% duty

# 离散被控对象（ZOH离散化）
ALPHA = np.exp(-TS / TAU_MOTOR)
KZ = K_PLANT * (1 - ALPHA)

# 当前PI参数（代码中默认值）
KP_CUR = 0.30
KI_CUR = 0.15


def print_system_info():
    """打印系统基本信息"""
    print("=" * 60)
    print("           系统传递函数分析")
    print("=" * 60)
    print(f"\n被控对象（连续域）：")
    print(f"    Gp(s) = {K_PLANT:.3f} / ({TAU_MOTOR:.3f}·s + 1)   [rpm/%]")
    print(f"\n关键特性：")
    print(f"  电机时间常数  tau = {TAU_MOTOR*1000:.0f} ms")
    print(f"  控制周期      Ts  = {TS*1000:.0f} ms = {TS/TAU_MOTOR:.1f}·tau")
    print(f"  被控对象增益  K   = {K_PLANT:.3f} rpm/%")
    print(f"  ZOH 离散: alpha = exp(-Ts/tau) = {ALPHA:.4e}")
    print(f"  ZOH 离散: Gp(z) ~ {KZ:.4f} / (z - {ALPHA:.4e})")


def compute_open_loop(Kp, Ki):
    """构建开环传递函数 L(s) = C_pi(s) * Gp(s)"""
    num_pi = [Kp, Ki]
    den_pi = [1, 0]
    num_plant = [K_PLANT]
    den_plant = [TAU_MOTOR, 1]
    num_ol = np.convolve(num_pi, num_plant)
    den_ol = np.convolve(den_pi, den_plant)
    return signal.TransferFunction(num_ol, den_ol)


def compute_closed_loop(Kp, Ki):
    """构建闭环传递函数 T(s) = L(s)/(1+L(s))"""
    num_pi = [Kp, Ki]
    den_pi = [1, 0]
    num_plant = [K_PLANT]
    den_plant = [TAU_MOTOR, 1]
    num_ol = np.convolve(num_pi, num_plant)
    den_ol = np.convolve(den_pi, den_plant)
    num_cl = num_ol
    den_cl = np.polyadd(den_ol, num_ol)
    return signal.TransferFunction(num_cl, den_cl)


def analyze_stability(Kp, Ki, name=""):
    """频域稳定性分析"""
    L = compute_open_loop(Kp, Ki)
    omega = np.logspace(-1, 3, 10000)
    w, mag, phase = signal.bode(L, omega)

    # 增益穿越频率 & 相位裕度
    pm = None
    wg = None
    for i in range(len(mag) - 1):
        if mag[i] > 0 and mag[i + 1] <= 0:
            wg = np.interp(0, [mag[i + 1], mag[i]], [w[i + 1], w[i]])
            ph_g = np.interp(wg, [w[i], w[i + 1]], [phase[i], phase[i + 1]])
            pm = 180 + ph_g
            break

    # 相位穿越频率 & 幅值裕度
    gm = None
    wp = None
    for i in range(len(phase) - 1):
        if phase[i] > -180 and phase[i + 1] <= -180:
            wp = np.interp(-180, [phase[i + 1], phase[i]], [w[i + 1], w[i]])
            mag_p = np.interp(wp, [w[i], w[i + 1]], [mag[i], mag[i + 1]])
            gm = -mag_p
            break

    return {"name": name, "Kp": Kp, "Ki": Ki,
            "PM": pm, "GM": gm, "wg": wg, "wp": wp,
            "w": w, "mag": mag, "phase": phase}


def simulate_discrete(Kp, Ki, setpoint, disturbance=None, noise_std=1.5, seed=42):
    """
    离散时域仿真（与代码完全一致的位置式PI + ZOH一阶对象）
    """
    rng = np.random.default_rng(seed)
    N = len(setpoint)
    rpm = np.zeros(N)
    duty = np.zeros(N)
    integral = 0.0
    x = 0.0  # 被控对象状态

    if disturbance is None:
        disturbance = np.zeros(N)

    for k in range(N):
        sp = setpoint[k]
        # 测量噪声
        noise = rng.normal(0, noise_std)
        rpm_meas = x + noise
        rpm[k] = rpm_meas

        # PI计算（与 controller.rs 逻辑一致）
        e = sp - rpm_meas
        integral += Ki * e * TS
        integral = np.clip(integral, -30.0, 30.0)  # 积分限幅
        u = Kp * e + integral
        u = np.clip(u, -100.0, 100.0)  # 输出限幅
        duty[k] = u

        # 被控对象更新（ZOH离散一阶系统）
        x = ALPHA * x + KZ * u + disturbance[k]

    return rpm, duty


def imc_tuning(lam):
    """IMC/Lambda整定法：一阶系统 + PI"""
    Kp = TAU_MOTOR / (K_PLANT * lam)
    Ki = 1.0 / (K_PLANT * lam)
    return Kp, Ki


def plot_step_response_comparison(candidates, output_dir="."):
    """图1：阶跃响应对比（报告插图）"""
    t = np.arange(0, 4.0, TS)
    N = len(t)
    setpoint = np.zeros(N)
    setpoint[t > 0.5] = 100.0  # 0.5s时100rpm阶跃

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    # 左图：转速响应
    ax = axes[0]
    for name, Kp, Ki, color in candidates:
        rpm, _ = simulate_discrete(Kp, Ki, setpoint, seed=42)
        ax.plot(t * 1000, rpm, color=color, linewidth=2.0, label=name)
    ax.axhline(100, color="gray", linestyle="--", alpha=0.6, linewidth=1.2)
    ax.axvline(500, color="gray", linestyle=":", alpha=0.4)
    ax.set_xlabel("时间 (ms)", fontsize=13)
    ax.set_ylabel("转速 (RPM)", fontsize=13)
    ax.set_title("阶跃响应：0 → 100 rpm", fontsize=14)
    ax.legend(loc="lower right", fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_xlim(0, 2500)
    ax.set_ylim(-5, 130)

    # 右图：占空比输出
    ax = axes[1]
    for name, Kp, Ki, color in candidates:
        _, duty = simulate_discrete(Kp, Ki, setpoint, seed=42)
        ax.plot(t * 1000, duty, color=color, linewidth=2.0, label=name)
    ax.set_xlabel("时间 (ms)", fontsize=13)
    ax.set_ylabel("占空比 (%)", fontsize=13)
    ax.set_title("控制器输出（仅反馈）", fontsize=14)
    ax.legend(loc="lower right", fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_xlim(0, 2500)

    plt.suptitle("PI 参数整定：阶跃响应对比",
                 fontsize=15, fontweight="bold")
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    path = f"{output_dir}/pi_tuning_step_response.png"
    plt.savefig(path, dpi=200, bbox_inches="tight")
    print(f"Saved: {path}")
    plt.close()


def plot_bode_comparison(candidates, output_dir="."):
    """图2：开环Bode图（报告插图）"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    omega = np.logspace(-1, 3, 5000)

    for name, Kp, Ki, color in candidates:
        L = compute_open_loop(Kp, Ki)
        w, mag, phase = signal.bode(L, omega)
        axes[0].semilogx(w, mag, color=color, linewidth=2.0, label=name)
        axes[1].semilogx(w, phase, color=color, linewidth=2.0, label=name)

    axes[0].axhline(0, color="black", linestyle="--", alpha=0.4, linewidth=1.0)
    axes[0].set_ylabel("幅值 (dB)", fontsize=13)
    axes[0].set_title("开环 Bode：幅值", fontsize=14)
    axes[0].legend(loc="lower left", fontsize=10)
    axes[0].grid(True, which="both", linestyle="--", alpha=0.3)
    axes[0].set_xlim(0.1, 500)

    axes[1].axhline(-180, color="black", linestyle="--", alpha=0.4, linewidth=1.0)
    axes[1].set_xlabel("频率 (rad/s)", fontsize=13)
    axes[1].set_ylabel("相位 (°)", fontsize=13)
    axes[1].set_title("开环 Bode：相位", fontsize=14)
    axes[1].legend(loc="lower left", fontsize=10)
    axes[1].grid(True, which="both", linestyle="--", alpha=0.3)
    axes[1].set_xlim(0.1, 500)

    plt.suptitle("开环频率响应",
                 fontsize=15, fontweight="bold")
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    path = f"{output_dir}/pi_tuning_bode.png"
    plt.savefig(path, dpi=200, bbox_inches="tight")
    print(f"Saved: {path}")
    plt.close()


def plot_closed_loop_comparison(candidates, output_dir="."):
    """图3：闭环特性对比（报告插图）"""
    omega = np.logspace(-1, 3, 5000)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    for name, Kp, Ki, color in candidates:
        T = compute_closed_loop(Kp, Ki)
        w, mag, phase = signal.bode(T, omega)
        axes[0].semilogx(w, mag, color=color, linewidth=2.0, label=name)
        axes[1].semilogx(w, phase, color=color, linewidth=2.0, label=name)

    axes[0].axhline(0, color="black", linestyle="--", alpha=0.4)
    axes[0].set_xlabel("频率 (rad/s)", fontsize=13)
    axes[0].set_ylabel("幅值 (dB)", fontsize=13)
    axes[0].set_title("闭环幅值", fontsize=14)
    axes[0].legend(loc="lower left", fontsize=10)
    axes[0].grid(True, which="both", linestyle="--", alpha=0.3)

    axes[1].set_xlabel("频率 (rad/s)", fontsize=13)
    axes[1].set_ylabel("相位 (°)", fontsize=13)
    axes[1].set_title("闭环相位", fontsize=14)
    axes[1].legend(loc="lower left", fontsize=10)
    axes[1].grid(True, which="both", linestyle="--", alpha=0.3)

    plt.suptitle("闭环频率响应",
                 fontsize=15, fontweight="bold")
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    path = f"{output_dir}/pi_tuning_closed_loop.png"
    plt.savefig(path, dpi=200, bbox_inches="tight")
    print(f"Saved: {path}")
    plt.close()


def plot_disturbance_rejection(candidates, output_dir="."):
    """图4：抗扰动性能对比（报告插图）"""
    t = np.arange(0, 6.0, TS)
    N = len(t)
    setpoint = np.full(N, 100.0)  # 稳态100rpm

    # 2.0s 时加入 -30% 占空比等效负载扰动（减速）
    disturbance = np.zeros(N)
    duty_disturb = -30.0  # % duty
    # 扰动持续1.0s
    mask = (t >= 2.0) & (t < 3.0)
    # 扰动折算为rpm衰减：等效为 duty 变化经过被控对象
    # 实际仿真中直接在对象状态上加等效转速扰动
    rpm_disturb = duty_disturb * K_PLANT  # ≈ -77.2 rpm
    disturbance[mask] = rpm_disturb

    fig, ax = plt.subplots(figsize=(12, 5.5))
    for name, Kp, Ki, color in candidates:
        rpm, _ = simulate_discrete(Kp, Ki, setpoint, disturbance=disturbance, seed=42)
        ax.plot(t * 1000, rpm, color=color, linewidth=2.0, label=name)

    ax.axhline(100, color="gray", linestyle="--", alpha=0.6, linewidth=1.2)
    ax.axvspan(2000, 3000, alpha=0.1, color="red", label="扰动 (-77 rpm)")
    ax.set_xlabel("时间 (ms)", fontsize=13)
    ax.set_ylabel("转速 (RPM)", fontsize=13)
    ax.set_title("抗扰动：2s 时施加 -77 rpm 负载",
                 fontsize=14)
    ax.legend(loc="lower right", fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_xlim(0, 5000)

    plt.suptitle("PI 整定：抗扰动性能对比",
                 fontsize=15, fontweight="bold")
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    path = f"{output_dir}/pi_tuning_disturbance.png"
    plt.savefig(path, dpi=200, bbox_inches="tight")
    print(f"Saved: {path}")
    plt.close()


def plot_parameter_table(candidates, output_dir="."):
    """图5：参数与性能汇总表（报告插图）"""
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.axis("off")

    # 稳定性分析
    data = []
    for name, Kp, Ki, _ in candidates:
        info = analyze_stability(Kp, Ki, name)
        pm_str = f"{info['PM']:.1f}°" if info['PM'] is not None else "N/A"
        gm_str = f"{info['GM']:.1f} dB" if info['GM'] is not None else "∞"
        wg_str = f"{info['wg']:.2f}" if info['wg'] is not None else "N/A"
        data.append([name, f"{Kp:.3f}", f"{Ki:.3f}", pm_str, gm_str, wg_str])

    headers = ["方案", "Kp", "Ki", "相位裕度", "幅值裕度", "穿越频率 wg"]
    table = ax.table(cellText=data, colLabels=headers,
                     loc="center", cellLoc="center",
                     colWidths=[0.25, 0.12, 0.12, 0.15, 0.15, 0.21])
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1.0, 2.0)
    for i in range(len(headers)):
        table[(0, i)].set_facecolor("#4472C4")
        table[(0, i)].set_text_props(color="white", fontweight="bold")

    ax.set_title("PI 参数整定汇总", fontsize=15,
                 fontweight="bold", pad=20)
    path = f"{output_dir}/pi_tuning_table.png"
    plt.savefig(path, dpi=200, bbox_inches="tight")
    print(f"Saved: {path}")
    plt.close()


def print_recommendation():
    """打印最终推荐"""
    lam_rec = TS / 2
    print("\n" + "=" * 60)
    print("           PI 参数整定结论")
    print("=" * 60)

    print("\n【系统传递函数】")
    print(f"  被控对象: Gp(s) = {K_PLANT:.3f} / ({TAU_MOTOR:.3f}·s + 1)")
    print(f"  PI控制器: C(s)  = Kp + Ki/s")
    print(f"  开环:     L(s)  = C(s)·Gp(s)")
    print(f"  闭环:     T(s)  = L(s) / (1 + L(s))")

    print("\n【关键发现】")
    print(f"  1. 电机时间常数 τ = {TAU_MOTOR*1000:.0f} ms << 控制周期 Ts = {TS*1000:.0f} ms")
    print(f"     → 电机动态在单个控制周期内已基本完成")
    print(f"  2. 前馈已承担稳态输出，反馈PI只需补偿误差和扰动")
    print(f"  3. 当前参数 (Kp=0.3, Ki=0.15) PM = 140°，非常保守")
    print(f"     → 响应慢，但鲁棒性极强")

    print("\n【推荐参数】")
    lam_use = lam_rec * 2  # λ = 0.050 s
    Kp_rec, Ki_rec = imc_tuning(lam_use)
    print(f"  Kp = {Kp_rec:.3f}")
    print(f"  Ki = {Ki_rec:.3f}")
    print(f"\n  整定方法: IMC/Lambda (λ = {lam_use:.3f}s = Ts)")
    print(f"  理由: 前馈已承担稳态输出，反馈只需补偿误差，取 λ = Ts 即可获得足够响应速度，同时保持高鲁棒性")

    print("\n【代码修改位置】")
    print("  src/main.rs: PID_KP, PID_KI")
    print("  src/controller.rs: CompositeController::new() 默认参数")
    print("=" * 60)


def main():
    output_dir = "."

    print_system_info()

    # 定义候选参数方案（推荐基于 lambda = Ts/2）
    lam_rec = TS / 2
    candidates = [
        ("当前 (Kp=0.30)", 0.30, 0.15, "#1f77b4"),
        (f"IMC λ={lam_rec*2:.3f}s", *imc_tuning(lam_rec*2), "#ff7f0e"),
    ]

    # 稳定性分析
    print("\n===== 频域稳定性分析 =====")
    print(f"{'方案':<22} {'Kp':>8} {'Ki':>8} {'PM':>10} {'GM':>10} {'ωg':>10}")
    print("-" * 70)
    for name, Kp, Ki, _ in candidates:
        info = analyze_stability(Kp, Ki, name)
        pm_str = f"{info['PM']:.1f}°" if info['PM'] is not None else "N/A"
        gm_str = f"{info['GM']:.1f}dB" if info['GM'] is not None else "∞"
        wg_str = f"{info['wg']:.2f}" if info['wg'] is not None else "N/A"
        print(f"{name:<22} {Kp:8.3f} {Ki:8.3f} {pm_str:>10} {gm_str:>10} {wg_str:>10}")

    # 生成报告插图
    print("\n===== 生成报告插图 =====")
    plot_step_response_comparison(candidates, output_dir)
    plot_bode_comparison(candidates, output_dir)
    plot_closed_loop_comparison(candidates, output_dir)
    plot_disturbance_rejection(candidates, output_dir)

    print_recommendation()


if __name__ == "__main__":
    main()
