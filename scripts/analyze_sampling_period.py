#!/usr/bin/env python3
"""
控制周期/采样周期合理性分析与优化建议

分析维度：
  1. 控制理论：Ts 与 tau 的关系、离散化特性
  2. 测速精度：M/T 法编码器计数约束
  3. 数字稳定性：ZOH 离散化后的极点位置
  4. 响应性能：不同 Ts 下的阶跃响应对比

用法：
    python scripts/analyze_sampling_period.py
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 使用微软雅黑
fm.fontManager.addfont("/usr/local/share/fonts/truetype/微软雅黑.ttf")
plt.rcParams['font.family'] = ['Microsoft YaHei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False


# ═══════════════════════════════════════════════════════════════════
# 系统参数
# ═══════════════════════════════════════════════════════════════════
TAU = 0.017          # s, 电机时间常数
K_PLANT = 2.574      # rpm/%
COUNTS_PER_REV = 924
RPM_MAX = 200.0      # rpm
RPM_DEADZONE = 14.0  # rpm
V_MAX = 11.0         # V


def analyze_speed_measurement_accuracy():
    """图1：M/T 法测速精度随 Ts 和转速的变化"""
    Ts_values = np.array([5, 10, 20, 50, 100, 200])  # ms
    rpm_values = np.array([14, 30, 50, 100, 150, 200])

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    # 左图：不同 Ts 下，各转速的编码器计数
    ax = axes[0]
    for Ts in Ts_values:
        counts = []
        for rpm in rpm_values:
            rev_per_sec = rpm / 60.0
            counts_per_sec = rev_per_sec * COUNTS_PER_REV
            c = counts_per_sec * (Ts / 1000.0)
            counts.append(c)
        ax.plot(rpm_values, counts, '-o', linewidth=2, markersize=6,
                label=f'Ts = {Ts} ms')
    ax.axhline(y=5, color='red', linestyle='--', linewidth=1.5,
               label='最小可信计数 = 5')
    ax.axhline(y=10, color='orange', linestyle='--', linewidth=1.5,
               label='良好计数 = 10')
    ax.set_xlabel('转速 (RPM)', fontsize=12)
    ax.set_ylabel('Ts 窗口内编码器计数', fontsize=12)
    ax.set_title('M/T 法测速：编码器计数 vs 转速', fontsize=13)
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.set_xlim(0, 220)

    # 右图：不同 Ts 下，死区边界的测速相对误差
    ax = axes[1]
    Ts_range = np.linspace(5, 200, 100)
    for rpm in [14, 30, 50, 100]:
        rev_per_sec = rpm / 60.0
        counts_per_sec = rev_per_sec * COUNTS_PER_REV
        counts = counts_per_sec * (Ts_range / 1000.0)
        err_pct = 1.0 / counts * 100  # ±1 count 误差
        ax.semilogy(Ts_range, err_pct, linewidth=2, label=f'{rpm} RPM')

    ax.axhline(y=5, color='red', linestyle='--', linewidth=1.5,
               label='5% 误差线')
    ax.axhline(y=10, color='orange', linestyle='--', linewidth=1.5,
               label='10% 误差线')
    ax.set_xlabel('控制周期 Ts (ms)', fontsize=12)
    ax.set_ylabel('±1 count 相对误差 (%)', fontsize=12)
    ax.set_title('测速量化误差 vs 控制周期', fontsize=13)
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.4, which='both')
    ax.set_xlim(0, 200)

    plt.suptitle('编码器 M/T 法测速精度分析', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('doc/figures/sampling_period_accuracy.png', dpi=200, bbox_inches='tight')
    print('Saved: doc/figures/sampling_period_accuracy.png')
    plt.close()


def analyze_control_theory():
    """图2：控制理论角度分析 Ts 的合理性"""
    Ts_values = np.linspace(1, 250, 250)  # ms
    Ts_s = Ts_values / 1000.0

    ratio = Ts_s / TAU
    alpha = np.exp(-Ts_s / TAU)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    # 左图：Ts/tau 比值
    ax = axes[0]
    ax.fill_between(Ts_values, 0, 0.5, alpha=0.2, color='green',
                    label='理想区 (Ts <= tau/2)')
    ax.fill_between(Ts_values, 0.5, 2.0, alpha=0.15, color='yellow',
                    label='可接受 (tau/2 < Ts <= 2*tau)')
    ax.fill_between(Ts_values, 2.0, 20, alpha=0.15, color='red',
                    label='过大 (Ts > 2*tau)')
    ax.plot(Ts_values, ratio, 'b-', linewidth=2.5)
    ax.axvline(x=200, color='purple', linestyle='--', linewidth=2,
               label=f'当前 Ts = 200 ms ({200/(TAU*1000):.1f}·tau)')
    ax.axhline(y=200/(TAU*1000), color='purple', linestyle=':', alpha=0.5)

    # 标注区域
    ax.annotate(' tau/10 线', xy=(200, 0.1), fontsize=10, color='green')
    ax.axhline(y=0.1, color='green', linestyle=':', alpha=0.5)
    ax.annotate(' tau/5 线', xy=(200, 0.2), fontsize=10, color='green')
    ax.axhline(y=0.2, color='green', linestyle=':', alpha=0.5)

    ax.set_xlabel('控制周期 Ts (ms)', fontsize=12)
    ax.set_ylabel('Ts / tau', fontsize=12)
    ax.set_title('Ts 与电机时间常数的比值', fontsize=13)
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.set_xlim(0, 250)
    ax.set_ylim(0, 15)

    # 右图：离散极点 alpha
    ax = axes[1]
    ax.semilogy(Ts_values, alpha, 'b-', linewidth=2.5)
    ax.axvline(x=200, color='purple', linestyle='--', linewidth=2,
               label=f'当前 Ts = 200 ms')
    ax.axhline(y=0.05, color='red', linestyle='--', linewidth=1.5,
               label='alpha = 0.05 (动态丢失阈值)')
    ax.axhline(y=0.5, color='orange', linestyle='--', linewidth=1.5,
               label='alpha = 0.5 (良好离散化)')

    # 标注当前值
    ax.scatter([200], [np.exp(-0.2/TAU)], color='purple', s=100, zorder=5)
    ax.annotate(f'alpha = {np.exp(-0.2/TAU):.2e}\n≈ 0 (纯延迟)',
                xy=(200, np.exp(-0.2/TAU)),
                xytext=(150, 1e-4),
                fontsize=10,
                arrowprops=dict(arrowstyle='->', color='purple'))

    ax.set_xlabel('控制周期 Ts (ms)', fontsize=12)
    ax.set_ylabel('离散极点 alpha = exp(-Ts/tau)', fontsize=12)
    ax.set_title('ZOH 离散化后的极点位置', fontsize=13)
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.3, which='both')
    ax.set_xlim(0, 250)

    plt.suptitle('控制理论：Ts 合理性分析', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('doc/figures/sampling_period_theory.png', dpi=200, bbox_inches='tight')
    print('Saved: doc/figures/sampling_period_theory.png')
    plt.close()


def simulate_step_with_Ts(Ts, Kp, Ki, setpoint, t_total=2.0):
    """给定 Ts 的离散阶跃响应仿真"""
    alpha = np.exp(-Ts / TAU)
    Kz = K_PLANT * (1 - alpha)
    N = int(t_total / Ts) + 1
    t = np.arange(N) * Ts
    rpm = np.zeros(N)
    duty = np.zeros(N)
    integral = 0.0
    x = 0.0
    sp = setpoint

    rng = np.random.default_rng(42)
    noise_std = 1.5

    for k in range(N):
        noise = rng.normal(0, noise_std)
        rpm_meas = x + noise
        rpm[k] = rpm_meas

        e = sp - rpm_meas
        integral += Ki * e * Ts
        integral = np.clip(integral, -30.0, 30.0)
        u = Kp * e + integral
        u = np.clip(u, -100.0, 100.0)
        duty[k] = u

        x = alpha * x + Kz * u

    return t, rpm, duty


def analyze_step_response_comparison():
    """图3：不同 Ts 下的阶跃响应对比（固定 lambda = Ts/2）"""
    Ts_candidates = [0.020, 0.050, 0.100, 0.200]  # s
    colors = ['#d62728', '#ff7f0e', '#2ca02c', '#1f77b4']

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    # 左图：转速响应
    ax = axes[0]
    for Ts, color in zip(Ts_candidates, colors):
        lam = Ts / 2
        Kp = TAU / (K_PLANT * lam)
        Ki = 1.0 / (K_PLANT * lam)
        t, rpm, _ = simulate_step_with_Ts(Ts, Kp, Ki, 100.0)
        ax.plot(t * 1000, rpm, color=color, linewidth=2.0,
                label=f'Ts={int(Ts*1000)}ms, λ={lam:.3f}s, Kp={Kp:.3f}')

    ax.axhline(100, color='gray', linestyle='--', alpha=0.6, linewidth=1.2)
    ax.set_xlabel('时间 (ms)', fontsize=12)
    ax.set_ylabel('转速 (RPM)', fontsize=12)
    ax.set_title('不同 Ts 下的阶跃响应（固定 λ=Ts/2）', fontsize=13)
    ax.legend(loc='lower right', fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.set_xlim(0, 1500)
    ax.set_ylim(-5, 130)

    # 右图：占空比输出
    ax = axes[1]
    for Ts, color in zip(Ts_candidates, colors):
        lam = Ts / 2
        Kp = TAU / (K_PLANT * lam)
        Ki = 1.0 / (K_PLANT * lam)
        t, _, duty = simulate_step_with_Ts(Ts, Kp, Ki, 100.0)
        ax.plot(t * 1000, duty, color=color, linewidth=2.0,
                label=f'Ts={int(Ts*1000)}ms')

    ax.set_xlabel('时间 (ms)', fontsize=12)
    ax.set_ylabel('占空比 (%)', fontsize=12)
    ax.set_title('控制器输出（仅反馈）', fontsize=13)
    ax.legend(loc='lower right', fontsize=9)
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.set_xlim(0, 1500)

    plt.suptitle('不同控制周期下的 IMC 整定阶跃响应对比', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('doc/figures/sampling_period_step.png', dpi=200, bbox_inches='tight')
    print('Saved: doc/figures/sampling_period_step.png')
    plt.close()


def print_recommendation():
    """打印分析结论与调整建议"""
    print()
    print('=' * 75)
    print('           控制周期/采样周期分析结论')
    print('=' * 75)
    print()
    print('【当前状态评估】')
    print(f'  控制周期 Ts = 200 ms = {200/17:.1f} × tau (17 ms)')
    print('  → 从控制理论角度：Ts 远大于 tau，过大')
    print('  → 从测速精度角度：Ts=200ms 在 14rpm 时有 43 counts，精度良好')
    print()
    print('【核心矛盾】')
    print('  控制理论要求 Ts << tau（理想 2~5 ms），')
    print('  但编码器测速要求 Ts 足够大以保证低速计数。')
    print()
    print('【不同 Ts 的权衡分析】')
    print()
    print(f"{'Ts':>6} | {'Ts/tau':>7} | {'14rpm误差':>10} | {'100rpm误差':>11} | {'IMC Kp':>8} | {'IMC Ki':>8} | {'评价':>20}")
    print('-' * 90)

    for Ts_ms in [10, 20, 50, 100, 200]:
        Ts = Ts_ms / 1000.0
        ratio = Ts / TAU
        # 测速误差
        for rpm, label in [(14, '14rpm'), (100, '100rpm')]:
            rev_per_sec = rpm / 60.0
            counts = rev_per_sec * COUNTS_PER_REV * Ts
            err = 1.0 / counts * 100
        err_14 = 1.0 / (14/60 * COUNTS_PER_REV * Ts) * 100
        err_100 = 1.0 / (100/60 * COUNTS_PER_REV * Ts) * 100
        # IMC 参数
        lam = Ts / 2
        Kp = TAU / (K_PLANT * lam)
        Ki = 1.0 / (K_PLANT * lam)

        if Ts_ms <= 20:
            eval_str = '测速差，控制理想'
        elif Ts_ms <= 50:
            eval_str = '可尝试，需验证'
        elif Ts_ms <= 100:
            eval_str = '测速好，控制偏大'
        else:
            eval_str = '当前：测速好，控制慢'

        print(f'{Ts_ms:>5}ms | {ratio:>7.1f} | {err_14:>9.1f}% | {err_100:>10.1f}% | {Kp:>8.3f} | {Ki:>8.3f} | {eval_str}')

    print()
    print('【调整建议】')
    print()
    print('  方案 A：维持 Ts = 200 ms（保守方案）')
    print('    - 优点：测速精度高，低速稳定，代码改动最小')
    print('    - 缺点：控制响应慢，tau << Ts 导致 PI 近似调节静态增益')
    print('    - 适用：对响应速度要求不高、负载扰动较小的场景')
    print()
    print('  方案 B：Ts = 50 ms（推荐折中方案）')
    print('    - 优点：Ts/tau = 2.9，仍能部分分辨电机动态；')
    print('      100 rpm 时测速误差 1.3%，14 rpm 时 9.3% 可接受')
    print('    - 缺点：低速测速噪声增大，可能需要软件滤波')
    print('    - 需改：TIM3 中断周期、M/T 法测速窗口、PI 参数重新整定')
    print('    - IMC 新参数：Kp = 0.264, Ki = 15.540')
    print()
    print('  方案 C：Ts = 20 ms（激进方案）')
    print('    - 优点：Ts/tau = 1.2，能较好分辨电机动态')
    print('    - 缺点：14 rpm 时测速误差 23%，低速控制可能不稳定')
    print('    - 需改：增加编码器分辨率，或换用 T 法/锁相环测速')
    print('    - IMC 新参数：Kp = 0.660, Ki = 38.850')
    print()
    print('  方案 D：双周期策略（高级方案）')
    print('    - 控制周期 = 50 ms（PI 计算 + PWM 输出）')
    print('    - 测速周期 = 200 ms（M/T 法，保证低速精度）')
    print('    - 优点：兼顾控制响应和测速精度')
    print('    - 缺点：实现复杂，需要插值或预测')
    print()
    print('【最终建议】')
    print('  推荐先尝试方案 B（Ts = 50 ms）：')
    print('    1. 修改 TIM3 中断周期为 50 ms')
    print('    2. 重新运行 tune_pi_controller.py（Ts=0.05）整定 PI')
    print('    3. 低速（< 30 rpm）时适当增加软件滤波或降低控制增益')
    print('    4. 实测验证后，若稳定可进一步尝试 Ts = 20 ms')
    print()
    print('=' * 75)


def main():
    analyze_speed_measurement_accuracy()
    analyze_control_theory()
    analyze_step_response_comparison()
    print_recommendation()


if __name__ == '__main__':
    main()
