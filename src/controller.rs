use crate::pid::{DerivativeMode, Pid};

// ═══════════════════════════════════════════════════════════════════════════
// 空载逆模型参数（由 scripts/fit_piecewise_linear.py 拟合得到）
// 正向模型：电压 → 转速（分段线性）
//   段 1 [0.00, V1]     : rpm = 0
//   段 2 [V1,  V2]      : rpm = K2 * (V - V1)
//   段 3 [V2,  V_MAX]   : rpm = K2*(V2-V1) + K3*(V-V2)
// ═══════════════════════════════════════════════════════════════════════════
const INV_V1: f32 = 1.872;      // V
const INV_V2: f32 = 3.300;      // V
const INV_K2: f32 = 27.0322;    // rpm/V
const INV_K3: f32 = 21.0;    // rpm/V
const INV_RPM2: f32 = 38.61;    // K2 * (V2 - V1)，段 2/3 分界转速

/// 100% 占空比时电机端实测电压（V）
const MOTOR_V_MAX: f32 = 11.0;

/// 积分分离阈值（rpm）：|误差| > 此值时冻结积分
const SEP_THRESHOLD_RPM: f32 = 20.0;

/// 前馈-反馈复合控制器
///
/// 结构：
///   u_total = u_ff + u_fb
///   u_ff    = 空载逆模型(目标转速) → 电压 → 占空比
///   u_fb    = PI(目标转速, 实际转速)   [带积分分离 + 输出饱和条件积分]
#[derive(Clone, Debug, defmt::Format)]
pub struct CompositeController {
    pid: Pid,
}

impl CompositeController {
    /// 创建复合控制器，使用默认 PI 参数
    ///
    /// 默认值基于 IMC/Lambda 整定（lambda = 0.05s = Ts），
    /// 对应被控对象 Gp(s) = 2.574 / (0.017*s + 1)，控制周期 Ts = 50 ms。
    /// 取 lambda = Ts 而非 Ts/2，以降低测量噪声引起的振荡。
    pub fn new() -> Self {
        let pid = Pid::new(0.132, 7.770, 0.0)
            .with_sample_time(0.05)
            .with_integral_limits(-30.0, 30.0)
            .with_output_limits(-100.0, 100.0)
            .with_derivative_mode(DerivativeMode::OnFeedback)
            .with_integral_separation(SEP_THRESHOLD_RPM);
        Self { pid }
    }

    /// 使用指定 PI 参数创建（用于从 main.rs 传入初始值）
    pub fn with_gains(kp: f32, ki: f32, kd: f32) -> Self {
        let mut s = Self::new();
        s.pid.set_kp(kp);
        s.pid.set_ki(ki);
        s.pid.set_kd(kd);
        s
    }

    // ── 前馈 ──

    /// 根据目标转速计算空载前馈电压（逆模型）。
    ///
    /// 返回值为带符号电压（正转>0，反转<0）。
    fn feedforward_voltage(&self, rpm: f32) -> f32 {
        let rpm_abs = rpm.abs();
        let v_base = if rpm_abs <= 0.0 {
            0.0
        } else if rpm_abs <= INV_RPM2 {
            INV_V1 + rpm_abs / INV_K2
        } else {
            INV_V2 + (rpm_abs - INV_RPM2) / INV_K3
        };
        if rpm >= 0.0 {
            v_base
        } else {
            -v_base
        }
    }

    // ── 反馈 ──

    /// 执行一次复合控制计算
    ///
    /// * `setpoint` — 目标转速（rpm）
    /// * `feedback` — 实际转速（rpm）
    ///
    /// 返回总输出（占空比 %，范围 [-100, 100]）
    pub fn compute(&mut self, setpoint: f32, feedback: f32) -> f32 {
        // 前馈：逆模型电压 → 占空比
        let v_ff = self.feedforward_voltage(setpoint);
        let duty_ff = (v_ff / MOTOR_V_MAX) * 100.0;

        // 反馈：PI
        let duty_fb = self.pid.compute(setpoint, feedback);

        // 复合输出并限幅
        let duty_total = duty_ff + duty_fb;
        duty_total.clamp(-100.0, 100.0)
    }

    /// 获取前馈占空比分量（用于调试/显示）
    pub fn feedforward_duty(&self, setpoint: f32) -> f32 {
        let v_ff = self.feedforward_voltage(setpoint);
        (v_ff / MOTOR_V_MAX) * 100.0
    }

    /// 重置 PI 内部状态
    pub fn reset(&mut self) {
        self.pid.reset();
    }

    /// 在线修改 PID 增益
    pub fn set_pid_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.pid.set_kp(kp);
        self.pid.set_ki(ki);
        self.pid.set_kd(kd);
    }

    /// 修改积分限幅
    pub fn set_integral_limits(&mut self, min: f32, max: f32) {
        self.pid.set_integral_limits(min, max);
    }

    /// 修改输出限幅
    pub fn set_output_limits(&mut self, min: f32, max: f32) {
        self.pid.set_output_limits(min, max);
    }
}

impl Default for CompositeController {
    fn default() -> Self {
        Self::new()
    }
}
