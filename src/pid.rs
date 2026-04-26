/// 微分模式
#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
pub enum DerivativeMode {
    /// 对误差微分（de/dt），设定值突变时微分项会有冲击
    OnError,
    /// 对反馈量微分（-df/dt），设定值突变时无冲击，更平滑
    OnFeedback,
}

/// 带积分限幅与输出限幅的位置式 PID 控制器
///
/// # 示例
/// ```rust,ignore
/// let mut pid = Pid::new(1.0, 0.1, 0.01)
///     .with_sample_time(0.01)          // 10 ms
///     .with_integral_limits(-10.0, 10.0)
///     .with_output_limits(-100.0, 100.0)
///     .with_derivative_mode(DerivativeMode::OnFeedback);
///
/// let output = pid.compute(50.0, 48.0); // setpoint=50, feedback=48
/// ```
#[derive(Clone, Debug, defmt::Format)]
pub struct Pid {
    // ── 参数 ──
    kp: f32,
    ki: f32,
    kd: f32,
    /// 采样周期 Ts（秒）
    ts: f32,
    /// 积分下限
    integral_min: f32,
    /// 积分上限
    integral_max: f32,
    /// 输出下限
    output_min: f32,
    /// 输出上限
    output_max: f32,
    derivative_mode: DerivativeMode,

    // ── 状态 ──
    /// 当前积分累积值
    integral: f32,
    /// 上一次误差（用于 OnError 微分）
    prev_error: f32,
    /// 上一次反馈量（用于 OnFeedback 微分）
    prev_feedback: f32,
    /// 是否为首次计算（用于初始化 prev_*）
    first_run: bool,
}

impl Default for Pid {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            ts: 1.0,
            integral_min: f32::NEG_INFINITY,
            integral_max: f32::INFINITY,
            output_min: f32::NEG_INFINITY,
            output_max: f32::INFINITY,
            derivative_mode: DerivativeMode::OnError,
            integral: 0.0,
            prev_error: 0.0,
            prev_feedback: 0.0,
            first_run: true,
        }
    }
}

impl Pid {
    /// 创建一个新的 PID 控制器
    ///
    /// * `kp` — 比例增益
    /// * `ki` — 积分增益
    /// * `kd` — 微分增益
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            ..Default::default()
        }
    }

    /// 设置采样周期（秒）
    pub fn with_sample_time(mut self, ts: f32) -> Self {
        self.ts = ts;
        self
    }

    /// 设置积分限幅（抗饱和）
    pub fn with_integral_limits(mut self, min: f32, max: f32) -> Self {
        self.integral_min = min;
        self.integral_max = max;
        self
    }

    /// 设置输出限幅
    pub fn with_output_limits(mut self, min: f32, max: f32) -> Self {
        self.output_min = min;
        self.output_max = max;
        self
    }

    /// 设置微分模式
    pub fn with_derivative_mode(mut self, mode: DerivativeMode) -> Self {
        self.derivative_mode = mode;
        self
    }

    // ── 运行时参数修改 ──

    /// 修改比例增益
    pub fn set_kp(&mut self, kp: f32) {
        self.kp = kp;
    }

    /// 修改积分增益
    pub fn set_ki(&mut self, ki: f32) {
        self.ki = ki;
    }

    /// 修改微分增益
    pub fn set_kd(&mut self, kd: f32) {
        self.kd = kd;
    }

    /// 修改采样周期（秒）
    pub fn set_sample_time(&mut self, ts: f32) {
        self.ts = ts;
    }

    /// 修改积分限幅
    pub fn set_integral_limits(&mut self, min: f32, max: f32) {
        self.integral_min = min;
        self.integral_max = max;
    }

    /// 修改输出限幅
    pub fn set_output_limits(&mut self, min: f32, max: f32) {
        self.output_min = min;
        self.output_max = max;
    }

    /// 重置控制器内部状态（积分、历史误差等）
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.prev_feedback = 0.0;
        self.first_run = true;
    }

    // ── 计算 ──

    /// 执行一次 PID 计算
    ///
    /// * `setpoint` — 设定值（目标）
    /// * `feedback` — 反馈值（实际测量值）
    ///
    /// 返回限幅后的输出值
    pub fn compute(&mut self, setpoint: f32, feedback: f32) -> f32 {
        let error = setpoint - feedback;

        // ── 比例项 ──
        let proportional = self.kp * error;

        // ── 积分项（带限幅）──
        self.integral += self.ki * error * self.ts;
        self.integral = clamp(self.integral, self.integral_min, self.integral_max);

        // ── 微分项 ──
        let derivative = if self.first_run {
            // 首次运行：初始化历史值，微分项为 0
            match self.derivative_mode {
                DerivativeMode::OnError => self.prev_error = error,
                DerivativeMode::OnFeedback => self.prev_feedback = feedback,
            }
            self.first_run = false;
            0.0
        } else {
            match self.derivative_mode {
                DerivativeMode::OnError => {
                    let d = (error - self.prev_error) / self.ts;
                    self.prev_error = error;
                    self.kd * d
                }
                DerivativeMode::OnFeedback => {
                    let d = (feedback - self.prev_feedback) / self.ts;
                    self.prev_feedback = feedback;
                    -self.kd * d // 负号使得反馈上升时产生制动效果
                }
            }
        };

        // ── 输出限幅 ──
        let output = proportional + self.integral + derivative;
        clamp(output, self.output_min, self.output_max)
    }

    // ── 查询 ──

    /// 当前积分累积值
    pub fn integral(&self) -> f32 {
        self.integral
    }

    /// 当前比例增益
    pub fn kp(&self) -> f32 {
        self.kp
    }

    /// 当前积分增益
    pub fn ki(&self) -> f32 {
        self.ki
    }

    /// 当前微分增益
    pub fn kd(&self) -> f32 {
        self.kd
    }

    /// 当前采样周期
    pub fn sample_time(&self) -> f32 {
        self.ts
    }
}

/// 通用限幅函数
#[inline]
fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

// ── 单元测试 ──
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_pid() {
        let mut pid = Pid::new(1.0, 0.0, 0.0);
        assert_eq!(pid.compute(10.0, 5.0), 5.0);
    }

    #[test]
    fn test_integral_windup() {
        // Ki=1, Ts=1，误差恒为 10，积分项会迅速累积
        let mut pid = Pid::new(0.0, 1.0, 0.0)
            .with_sample_time(1.0)
            .with_integral_limits(-5.0, 5.0)
            .with_output_limits(-100.0, 100.0);

        // 第一次：积分 = 10，被限幅到 5，输出 = 5
        let out1 = pid.compute(10.0, 0.0);
        assert!((out1 - 5.0).abs() < 1e-6);
        assert!((pid.integral() - 5.0).abs() < 1e-6);

        // 第二次：积分尝试到 15，仍被限幅到 5
        let out2 = pid.compute(10.0, 0.0);
        assert!((out2 - 5.0).abs() < 1e-6);
        assert!((pid.integral() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_output_clamp() {
        let mut pid = Pid::new(10.0, 0.0, 0.0).with_output_limits(-50.0, 50.0);
        // error=10, P=100，输出应被限幅到 50
        assert_eq!(pid.compute(10.0, 0.0), 50.0);
        // error=-10, P=-100，输出应被限幅到 -50
        assert_eq!(pid.compute(0.0, 10.0), -50.0);
    }

    #[test]
    fn test_derivative_on_feedback() {
        let mut pid = Pid::new(0.0, 0.0, 1.0)
            .with_sample_time(1.0)
            .with_derivative_mode(DerivativeMode::OnFeedback);

        // 第一次运行无微分
        let _ = pid.compute(100.0, 0.0);
        // 第二次：feedback 从 0→10，df/dt=10，微分项 = -10
        let out = pid.compute(100.0, 10.0);
        assert!((out + 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_reset() {
        let mut pid = Pid::new(0.0, 1.0, 0.0)
            .with_sample_time(1.0)
            .with_integral_limits(0.0, 10.0);

        pid.compute(10.0, 0.0);
        assert!(pid.integral() > 0.0);

        pid.reset();
        assert_eq!(pid.integral(), 0.0);
        assert!(pid.compute(10.0, 0.0) > 0.0); // 首次运行重新初始化
    }
}
