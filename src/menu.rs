use core::fmt::Write;
use crate::font::{F6X8, F8X16};
use crate::keypad::{Key, KeyEvent};
use crate::sh1106::Sh1106;

/// 速度控制时，每次按键增减的设定值步长（rpm）
const SPEED_SETPOINT_STEP: f32 = 5.0;
/// PI 参数在线调参步长
const PID_TUNE_STEP: f32 = 0.05;

/// 顶层菜单模式
#[derive(Clone, Copy, PartialEq)]
pub enum TopMode {
    Speed,
    Pi,
}

/// 速度子模式
#[derive(Clone, Copy, PartialEq)]
pub enum SpeedSubMode {
    Following,
    Step,
}

/// PID 参数选择
#[derive(Clone, Copy, PartialEq)]
pub enum PidParam {
    P,
    I,
    D,
}

/// 应用状态
pub struct AppState {
    pub top_mode: TopMode,
    pub speed_sub: SpeedSubMode,
    pub setpoint: f32,        // rpm
    pub actual: f32,          // rpm
    pub pwm_duty: f32,        // -100.0 ~ 100.0
    pub step_hold_speed: f32, // 进入阶跃模式时的转速
    pub p_val: f32,
    pub i_val: f32,
    pub d_val: f32,
    pub pid_sel: PidParam,
    /// 当前警告信息（空字符串表示无警告）
    pub warning: &'static str,

    // ---- 上一次绘制的状态，用于局部刷新 ----
    last_top_mode: TopMode,
    last_speed_sub: SpeedSubMode,
    last_setpoint: f32,
    last_actual: f32,
    last_pwm_duty: f32,
    last_step_hold_speed: f32,
    last_p_val: f32,
    last_i_val: f32,
    last_d_val: f32,
    last_pid_sel: PidParam,
    last_warning: &'static str,
    first_draw: bool,
}

impl AppState {
    pub const fn new() -> Self {
        Self {
            top_mode: TopMode::Speed,
            speed_sub: SpeedSubMode::Following,
            setpoint: 100.0,
            actual: 0.0,
            pwm_duty: 0.0,
            step_hold_speed: 0.0,
            p_val: 1.0,
            i_val: 0.1,
            d_val: 0.0,
            pid_sel: PidParam::P,
            warning: "",
            // 初始值与 last 不同，确保第一次强制全量绘制
            last_top_mode: TopMode::Pi,
            last_speed_sub: SpeedSubMode::Step,
            last_setpoint: -1.0,
            last_actual: -1.0,
            last_pwm_duty: -1.0,
            last_step_hold_speed: -1.0,
            last_p_val: -1.0,
            last_i_val: -1.0,
            last_d_val: -1.0,
            last_pid_sel: PidParam::D,
            last_warning: "!",
            first_draw: true,
        }
    }

    /// 处理按键事件
    pub fn handle_event(&mut self, ev: KeyEvent) {
        match self.top_mode {
            TopMode::Speed => self.handle_speed_event(ev),
            TopMode::Pi => self.handle_pi_event(ev),
        }
    }

    fn handle_speed_event(&mut self, ev: KeyEvent) {
        match ev {
            KeyEvent::Pressed(Key::K1) => {
                self.setpoint += SPEED_SETPOINT_STEP;
                defmt::info!("Setpoint +{} => {} rpm", SPEED_SETPOINT_STEP, self.setpoint);
            }
            KeyEvent::Pressed(Key::K2) => {
                self.setpoint -= SPEED_SETPOINT_STEP;
                defmt::info!("Setpoint -{} => {} rpm", SPEED_SETPOINT_STEP, self.setpoint);
            }
            KeyEvent::Pressed(Key::K3) => {
                if self.speed_sub == SpeedSubMode::Step {
                    self.speed_sub = SpeedSubMode::Following;
                    defmt::info!("Confirmed: switch to FOLLOWING mode");
                } else {
                    self.top_mode = TopMode::Pi;
                    defmt::info!("Switch to PID TUNE menu");
                }
            }
            KeyEvent::Pressed(Key::K4) => {
                if self.speed_sub == SpeedSubMode::Following {
                    self.speed_sub = SpeedSubMode::Step;
                    self.step_hold_speed = self.actual;
                    defmt::info!(
                        "Enter STEP mode, hold speed = {} rpm",
                        self.step_hold_speed
                    );
                }
            }
            _ => {}
        }
    }

    fn handle_pi_event(&mut self, ev: KeyEvent) {
        match ev {
            KeyEvent::Pressed(Key::K1) => {
                match self.pid_sel {
                    PidParam::P => {
                        self.p_val += PID_TUNE_STEP;
                        defmt::info!("P +{} => {}", PID_TUNE_STEP, self.p_val);
                    }
                    PidParam::I => {
                        self.i_val += PID_TUNE_STEP;
                        defmt::info!("I +{} => {}", PID_TUNE_STEP, self.i_val);
                    }
                    PidParam::D => {
                        self.d_val += PID_TUNE_STEP;
                        defmt::info!("D +{} => {}", PID_TUNE_STEP, self.d_val);
                    }
                }
            }
            KeyEvent::Pressed(Key::K2) => {
                match self.pid_sel {
                    PidParam::P => {
                        self.p_val -= PID_TUNE_STEP;
                        if self.p_val < 0.0 {
                            self.p_val = 0.0;
                        }
                        defmt::info!("P -{} => {}", PID_TUNE_STEP, self.p_val);
                    }
                    PidParam::I => {
                        self.i_val -= PID_TUNE_STEP;
                        if self.i_val < 0.0 {
                            self.i_val = 0.0;
                        }
                        defmt::info!("I -{} => {}", PID_TUNE_STEP, self.i_val);
                    }
                    PidParam::D => {
                        self.d_val -= PID_TUNE_STEP;
                        if self.d_val < 0.0 {
                            self.d_val = 0.0;
                        }
                        defmt::info!("D -{} => {}", PID_TUNE_STEP, self.d_val);
                    }
                }
            }
            KeyEvent::Pressed(Key::K3) => {
                self.top_mode = TopMode::Speed;
                self.speed_sub = SpeedSubMode::Following;
                defmt::info!("Switch to SPEED menu (FOLLOWING)");
            }
            KeyEvent::Pressed(Key::K4) => {
                self.pid_sel = match self.pid_sel {
                    PidParam::P => PidParam::I,
                    PidParam::I => PidParam::D,
                    PidParam::D => PidParam::P,
                };
                defmt::info!(
                    "Select {}",
                    match self.pid_sel {
                        PidParam::P => "P",
                        PidParam::I => "I",
                        PidParam::D => "D",
                    }
                );
            }
            _ => {}
        }
    }

    /// 真实系统中 actual / pwm_duty 由硬件任务更新，
    /// 此函数保留用于兼容原有 100 ms 调用周期。
    pub fn update_motor(&mut self) {
        // no-op in real hardware
    }

    // ========================================================================
    // 绘制接口 —— 局部刷新
    // ========================================================================

    /// 绘制到 OLED（自动判断全量 / 局部刷新）
    pub fn render<I2C: embedded_hal::i2c::I2c>(&mut self, oled: &mut Sh1106<I2C>) {
        let mode_changed = self.top_mode != self.last_top_mode || self.first_draw;

        if mode_changed {
            // 菜单切换：全清并重绘
            oled.clear();
            match self.top_mode {
                TopMode::Speed => self.render_speed_full(oled),
                TopMode::Pi => self.render_pi_full(oled),
            }
        } else {
            // 同菜单内：只刷新变化字段
            match self.top_mode {
                TopMode::Speed => self.render_speed_delta(oled),
                TopMode::Pi => self.render_pi_delta(oled),
            }
        }

        self.sync_last_state();
    }

    fn sync_last_state(&mut self) {
        self.last_top_mode = self.top_mode;
        self.last_speed_sub = self.speed_sub;
        self.last_setpoint = self.setpoint;
        self.last_actual = self.actual;
        self.last_pwm_duty = self.pwm_duty;
        self.last_step_hold_speed = self.step_hold_speed;
        self.last_p_val = self.p_val;
        self.last_i_val = self.i_val;
        self.last_d_val = self.d_val;
        self.last_pid_sel = self.pid_sel;
        self.last_warning = self.warning;
        self.first_draw = false;
    }

    // ---------------- Speed 菜单：全量绘制 ----------------
    fn render_speed_full<I2C: embedded_hal::i2c::I2c>(&self, oled: &mut Sh1106<I2C>) {
        let title = match self.speed_sub {
            SpeedSubMode::Following => "SPEED-FOLLOW",
            SpeedSubMode::Step => "SPEED-STEP",
        };
        oled.draw_string_8x16(0, 0, title, &F8X16);

        let mut buf = heapless::String::<32>::new();
        let _ = write!(buf, "SP:{:5.0}rpm", self.setpoint);
        oled.draw_string_6x8(2, 0, &buf, &F6X8);

        buf.clear();
        let _ = write!(buf, "AC:{:5.1}rpm", self.actual);
        oled.draw_string_6x8(3, 0, &buf, &F6X8);

        buf.clear();
        let _ = write!(buf, "PWM:{:5.1}%", self.pwm_duty);
        oled.draw_string_6x8(4, 0, &buf, &F6X8);

        // page 5: warning
        if !self.warning.is_empty() {
            oled.draw_string_6x8(5, 0, self.warning, &F6X8);
        }

        // page 6: HOLD（Step 模式）
        if self.speed_sub == SpeedSubMode::Step {
            buf.clear();
            let _ = write!(buf, "HOLD:{:5.1}rpm", self.step_hold_speed);
            oled.draw_string_6x8(6, 0, &buf, &F6X8);
        }

        // page 7: 按键提示（常驻底行）
        oled.draw_string_6x8(7, 0, "S1+ S2- S3 OK S4 STEP", &F6X8);
    }

    // ---------------- Speed 菜单：局部刷新 ----------------
    fn render_speed_delta<I2C: embedded_hal::i2c::I2c>(&self, oled: &mut Sh1106<I2C>) {
        let mut buf = heapless::String::<32>::new();

        // Title: page 0-1, 8x16 font
        if self.speed_sub != self.last_speed_sub {
            // 清除标题区域（最多 12 字符 × 8 = 96 px）
            oled.clear_cols(0, 0, 100);
            oled.clear_cols(1, 0, 100);
            let title = match self.speed_sub {
                SpeedSubMode::Following => "SPEED-FOLLOW",
                SpeedSubMode::Step => "SPEED-STEP",
            };
            oled.draw_string_8x16(0, 0, title, &F8X16);
        }

        // SP line: page 2, 6x8 font, "SP:xxxxrpm" ≤ 10×6 = 60 px
        if self.setpoint != self.last_setpoint {
            oled.clear_cols(2, 0, 70);
            let _ = write!(buf, "SP:{:5.0}rpm", self.setpoint);
            oled.draw_string_6x8(2, 0, &buf, &F6X8);
            buf.clear();
        }

        // AC line: page 3
        if self.actual != self.last_actual {
            oled.clear_cols(3, 0, 70);
            let _ = write!(buf, "AC:{:5.1}rpm", self.actual);
            oled.draw_string_6x8(3, 0, &buf, &F6X8);
            buf.clear();
        }

        // PWM line: page 4
        if self.pwm_duty != self.last_pwm_duty {
            oled.clear_cols(4, 0, 70);
            let _ = write!(buf, "PWM:{:5.1}%", self.pwm_duty);
            oled.draw_string_6x8(4, 0, &buf, &F6X8);
            buf.clear();
        }

        // Warning line: page 5
        if self.warning != self.last_warning {
            oled.clear_cols(5, 0, 128);
            if !self.warning.is_empty() {
                oled.draw_string_6x8(5, 0, self.warning, &F6X8);
            }
        }

        // HOLD line: page 6 —— 仅在 Step 模式或从 Step 退出时处理
        if self.speed_sub != self.last_speed_sub || self.step_hold_speed != self.last_step_hold_speed
        {
            oled.clear_cols(6, 0, 80); // "HOLD:xxxxrpm" ≤ 12×6 = 72 px
            if self.speed_sub == SpeedSubMode::Step {
                let _ = write!(buf, "HOLD:{:5.1}rpm", self.step_hold_speed);
                oled.draw_string_6x8(6, 0, &buf, &F6X8);
            }
        }
    }

    // ---------------- PID 菜单：全量绘制 ----------------
    fn render_pi_full<I2C: embedded_hal::i2c::I2c>(&self, oled: &mut Sh1106<I2C>) {
        oled.draw_string_8x16(0, 0, "PID TUNE", &F8X16);

        let mut buf = heapless::String::<32>::new();
        let p_marker = if self.pid_sel == PidParam::P { ">" } else { " " };
        let _ = write!(buf, "{}P:{:.2}", p_marker, self.p_val);
        oled.draw_string_6x8(2, 0, &buf, &F6X8);

        buf.clear();
        let i_marker = if self.pid_sel == PidParam::I { ">" } else { " " };
        let _ = write!(buf, "{}I:{:.2}", i_marker, self.i_val);
        oled.draw_string_6x8(3, 0, &buf, &F6X8);

        buf.clear();
        let d_marker = if self.pid_sel == PidParam::D { ">" } else { " " };
        let _ = write!(buf, "{}D:{:.2}", d_marker, self.d_val);
        oled.draw_string_6x8(4, 0, &buf, &F6X8);

        oled.draw_string_6x8(6, 0, "S1+ S2- S3 SPD S4 SEL", &F6X8);
    }

    // ---------------- PID 菜单：局部刷新 ----------------
    fn render_pi_delta<I2C: embedded_hal::i2c::I2c>(&self, oled: &mut Sh1106<I2C>) {
        let mut buf = heapless::String::<32>::new();

        // P line: page 2, ">P:x.x" ≤ 7×6 = 42 px
        if self.p_val != self.last_p_val || self.pid_sel != self.last_pid_sel {
            oled.clear_cols(2, 0, 50);
            let p_marker = if self.pid_sel == PidParam::P { ">" } else { " " };
            let _ = write!(buf, "{}P:{:.2}", p_marker, self.p_val);
            oled.draw_string_6x8(2, 0, &buf, &F6X8);
            buf.clear();
        }

        // I line: page 3
        if self.i_val != self.last_i_val || self.pid_sel != self.last_pid_sel {
            oled.clear_cols(3, 0, 50);
            let i_marker = if self.pid_sel == PidParam::I { ">" } else { " " };
            let _ = write!(buf, "{}I:{:.2}", i_marker, self.i_val);
            oled.draw_string_6x8(3, 0, &buf, &F6X8);
            buf.clear();
        }

        // D line: page 4
        if self.d_val != self.last_d_val || self.pid_sel != self.last_pid_sel {
            oled.clear_cols(4, 0, 50);
            let d_marker = if self.pid_sel == PidParam::D { ">" } else { " " };
            let _ = write!(buf, "{}D:{:.2}", d_marker, self.d_val);
            oled.draw_string_6x8(4, 0, &buf, &F6X8);
        }
    }
}
