use core::fmt::Write;
use dc_motor_speed_control_embassy::font::{F6X8, F8X16};
use dc_motor_speed_control_embassy::keypad::{Key, KeyEvent};
use dc_motor_speed_control_embassy::sh1106::Sh1106;

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

/// PI 参数选择
#[derive(Clone, Copy, PartialEq)]
pub enum PiParam {
    P,
    I,
}

/// 应用状态
pub struct AppState {
    pub top_mode: TopMode,
    pub speed_sub: SpeedSubMode,
    pub setpoint: i32,       // rpm
    pub actual: i32,         // rpm
    pub step_hold_speed: i32, // 进入阶跃模式时的转速
    pub p_val: f32,
    pub i_val: f32,
    pub pi_sel: PiParam,

    // ---- 上一次绘制的状态，用于局部刷新 ----
    last_top_mode: TopMode,
    last_speed_sub: SpeedSubMode,
    last_setpoint: i32,
    last_actual: i32,
    last_step_hold_speed: i32,
    last_p_val: f32,
    last_i_val: f32,
    last_pi_sel: PiParam,
    first_draw: bool,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            top_mode: TopMode::Speed,
            speed_sub: SpeedSubMode::Following,
            setpoint: 100,
            actual: 0,
            step_hold_speed: 0,
            p_val: 1.0,
            i_val: 0.5,
            pi_sel: PiParam::P,
            // 初始值与 last 不同，确保第一次强制全量绘制
            last_top_mode: TopMode::Pi,
            last_speed_sub: SpeedSubMode::Step,
            last_setpoint: -1,
            last_actual: -1,
            last_step_hold_speed: -1,
            last_p_val: -1.0,
            last_i_val: -1.0,
            last_pi_sel: PiParam::I,
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
                self.setpoint += 10;
                defmt::info!("Setpoint +10 => {} rpm", self.setpoint);
            }
            KeyEvent::Pressed(Key::K2) => {
                self.setpoint -= 10;
                defmt::info!("Setpoint -10 => {} rpm", self.setpoint);
            }
            KeyEvent::Pressed(Key::K3) => {
                if self.speed_sub == SpeedSubMode::Step {
                    self.speed_sub = SpeedSubMode::Following;
                    defmt::info!("Confirmed: switch to FOLLOWING mode");
                } else {
                    self.top_mode = TopMode::Pi;
                    defmt::info!("Switch to PI TUNE menu");
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
                match self.pi_sel {
                    PiParam::P => {
                        self.p_val += 0.1;
                        defmt::info!("P +0.1 => {}", self.p_val);
                    }
                    PiParam::I => {
                        self.i_val += 0.1;
                        defmt::info!("I +0.1 => {}", self.i_val);
                    }
                }
            }
            KeyEvent::Pressed(Key::K2) => {
                match self.pi_sel {
                    PiParam::P => {
                        self.p_val -= 0.1;
                        if self.p_val < 0.0 {
                            self.p_val = 0.0;
                        }
                        defmt::info!("P -0.1 => {}", self.p_val);
                    }
                    PiParam::I => {
                        self.i_val -= 0.1;
                        if self.i_val < 0.0 {
                            self.i_val = 0.0;
                        }
                        defmt::info!("I -0.1 => {}", self.i_val);
                    }
                }
            }
            KeyEvent::Pressed(Key::K3) => {
                self.top_mode = TopMode::Speed;
                self.speed_sub = SpeedSubMode::Following;
                defmt::info!("Switch to SPEED menu (FOLLOWING)");
            }
            KeyEvent::Pressed(Key::K4) => {
                self.pi_sel = match self.pi_sel {
                    PiParam::P => PiParam::I,
                    PiParam::I => PiParam::P,
                };
                defmt::info!(
                    "Select {}",
                    match self.pi_sel {
                        PiParam::P => "P",
                        PiParam::I => "I",
                    }
                );
            }
            _ => {}
        }
    }

    /// 模拟电机动态（每 100ms 调用一次）
    pub fn update_motor(&mut self) {
        if self.top_mode == TopMode::Speed && self.speed_sub == SpeedSubMode::Following {
            // 简单一阶惯性跟随：actual += (setpoint - actual) * 0.1
            let diff = self.setpoint - self.actual;
            if diff.abs() < 2 {
                self.actual = self.setpoint;
            } else {
                self.actual += diff / 10;
            }
        } else if self.top_mode == TopMode::Speed && self.speed_sub == SpeedSubMode::Step {
            // 阶跃模式：保持进入时的转速
            self.actual = self.step_hold_speed;
        }
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
        self.last_step_hold_speed = self.step_hold_speed;
        self.last_p_val = self.p_val;
        self.last_i_val = self.i_val;
        self.last_pi_sel = self.pi_sel;
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
        let _ = write!(buf, "SP:{:4}rpm", self.setpoint);
        oled.draw_string_6x8(2, 0, &buf, &F6X8);

        buf.clear();
        let _ = write!(buf, "AC:{:4}rpm", self.actual);
        oled.draw_string_6x8(3, 0, &buf, &F6X8);

        if self.speed_sub == SpeedSubMode::Step {
            buf.clear();
            let _ = write!(buf, "HOLD:{:4}rpm", self.step_hold_speed);
            oled.draw_string_6x8(4, 0, &buf, &F6X8);
        }

        oled.draw_string_6x8(6, 0, "S1+ S2- S3 PID S4 STEP", &F6X8);
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
            let _ = write!(buf, "SP:{:4}rpm", self.setpoint);
            oled.draw_string_6x8(2, 0, &buf, &F6X8);
            buf.clear();
        }

        // AC line: page 3
        if self.actual != self.last_actual {
            oled.clear_cols(3, 0, 70);
            let _ = write!(buf, "AC:{:4}rpm", self.actual);
            oled.draw_string_6x8(3, 0, &buf, &F6X8);
            buf.clear();
        }

        // HOLD line: page 4 —— 仅在 Step 模式或从 Step 退出时处理
        if self.speed_sub != self.last_speed_sub || self.step_hold_speed != self.last_step_hold_speed {
            oled.clear_cols(4, 0, 80); // "HOLD:xxxxrpm" ≤ 12×6 = 72 px
            if self.speed_sub == SpeedSubMode::Step {
                let _ = write!(buf, "HOLD:{:4}rpm", self.step_hold_speed);
                oled.draw_string_6x8(4, 0, &buf, &F6X8);
            }
        }
    }

    // ---------------- PI 菜单：全量绘制 ----------------
    fn render_pi_full<I2C: embedded_hal::i2c::I2c>(&self, oled: &mut Sh1106<I2C>) {
        oled.draw_string_8x16(0, 0, "PI TUNE", &F8X16);

        let mut buf = heapless::String::<32>::new();
        let p_marker = if self.pi_sel == PiParam::P { ">" } else { " " };
        let _ = write!(buf, "{}P:{:.1}", p_marker, self.p_val);
        oled.draw_string_6x8(2, 0, &buf, &F6X8);

        buf.clear();
        let i_marker = if self.pi_sel == PiParam::I { ">" } else { " " };
        let _ = write!(buf, "{}I:{:.1}", i_marker, self.i_val);
        oled.draw_string_6x8(3, 0, &buf, &F6X8);

        oled.draw_string_6x8(6, 0, "S1+ S2- S3 SPD S4 SEL", &F6X8);
    }

    // ---------------- PI 菜单：局部刷新 ----------------
    fn render_pi_delta<I2C: embedded_hal::i2c::I2c>(&self, oled: &mut Sh1106<I2C>) {
        let mut buf = heapless::String::<32>::new();

        // P line: page 2, ">P:x.x" ≤ 7×6 = 42 px
        if self.p_val != self.last_p_val || self.pi_sel != self.last_pi_sel {
            oled.clear_cols(2, 0, 50);
            let p_marker = if self.pi_sel == PiParam::P { ">" } else { " " };
            let _ = write!(buf, "{}P:{:.1}", p_marker, self.p_val);
            oled.draw_string_6x8(2, 0, &buf, &F6X8);
            buf.clear();
        }

        // I line: page 3
        if self.i_val != self.last_i_val || self.pi_sel != self.last_pi_sel {
            oled.clear_cols(3, 0, 50);
            let i_marker = if self.pi_sel == PiParam::I { ">" } else { " " };
            let _ = write!(buf, "{}I:{:.1}", i_marker, self.i_val);
            oled.draw_string_6x8(3, 0, &buf, &F6X8);
        }
    }
}
