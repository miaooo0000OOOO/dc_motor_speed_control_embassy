use embassy_stm32::gpio::{Input, Pin, Pull};
use embassy_stm32::Peri;

/// 按键编号，对应 C1~C4（PA3~PA6）
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum Key {
    K1 = 0,
    K2 = 1,
    K3 = 2,
    K4 = 3,
}

impl Key {
    pub fn from_index(idx: usize) -> Option<Self> {
        match idx {
            0 => Some(Key::K1),
            1 => Some(Key::K2),
            2 => Some(Key::K3),
            3 => Some(Key::K4),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Key::K1 => "KEY1",
            Key::K2 => "KEY2",
            Key::K3 => "KEY3",
            Key::K4 => "KEY4",
        }
    }
}

/// 按键事件
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum KeyEvent {
    Pressed(Key),
    Released(Key),
}

/// 矩阵键盘驱动（4×4 矩阵的简化版，仅使用 R1 行 + C1~C4 列）
/// 行线 R1 接 3.3V，列线通过下拉输入 GPIO 读取。
/// 按键按下时 GPIO 读到高电平，松开时为低电平。
pub struct Keypad {
    cols: [Input<'static>; 4],
    /// 当前硬件读值（经过简单消抖后的稳定状态）
    state: [bool; 4],
    /// 上一次扫描的稳定状态，用于检测边沿
    last_state: [bool; 4],
    /// 消抖计数器
    debounce: [u8; 4],
}

/// 消抖阈值：连续 3 次（30ms）状态一致才确认
const DEBOUNCE_THRESHOLD: u8 = 3;

impl Keypad {
    pub fn new(
        col1: Peri<'static, impl Pin>,
        col2: Peri<'static, impl Pin>,
        col3: Peri<'static, impl Pin>,
        col4: Peri<'static, impl Pin>,
    ) -> Self {
        Self {
            cols: [
                Input::new(col1, Pull::Down),
                Input::new(col2, Pull::Down),
                Input::new(col3, Pull::Down),
                Input::new(col4, Pull::Down),
            ],
            state: [false; 4],
            last_state: [false; 4],
            debounce: [0; 4],
        }
    }

    /// 执行一次扫描，返回发生变化的按键事件列表
    /// 调用者应以固定周期（建议 10ms）调用本函数
    pub fn scan(&mut self) -> heapless::Vec<KeyEvent, 4> {
        let mut events = heapless::Vec::new();

        for i in 0..4 {
            let raw = self.cols[i].is_high();

            if raw != self.state[i] {
                // 状态与当前稳定值不同，开始/继续消抖
                self.debounce[i] += 1;
                if self.debounce[i] >= DEBOUNCE_THRESHOLD {
                    // 消抖完成，更新稳定状态
                    self.state[i] = raw;
                    self.debounce[i] = 0;
                }
            } else {
                // 状态与稳定值一致，重置消抖计数
                self.debounce[i] = 0;
            }

            // 检测边沿（基于稳定状态）
            if self.state[i] && !self.last_state[i] {
                // 上升沿：按下
                let _ = events.push(KeyEvent::Pressed(Key::from_index(i).unwrap()));
            } else if !self.state[i] && self.last_state[i] {
                // 下降沿：松开
                let _ = events.push(KeyEvent::Released(Key::from_index(i).unwrap()));
            }

            self.last_state[i] = self.state[i];
        }

        events
    }
}
