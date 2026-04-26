# 直流电机转速闭环控制系统

基于 **STM32F103C8T6** + **Rust Embassy** 框架的直流电机 PID 转速闭环控制项目。

## 系统功能

- **编码器测速**：TIM1 QEI 正交编码器输入，M/T 法浮点转速计算
- **PID 闭环控制**：位置式 PID，支持 P/I/D 在线调参、积分限幅、输出限幅
- **PWM 电机驱动**：TIM2 驱动 L298N，20kHz PWM，支持正反转
- **OLED 显示**：SH1106 1.3寸屏，I2C+局部刷新，实时显示转速与占空比
- **矩阵键盘**：4键输入，支持速度设定、PI 参数调节、阶跃保持模式

## 硬件连接

| 功能 | STM32 引脚 | 外设引脚 |
|------|-----------|---------|
| PWM IN1 | PB10 (TIM2 CH3) | L298N IN1 |
| PWM IN2 | PB11 (TIM2 CH4) | L298N IN2 |
| ENA 使能 | PA7 | L298N ENA |
| 编码器 A 相 | PA8 (TIM1 CH1) | 编码器 A |
| 编码器 B 相 | PA9 (TIM1 CH2) | 编码器 B |
| OLED SCL | PB8 (I2C1) | OLED SCL |
| OLED SDA | PB9 (I2C1) | OLED SDA |
| 键盘 C1~C4 | PA3~PA6 | 矩阵键盘列线 |
| 调试串口 TX | PB6 | DAPLINK RX |
| 调试串口 RX | PB7 | DAPLINK TX |

## 电机参数

- 12V 减速电机，空载 201 RPM，负载 168 RPM
- 减速比 21:1，编码器 11 PPR × 4 倍频 = **924 counts/rev**

## 构建与烧录

```bash
# Release 编译（必须，debug 会超 Flash）
cargo build --release --bin dc_motor_speed_control_embassy

# probe-rs 下载并运行
cargo run --release --bin dc_motor_speed_control_embassy

# 仅下载
probe-rs download --chip STM32F103C8 target/thumbv7m-none-eabi/release/dc_motor_speed_control_embassy
```

## 按键操作

### 速度菜单（SPEED）
| 按键 | 功能 |
|-----|------|
| K1 | 设定值 +10 RPM |
| K2 | 设定值 -10 RPM |
| K3 | 进入 PID 调节菜单 |
| K4 | 进入阶跃保持模式（锁定当前转速） |

### PID 调节菜单（PID TUNE）
| 按键 | 功能 |
|-----|------|
| K1 | 当前选中参数 +0.1 |
| K2 | 当前选中参数 -0.1 |
| K3 | 返回速度菜单 |
| K4 | 循环切换 P → I → D |

## 测试程序

```bash
# 编码器测速 + 固定 70% PWM
cargo run --release --bin test_encoder

# L298N PWM 梯形波测试
cargo run --release --bin test_l298n

# OLED 显示测试
cargo run --release --bin test_oled

# 矩阵键盘测试
cargo run --release --bin test_keyboard

# PID 阶跃响应仿真
cargo run --release --bin test_pid

# 菜单系统测试
cargo run --release --bin test_menu
```

## PID 参数说明

默认参数（50ms 控制周期）：
- **P = 0.4**：比例增益，决定响应速度
- **I = 0.2**：积分增益，消除静差
- **D = 0.0**：微分增益，初始为 0（避免放大噪声），可在线调节

参数统一在 `src/main.rs` 中定义，通过 `APP_STATE` 同步到菜单显示，避免代码中重复出现。
