# 接线说明

本文件只列出**能从源码中直接确认**的引脚。凡代码里看不出来的，一律标注 `TODO(待补充)`，请自行填写。

> ⚠️ **所有串口与 I²C 连接都必须共地（GND 对接）。**
> K230 与 MSPM0G3507 是两块独立供电的板子，不共地时电平没有共同参考点，串口会出现随机丢帧或完全收不到数据。

---

## 1. K230 与 MSPM0G3507 之间的主控链路

这条链路承载任务握手与钢球位置回传，是整题的核心通信。

| K230 侧 | 方向 | MSPM0G3507 侧 | 出处 |
|---|---|---|---|
| GPIO48 / UART4_TXD | → | PB3 / UART3_RX | `k230/main.py:294` |
| GPIO49 / UART4_RXD | ← | PB2 / UART3_TX | `k230/main.py:295` |
| GND | — | GND | 必须连接 |

**串口参数**：115200，8 数据位，无校验，1 停止位（8N1）。

**⚠️ 命名陷阱**：`mspm0g3507/empty.syscfg` 里的变量名与外设编号是**错位**的，看配置时容易搞反：

| syscfg 中的变量名 | 实际映射的外设 | 引脚 | 用途 |
|---|---|---|---|
| `UART1`，名字叫 `UART_OPENMV` | **外设 UART3** | PB3(RX) / PB2(TX) | **接 K230** |
| `UART3`，名字叫 `UART_openmv` | **外设 UART0** | PB1(RX) / PB0(TX) | 代码中未见使用 |
| `UART2`，名字叫 `UART_Bluetooth` | 外设 UART2 | PB18(RX) / PB17(TX) | 蓝牙 |

`k230_link.c` 通过 `Driver/Hardware/UART3_OPENMV/` 访问 K230，目录名里的 "OPENMV" 是历史遗留命名——那里接的其实是 K230，不是 OpenMV 摄像头。

---

## 2. K230 侧

| 功能 | 引脚 | 参数 | 出处 |
|---|---|---|---|
| 步进电机（ZDT X42S，Emm FD 模式） | UART2，IO11(TX) / IO12(RX) | 115200 8N1，ID=1 | `k230/main.py:230-231, 2978` |
| 主控链路 | UART4，GPIO48/GPIO49 | 115200 8N1 | `k230/main.py:306-307` |
| 板载用户按键（任务 6 标定用） | GPIO53 | 下拉输入，**按下为高电平** | `k230/main.py:212-213` |

**⚠️ 板型差异**：`k230/main.py:212` 注释指出，若使用 **Lite-K230D**，板载按键应改为 **GPIO64**。对应常量是 `TASK6_BUTTON_GPIO`。

**Wi-Fi**：K230 使用板载 RTL8189FTV，由程序创建 2.4 GHz 热点（`k230/main.py:91`）。无需额外接线。

---

## 3. MSPM0G3507 侧

以下全部取自 `mspm0g3507/empty.syscfg`——该文件是本工程引脚配置的唯一来源。

### 3.1 电机与编码器

| 信号 | 引脚 | 说明 |
|---|---|---|
| PIN_L1 | PB23 | 左轮方向 1 |
| PIN_L2 | PB21 | 左轮方向 2 |
| PIN_R1 | PA22 | 右轮方向 1 |
| PIN_R2 | PB19 | 右轮方向 2 |
| PIN_STBY | PB20 | TB6612 使能（内部上拉） |
| Motor PWM CCP0 | PA26 | 定时器 TIMG8 |
| Motor PWM CCP1 | PB22 | 定时器 TIMG8 |
| SPEED_1 | PA24 | 左轮编码器脉冲（上升沿+下降沿中断） |
| DIRECTION_1 | PB24 | 左轮编码器方向 |
| SPEED_2 | PA13 | 右轮编码器脉冲（上升沿+下降沿中断） |
| DIRECTION_2 | PA14 | 右轮编码器方向 |

### 3.2 人机交互

| 信号 | 引脚 | 说明 |
|---|---|---|
| BUTTON_1 | PA12 | 菜单下一项（K1），内部上拉，低电平按下 |
| BUTTON_2 | PB15 | 菜单上一项（K2） |
| BUTTON_3 | PB14 | 执行当前任务（K3），也是计时起点 |
| BUTTON_4 | PB13 | 取消/返回（K4） |
| OLED SDA | PA16 | I²C_OLED，Fast 模式 |
| OLED SCL | PA15 | I²C_OLED |
| RED / YELLOW / BLUE / GREEN | PA7 / PB4 / PB5 / PB6 | 四色 LED |
| BUZZER | PB27 | 蜂鸣器 |

OLED 为 **SSD1306**，128×64，7 位地址 `0x3C`（`mspm0g3507/Driver/Hardware/oled.c:3,20`）。

### 3.3 传感器

| 信号 | 引脚 | 说明 |
|---|---|---|
| I²C_GYRO SDA | PA10 | I²C0 |
| I²C_GYRO SCL | PA11 | I²C0 |
| LINE_1 … LINE_8 | PA25 / PB25 / PB26 / PA27 / PA28 / PA29 / PA30 / PA31 | 八路循迹传感器 |
| PIN_1 / PIN_2 | PB12 / PB16 | 光电传感器 |
| (磁力计) | 与 BMI088 共用 I²C_GYRO | MMC5983MA，地址见驱动 |

IMU 为 **BMI088**（陀螺仪 0x68/0x69，加速度计 0x18/0x19），磁力计为 **MMC5983MA**。两者经 `Driver/PeripheralTest/bsp_i2c.c` 挂在两条独立 I²C 上。

### 3.4 其他执行器

| 信号 | 引脚 | 说明 |
|---|---|---|
| Servo PWM CCP0 / CCP1 | PA8 / PA9 | 定时器 TIMA0，timerCount=40000，clockDivider=8 |
| RELAY PIN_0 | PA2 | 继电器 |

---

## 4. 供电

`TODO(待补充)` —— 代码里看不出供电方案。请补充：

- K230 与 MSPM0 各自的供电来源与电压
- 步进电机（ZDT X42S）的供电
- TB6612 的 VM/VCC
- 是否需要电平转换（K230 IO 为 3.3 V，MSPM0 亦为 3.3 V，通常可直连，但请以实际为准）

---

## 5. 机械结构

`TODO(待补充)` —— 本仓库只含软件，无机械图纸。已知代码中出现的几何参数（可用于反推或标定）：

| 参数 | 值 | 出处 |
|---|---|---|
| 曲柄半径（电机轴心 → 曲柄销） | 47.5 mm | `k230/main.py:245` |
| 摆杆支点 → 连杆上端铰点 | 280.0 mm | `k230/main.py:246` |
| 步进电机步距角 | 1.8° | `k230/main.py:236` |
| 细分数 | 16 | `k230/main.py:237` |
| 电机角度硬限位 | ±20° | `k230/main.py:258` |

摆杆长度、轨道长度、钢球直径等**代码中没有**，请补充。
