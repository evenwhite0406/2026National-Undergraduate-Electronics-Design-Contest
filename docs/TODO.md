# 待办与已知问题

本文件记录**已知但未修改**的问题。整理本仓库时遵守的原则是：**只脱敏、只勘误注释、只补文档，不改动任何控制逻辑与算法**。因此下面所有代码层面的建议都只记录、不执行。

---

## 1. 需要作者补充的信息

| 项 | 位置 | 说明 |
|---|---|---|
| 比赛信息 | `README.md` | 赛区、奖项等级、年份、题目全称 |
| 供电方案 | `docs/wiring.md` §4 | 代码中看不出 |
| 机械结构尺寸 | `docs/wiring.md` §5 | 摆杆长度、轨道长度、钢球直径等 |
| 前馈加速度字段单位 | `docs/protocol.md` §7.1 | `reference_accel` / `measured_accel` |
| 实物照片 | `images/` | 目录已建好，等待放入 |

---

## 2. 悬空引用：4 处指向不存在的文件

扫描全部 81 个源文件后发现的。**所在文件均已被 `.cproject` 排除编译**，因此不影响当前构建；但若有人重新启用这些文件，会立刻编译失败。

| 文件 | 行 | 引用了不存在的 |
|---|---|---|
| `Driver/Control/k230_line_follow.h` | 9 | `vision_line_follow.h` |
| `Driver/Control/openmv_line_follow.h` | 9 | `vision_line_follow.h` |
| `Driver/Control/motion_control.c` | — | `balance_control.h` |
| `Driver/PeripheralTest/peripheral_test.c` | — | `../Control/balance_control.h` |

附注：`Debug/.clangd/.cache/` 里留有 `vision_line_follow.h` 的索引缓存，说明该文件**曾经存在**，后来被删除。

---

## 3. 被排除编译、但保留在仓库中的文件

`.cproject` 的 `excluding` 列表排除了以下文件。整理时**全部保留**，因为无法判断它们是废弃代码还是作者有意留作参考。

**`Driver/Control/`**（3 对）
- `k230_line_follow.c/.h`
- `motion_control.c/.h`
- `openmv_line_follow.c/.h`

**`Driver/PeripheralTest/`**（9 对）
- `angle_hold_test.c/.h`
- `line_follow_task.c/.h`
- `line_sensor_bt_test.c/.h`
- `motor_encoder_check.c/.h`
- `motor_test.c/.h`
- `peripheral_test.c/.h`
- `position_test.c/.h`
- `speed_pid_tune.c/.h`
- `yaw_pid_tune.c/.h`

> 这些大多是外设联调与 PID 调参程序，对复现调试有参考价值，但**不参与正式固件编译**。

---

## 4. 协议中存疑的地方

### 4.1 `[L6,±XXXX]` 帧无人接收

K230 在 `main.py:3402` 构造、`5078` 与 `5259` 两处发送，但 MSPM0 的解析器（`k230_link.c:223-333`）没有 `L6` 分支，该帧被静默丢弃。

**需要确认**：是有意设计（MSPM0 不需要这个中间状态），还是漏实现？

### 4.2 `reference_accel` / `measured_accel` 是只写字段

K230 解析后保存（`main.py:3208-3217`），但**全文件没有任何地方读取**。即 `FA` 帧的这两个字段实际上白占带宽。

**可能的原因**：原本打算用于前馈调试显示，后来改为打印到终端。

### 4.3 `[PREP6]` 可能落在接收方的空档

K230 发送 `"PREP%d" % event[1]`（`main.py:5070`），而 MSPM0 只接受 `PREP3`–`PREP5`（`k230_link.c:272`）。若 `event[1]` 取到 6，该回执会被忽略。

代码路径上任务 6 走 `SET6` 分支，但无法静态断定 `event[1]` 不会取 6。

---

## 5. 哨兵值与量程重叠

上行球位置帧 `[±XXXX*]` 用 `9999` 表示丢球、`9998` 表示序列完成（`main.py:320-321`），而真实位置量程是 ±999.9 mm，**理论上会撞车**。

K230 侧已把真实值压到 ±9899 规避（`main.py:3369`），代码里有注释说明，属于**已知且已处理**的隐患。移植到其它量程时需要重新设计哨兵。

---

## 6. 重构建议（**仅供参考，本次未执行**）

以下都是"可以改但没必要现在改"的项，列出以备日后参考。

| # | 位置 | 建议 | 为什么不建议现在动 |
|---|---|---|---|
| 1 | `k230/main.py` 全文 5513 行 | 按功能拆成多个模块 | 改动面极大，且 K230 上多文件部署会增加部署复杂度 |
| 2 | `AI_WIDTH` / `AI_HEIGHT` | 直接改用 `CONTROL_WIDTH` / `CONTROL_HEIGHT` | 命名遗留但功能正常，改名会触及控制路径 |
| 3 | `empty.syscfg` 的 UART 变量名 | `UART1`(名为 `UART_OPENMV`) 映射外设 UART3；`UART3`(名为 `UART_openmv`) 映射外设 UART0，名实相反 | 改 SysConfig 变量名会连带重新生成 `ti_msp_dl_config.h`，影响面大 |
| 4 | `Driver/Hardware/UART3_OPENMV/` 目录名 | 那里接的是 K230，不是 OpenMV | 同上，属历史命名 |
| 5 | `k230/main.py` 的 `CarMotionFeedforward` 命名 | 含 "Car"，但本项目确实有车载环节，命名尚可 | 不必改 |

---

## 7. 第三方内容的许可待确认

`Driver/Hardware/oled_font.h` 里的 `F6x8[][6]` 是嵌入式领域广泛流传的 6×8 ASCII 点阵字库，**代码中没有任何出处或版权声明**，无法从代码判断其原始许可。

本仓库当前采用 **MIT**。若该字库实际来源于 GPL 项目，则需重新考虑许可证。**请作者确认字库来源后决定。**

---

## 8. 已在本仓库中处理掉的事项（留档）

| 项 | 处理方式 |
|---|---|
| K230 里的 Wi-Fi 热点名（原值疑似姓名缩写） | 改为占位符 `K230_AP` |
| K230 里的 Wi-Fi 热点密码 | 改为占位符 `CHANGE_ME` |
| K230 运行时打印热点密码 | 删除该行打印 |
| `.ccsproject` 中 1 处 Windows 用户目录路径 | 置空 |
| `.cproject` 中 2 处 Windows 用户目录路径（includePath） | 改为 `${PROJECT_ROOT}/...`，行为不变 |
| 名为 `true` 的 CCS 调试日志（含多处用户目录路径） | 未复制 |
| `.vscode/settings.json`（含本地用户目录路径） | 未复制 |
| 9 个 `ros2_*.sh`（含本地用户目录路径，且与本项目无关） | 未复制 |
| `.codex_*` 等 191 MB 的 AI 工具临时目录 | 未复制 |
| `Debug/` 构建产物 | 未复制 |
| 旧版 `oled.*`、`Driver/motor.*`、`Driver/Servo.*` | 删除（均已排除编译且自我声明为旧版） |
| 注释误写 `YOLO`（6 处） | 改为「检测框」 |
| 注释误写微步数 16 / 8 | 改为 10.7 / 10 |
| 注释误指 `k230_ball.h` | 改为 `k230_link.h` |
| 注释误写链路超时 100 ms | 改为 500 ms |
| `empty.c` 头部注释写 "Seven-event"，实际 8 项 | 改为 "Eight-event" 并列出任务 |

> 说明：具体被移除的凭据值与用户目录名**不在本文件中重复记录**，以免二次泄漏。
