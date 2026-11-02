# Shooter 四球连射调参

在 Driver Station 选择 **Testing → TEST - Shooter PIDF Tuner**。这是单独的调参 OpMode，不控制底盘。硬件沿用 `shooter`（FORWARD）和 `intake`（REVERSE）。需要 Shooter 电机编码器接入 Hub；只连接供电线无法做速度闭环。所有速度单位是 **编码器 ticks/s，不是 RPM**。

程序使用 Hub 的速度 PIDF，与现有 Shooter 的 `setVelocity()` 相同。F 提供维持转速的基础输出，P/I/D 对速度误差作修正。不要套用其他软件 PID 控制器的系数，因为单位和实现可能不同。

## 操作

INIT 时电机保持零输出，并读取当前 Hub 的 PIDF 到 Panels。**在 INIT 之后改 P/I/D/F**；重新 INIT 会再次从 Hub 读取。初始 `TARGET_TPS` 沿用 Shooter 的 1560，目标速度也可以修改。

调参器明确使用 PIDF 算法。`Algorithm` 行同时显示进入时的算法；若原先为 LegacyPID，原系数仅作为起点，切换后不代表与原控制行为等价。每次写入系数后会读回核对，不匹配则退出并尝试停机。

| 手柄 1 | 功能 |
|---|---|
| 按住 RB | Shooter 持续控速；松开立即命令停止 |
| 按住 LB | 现有 intake 供球；必须同时按住 RB |
| A，每次按一下 | 在每个球送入前人工标记一次，最多四球；不触发自动供球 |
| X | 清空本轮四球记录 |
| B | 停止两电机；松开 RB 后才能重新启动 |
| 方向键 左/右 | 选择 P、I、D、F 或目标速度 |
| 方向键 上/下 | 每次按键增减所选值 |
| Y | 松开 RB 后保存本轮 CSV 摘要 |

手柄调参步长由 `PIDF_STEP`（默认 0.05）和 `TARGET_STEP_TPS`（默认 10）控制。按住方向键不会连续加数。Panels 可直接输入数值。

连接机器人网络，在 Panels 的 Configurables 面板中展开 `ShooterPidfTuner`。在图表中同时选择 `targetTps`、`velocityTps`；也可观察 `errorTps`、`ready`、`intakeCommand`、`manualMarkers` 和 `batteryVolts`。使用已在 `DashTest` 验证可用的 Panels 连接地址，并打开 Telemetry 和 Graph 面板。

任何 PIDF、目标、供球功率、容差、等待时间或供球模式变更都会清空本轮数据并重新等待速度稳定。INIT 和 START 都不会自动转轮；若 START 时已按住 RB，需要先松开再按。无效参数会停机，修正后也要松开 RB 再按。

## 怎样调

1. **固定射击距离、仰角、机器人位置和球的顺序。** 先不供球，按住 RB，确认实际速度为正，能接近目标。速度一直为 0 或符号不对时，先检查编码器、硬件配置和方向。不要用调大 PID 来掩盖反馈问题。
2. **记下 INIT 读出的原始 PIDF，先跑基线。** 看不供球时是否稳定，再按同一节奏射四球。每个球进入前按 A；更方便时由另一人负责观察球和落点。第四球之后保持 RB 至少 `SHOT_WINDOW_SECONDS`（默认 2 秒），再松开 RB、按 Y 保存。
3. **先调 F，再调 P。** 从当前 Hub 系数出发，做小幅修改。可暂设 I=0、D=0，在较小 P 下观察无球稳态速度；持续偏低时小幅加 F，偏高时减 F。然后逐步加 P，让进球后的掉速恢复更快；开始来回振荡、明显过冲时回退。每次只改一项，再跑相同四球测试。
4. **I 和 D 只在需要时加。** 已有 F/P 仍存在持续稳态误差，再少量增加 I；I 太大会积累误差并加重过冲。D 可能放大编码器噪声，先保持 0；若需要抑制快速过冲，再非常小幅尝试。不要把射球瞬间不可避免的机械能损失当成稳态误差。
5. **比较多组四球。** 优先比较射前速度差、最低速度、恢复时间和实际落点；同时参考电池电压曲线。记录每球偏近/偏远/左右偏移。不同电量也要复测。图表稳定不等于落点已验证。

若提高目标后实际速度始终追不上，可能已没有电机/电池输出余量；继续增加系数不会凭空提供功率。需要实测降低目标、延长球间隔，或改善飞轮惯量、传动和供球一致性。

## 四球记录怎么看

- `pre`：按 A 那一刻的速度，是射前速度的人工近似。
- `min`：该标记后的最低观测速度。
- `drop`：相对目标速度的最大下降百分比，不是相对 pre。
- `over`：高于目标的最大观测速度（ticks/s）。
- `recover`：从人工标记到“发生超容差扰动后，重新连续处于容差内达到 dwell”的时间，包含供球延迟和确认等待时间。默认容差 ±40 ticks/s、dwell 0.15 秒。
- `not observed`：该窗口里没确认恢复，不能按 0 毫秒理解。
- `no out-of-band dip observed`：没有观察到超容差扰动，不能据此确认球已射出。
- `NOT READY at marker`：标记时尚未连续稳定达标。
- `Pre-shot spread`：本轮人工标记速度的最大值减最小值。

每个观测窗口在**下一次标记、2 秒到时或松开 RB**时结束，以先到者为准。连续射球时，上一球可能还没恢复就进入下一球；此时显示“未观察到恢复”是有效结果。停机后的自然降速不计入掉速。手动标记、SDK 速度反馈和程序采样都有延迟，因此数据用于对比调参，不是精确的球出膛时刻测量。四个标记只表示人工记录了四次，不是传感器确认了四颗球。

## 连射和等转速供球的对比

`GATE_INTAKE_ON_READY = false` 为默认：同时按住 RB 和 LB，供球连续运行，可以直接暴露四球连射的掉速。

设为 `true` 后：仍需按住 RB 和 LB，但实际速度连续达标后 intake 才运行；掉速超出容差就暂停，再恢复后继续。这样可以测试“增加恢复等待”是否改善一致性。它不是单球分离器：已经进入发射轮的球无法撤回，队列中的多球也可能在速度反馈前一起前进。若机构不能可靠分球，应由操作者控制 LB 的供球节奏。

## 保存并用于正式 TeleOp

### 用 USB 自动传回电脑（Mac）

1. 给 Control Hub 正常供电，用支持数据传输的 USB 线连接电脑与 **Control Hub 的电脑连接端口**。连接的是运行 Robot Controller 的设备，而不是 Driver Station。首次连接若出现 USB 调试授权提示，请允许这台电脑。
2. 双击项目 `scripts/接收Shooter数据.command`。保持弹出的终端窗口运行；它会每隔约 2 秒检查一次 USB 设备。需要本机 Python 3.8+ 和 Android SDK Platform Tools；这台 Mac 已找到 Android Studio 的 adb。
3. 在调参器完成一组后，松开 RB，按 Y。通常约 2–4 秒后，电脑终端显示 `已传回：完整文件路径`；设备较慢或文件较多时可能更久。
4. 默认保存在项目根目录 `shooter-data/<设备标识>/trial-时间戳.csv`。该目录已加入 Git 忽略列表。机器人端原文件保留；脚本重启后也会补传之前的文件。
5. 按 Ctrl+C 结束接收。断线时脚本保留已收到的文件并等待重连；若提示 unauthorized，请处理 USB 调试授权；若提示多台 USB 设备，请暂时只连接需要的 Control Hub。

也可从项目目录运行：

```bash
python3 scripts/sync_shooter_csv.py
```

只传输当前已有文件后退出：

```bash
python3 scripts/sync_shooter_csv.py --once
```

保存到桌面的指定文件夹：

```bash
python3 scripts/sync_shooter_csv.py --output "$HOME/Desktop/Shooter数据"
```

这是接收**按 Y 生成的 CSV 摘要**，不改变其内容，也不会自动记录完整速度波形。只更新电脑脚本即可使用，机器人程序无需为此重新部署。默认只使用 USB（ADB `-d`）；不会自动转用 Wi-Fi。若机器人显示的 CSV 目录不同，可用 `--remote-dir` 指定。ADB 的 USB 调试、设备授权和文件传输说明见 [Android 官方文档](https://developer.android.com/tools/adb)。

### 机器人端原文件与正式参数

松开 RB 后按 Y，把当前设置和四球摘要写入 Robot Controller 应用文件目录下的 `shooter-tuning/trial-时间戳.csv`。Driver Station 的 `CSV` 行显示完整路径。可通过 Android Studio Device Explorer 或已有的 ADB 连接提取。CSV 的电压是保存时电压；射击过程中的负载电压请看 Panels 图表。CSV 不包含落点和完整波形，请另行记录落点。

Panels 改值和 CSV 保存**不会自动修改 Java 源码或安装正式参数**。确认一组参数后，修改 `subsystems/Shooter.java`：

- `SHOOT_VEL` 写入测得的目标 ticks/s。
- `VELOCITY_P`、`VELOCITY_I`、`VELOCITY_D`、`VELOCITY_F` 写入选定值。
- `USE_TUNED_PIDF = true`，重新编译部署。默认 false 保留原有 Hub 系数。

调参器退出时会尝试停止两电机，并恢复进入时的 Hub PIDF。正式 Shooter 每次初始化时会在启用开关后设置你填写的系数。不要只依赖 Hub 暂存参数：FTC 官方说明 PIDF 修改不跨 Hub 断电保存，参见 [Changing PIDF Coefficients](https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/pidf_coefficients/pidf-coefficients.html)。

本程序帮助稳定轮速并比较供球节奏；同一落点仍需靠实机确认仰角、球的压缩量/摩擦、供球路径和机器人姿态。尚未进行实机四球调参。
