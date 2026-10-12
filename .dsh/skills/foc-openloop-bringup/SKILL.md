---
name: foc-openloop-bringup
description: 本工程的 FOC 固件改动与开环上电调试流程。改 SMO/VF/中断代码、查"电机不转/STALL/观测器发散"、动控制频率、跑 USB 台架采集或离线自检脚本时用。含中断预算与边界层 δ 等硬性约束。
whenToUse: 任务涉及 FOC/src 下的 VF/SMO/SVPWM/PID、BSP/src/App_FOC.c 的遥测与命令、控制频率或 PWM 周期、M3508 开环试转、SMO 数据采集与判读、或"板子起不来/电机不转"排查时加载。纯 CAN 协议、纯上位机 Excel 工具、纯 USB 描述符改动不需要。
---

# FOC 开环调试与固件改动

STM32F405 + M3508（19:1 行星减速，空心杯转子），20kHz 电流环，无感开环 V/F 驱动 +
滑模观测器（SMO）只做诊断。**观测角不参与驱动**，驱动角由固件积分（`drv 0`）。

> ⚠ **路径约定**：本 skill 的 base directory 是**仓库根**（多工程仓库：F405 / G431 / G474
> EtherCAT / FOC_Simulation），但下面所有相对路径都是**相对 `F405_M3508_FOC/`** 这个子工程，
> 例如 `tools/build.ps1` 的实际路径是 `<仓库根>/F405_M3508_FOC/tools/build.ps1`。
> **不要**按 base directory 直接拼接。下文命令都以 `F405_M3508_FOC/` 为工作目录。

工程长期知识在两份文档里，**改代码或上电前先读对应那份**，本 skill 只给流程与硬约束
（均位于 `F405_M3508_FOC/docs/`）：

- `SMO开环测试说明.md` —— 开环 + SMO 的完整使用说明、参数拐点、排查表、附录
- `板子起不来排查顺序.md` —— 分层排查（固件在跑吗 → 电源 → SWD → 电机）

## 三条硬规矩

1. **改完 SMO/VF 数值，先跑离线自检再烧板**（不上电、秒级）：
   ```powershell
   python tools/verify_smo.py          # 参数直接读固件头文件; 自带 δ 离散稳定性自检
   python tools/test_openloop_state.py # 开环保护状态机: 保护有效 + 可重复使能 + 不误判
   python tools/vf_sim.py --sweep      # 哪个 vlo/fres 组合能起来(起动能力估算)
   ```
   `verify_smo.py` 发现 δ 不满足稳定条件会**直接报错退出**——这是设计好的拦阻，不要绕过它。
2. **台架永远从小转速、短时间开始**：`vf 300` → 看 1~2s 内 `|E|` 是否建立 → 再往上加。
   电机必须固定牢、轴上空载、人不要碰联轴器；听到异响/闻到味道立刻断电。
   出现 `# STALL: ... -> 封驱动` **先 `stop`，再调参数，不要反复硬冲**。
3. **动控制频率前，五个地方必须一起改**（漏一处就出怪问题），见下节。

## 构建与烧写（本机特有）

```powershell
pwsh -File tools/build.ps1                 # 常规: 补齐改动文件并链接
pwsh -File tools/build.ps1 -Force          # 全量重编(只在上面出问题时)
pwsh -File tools/build.ps1 -Files FOC/src/SMO.c -NoLink   # 只编单个文件
STM32_Programmer_CLI -c port=SWD -w build/Debug/F405_M3508_FOC.elf -v -rst
```

**不要用 `cmake --build`**：本机 `C:\ST\STM32CubeCLT_1.19.0` 的 cmake/ninja 会卡在启动阶段
（0% CPU、十几分钟无输出），而 `arm-none-eabi-gcc` 单跑只要 0.5s。`build.ps1` 直接解析
`build/Debug/build.ninja` 的"目标文件→源文件"表复刻编译/链接命令。

## ★ 控制频率（`APP_CTRL_HZ`）改动的 5 处联动

| # | 文件 | 改什么 | 漏了会怎样 |
|---|------|--------|-----------|
| 1 | `BSP/inc/App_Config.h` | `APP_CTRL_HZ` | 时间基全错 |
| 2 | `FOC/inc/SVPWM.h` | `PWM_PERIOD` | 占空比算错，电压不对 |
| 3 | `Core/Src/tim.c` | `htim1.Init.Period` | 实际频率和代码不一致 |
| 4 | `BSP/src/BSP_Timer.c` | `CCR4 = PWM_PERIOD − 50` | ADC 采样点错位 |
| 5 | `FOC/inc/SMO.h` | **`SMO_BOUNDARY`（δ）** | ★观测器**发散** |

**最坑的一条：δ 与 Ts 成正比。** 前向欧拉下 `G=(1−F)/R` 随 Ts 增大，稳定性由 `G·k/δ` 决定：

| 频率 | Ts | G | δ | G·k/δ | 闭环极点 | |
|------|-----|---|-----|-------|---------|---|
| 40kHz | 25us | 0.2514 | 8A | 1.571 | −0.620 | ✅ |
| 20kHz | 50us | 0.4905 | 8A | 3.066 | −2.161 | ❌ 发散 |
| 20kHz | 50us | 0.4905 | **16A** | 1.533 | −0.628 | ✅ |

稳定下限 `δ > G·k/(1+F)`：40kHz 是 6.4A，**20kHz 是 12.9A**。当前值 20kHz/δ=16A。
δ 或 α 一改，滞后补偿要重标，然后更新 `OL_ANGLE_OFF_DEF`（当前 `+0.4936f`）。

**开机打印的中断预算是硬约束**（`BSP/src/App_FOC.c` 开机 3s 打一次）：

```
# ctrl_step_max=<n> cyc = ??.?? us (budget 50.00 us @ 20000 Hz)   ← 必须 < 50
# prof(2s内峰值): total=??.??us | smo=??.??us vf=??.??us | 其余=??.??us
```

超标（历史上 40kHz 实测 31.67us > 25us 预算）会导致 ADC 丢拍、角度积分和真实时间对不上，
**表现就是电机不转**——所以看到超预算先把中断耗时解决掉，别去调 V/F 参数。

## 台架操作顺序（全程走 USB CDC，不需要 CAN 盒）

```powershell
# 1) 确认命令通: 复位后应有 "# boot: ..." 和一行 "# ctrl_step_max=..."
#    板子自己的 USB 能枚举出 COM 口 = 芯片在跑(这也是最省事的"板子活着吗"判据)
# 2) 逐档试转(每档等 1~2s 看 |E|)
vf 300 ; stream 1 ; stat ; vf 1200 ; vf 2000 ; stop
# 3) 自动扫速采集 + 自动判读
python tools/smo_capture.py --port COM7 --sweep 600,1200,2000,3000 --dwell 5 --out data/sweep1.csv
python tools/ol_watch.py --port COM7 --seconds 15        # 实时看数, 不落盘
python tools/verify_fastatan2.py                          # 观测器 atan2 精度离线自检
```

`vf` 只进开环不改目标；`stop` 退出开环、三相封零。`smo_capture.py` 自己发
`stream 1` / `vf <rpm>` / `stop`，分析模式按 CSV 头 `drv=` 自动判定，只依赖标准库
（`--plot` 要 matplotlib、`--list-ports` 要 pyserial，本机都没装）。

USB 命令行全部命令见 `BSP/src/App_FOC.c` 的 `App_UsbCommand()`，含
`vf/stop/cur/spd/slope/vmax/vboost/vlo/fres/ramp/fstart/valign/drv/prot/park/angle/off/lag/gain/delta/stream/stat/help`。
其中 `park <deg> <V>` 固定角加恒压（有力=功率级正常，没感觉=驱动或接线有问题，★持续发热用完必退 `park 0 0`）。

## 判据：怎么知道电机真的在转

**没有编码器时 `theta_enc / err / rpm_enc` 三列是垃圾，别理。** 反电动势是唯一证据：

| 电机侧 rpm | 600 | 1200 | 2000 | 3000 |
|---|---|---|---|---|
| `emf` 期望 V | 0.44 | 0.88 | 1.47 | 2.20 |

- `imag` 正常运转 3~8A；堵转时顶到限流。
- `vcmd` 起步段应是 `vlo`(2.8V)，**牵入后必须掉回** `1.0+0.05·fe`；长期停在 2.8V = 一直没牵入。
- `flags`：bit0 已牵入 / bit1 低频强制段 / bit2 失步 / bit3 对齐完成。**bit1 长期为 1 = 没牵入。**
- 实时诊断行：`# OL t=... v_out=... I=... |E|=... rpm_cmd=... rpm_obs=... vdc=...` 后面跟状态词
  （`aligned/aligning`、`running/no-emf`、`boost_lo/curve`、`prot=`）——开环模式下不开打点流也每 500ms 打一行，
  是排查"不转"的第一手材料。`usb_drop` 非 0 → 打点丢帧，把 `SMO_STREAM_HZ`（`App_FOC.c`，现 1000）降到 500。

## 参数默认值与拐点

| 命令 | 默认 | 现象 → 怎么调 |
|------|------|--------------|
| `vlo` | 2.8 V | **堵转/一冲就停 → 加到 3.5~4.5**；电流大发热 → 回调 |
| `fres` | 6 Hz | 起步无力/牵不进 → 提到 8~10 |
| `vboost` | 1.0 V | 牵入后低速段抖 → 加一点 |
| `slope` | 0.05 V/Hz | 转速上不去、vdc 打满 → 调小；带载掉速 → 调大 |
| `vmax` | 6.5 V | **12V 母线线性区 ≈ vdc/√3 ≈ 6.9V，别超**；24V 可到 13 |
| `fstart` | 2.0 Hz | 起步就堵 → 3~5；起步顿挫 → 保持小值 |
| `valign` | 1.5 V | 对齐"咔"一声正常；不想对齐设 0 |
| `ramp` | 600 rpm/s | 起步电流大 → 降到 200~300 |
| `prot` | 1 | 想排除"保护在捣乱"→ `prot 0`（会失去堵转保护） |

为什么需要 `vlo`：V/F 曲线在 `fe→0` 时 `V→vboost`(1V)，电流 ≈1/0.194≈5A，折算输出轴转矩 ≈1N·m，
**顶不住 19:1 减速箱的静摩擦** → 原地堵转。所以牵入前把电压强抬到 `vlo`，`|E|` 一建立立刻退回曲线。

## 物理常量与量程（来自 FOC/inc/FOC_Param.h / SMO.h）

- `FOC_POLE_PAIRS = 7`；电机侧 rpm ↔ 电频率：`fe[Hz] = rpm·7/60`（3000rpm ≈ 350Hz）。
- `FOC_RS = 0.194Ω`、`FOC_LS = 97µH`、`FOC_FLUX = 1.0mWb`（实测反推 1.10e-3 / 0.96e-3）。
- `OBS_PLL_FN_HZ = 60 / ζ = 0.707`，`OBS_PLL_WMAX = 2π·1500`（防跑飞限幅）。
- `OBS_EMF_MIN_V = 0.3V` → **电机侧约 478rpm 以下没有可靠观测角**，低速只能靠 V/F 硬拖；HFI 已删。
- 失步保护：对齐 300ms 后、指令 `fe > fe_min` 起计时，2000ms 内 `|E| < 0.35V` → 封驱动 + 关 EN + 退开环。
  **被保护封过之后直接再发 `vf` 就能重来**（使能时清失步锁）——别让"封一次就再也起不来"的回归复现。

## 长尾坑

- 连不上调试器：1000/500/100kHz 三档都报 `Error connecting DP: cannot read IDR` = 探针 OK 但**芯片不应答**，
  查供电/共地/接线/RDP，**不要在速度上折腾**。固件没有任何复用/关闭 SWD 引脚的代码。
  GDB 报 `localhost:50000` 连不上只是 OpenOCD 已退出的**后果**；用 `pwsh -File tools/swd_test.ps1` 看原始报错。
- Cortex-Debug 报 objdump/nm ENOENT = `cortex-debug.armToolchainPath` 写成了可执行文件，应为工具链 **bin 目录**。
- 编码器模式下 `rpm_obs` 符号与指令相反 = 观测角镜像：`FOC/src/SMO.c` 第 6 步
  `atan2f(-e_lpf[0], e_lpf[1])` → `atan2f(e_lpf[0], e_lpf[1])`。
- 目前**没做**无感闭环（观测角只诊断）、没实测极限转速、没低速 HFI。要做闭环交接，
  先拿数据证明 `|E|` 稳定、`rpm_obs` 跟得上指令。
