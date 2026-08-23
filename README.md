# Sensor_FOC

基于**磁场定向控制（FOC）**的无刷直流/永磁同步电机驱动项目，目标电机为**大疆 DJI M3508**（7 极对数，24 V 母线），以磁编码器为位置传感器，配套 MATLAB/Simulink 仿真验证。

> 项目面向实验和二次开发。上电前请确认母线电压、电流采样零点、编码器方向和功率级保护均已验证；固件参数不应直接用于未经测试的硬件。

## 目录结构

```
Sensor_FOC/
├── F405_M3508_FOC/      # STM32F405 固件（第一版方案）
├── G431_M3508FOC/       # STM32G431 固件（当前主力方案）
└── FOC_Simulation/      # MATLAB/Simulink 仿真模型（FOC.slx）
```

## 子项目简介

### F405_M3508_FOC — STM32F405 方案

- MCU：STM32F405 @ 168 MHz，CMake + STM32CubeMX 工程，集成 FreeRTOS、USB CDC 和 CAN1（1 Mbps）
- **10 kHz 中心对齐 PWM**（TIM1，ARR=8400）
- 三路 ADC 注入转换（TIM1_CC4 触发），同步采样相电压 VA/VB/VC、相电流 IA/IB/IC、母线电压、磁编码器 SIN/COS 及温度
- 开环 **V/F 起转**（`VF.c`，100 RPM），**SVPWM 零序注入法**输出（`SVPWM.c`）
- 磁编码器 SIN/COS 通道 ADC 采样 + `atan2` 解算，与合成角 `vf_theta` 对比输出误差
- 速度/位置/电流三种目标模式，电压欠压、过压、编码器和温度故障保护，LED 心跳/故障诊断

### G431_M3508FOC — STM32G431 方案（当前主力）

- MCU：STM32G431 @ 168 MHz（BOOST 模式），CMake + Keil MDK-ARM 双构建
- **10 kHz PWM**（TIM1，ARR=8399）+ 互补通道，TIM1 更新中断内执行控制环
- 电流采样：内置运放 OPAMP1/2/3 + ADC1/2 注入转换，8 点 MAF + 主循环 EMA + 尖刺抑制滤波
- 位置传感器：
  - 磁编码器（SIN/COS 经 UART4 外部采集），归一化 + `atan2` + 角度展开，在线动态校准（中心/幅值硬编码收敛值）
  - 预留 **AS5600**（I2C）、**MT6701**（SPI/SSI 14-bit）、**Hall**（TIM3 输入捕获 + XOR 模式，120° 扇区表）驱动
- 启动流程：**对齐（Vd 预定位 500 ms）→ 斜坡加速（1 s）→ 编码器角度融合过渡（300 ms）→ 双闭环（速度环 + 电流环）**
- **电流闭环（10 kHz）**：ADC 采样 → Clarke/Park → iq/id 增量式 PI → 反 Park → SVPWM，限幅 ±13 V（VDC/√3 线性区）
- **速度环（1 kHz）**：编码器窗口平均测速，PI 输出 iq 参考（A），形成"速度外环 + 电流内环"标准双闭环结构
- **SMO 滑模观测器**（预测-校正结构 + PLL，`observer.c`）：已完成实现，因空载电流过小无法提取反电动势暂禁用，代码保留
- 调试输出：USART2 1 kHz 打印 **VOFA+ 格式**数据流（`enc_deg, foc_deg, iq, id, I_mag` 等），UART4 DMA 用于编码器数据采集与动态校准

### FOC_Simulation — MATLAB/Simulink 仿真

- Simulink 模型（`FOC.slx`）：PMSM 电机模型 + Two-Level Converter + DC Voltage Source + powergui
- **包含电流环与速度环**：IdPID/IqPID 电流环 PID + 速度环，转速参考由 Repeating Sequence 给定
- 变换模块：Clark / Park / 逆 Park（MATLAB Function）
- **SVPWM**：扇区判定 N → Ta/Tb 时间计算 → CCR 输出（10 kHz，ARR=8400-1），带 Va/Vb、扇区、CCR 等观测 Scope
- **Hall 角度估计**：`Hall_To_FOC_Fix_180`（60° 离散角 + 速度外推线性插值）
- **SMO 观测器**（L=0.097 mH，R=0.194 Ω，k=40）
- 调试 Scope：三相电流、速度（估计 vs 实际）对比、扇区等

## 关键技术

| 模块 | 说明 |
|------|------|
| Clarke / Park / 逆 Park | 三相↔αβ↔dq 坐标变换 |
| SVPWM | 零序注入法，10 kHz，中心对齐 |
| V/F 开环 | 启动/测试用，含对齐 → 斜坡 → 编码器融合 |
| 电流闭环 | 10 kHz iq/id 双 PI（增量式，抗饱和），Vq/Vd 限幅 |
| 速度闭环 | 窗口平均测速 + PI 输出 iq 参考，与电流环级联 |
| SMO + PLL | 无传感器反电动势观测（仿真可用，固件待负载验证） |
| 磁编码器校准 | sin/cos 中心与幅值在线标定、椭圆/幅值补偿、相位对齐 |

## F405 CAN 协议

F405 固件接收标准帧 `0x91`（DLC=8），数据格式为 `mode, value_hi, value_lo, 0, 0, 0, 0, 0`。`value` 为有符号大端 `int16`：

| `mode` | 控制目标 | 原始值换算 |
|--------|----------|------------|
| `0x01` | `iq` 电流 | `value / 1000` A |
| `0x02` | 速度 | `value` rpm |
| `0x03` | 单圈机械位置 | `value * pi / 30000` rad |

每 10 ms 发送标准反馈帧 `0x78`：`iq_mA_hi, iq_mA_lo, speed_hi, speed_lo, position_hi, position_lo, temperature, error`。错误码 `0x01/0x02/0x03/0x04` 分别表示欠压、过压、编码器无效和过温。F405 目录下的 [`CAN_Control_Commands.xlsx`](F405_M3508_FOC/CAN_Control_Commands.xlsx) 提供可直接复制的示例帧。

## 硬件参数

- 电机：DJI M3508（7 极对数）
- 母线电压：24 V
- 电机参数（实测）：Rs = 0.194 Ω，Ls = 0.097 mH，磁链 ≈ 0.005 Wb

## 构建方法

### 环境要求

- CMake 3.20+、Ninja 和 `arm-none-eabi-gcc` 工具链
- STM32CubeMX（仅在修改 `.ioc` 后重新生成代码时需要）
- G431 方案也可使用 Keil MDK-ARM；仿真需要 MATLAB/Simulink

### F405_M3508_FOC（CMake）

```bash
cd F405_M3508_FOC
cmake --preset Debug
cmake --build --preset Debug
```

### G431_M3508FOC（CMake 或 Keil）

```bash
cd G431_M3508FOC
cmake --preset Debug
cmake --build --preset Debug
```

或直接打开 `G431_M3508FOC/MDK-ARM/FOC.uvprojx`（Keil MDK-ARM）编译。

生成文件默认位于对应工程的 `build/Debug` 或 `build/Release` 目录。修改工程配置后，建议先删除该工程的构建目录再重新配置。

### 仿真

在 MATLAB/Simulink 中打开 `FOC_Simulation/FOC.slx`。运行前请确认已安装 Simulink 及对应版本的 Simscape Electrical（或 Specialized Power Systems）。`FOC_Simulation/slprj` 为 Simulink 生成的缓存目录，不需要手工修改。

## 当前状态与待办

- [x] 开环 V/F 起转 + SVPWM 驱动 M3508
- [x] 磁编码器在线校准与角度融合
- [x] 电流闭环（10 kHz iq/id 双 PI）
- [x] 速度环 + 电流环双闭环级联
- [x] F405 CAN 目标指令、状态反馈和基础故障码
- [ ] 消除 iq/id 残余交流分量（编码器幅值/椭圆补偿）
- [ ] SMO 观测器在带载条件下重新启用验证

## 调试工具

- G431：USART2 以 1 kHz 输出 VOFA+ 数据，UART4 DMA 接收编码器数据
- F405：USB CDC 输出诊断信息，CAN 用于目标值下发和状态反馈
- `filter_dump.txt`、`Video/` 和工程内 PNG 文件是实验记录或分析中间产物，具体用途以文件名和源码为准

## 许可

仓库当前未声明开源许可证。如需在其他项目中分发或商用，请先获得作者授权。
