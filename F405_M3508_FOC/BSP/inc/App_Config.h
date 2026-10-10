#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Common application constants. Keep all tunable values in one place. */
#define APP_USE_CMSIS_OS2          0
#define APP_TWO_PI                 6.28318530718f
#define APP_PI                     3.14159265359f

/* ---------------------------------------------------------------------------
 * ★开关: 是否在中断里运行滑模观测器(APP_SMO_ENABLE)
 *
 *   0 = 完全不跑观测器(默认, 上电排查用)。
 *       只保留"电流环 + V/F 开环 + park 点动", 也就是纯粹的开环驱动。
 *       开环驱动角是固件积分的, 本来就不需要观测器 —— 去掉它能把
 *       出问题的面缩到最小, 先用 park/vf 确认功率级能出电流。
 *   1 = 打开观测器(诊断用: |E|/转速/角度对照)。
 *
 *   ★注意: 关掉观测器后, 失步保护(|E| 判定)也随之失效, 因为没有 |E| 了。
 *     所以这种模式下**必须**自己看着电流/温度, 别长时间堵转。
 * ------------------------------------------------------------------------- */
#define APP_SMO_ENABLE             0
#define APP_ADC_VREF               3.3f
#define APP_ADC_MAX_COUNTS         4095.0f
#define APP_ADC_COUNTS_INV         0.00024420024420f /* 1 / 4095 */
#define APP_ADC_VOLT_SCALE         (APP_ADC_VREF * APP_ADC_COUNTS_INV)

#define APP_VDC_DIVIDER            18.727272f
#define APP_PHASE_VOLT_DIVIDER     (41.2f / 2.2f)
#define APP_CURRENT_SCALE           (APP_ADC_VOLT_SCALE * 20.0f)

/* ---- 控制/采样频率: 必须与 TIM1 ARR 一致(中心对齐) ----
 * ★2026-10 从 40kHz 改回 20kHz: 观测器把中断撑到 31.67us, 而 40kHz 预算只有 25us,
 *   超预算 27% -> 采样丢拍、控制时序错乱、电机转不起来。
 *   20kHz 预算 50us, 余量充足(这工程本来就是 20kHz; 为 HFI 提到 40kHz, HFI 已删)。
 *   ★改这里必须同步改另外三处(否则频率/时序全错):
 *       FOC/inc/SVPWM.h     PWM_PERIOD = 4200
 *       Core/Src/tim.c      htim1.Init.Period = 4200
 *       BSP/src/BSP_Timer.c CCR4 = PWM_PERIOD - 50(死区后触发采样) */
#define APP_CTRL_HZ                 20000u
#define APP_TICKS_PER_MS            (APP_CTRL_HZ / 1000u)

#define APP_ADC_CAL_WINDOW          100u
#define APP_ADC_CAL_TIMEOUT_MS      100u
#define APP_ADC_STAT_WINDOW         250u

#define APP_CAN_FEEDBACK_PERIOD_MS  10u
#define APP_CAN_IQ_SCALE            1000.0f
#define APP_CAN_POSITION_SCALE      30000.0f
#define APP_CAN_POSITION_TO_RAW     (APP_CAN_POSITION_SCALE * 0.31830988618f)
#define APP_CAN_RAW_TO_POSITION     (0.00010471975512f)
#define APP_CAN_SPEED_LIMIT_RPM     20000.0f

#define APP_VDC_MIN                 11.0f
#define APP_VDC_RECOVER             11.5f
#define APP_VDC_FAULT_MS            50u
#define APP_VDC_RECOVER_MS          200u
#define APP_VDC_FAULT_TICKS         (APP_VDC_FAULT_MS * APP_TICKS_PER_MS)
#define APP_VDC_RECOVER_TICKS       (APP_VDC_RECOVER_MS * APP_TICKS_PER_MS)

#define APP_SPEED_WINDOW_TICKS      80u      /* 80 拍 = 2ms */
#define APP_SPEED_WINDOW_INV        0.0125f
#define APP_SPEED_LPF_ALPHA         0.25f
#define APP_CURRENT_PID_DT_INV      ((float)APP_CTRL_HZ)
#define APP_SPEED_RPM_SCALE         (60.0f * 0.15915494309f * APP_CURRENT_PID_DT_INV * APP_SPEED_WINDOW_INV)
#define APP_ENCODER_TWO_PI          APP_TWO_PI
#define APP_ENCODER_HALF_PI         APP_PI
#define APP_ENCODER_POLE_PAIRS_INV  0.142857142857f

#define APP_CURRENT_FILTER_WINDOW   5u
#define APP_CURRENT_LPF_ALPHA       0.0125f  /* 保持原时间常数(系数随频率 4x) */
#define APP_CURRENT_MAG_ALPHA       0.005f

#define APP_CURRENT_PID_KP           0.10f
#define APP_CURRENT_PID_KI           40.0f
#define APP_CURRENT_PID_DT           (1.0f / (float)APP_CTRL_HZ)
#define APP_CURRENT_PID_LIMIT        18.0f
#define APP_CURRENT_PID_ILIMIT       15.0f
#define APP_SPEED_PID_KP             0.020f
#define APP_SPEED_PID_KI             0.01f
#define APP_SPEED_PID_DT             (1.0f / (float)APP_CTRL_HZ)
#define APP_SPEED_PID_LIMIT          8.0f
#define APP_SPEED_PID_ILIMIT         5.0f
#define APP_POSITION_PID_KP           500.0f
#define APP_POSITION_PID_KI           0.0f
#define APP_POSITION_PID_DT          (1.0f / (float)APP_CTRL_HZ)
#define APP_POSITION_PID_LIMIT        500.0f
#define APP_POSITION_PID_ILIMIT       300.0f

#endif
