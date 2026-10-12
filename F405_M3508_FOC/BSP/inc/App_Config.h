#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Common application constants. Keep all tunable values in one place. */
#define APP_USE_CMSIS_OS2          0
#define APP_TWO_PI                 6.28318530718f
#define APP_PI                     3.14159265359f
#define APP_ADC_VREF               3.3f
#define APP_ADC_MAX_COUNTS         4095.0f
#define APP_ADC_COUNTS_INV         0.00024420024420f /* 1 / 4095 */
#define APP_ADC_VOLT_SCALE         (APP_ADC_VREF * APP_ADC_COUNTS_INV)

#define APP_VDC_DIVIDER            18.727272f
#define APP_PHASE_VOLT_DIVIDER     (41.2f / 2.2f)
#define APP_CURRENT_SCALE           (APP_ADC_VOLT_SCALE * 20.0f)

#define APP_ADC_CAL_WINDOW          100u
#define APP_ADC_CAL_TIMEOUT_MS      100u
#define APP_ADC_STAT_WINDOW         250u

#define APP_CAN_FEEDBACK_PERIOD_MS  10u
#define APP_CAN_IQ_SCALE            1000.0f
#define APP_CAN_POSITION_SCALE      30000.0f
#define APP_CAN_POSITION_TO_RAW     (APP_CAN_POSITION_SCALE * 0.31830988618f)
#define APP_CAN_RAW_TO_POSITION     (0.00010471975512f)
#define APP_CAN_SPEED_LIMIT_RPM     20000.0f

/* ★减速比: 固件里所有"rpm"都是**电机侧**(编码器所在轴), 输出轴 = 电机/19。
 *   CAN 收发两边也都是电机侧 rpm —— 上位机按"输出轴 rpm"发 1000,
 *   就是 19000 电机 rpm(超出电机能力), 现象是"一给就顶到极限跑"。 */
#define APP_GEAR_RATIO              19.0f
#define APP_ELEC_PER_REV            (7.0f)     /* 每个机械转 7 个电周期 */

#define APP_VDC_MIN                 11.0f
#define APP_VDC_RECOVER             11.5f
#define APP_VDC_FAULT_MS            50u
#define APP_VDC_RECOVER_MS          200u
#define APP_VDC_FAULT_TICKS         (APP_VDC_FAULT_MS * 10u)
#define APP_VDC_RECOVER_TICKS       (APP_VDC_RECOVER_MS * 10u)

/* 编码器失效保护。
 *   ★为什么必须有: 编码器一失效(信号丢/断线), 角度就冻结, 转速读数恒为 0 ——
 *     速度环会把它当成"还没到目标", 电流一路加到限幅, 电机直接冲到底。
 *     这就是"机械角积分坏了"那次的同一类现象, 只是触发源不同。
 *   连续 APP_ENC_FAULT_MS 没有有效样本 -> 封驱动+关 EN; 恢复要连续有效 APP_ENC_RECOVER_MS */
#define APP_ENC_FAULT_MS            20u
#define APP_ENC_RECOVER_MS          200u
#define APP_ENC_FAULT_TICKS         (APP_ENC_FAULT_MS * 10u)
#define APP_ENC_RECOVER_TICKS       (APP_ENC_RECOVER_MS * 10u)

#define APP_SPEED_WINDOW_TICKS      20u
#define APP_SPEED_WINDOW_INV        0.05f
#define APP_SPEED_LPF_ALPHA         0.25f
#define APP_CURRENT_PID_DT_INV      10000.0f
#define APP_SPEED_RPM_SCALE         (60.0f * 0.15915494309f * APP_CURRENT_PID_DT_INV * APP_SPEED_WINDOW_INV)
#define APP_ENCODER_TWO_PI          APP_TWO_PI
#define APP_ENCODER_HALF_PI         APP_PI
#define APP_ENCODER_POLE_PAIRS_INV  0.142857142857f

#define APP_CURRENT_FILTER_WINDOW   5u
#define APP_CURRENT_LPF_ALPHA       0.05f
#define APP_CURRENT_MAG_ALPHA       0.02f

/* ---------------------------------------------------------------------------
 * 编码器电角度标定(2000 点查表) —— 用低频旋转电压矢量拖转子当基准
 *
 *   采样率约束(硬约束): 控制 10kHz, 要分辨 2000 格 ⇒ 电频率 ≤ 5Hz。
 *     2Hz 时每个电周期 5000 拍(2.5 拍/格), 40 个周期累计 20 万个样本
 *     ⇒ 每格约 100 个样本, 平均后格子噪声可忽略。
 *   电压幅值: 低速反电动势≈0, 电流≈V/Rs(0.194Ω)。0.6V→约3A, 足够把转子拖住;
 *     电压越大磁场越"硬", 齿槽力矩引起的转子滞后越小(表越准), 但发热也越大。
 * ------------------------------------------------------------------------- */
#define APP_CAL_VOLT            0.60f    /* 拖转电压幅值 V */
#define APP_CAL_FREQ_HZ         2.0f     /* 拖转电频率 Hz(≤5Hz!) */
#define APP_CAL_RAMP_S          2.0f     /* 频率从 0 升到标定值的时间 s */
#define APP_CAL_ALIGN_MS        500u     /* 先固定角对齐, 把转子拉到已知位置 */
#define APP_CAL_CYCLES          40u      /* 采集多少个电周期(2Hz×40 = 20s) */
#define APP_CAL_TIMEOUT_MS      90000u   /* 兜底超时 */

#define APP_CURRENT_PID_KP           0.10f
#define APP_CURRENT_PID_KI           40.0f
#define APP_CURRENT_PID_DT           0.0001f
#define APP_CURRENT_PID_LIMIT        18.0f
#define APP_CURRENT_PID_ILIMIT       15.0f
#define APP_SPEED_PID_KP             0.020f
#define APP_SPEED_PID_KI             0.01f
#define APP_SPEED_PID_DT             0.0001f
#define APP_SPEED_PID_LIMIT          8.0f
#define APP_SPEED_PID_ILIMIT         5.0f
#define APP_POSITION_PID_KP           500.0f
#define APP_POSITION_PID_KI           0.0f
#define APP_POSITION_PID_DT           0.0001f
#define APP_POSITION_PID_LIMIT        500.0f
#define APP_POSITION_PID_ILIMIT       300.0f

#endif
