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

#define APP_VDC_MIN                 11.0f
#define APP_VDC_RECOVER             11.5f
#define APP_VDC_FAULT_MS            50u
#define APP_VDC_RECOVER_MS          200u
#define APP_VDC_FAULT_TICKS         (APP_VDC_FAULT_MS * 10u)
#define APP_VDC_RECOVER_TICKS       (APP_VDC_RECOVER_MS * 10u)

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
