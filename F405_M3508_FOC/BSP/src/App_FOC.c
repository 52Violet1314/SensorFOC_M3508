#include "App_FOC.h"
#include "BSP_AdcIrq.h"
#include "App_Config.h"
#include "adc.h"
#include "BSP_CAN.h"
#include "BSP_Timer.h"
#include "Control.h"
#include "Encoder.h"
#include "FOC_Param.h"
#include "gpio.h"
#include "HFI.h"
#include "PLL.h"
#include "SMO.h"
#include "SVPWM.h"
#include "tim.h"
#include "Transformer.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <math.h>

float ADC_Current[3];
float ADC_Voltage[3];
float ADC_Power;
float ADC_Temp;
float Current[3];
float ialpha;
float ibeta;
float ia_filt;
float ib_filt;
float cur_mag;
float cur_mag_filt;
float id;
float iq;
float id_filt;
float iq_filt;
float Voltage[3];
float Power;
float Temp;
int Encoder_Sin;
int Encoder_Cos;
float Encoder_Angle;
float Encoder_Elec_Angle;
float Encoder_Mech_Position;
float csa_vmid[3] = {2047.0f, 2047.0f, 2047.0f};
float spd_meas_rpm;

static volatile uint8_t adc_cal_done;
static uint32_t cal_sum[3];
static uint32_t cal_count[3];
static volatile uint8_t adc_group_mask;
static uint8_t hfi_trip;
static uint8_t hfi_hdr_done;
static uint8_t isr_cyc_reported;
static uint32_t hfi_debug_ms;
static uint32_t encoder_last_valid_ms;
static uint32_t can_feedback_ms;
static uint32_t heartbeat_ms;
static uint32_t obs_debug_ms;
static uint8_t obs_hdr_done;
static SMO_Config_t smo_obs;   /* 滑模观测器 */
static PLL_t        smo_pll;   /* 观测角度的锁相环: 平滑角度 + 直接给速度 */
/* 观测定角(高速区): 控制角在"编码器角"和"观测角"之间按 ramp 平滑交接 */
static float   theta_ctrl;      /* 本拍 Park/DePark 统一使用的角度 */
static float   obs_ramp;        /* 0=编码器角, 1=观测角 */
static float   obs_ang_off;     /* 观测角常数偏置 rad */
static float   obs_cmd_theta;   /* 观测角命令值 = PLL角 + 群延迟补偿 + 偏置(控制/打印都用它) */
static float   obs_switch_rpm;  /* 切换阈值(电机侧 rpm) */
static uint8_t obs_angle_en;    /* 1 = 允许自动切换(默认开, CAN 0x15 可改) */
static uint32_t obs_arm_cnt;    /* 切换条件已连续成立的拍数 */
static uint8_t theta_ctrl_init;

/* 观测器入口(实现在下面, 换观测器只改这一个函数) */
static uint8_t App_ObserverStep(float ia, float ib, float va, float vb);

static float cur_id_ref;
static float cur_iq_ref;
static float cur_id_err;
static float cur_iq_err;
static float cur_vd;
static float cur_vq;
static float speed_ref_cmd_rpm;
static float pos_err_rad;
static float cur_mech_prev;
static float cur_spd_accum;
static float spd_err_rpm;
static uint16_t cur_spd_count;
static uint16_t speed_loop_count;
static uint8_t cur_mech_valid;
static Control_Mode_t previous_mode = CONTROL_MODE_SPEED;
static volatile uint8_t vdc_fault;
static uint32_t vdc_low_count;
static uint32_t vdc_ok_count;

typedef struct
{
    float sample[APP_CURRENT_FILTER_WINDOW];
    uint8_t index;
    float low_pass;
} CurrentFilter_t;

static CurrentFilter_t ia_filter;
static CurrentFilter_t ib_filter;
static CurrentFilter_t id_filter;
static CurrentFilter_t iq_filter;

static float App_WrapAngle(float angle)
{
    angle = fmodf(angle, APP_TWO_PI);
    if (angle > APP_PI) angle -= APP_TWO_PI;
    if (angle < -APP_PI) angle += APP_TWO_PI;
    return angle;
}

static float App_FilterCurrent(CurrentFilter_t *filter, float input)
{
    float sorted[APP_CURRENT_FILTER_WINDOW];
    float median;

    filter->sample[filter->index] = input;
    filter->index = (uint8_t)((filter->index + 1u) % APP_CURRENT_FILTER_WINDOW);
    for (uint8_t i = 0u; i < APP_CURRENT_FILTER_WINDOW; i++)
    {
        sorted[i] = filter->sample[i];
    }
    for (uint8_t i = 1u; i < APP_CURRENT_FILTER_WINDOW; i++)
    {
        float key = sorted[i];
        uint8_t j = i;
        while (j > 0u && sorted[j - 1u] > key)
        {
            sorted[j] = sorted[j - 1u];
            j--;
        }
        sorted[j] = key;
    }
    median = sorted[APP_CURRENT_FILTER_WINDOW / 2u];
    filter->low_pass += APP_CURRENT_LPF_ALPHA * (median - filter->low_pass);
    return filter->low_pass;
}

static int16_t App_SaturateI16(float value)
{
    if (value >= 32767.0f) return 32767;
    if (value <= -32768.0f) return -32768;
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void App_PutI16(uint8_t *dst, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    dst[0] = (uint8_t)(raw >> 8);
    dst[1] = (uint8_t)raw;
}

static uint8_t App_GetErrorCode(void)
{
    if (Power < APP_VDC_MIN) return 0x01u;
    if (Power >= 27.0f) return 0x02u;
    if (Encoder_IsValid() == 0u ||
        (HAL_GetTick() - encoder_last_valid_ms) > 20u) return 0x03u;
    if (Temp > 60.0f) return 0x04u;
    return 0x00u;
}

static void App_SendCanFeedback(void)
{
    uint8_t data[8] = {0u};
    float current = fminf(fmaxf(iq_filt, -30.0f), 30.0f);
    float speed = fminf(fmaxf(spd_meas_rpm,
                              -APP_CAN_SPEED_LIMIT_RPM),
                        APP_CAN_SPEED_LIMIT_RPM);
    float position = App_WrapAngle(Encoder_Mech_Position);
    float temperature = fminf(fmaxf(Temp, 0.0f), 255.0f);

    App_PutI16(&data[0], App_SaturateI16(current * APP_CAN_IQ_SCALE));
    App_PutI16(&data[2], App_SaturateI16(speed));
    App_PutI16(&data[4], App_SaturateI16(position *
                                         APP_CAN_POSITION_TO_RAW));
    data[6] = (uint8_t)(temperature + 0.5f);
    data[7] = App_GetErrorCode();
    (void)BSP_CAN_SendStd(0x78u, data, 8u);
}

/* 打印 HFI 辨识结果: e2 = 2*编码器电角度, d2 = 2*辨识电角度,
   两者应同步变化(可相差一个常数偏置); L0/L1 单位 mH; sal = L1/L0 百分比 */
static void App_PrintHfiDebug(void)
{
    float e2 = Encoder_Elec_Angle * 2.0f;
    float d2 = hfi_theta * 2.0f;
    float sal = 0.0f;

    while (e2 >= APP_TWO_PI) e2 -= APP_TWO_PI;
    while (d2 >= APP_TWO_PI) d2 -= APP_TWO_PI;
    if (hfi_l0 > 1.0e-5f) sal = hfi_l1 / hfi_l0 * 100.0f;

    unsigned drv = (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET) ? 1u : 0u;

    (void)CDC_Printf("# en=%u trip=%u drv=%u probe=%u U=%.2f fb=%u vdc=%.1f pol=%.0f "
                     "fit=%u e2=%.3f d2=%.3f L0=%.3f L1=%.3f sal=%.2f I=%.2f\r\n",
                     (unsigned)hfi_enable, (unsigned)hfi_trip, drv,
                     (unsigned)hfi_probe, hfi_u_inj, (unsigned)hfi_foldback,
                     Power, hfi_pol, (unsigned)hfi_fit_count,
                     e2, d2, hfi_l0 * 1000.0f, hfi_l1 * 1000.0f, sal,
                     hfi_i_peak);
}

/* ---------------------------------------------------------------------------
   纯数字行: 每个采样周期一行, 逗号分隔, 直接喂 Excel / Python / 串口绘图, 无任何标签。
   列序(t_ms,enc,hfi,err,L0,L1,sal,I,U,vdc):
     enc : 磁编码器换算出的电角度 rad [0,2pi)
     hfi : HFI 辨识出的电角度 rad [0,2pi)
     err : enc - hfi, 归一化到 [-pi,pi)
   凸极法只能给出模 pi 的轴(高电感轴), 这里的 hfi 用编码器在 ±pi/2 内选最近的一支,
   只是为了把两条曲线画在一起; 若 err 基本恒定 -> 差值就是常数偏置; err 的波动 = 精度;
   若 err 呈斜坡/镜像, 说明有速度滞后或符号问题。
--------------------------------------------------------------------------- */
static void App_PrintHfiRow(void)
{
#if (HFI_FIX_PHI != 0u)
    /* 诊断模式(见 HFI.h): 注入方向固定, 打印 HF 电流矢量的幅值与倾斜角。
     *   tilt_pp = 2*atan(L1/L0): 手拨转子转, 它会摆动 -> 真凸极; 纹丝不动 -> 伪各向异性。
     *   phi 恒为 0 是本模式生效的自检。 */
    (void)CDC_Printf("%u,%.2f,%.2f,%.1f,%.1f,%.3f,%.2f,%.2f,%.3f\r\n",
                     (unsigned)HAL_GetTick(),
                     hfi_diag_tilt_ctr, hfi_diag_tilt_pp,
                     hfi_diag_mag_ctr, hfi_diag_mag_pp,
                     hfi_i_peak, hfi_u_inj, Power, hfi_phi);
    return;
#else
    float enc = Encoder_Elec_Angle;
    float hfi = hfi_theta;
    float err;
    float sal = 0.0f;

    if (hfi_l0 > 1.0e-5f) sal = hfi_l1 / hfi_l0 * 100.0f;

    while (hfi < enc - 1.5707963268f) hfi += APP_PI;
    while (hfi > enc + 1.5707963268f) hfi -= APP_PI;
    while (hfi < 0.0f) hfi += APP_TWO_PI;
    while (hfi >= APP_TWO_PI) hfi -= APP_TWO_PI;

    err = enc - hfi;
    while (err >= APP_PI) err -= APP_TWO_PI;
    while (err < -APP_PI) err += APP_TWO_PI;

    (void)CDC_Printf("%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.3f,%.2f,%.1f\r\n",
                     (unsigned)HAL_GetTick(), enc, hfi, err,
                     hfi_l0 * 1000.0f, hfi_l1 * 1000.0f, sal,
                     hfi_i_peak, hfi_u_inj, Power);
#endif
}

void BSP_CAN_RxCallback(const CAN_RxHeaderTypeDef *header, const uint8_t *data)
{
    int16_t command;

    if (header == 0 || data == 0 || header->IDE != CAN_ID_STD ||
        header->StdId != 0x91u || header->DLC < 3u)
    {
        return;
    }

    command = (int16_t)(((uint16_t)data[1] << 8) | data[2]);
    switch (data[0])
    {
        case 0x01u:
        case 0x02u:
        case 0x03u:
            /* 发运动指令 = 要用电动机模式: 自动退出 HFI(否则电流环被旁路, 电机不会动) */
            if (hfi_enable != 0u) HFI_SetEnable(0u);
            if (data[0] == 0x01u)
            {
                Control_SetCurrentTarget(0.0f, (float)command * 0.001f);
            }
            else if (data[0] == 0x02u)
            {
                Control_SetSpeedTarget((float)command);
            }
            else
            {
                Control_SetPositionTarget((float)command *
                                           APP_CAN_RAW_TO_POSITION);
            }
            break;
        case 0x10u:  /* HFI 注入开关: 非 0 = 使能并解除过流封锁 */
            if (command != 0)
            {
                hfi_trip = 0u;
                HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_SET);
                HFI_SetEnable(1u);
            }
            else
            {
                HFI_SetEnable(0u);
            }
            break;
        case 0x11u:  /* HFI 注入幅值 mV */
            HFI_SetUInj((float)command * 0.001f);
            break;
        case 0x15u:  /* 观测定角开关: 非 0 = 允许高速自动切到观测角 */
            obs_angle_en = (command != 0) ? 1u : 0u;
            break;
        case 0x16u:  /* 观测定角切换阈值(电机侧 rpm) */
            if (command > 0)
            {
                obs_switch_rpm = (float)command;
            }
            break;
        case 0x17u:  /* 观测角常数偏置 mrad */
            obs_ang_off = (float)command * 0.001f;
            break;
        default:
            break;
    }
}

static void App_UpdateEncoder(void)
{
    Encoder_Update(Encoder_Sin, Encoder_Cos);
    Encoder_Angle = Encoder_GetMechAngle();
    Encoder_Elec_Angle = Encoder_GetElecAngle();
    Encoder_Mech_Position = Encoder_GetMechanicalPosition();
    if (Encoder_IsValid() != 0u)
    {
        encoder_last_valid_ms = HAL_GetTick();
    }
}

static void App_UpdateSpeed(void)
{
    if (cur_mech_valid == 0u)
    {
        cur_mech_prev = Encoder_Mech_Position;
        cur_mech_valid = 1u;
        return;
    }

    cur_spd_accum += Encoder_Mech_Position - cur_mech_prev;
    cur_mech_prev = Encoder_Mech_Position;
    if (++cur_spd_count >= APP_SPEED_WINDOW_TICKS)
    {
        spd_meas_rpm += APP_SPEED_LPF_ALPHA *
                        (cur_spd_accum * APP_SPEED_RPM_SCALE - spd_meas_rpm);
        cur_spd_accum = 0.0f;
        cur_spd_count = 0u;
    }
}

static void App_UpdateProtection(float vdc)
{
    if (vdc_fault == 0u)
    {
        if (vdc < APP_VDC_MIN)
        {
            if (++vdc_low_count >= APP_VDC_FAULT_TICKS)
            {
                vdc_fault = 1u;
                vdc_low_count = 0u;
                vdc_ok_count = 0u;
                Control_ResetCurrentPID();
                Control_ResetSpeedPID();
                Control_ResetPositionPID();
                cur_iq_ref = 0.0f;
                speed_ref_cmd_rpm = 0.0f;
                spd_err_rpm = 0.0f;
                spd_meas_rpm = 0.0f;
                cur_spd_accum = 0.0f;
                cur_spd_count = 0u;
                cur_id_err = 0.0f;
                cur_iq_err = 0.0f;
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
                HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_RESET);
            }
        }
        else
        {
            vdc_low_count = 0u;
        }
    }
    else
    {
        if (vdc > APP_VDC_RECOVER)
        {
            if (++vdc_ok_count >= APP_VDC_RECOVER_TICKS)
            {
                vdc_fault = 0u;
                vdc_ok_count = 0u;
                Control_ResetCurrentPID();
                Control_ResetSpeedPID();
                Control_ResetPositionPID();
                HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_SET);
            }
        }
        else
        {
            vdc_ok_count = 0u;
        }
    }
}

static uint8_t App_ProtectionActive(void)
{
    if (vdc_fault == 0u) return 0u;
    Control_ResetSpeedPID();
    Control_ResetPositionPID();
    cur_iq_ref = 0.0f;
    speed_ref_cmd_rpm = 0.0f;
    spd_err_rpm = 0.0f;
    spd_meas_rpm = 0.0f;
    cur_spd_accum = 0.0f;
    cur_spd_count = 0u;
    cur_id_err = 0.0f;
    cur_iq_err = 0.0f;
    pos_err_rad = 0.0f;
    cur_vd = 0.0f;
    cur_vq = 0.0f;
    return 1u;
}

/* HFI 过流封锁: 撤注入 + 关驱动, 需 CAN 0x10 重新使能 */
static void App_HfiTrip(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
    HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_RESET);
    HFI_SetEnable(0u);
    cur_id_ref = 0.0f;
    cur_iq_ref = 0.0f;
}

/* 反电动势低通的相位滞后补偿量 = atan(we/omega_c), 带符号(反转自动反号)。
 *   用有理逼近代替 atanf: ISR 每拍都要算, 逼近式只要几次乘加(误差 <0.002rad)。
 *   12V 台架上 we/omega_c 最大约 0.95(<1), 超出按 1 截断(连续, 不会跳变)。 */
static float Obs_EmfLag(float we)
{
    float a = we / OBS_EMF_WC;
    float s = (a < 0.0f) ? -1.0f : 1.0f;
    float m = fabsf(a);

    if (m > 1.0f) m = 1.0f;
    return s * (0.7853982f * m - m * (m - 1.0f) * (0.2447f + 0.0663f * m));
}

static void App_CurrentLoopTick(float vdc, float theta_elec)
{
    Control_Mode_t mode;

    App_UpdateSpeed();
    App_UpdateProtection(vdc);
    if (App_ProtectionActive() != 0u)
    {
        return;
    }

    /* HFI 实验模式: 只做高频注入, 电流/速度/位置环全部旁路, 不输出转矩 */
    if (hfi_enable != 0u)
    {
        HFI_Tick(ialpha, ibeta, vdc);
        if (hfi_fault != HFI_FAULT_NONE)
        {
            hfi_trip = 1u;
            App_HfiTrip();
        }
        return;
    }
    if (hfi_trip != 0u)
    {
        App_HfiTrip();
        return;
    }

    /* 控制角: 编码器角 与 观测角 按 ramp 平滑交接(在下面的观测器接口位更新) */
    theta_elec = theta_ctrl;

    mode = Control_GetMode();
    if (mode != previous_mode)
    {
        Control_ResetSpeedPID();
        Control_ResetPositionPID();
        speed_loop_count = 0u;
        cur_iq_ref = 0.0f;
        cur_id_ref = 0.0f;
        speed_ref_cmd_rpm = 0.0f;
        pos_err_rad = 0.0f;
        previous_mode = mode;
    }

    if (mode == CONTROL_MODE_CURRENT)
    {
        Control_GetCurrentTarget(&cur_id_ref, &cur_iq_ref);
        speed_ref_cmd_rpm = 0.0f;
        spd_err_rpm = 0.0f;
        pos_err_rad = 0.0f;
    }
    else if (++speed_loop_count >= Control_GetSpeedLoop())
    {
        speed_loop_count = 0u;
        if (mode == CONTROL_MODE_SPEED)
        {
            speed_ref_cmd_rpm = Control_GetSpeedTarget();
            spd_err_rpm = speed_ref_cmd_rpm - spd_meas_rpm;
            cur_iq_ref = Control_SpeedStep(spd_err_rpm);
            cur_id_ref = 0.0f;
        }
        else if (mode == CONTROL_MODE_POSITION)
        {
            float position = App_WrapAngle(Encoder_Mech_Position);
            pos_err_rad = Control_GetPositionError(position);
            speed_ref_cmd_rpm = Control_PositionStep(position);
            spd_err_rpm = speed_ref_cmd_rpm - spd_meas_rpm;
            cur_iq_ref = Control_SpeedStep(spd_err_rpm);
            cur_id_ref = 0.0f;
        }
    }

    cur_id_err = cur_id_ref - id_filt;
    cur_iq_err = cur_iq_ref - iq_filt;
    cur_vd = Control_CurrentDStep(cur_id_err);
    cur_vq = Control_CurrentQStep(cur_iq_err);
    {
        float valpha;
        float vbeta;
        uint16_t ccr1;
        uint16_t ccr2;
        uint16_t ccr3;
        DeParkTransformer(&valpha, &vbeta, theta_elec, cur_vq, cur_vd);
        /* ===== 观测器 + PLL + 观测定角 =====
         * 只在 HFI 关闭时跑(HFI 开着时电压是纯注入, 平均为 0, 观测器没有可用模型)。 */
        if (hfi_enable == 0u)
        {
            float rpm_e, thr, dth, step;
            uint8_t obs_ok;

            /* ---- 观测器(唯一入口, 见 App_ObserverStep) ---- */
            obs_ok = App_ObserverStep(ialpha, ibeta, valpha, vbeta);

            /* 电机侧 rpm(由电角速度换算): rpm = ωe / 极对数 × 60/2π */
            rpm_e = PLL_GetOmega(&smo_pll) * 60.0f /
                    (APP_TWO_PI * (float)FOC_POLE_PAIRS);

            /* 切换判据: 使能 + 反电动势有效 + 转速过阈值(带回差), 否则斜坡退回 */
            thr = (obs_ramp > 0.5f) ? (obs_switch_rpm - OBS_SWITCH_HYST)
                                    :  obs_switch_rpm;
            step = FOC_TS / OBS_RAMP_TIME_S;
            if ((obs_angle_en != 0u) && (obs_ok != 0u) &&
                (fabsf(rpm_e) > thr))
            {
                if (obs_arm_cnt < OBS_ARM_TICKS) obs_arm_cnt++;
            }
            else
            {
                obs_arm_cnt = 0u;
            }
            if (obs_arm_cnt >= OBS_ARM_TICKS)
            {
                obs_ramp += step;
                if (obs_ramp > 1.0f) obs_ramp = 1.0f;
            }
            else
            {
                obs_ramp -= step;
                if (obs_ramp < 0.0f) obs_ramp = 0.0f;
            }

            /* 控制角 = 编码器角 + ramp·[(观测角+偏置) − 编码器角]
             *   ramp=1 时恒等于 观测角+偏置, 与编码器无关(真正的 sensorless) */
            if (theta_ctrl_init == 0u)
            {
                theta_ctrl = Encoder_Elec_Angle;
                theta_ctrl_init = 1u;
            }
            dth = obs_cmd_theta - Encoder_Elec_Angle;
            while (dth >=  APP_PI) dth -= APP_TWO_PI;
            while (dth <  -APP_PI) dth += APP_TWO_PI;
            theta_ctrl = Encoder_Elec_Angle + obs_ramp * dth;
            while (theta_ctrl >= APP_TWO_PI) theta_ctrl -= APP_TWO_PI;
            while (theta_ctrl <  0.0f)        theta_ctrl += APP_TWO_PI;
        }
        SVPWM(valpha, vbeta, vdc, &ccr1, &ccr2, &ccr3);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
    }
}

/* ============================================================================
 * 观测器接口 —— 每拍调用一次, 这是"换观测器只改这里"的唯一入口
 *
 *   输入: ia, ib  本拍 αβ 电流 (A)
 *         va, vb  本拍 αβ 指令电压 (V, 不含任何注入)
 *   输出: 写全局 obs_cmd_theta —— 电角度, 已含相位补偿与常数偏置
 *   返回: 1 = 本拍角度有效;  0 = 无效(调用方保持上一次的值)
 *
 *   ★下面这段 SMO + PLL 只是参考实现, 可以整段替换成你自己的观测器;
 *     唯一约定就是"吃 ialpha/ibeta/valpha/vbeta, 吐 obs_cmd_theta"。
 * ========================================================================== */
static uint8_t App_ObserverStep(float ia, float ib, float va, float vb)
{
    float e_sq;

    SMO_Update(&smo_obs, ia, ib, va, vb);
    e_sq = smo_obs.E_alpha * smo_obs.E_alpha +
           smo_obs.E_beta  * smo_obs.E_beta;
    if (e_sq <= OBS_EMF_MIN_SQ)
    {
        return 0u;                    /* |E| 太小, 角度不可信: 不更新 */
    }
    PLL_Step(&smo_pll, smo_obs.theta_e);

    /* PLL 角 + 反电动势低通相位滞后补偿 + 常数偏置 */
    obs_cmd_theta = PLL_GetTheta(&smo_pll)
                  + Obs_EmfLag(PLL_GetOmega(&smo_pll))
                  + obs_ang_off;
    return 1u;
}

/* 三相电流同源后执行的统一处理(原 ADC1 分支内容) */
static void App_AdcStep(void)
{
    /* 原始值 -> 工程量(中断只搬原始值, 换算是应用层的事) */
    ADC_Voltage[0] = BSP_AdcRaw_PhaseV[0];
    ADC_Voltage[1] = BSP_AdcRaw_PhaseV[1];
    ADC_Voltage[2] = BSP_AdcRaw_PhaseV[2];
    Encoder_Sin = (int)BSP_AdcRaw_EncSin;
    Encoder_Cos = (int)BSP_AdcRaw_EncCos;
    ADC_Current[0] = BSP_AdcRaw_Current[0];
    ADC_Current[1] = BSP_AdcRaw_Current[1];
    ADC_Current[2] = BSP_AdcRaw_Current[2];
    ADC_Power = BSP_AdcRaw_Power;
    ADC_Temp  = BSP_AdcRaw_Temp;
    Voltage[0] = (float)ADC_Voltage[0] * APP_ADC_VOLT_SCALE * APP_PHASE_VOLT_DIVIDER;
    Voltage[1] = (float)ADC_Voltage[1] * APP_ADC_VOLT_SCALE * APP_PHASE_VOLT_DIVIDER;
    Voltage[2] = (float)ADC_Voltage[2] * APP_ADC_VOLT_SCALE * APP_PHASE_VOLT_DIVIDER;
    Temp = (float)ADC_Temp * APP_ADC_VOLT_SCALE;
    if (adc_cal_done == 0u)
    {
        cal_sum[0] += (uint32_t)ADC_Current[0]; cal_count[0]++;
        cal_sum[1] += (uint32_t)ADC_Current[1]; cal_count[1]++;
        cal_sum[2] += (uint32_t)ADC_Current[2]; cal_count[2]++;
    }

    if (adc_cal_done == 0u &&
        cal_count[0] >= APP_ADC_CAL_WINDOW &&
        cal_count[1] >= APP_ADC_CAL_WINDOW &&
        cal_count[2] >= APP_ADC_CAL_WINDOW)
    {
        csa_vmid[0] = (float)cal_sum[0] / (float)cal_count[0];
        csa_vmid[1] = (float)cal_sum[1] / (float)cal_count[1];
        csa_vmid[2] = (float)cal_sum[2] / (float)cal_count[2];
        adc_cal_done = 1u;
    }

    Current[0] = ((float)ADC_Current[0] - csa_vmid[0]) * APP_CURRENT_SCALE;
    Current[1] = ((float)ADC_Current[1] - csa_vmid[1]) * APP_CURRENT_SCALE;
    Current[2] = ((float)ADC_Current[2] - csa_vmid[2]) * APP_CURRENT_SCALE;
    ClarkTransformer(Current[0], Current[1], Current[2], &ialpha, &ibeta);
    ia_filt = App_FilterCurrent(&ia_filter, ialpha);
    ib_filt = App_FilterCurrent(&ib_filter, ibeta);
    cur_mag = sqrtf(ia_filt * ia_filt + ib_filt * ib_filt);
    cur_mag_filt += APP_CURRENT_MAG_ALPHA * (cur_mag - cur_mag_filt);
    Power = (float)ADC_Power * APP_ADC_VOLT_SCALE * APP_VDC_DIVIDER;
    App_UpdateEncoder();
    ParkTransformer(ialpha, ibeta, Encoder_Elec_Angle, &iq, &id);
    id_filt = App_FilterCurrent(&id_filter, id);
    iq_filt = App_FilterCurrent(&iq_filter, iq);
    App_CurrentLoopTick(Power, Encoder_Elec_Angle);
    HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin,
                      (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET) ?
                      GPIO_PIN_SET : GPIO_PIN_RESET);
    if (HAL_GetTick() - heartbeat_ms >= 500u)
    {
        HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
        heartbeat_ms = HAL_GetTick();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        HAL_IncTick();
    }
}

void App_FOC_Init(void)
{
    Control_Init();
    Control_SetCurrentPID(APP_CURRENT_PID_KP, APP_CURRENT_PID_KI,
                          APP_CURRENT_PID_DT, APP_CURRENT_PID_LIMIT,
                          APP_CURRENT_PID_ILIMIT);
    Control_SetSpeedPID(APP_SPEED_PID_KP, APP_SPEED_PID_KI,
                        APP_SPEED_PID_DT, APP_SPEED_PID_LIMIT,
                        APP_SPEED_PID_ILIMIT);
    Control_SetPositionPID(APP_POSITION_PID_KP, APP_POSITION_PID_KI,
                            APP_POSITION_PID_DT, APP_POSITION_PID_LIMIT,
                            APP_POSITION_PID_ILIMIT);
    Control_SetSpeedLoop(1u);
    Encoder_Init();
    SMO_Init(&smo_obs, FOC_RS, FOC_LS, FOC_TS);
    PLL_Init(&smo_pll, OBS_PLL_FN_HZ, OBS_PLL_ZETA, FOC_TS, OBS_PLL_WMAX);
    theta_ctrl = 0.0f;
    obs_ramp = 0.0f;
    obs_ang_off = OBS_ANGLE_OFF_DEF;
    obs_switch_rpm = OBS_SWITCH_RPM;
    obs_angle_en = OBS_ANGLE_EN_DEF;
    obs_arm_cnt = 0u;
    theta_ctrl_init = 0u;
    if (BSP_CAN_Init() != HAL_OK)
    {
        Error_Handler();
    }
    MX_USB_DEVICE_Init();
    BSP_Timer_Init();
    /* 把应用层处理函数挂到 ADC 中断上(底层只拿一个函数指针, 不认识它是什么) */
    BSP_AdcIrq_SetGroupHook(App_AdcStep);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc3);
    BSP_Timer_PWM_Start();

    {
        uint32_t calibration_start = HAL_GetTick();
        while (adc_cal_done == 0u)
        {
            if (HAL_GetTick() - calibration_start >= APP_ADC_CAL_TIMEOUT_MS)
            {
                adc_cal_done = 1u;
            }
        }
    }
    HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_SET);

    /* DWT 周期计数器: 用来量控制中断本体耗时(40kHz 下必须远小于 25us) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* HFI 实验: 必须在 CSA 零点标定完成之后再启动注入 */
    HFI_Init();
    if (HFI_AUTO_START != 0u)
    {
        HFI_SetEnable(1u);
    }
}

void App_FOC_MainLoop(void)
{
    if (HAL_GetTick() - can_feedback_ms >= APP_CAN_FEEDBACK_PERIOD_MS)
    {
        can_feedback_ms = HAL_GetTick();
        App_SendCanFeedback();
    }
    if (isr_cyc_reported == 0u && HAL_GetTick() >= 3000u)
    {
        isr_cyc_reported = 1u;
        (void)CDC_Printf("# ctrl_step_max=%u cyc = %.2f us (budget %.2f us @ %u Hz)\r\n",
                         (unsigned)BSP_AdcIrq_GetMaxCycle(),
                         (float)BSP_AdcIrq_GetMaxCycle() / 168.0f,
                         1.0e6f / (float)APP_CTRL_HZ,
                         (unsigned)APP_CTRL_HZ);
    }
    if (hfi_enable != 0u || hfi_trip != 0u)
    {
        if (HAL_GetTick() - hfi_debug_ms >= HFI_PRINT_MS)
        {
            hfi_debug_ms = HAL_GetTick();
            if (hfi_trip != 0u || hfi_fault != HFI_FAULT_NONE ||
                hfi_probe != 0u ||
                HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET)
            {
                App_PrintHfiDebug();      /* 异常/探针期间: 带标签整行, 便于诊断 */
            }
            else
            {
                if (hfi_hdr_done == 0u)   /* 首行前打印一次列名( # 开头 ) */
                {
                    hfi_hdr_done = 1u;
#if (HFI_FIX_PHI != 0u)
                    (void)CDC_Printf("# t_ms,tilt_ctr(deg),tilt_pp(deg),mag_ctr(mA),"
                                     "mag_pp(mA),Ipk(A),U(V),vdc(V),phi(rad)\r\n");
                    (void)CDC_Printf("# HFI diag: phi=0(alpha), U=%.2fV, win=%ums | "
                                     "tilt_pp=2*atan(L1/L0); 手转转子看 tilt_pp 是否变化\r\n",
                                     hfi_u_inj, (unsigned)HFI_DIAG_PRINT_MS);
#else
                    (void)CDC_Printf("# t_ms,enc,hfi,err,L0(mH),L1(mH),sal(%),"
                                     "I(A),U(V),vdc(V)\r\n");
#endif
                }
                App_PrintHfiRow();        /* 正常: 纯数字 */
            }
        }
    }
    /* 母线欠压故障事件: 一触发就封驱动 -> 电机不转, 这里明确打出来 */
    {
        static uint8_t vdc_fault_prev;
        if (vdc_fault != vdc_fault_prev)
        {
            vdc_fault_prev = vdc_fault;
            (void)CDC_Printf("# vdc_fault=%u vdc=%.2f V (阈值 %.1f/%.1f) trip=%u drv=%u\r\n",
                             (unsigned)vdc_fault, Power, APP_VDC_MIN, APP_VDC_RECOVER,
                             (unsigned)hfi_trip,
                             (unsigned)((HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET) ? 1u : 0u));
        }
    }
    /* 观测器诊断输出(仅 HFI 关闭时): 纯数字, 逗号分隔
       列: t_ms, enc, smo_theta, cmd_theta, err, pll_omega, E(V), vdc, ramp */
    if (hfi_enable == 0u && HAL_GetTick() - obs_debug_ms >= 50u)
    {
        obs_debug_ms = HAL_GetTick();
        if (obs_hdr_done == 0u)
        {
            obs_hdr_done = 1u;
            (void)CDC_Printf("# t_ms,enc,smo_theta,cmd_theta,err,pll_omega,E(V),vdc(V),ramp\r\n");
        }
        {
            float obs_err = Encoder_Elec_Angle - obs_cmd_theta;
            while (obs_err >=  APP_PI) obs_err -= APP_TWO_PI;
            while (obs_err <  -APP_PI) obs_err += APP_TWO_PI;
            (void)CDC_Printf("%u,%.4f,%.4f,%.4f,%.4f,%.1f,%.3f,%.1f,%.2f\r\n",
                             (unsigned)HAL_GetTick(),
                             Encoder_Elec_Angle,
                             smo_obs.theta_e,
                             obs_cmd_theta,
                             obs_err,
                             PLL_GetOmega(&smo_pll),
                             sqrtf(smo_obs.E_alpha * smo_obs.E_alpha +
                                   smo_obs.E_beta  * smo_obs.E_beta),
                             Power, obs_ramp);
        }
    }
    HAL_Delay(1u);
}
