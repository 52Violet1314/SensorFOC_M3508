#include "App_FOC.h"
#include "App_Config.h"
#include "adc.h"
#include "BSP_CAN.h"
#include "BSP_Timer.h"
#include "Control.h"
#include "Encoder.h"
#include "gpio.h"
#include "SVPWM.h"
#include "tim.h"
#include "Transformer.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SMO.h"
#include "HFI_INJ.h"

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
static uint8_t encoder_sample_mask;
static uint32_t encoder_last_valid_ms;
static uint32_t can_feedback_ms;
static uint32_t heartbeat_ms;

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
/* 编码器失效保护状态(见 App_Config.h 里的说明) */
static volatile uint8_t enc_fault;
static uint32_t enc_bad_count;
static uint32_t enc_ok_count;

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

/* 标定状态机在前向声明: 它定义在文件后半段, 而 ADC 中断(前半段)要调它 */
static uint8_t App_CalTick(float vdc);
/* VOFA 采样同理: 它要用到标定区那批静态量, 所以定义在后面 */
static void App_VofaSample(void);

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
            Control_SetCurrentTarget(0.0f, (float)command * 0.001f);
            break;
        case 0x02u:
            Control_SetSpeedTarget((float)command);
            break;
        case 0x03u:
            Control_SetPositionTarget((float)command *
                                       APP_CAN_RAW_TO_POSITION);
            break;
        default:
            break;
    }
}

static void App_UpdateEncoder(void)
{
    if ((encoder_sample_mask & 0x03u) != 0x03u)
    {
        return;
    }
    encoder_sample_mask = 0u;
    Encoder_Update(Encoder_Sin, Encoder_Cos);
    Encoder_Angle = Encoder_GetMechAngle();
    Encoder_Elec_Angle = Encoder_GetElecAngle();
    Encoder_Mech_Position = Encoder_GetMechanicalPosition();
    if (Encoder_IsValid() != 0u)
    {
        encoder_last_valid_ms = HAL_GetTick();
    }
}

/* --------------------------------------------------------------------------
 * 滑模观测器(SMO) —— ★只观测, 不参与驱动
 *   驱动角永远来自编码器(Encoder_Elec_Angle); 这里只是把同一拍的 αβ 电压/电流
 *   喂给观测器, 算出它自己的电角度, 和编码器角对照着看 —— 用来独立验证编码器
 *   标定/对齐对不对(观测器完全不用编码器, 是另一条独立的测量路径)。
 *
 *   两个容易踩的点:
 *   1) 参数按控制频率重标: 本工程 10kHz, δ 必须 > G·k/(1+F) = 25.7A (见 SMO.h 表)。
 *   2) 观测角与编码器角之间有一个**固定常数差**(两套角度口径不同, 存档那版用
 *      OL_ANGLE_OFF_DEF = +0.4936rad 抵消)。这个常数先自适应对齐一次、之后慢速跟踪,
 *      否则你会看到"差 26°"而误判成编码器没标好。只影响显示, 不碰控制。
 * ------------------------------------------------------------------------ */
static float   smo_ia, smo_ib;        /* 同一拍的 αβ 电流(中值滤波后, 无相位滞后) */
static float   smo_va, smo_vb;        /* 同一拍的 αβ 指令电压 */
static float   smo_off;               /* 观测角与编码器角的常数差(自适应) */
static float   smo_theta;             /* 对齐后的观测电角度 rad */
static float   smo_err;               /* 对齐后与编码器角的差 rad(打点用) */
static uint8_t smo_seeded;
static uint8_t smo_valid;
static uint8_t smo_cal_busy;   /* 标定期间别喂观测器: 那时电压快照是旧的 */

static void App_SmoTick(void)
{
    float raw;
    float corr;

    if (smo_cal_busy != 0u)
    {
        /* 标定时由标定状态机直接给 SVPWM 加矢量, smo_va/smo_vb 停在旧值,
         *   喂进去算出来的 |E| 和角度都是假的(还可能误判 Valid) */
        smo_valid = 0u;
        return;
    }

    SMO.Input.Ialpha = smo_ia;
    SMO.Input.Ibeat  = smo_ib;
    SMO.Input.Valpha = smo_va;
    SMO.Input.Vbeta  = smo_vb;
    SMO.Output.ThetaEnc = Encoder_Elec_Angle;   /* 对照量: 同一拍的编码器角 */
    SMO_Run(&SMO);

    smo_valid = SMO.Output.Valid;
    if (smo_valid == 0u)
    {
        return;      /* |E| 不够(低速), 观测角不可信, 不参与对齐 */
    }

    raw  = App_WrapAngle(SMO.Output.Theta + SMO.Output.Lag);  /* 含滞后补偿 */
    corr = App_WrapAngle(Encoder_Elec_Angle);

    if (smo_seeded == 0u)
    {
        smo_off = App_WrapAngle(corr - raw);   /* 第一次直接对齐, 省得等收敛 */
        smo_seeded = 1u;
    }
    smo_theta = App_WrapAngle(raw + smo_off);
    smo_err   = App_WrapAngle(smo_theta - corr);
    smo_off   = App_WrapAngle(smo_off - 0.0005f * smo_err);   /* τ≈0.2s 慢速跟踪 */
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
    /* 编码器失效: 角度冻结 -> 转速读数恒 0 -> 速度环把电流顶到限幅 -> 冲飞。
     *   和 vdc_fault 同样处理: 封驱动 + 关 EN; 恢复要连续有效一段时间。 */
    {
        uint8_t enc_bad = ((Encoder_IsValid() == 0u) ||
                           ((HAL_GetTick() - encoder_last_valid_ms) > APP_ENC_FAULT_MS))
                              ? 1u : 0u;

        if (enc_fault == 0u)
        {
            if (enc_bad != 0u)
            {
                if (++enc_bad_count >= APP_ENC_FAULT_TICKS)
                {
                    enc_fault = 1u;
                    enc_bad_count = 0u;
                    enc_ok_count = 0u;
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
                enc_bad_count = 0u;
            }
        }
        else
        {
            if (enc_bad == 0u)
            {
                if (++enc_ok_count >= APP_ENC_RECOVER_TICKS)
                {
                    enc_fault = 0u;
                    enc_ok_count = 0u;
                    Control_ResetCurrentPID();
                    Control_ResetSpeedPID();
                    Control_ResetPositionPID();
                    HAL_GPIO_WritePin(EN_Gate_GPIO_Port, EN_Gate_Pin, GPIO_PIN_SET);
                }
            }
            else
            {
                enc_ok_count = 0u;
            }
        }
    }

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
    if ((vdc_fault == 0u) && (enc_fault == 0u)) return 0u;
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

static void App_CurrentLoopTick(float vdc, float theta_elec)
{
    Control_Mode_t mode;

    App_UpdateSpeed();
    App_UpdateProtection(vdc);
    if (App_ProtectionActive() != 0u)
    {
        return;
    }

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
        smo_va = valpha;       /* 观测器输入快照(本拍真正加下去的 αβ 电压) */
        smo_vb = vbeta;
        /* ★HFI 注入: 方波电压矢量沿 **PLL 估计的 d 轴**注入(不是编码器轴!)
         *   ① 放在 SMO 快照**之后**: 观测器的电流输入是低通后的 αβ, 电压若带 1kHz
         *      注入, 两边口径不一致会给 SMO 灌进假反电动势(只影响诊断, 但没必要)。
         *   ② 关着的时候 hfi_enable=0, 一次乘加都不花(热点路径零成本)。
         *   ③ 与采样同拍: 本工程 ADC 注入组在 CCR4 时刻采完, 中断里立刻写 CCR,
         *      所以"这一拍写的电压"正好作用在下一个采样区间上 —— 解调极性才对得上。
         *   ④ 沿编码器轴注入是错的: 电流会跟着编码器轴走, 解调量与凸极无关 —— 见
         *      FOC/inc/HFI_INJ.h 的"符号约定"和 tools/verify_hfi.py 的 [1b] 节。 */
        if ((hfi_enable != 0u) && (Encoder_IsValid() != 0u))
        {
            float via;
            float vib;

            HFI_INJ(&via, &vib);
            valpha += via;
            vbeta  += vib;
        }
        SVPWM(valpha, vbeta, vdc, &ccr1, &ccr2, &ccr3);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        ADC_Voltage[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        Encoder_Sin = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        ADC_Current[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
        ADC_Power = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);
        Voltage[0] = (float)ADC_Voltage[0] * APP_ADC_VOLT_SCALE *
                     APP_PHASE_VOLT_DIVIDER;
        if (adc_cal_done == 0u)
        {
            cal_sum[0] += (uint32_t)ADC_Current[0];
            cal_count[0]++;
            if (cal_count[0] >= APP_ADC_CAL_WINDOW &&
                cal_count[1] >= APP_ADC_CAL_WINDOW &&
                cal_count[2] >= APP_ADC_CAL_WINDOW)
            {
                csa_vmid[0] = (float)cal_sum[0] / (float)cal_count[0];
                csa_vmid[1] = (float)cal_sum[1] / (float)cal_count[1];
                csa_vmid[2] = (float)cal_sum[2] / (float)cal_count[2];
                adc_cal_done = 1u;
            }
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
        encoder_sample_mask |= 0x01u;
        App_UpdateEncoder();
        ParkTransformer(ialpha, ibeta, Encoder_Elec_Angle, &iq, &id);
        id_filt = App_FilterCurrent(&id_filter, id);
        iq_filt = App_FilterCurrent(&iq_filter, iq);
        /* ★标定期间由标定状态机接管输出(它直接给 SVPWM 加旋转矢量),
         *   标定不跑电流环 —— 电流环的反馈就是这个编码器, 会把要测的误差抵消掉 */
        if (App_CalTick(Power) == 0u)
        {
            App_CurrentLoopTick(Power, Encoder_Elec_Angle);
            smo_cal_busy = 0u;
        }
        else
        {
            smo_cal_busy = 1u;
        }
        smo_ia = ia_filt;      /* 观测器输入快照(与电流环同一拍) */
        smo_ib = ib_filt;
        App_VofaSample();
        HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin,
                          (HAL_GPIO_ReadPin(FAULT_GPIO_Port, FAULT_Pin) == GPIO_PIN_RESET) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET);
        if (HAL_GetTick() - heartbeat_ms >= 500u)
        {
            HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
            heartbeat_ms = HAL_GetTick();
        }
    }
    else if (hadc->Instance == ADC2)
    {
        ADC_Voltage[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        Encoder_Cos = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        ADC_Current[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
        ADC_Temp = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);
        Temp = (float)ADC_Temp * APP_ADC_VOLT_SCALE;
        Voltage[1] = (float)ADC_Voltage[1] * APP_ADC_VOLT_SCALE *
                     APP_PHASE_VOLT_DIVIDER;
        encoder_sample_mask |= 0x02u;
        App_UpdateEncoder();
        /* ★观测器在这里跑(ADC2 分支), 不放在 ADC1 分支:
         *   Encoder_Elec_Angle 是在上面这行更新的, 放在 ADC1 里会拿到**上一拍**的角度,
         *   1000rpm 下一拍电角度走 4.2° —— 对照就白对了。这里两边的量都是同一拍的。 */
        App_SmoTick();
        /* ★HFI 诊断观测器: **每拍都调**(哪怕注入关着) —— 它内部要维护编码器角速度
         *   估计用于转速门限; 停了不调会让差分变陈旧、下次使能立刻误判超速。
         *   输入用**未滤波**的 ialpha/ibeta(注入是 1kHz 方波, 低通/中值都会把它修掉),
         *   编码器角是这一拍刚更新的; 编码器无效/注入关着时函数内部自己会早退。 */
        HFI_Tick(ialpha, ibeta, Power, Encoder_Elec_Angle);
        if (adc_cal_done == 0u)
        {
            cal_sum[1] += (uint32_t)ADC_Current[1];
            cal_count[1]++;
        }
    }
    else if (hadc->Instance == ADC3)
    {
        ADC_Voltage[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        ADC_Current[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        Voltage[2] = (float)ADC_Voltage[2] * APP_ADC_VOLT_SCALE *
                     APP_PHASE_VOLT_DIVIDER;
        if (adc_cal_done == 0u)
        {
            cal_sum[2] += (uint32_t)ADC_Current[2];
            cal_count[2]++;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        HAL_IncTick();
    }
}

/* ==========================================================================
 * 编码器电角度标定: 用低频旋转电压矢量"拖"转子, 拿自己积分出来的 φ 当真值
 *
 *   为什么必须拖转而不是用闭环: 闭环的反馈就是这个编码器, 它会自动把要测的
 *   角度误差抵消掉 —— 闭环下标出来的表恒等于 0。
 *
 *   流程:  ALIGN(固定角吸住 0.5s) -> RAMP(频率 0→2Hz) -> RUN(采 40 个电周期)
 *          -> READY(主循环生成 2000 点表)
 *   每拍做两件事: ① φ += 2π·f·Ts  ② 把 (φ − 粗角度) 累加到粗角度所在的格子。
 * ========================================================================== */
typedef enum
{
    CAL_OFF = 0,
    CAL_ALIGN,
    CAL_RAMP,
    CAL_RUN,
    CAL_READY,      /* 采完, 等主循环算表 */
    CAL_ABORT
} App_CalState_t;

static volatile App_CalState_t cal_state = CAL_OFF;
static volatile float    cal_phi;          /* 驱动矢量角度(就是基准真值) */
static volatile float    cal_freq;         /* 当前拖转电频率 Hz */
static volatile float    cal_cycles;       /* 已采集的电周期数 */
static volatile uint32_t cal_tick_ms;
static volatile uint8_t  cal_stop_req;     /* 主循环请求中止 */
static volatile uint8_t  cal_fail_code;    /* 1=母线欠压 2=超时 3=编码器无效 */
static uint8_t  cal_result_code;      /* 0=OK 1=样本不足 2=覆盖不足 3=打滑 */
static float    cal_result_mean;
static float    cal_result_max;
static float    cal_result_resid;
static uint16_t cal_result_bins;
static uint32_t cal_progress_ms;
/* 电角度打点: `ang [hz]` 开关。放在主循环里(不占中断), 默认 100Hz。
 *   看三件事: corr 是否平滑单调(表要是引入了跳变, 这里一眼看出来)、
 *   lut 那一列就是表在实时补多少、以及一圈里 raw 走 7 个电周期(机械角对不对)。 */
static uint16_t ang_hz;
static uint32_t ang_ms;

/* --------------------------------------------------------------------------
 * VOFA+ 打点(FireWater: 一行一帧, 逗号分隔, 换行结尾; VOFA+ 里协议选 FireWater)
 *   通道顺序: corr,raw,lut,mech,spd,out,iq,vdc,obs,err,hfi_theta,hfi_err
 *     corr/raw/lut  电角度(rad): 查表后 / 粗角度 / 表补的量 —— 看表在干什么
 *     mech          电机轴机械角(rad)
 *     spd/out       电机侧 rpm / 输出轴 rpm
 *     iq,vdc        转矩电流(A) / 母线电压(V)
 *     obs,err       SMO 观测角 / (观测角 − 编码器角)
 *     hfi_theta     HFI(PLL)电角度 [0,2π); ★凸极法只有模 π 的轴, 差 π 是正常的
 *     hfi_err       HFI 角 − 编码器角, 折到 ±π/2(rad) —— 这个才是判据
 *
 *   ★采样在 10kHz 的 ADC 中断里按整数分频做, 不是在主循环里凑时间:
 *     主循环的粒度是 1ms(HAL_Delay), 500Hz 会让采样间隔在 2/3ms 之间跳,
 *     波形的时间轴就歪了。中断里分频 = 采样间隔严格是 Ts 的整数倍。
 *   中断只往环形缓冲里塞 12 个 float(约二十个周期), 打印在主循环做。
 *   ★12 通道 @500Hz ≈ 54kB/s, 已接近 USB CDC 的 ~60kB/s: 丢帧看 `vofa 0`
 *     打印的计数, 或把频率降到 300。
 * ------------------------------------------------------------------------ */
#define APP_VOFA_CH   12u
#define APP_VOFA_BUF  64u

static volatile uint16_t vofa_div;          /* 0 = 关; >0 = 每 div 拍采一帧 */
static volatile uint16_t vofa_tick;
static volatile uint8_t  vofa_head;
static volatile uint8_t  vofa_tail;
static volatile uint32_t vofa_drop;
static volatile float    vofa_buf[APP_VOFA_BUF][APP_VOFA_CH];
static uint16_t vofa_hz;

/* 中断里调用: 每 div 拍把当前量快照进环形缓冲(满了就丢并计数) */
static void App_VofaSample(void)
{
    uint8_t next;
    float *s;

    if (vofa_div == 0u) return;
    if (++vofa_tick < vofa_div) return;
    vofa_tick = 0u;

    next = (uint8_t)((vofa_head + 1u) % APP_VOFA_BUF);
    if (next == vofa_tail)
    {
        vofa_drop++;      /* 主循环没跟上/串口堵了: 丢帧但绝不阻塞中断 */
        return;
    }
    {
        float raw = Encoder_GetRawElecAngle();

        s = (float *)vofa_buf[vofa_head];
        s[0] = Encoder_Elec_Angle;
        s[1] = raw;
        s[2] = Encoder_LutGet(Encoder_LutIndex(raw));
        s[3] = Encoder_Mech_Position;
        s[4] = spd_meas_rpm;
        s[5] = spd_meas_rpm / APP_GEAR_RATIO;
        s[6] = iq_filt;
        s[7] = Power;
        s[8] = smo_theta;      /* 观测器电角度(已扣常数差) */
        s[9] = smo_err;        /* 观测角 − 编码器角, rad */
        s[10] = HFI_GetStreamTheta();  /* HFI/PLL 电角度 [0,2π)(模 π 口径) */
        s[11] = HFI_GetStreamErr();    /* HFI 角 − 编码器角, 折到 ±π/2, rad */
    }
    vofa_head = next;
}
/* 标定拖转电压: 可用 `calv <V>` 运行时改。
 *   ★这个值直接决定"转子被齿槽拽住、滞后磁场多少": 电压越大磁场越硬、滞后越小,
 *     混进表里的机械滞后就越小 —— 它是"标定纯度"的主旋钮。
 *     判定办法: 同一台电机用两档电压各标一次, 看表的峰峰值是否跟着往下掉:
 *       掉了  = 锯齿那部分主要是转子滞后(物理效应), 加大电压就能压
 *       不动  = 那部分是编码器自身的非线性, 只能靠表修
 *     残差(均|残差|)两档对比也能看出来, 但峰峰值最直观。 */
static float    cal_voltage = APP_CAL_VOLT;

static void App_CalOutputZero(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
}

/* 标定期间 App_UpdateSpeed() 一次都没跑, 机械位置却转了好几圈 ——
 *   恢复控制前必须把速度估计复位, 否则第一拍会算出"20 秒的巨大位移"当速度尖峰,
 *   速度环直接把电流打到底(现象: 标定一结束电机猛冲一下)。
 *   中断里(采完那一拍)和主循环里(中止)都要调, 所以单独抽出来。 */
static void App_CalReleaseEstimator(void)
{
    spd_meas_rpm  = 0.0f;
    cur_spd_accum = 0.0f;
    cur_spd_count = 0u;
    cur_mech_valid = 0u;      /* 下一拍重新取基准点 */
}

static void App_CalStop(void)
{
    App_CalOutputZero();
    cal_state = CAL_OFF;
    cal_freq = 0.0f;

    /* 交还控制权前把三个环全部清零并锁住目标, 免得出标定瞬间窜一下 */
    Control_ResetCurrentPID();
    Control_ResetSpeedPID();
    Control_ResetPositionPID();
    Control_SetSpeedTarget(0.0f);
    Control_SetCurrentTarget(0.0f, 0.0f);
    Control_SetPositionTarget(App_WrapAngle(Encoder_Mech_Position));
    App_CalReleaseEstimator();
}

/* 返回 1 = 本拍由标定接管(不跑电流环) */
static uint8_t App_CalTick(float vdc)
{
    float va;
    float vb;
    uint16_t ccr1;
    uint16_t ccr2;
    uint16_t ccr3;
    uint32_t now = HAL_GetTick();

    if (cal_state == CAL_OFF || cal_state == CAL_READY || cal_state == CAL_ABORT)
    {
        return 0u;
    }

    if (vdc < APP_VDC_MIN)                  /* 标定期间也不能丢了欠压保护 */
    {
        cal_fail_code = 1u;
        cal_state = CAL_ABORT;
        App_CalOutputZero();
        return 0u;
    }
    if (cal_stop_req != 0u)
    {
        cal_fail_code = 0u;
        cal_state = CAL_ABORT;
        App_CalOutputZero();
        return 0u;
    }
    if ((now - cal_tick_ms) > APP_CAL_TIMEOUT_MS)
    {
        cal_fail_code = 2u;
        cal_state = CAL_ABORT;
        App_CalOutputZero();
        return 0u;
    }

    switch (cal_state)
    {
        case CAL_ALIGN:
            cal_phi = 0.0f;                 /* 固定角吸住转子, 建立已知起点 */
            if ((now - cal_tick_ms) >= APP_CAL_ALIGN_MS)
            {
                cal_state = CAL_RAMP;
                cal_tick_ms = now;
                cal_freq = 0.0f;
                Encoder_CalReset();
            }
            break;

        case CAL_RAMP:
        {
            /* 频率线性上升, 让转子跟得上(直接给 2Hz 会有一下顿挫) */
            float dt_s = APP_CURRENT_PID_DT;
            cal_freq += (APP_CAL_FREQ_HZ / APP_CAL_RAMP_S) * dt_s;
            if (cal_freq >= APP_CAL_FREQ_HZ)
            {
                cal_freq = APP_CAL_FREQ_HZ;
                cal_state = CAL_RUN;
                cal_cycles = 0.0f;
                Encoder_CalReset();        /* 升频段不采, 从匀速段重新开始 */
            }
            cal_phi += APP_TWO_PI * cal_freq * dt_s;
            break;
        }

        case CAL_RUN:
        {
            float dt_s = APP_CURRENT_PID_DT;
            float dphi = APP_TWO_PI * APP_CAL_FREQ_HZ * dt_s;

            cal_phi += dphi;
            cal_cycles += dphi / APP_TWO_PI;

            if (Encoder_IsValid() == 0u)
            {
                cal_fail_code = 3u;
                cal_state = CAL_ABORT;
                App_CalOutputZero();
                return 0u;
            }
            /* ★基准 φ 就是这个函数积分出来的角度, 不依赖时钟/转速恒定 */
            Encoder_CalSample(Encoder_GetRawElecAngle(), cal_phi);

            if (cal_cycles >= (float)APP_CAL_CYCLES)
            {
                /* ★在中断里就把速度估计复位: 下一拍电流环立刻恢复, 不能等主循环 */
                App_CalReleaseEstimator();
                cal_state = CAL_READY;
                App_CalOutputZero();
                return 0u;
            }
            break;
        }

        default:
            break;
    }

    while (cal_phi >= APP_TWO_PI) cal_phi -= APP_TWO_PI;
    while (cal_phi <  0.0f)      cal_phi += APP_TWO_PI;

    va = cal_voltage * cosf(cal_phi);
    vb = cal_voltage * sinf(cal_phi);
    SVPWM(va, vb, vdc, &ccr1, &ccr2, &ccr3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
    return 1u;
}

static void App_CalStart(void)
{
    if (Encoder_IsValid() == 0u)
    {
        (void)CDC_Printf("# cal: 编码器无效, 先查 SIN/COS(ref enc)\r\n");
        return;
    }
    Encoder_CalReset();
    cal_stop_req   = 0u;
    cal_fail_code  = 0u;
    cal_cycles     = 0.0f;
    cal_freq       = 0.0f;
    cal_phi        = 0.0f;
    cal_tick_ms    = HAL_GetTick();
    cal_progress_ms = 0u;
    cal_state      = CAL_ALIGN;
    (void)CDC_Printf("# cal START: 对齐 %ums -> 升频 %.1fs -> 拖转 %.0fs "
                     "(V=%.2f f=%.1fHz %u 个电周期, 轴上别带负载)\r\n",
                     (unsigned)APP_CAL_ALIGN_MS, (double)APP_CAL_RAMP_S,
                     (double)((float)APP_CAL_CYCLES / APP_CAL_FREQ_HZ),
                     (double)cal_voltage, (double)APP_CAL_FREQ_HZ,
                     (unsigned)APP_CAL_CYCLES);
}

/* READY -> 生成表(在主循环里做, 2000 格的统计不占中断时间) */
static void App_CalCollect(void)
{
    if (cal_state != CAL_READY) return;

    cal_result_code = Encoder_CalFinish(&cal_result_mean, &cal_result_max,
                                        &cal_result_resid, &cal_result_bins);
    cal_state = CAL_OFF;
    cal_freq = 0.0f;

    (void)CDC_Printf("# cal DONE: %s | N=%u(原始粗角度扫过 %.1f 圈) | 覆盖格 %u/%u | "
                     "原始均|err|=%.4frad(%.2fdeg) max=%.4frad | ★查表后残差=%.4frad(%.2fdeg)\r\n",
                     (cal_result_code == 0u) ? "OK"
                         : ((cal_result_code == 1u) ? "★失败: 样本不足"
                            : ((cal_result_code == 2u) ? "★失败: 覆盖不足(转子没转够/卡住)"
                               : "★失败: 平均误差过大(大概率打滑)")),
                     (unsigned)Encoder_CalCount(), (double)Encoder_CalSweepRev(),
                     (unsigned)cal_result_bins, (unsigned)ENCODER_LUT_SIZE,
                     (double)cal_result_mean,
                     (double)(cal_result_mean * 57.29578f),
                     (double)cal_result_max,
                     (double)cal_result_resid,
                     (double)(cal_result_resid * 57.29578f));
    if (cal_result_code == 0u)
    {
        float dc;
        float pp;

        Encoder_LutStats(&dc, &pp);
        (void)CDC_Printf("# 表的构成: 直流(对齐偏差)=%+.2fdeg  峰峰(非线性)=%.2fdeg "
                         "→ %s\r\n",
                         (double)(dc * 57.29578f), (double)(pp * 57.29578f),
                         (fabsf(dc) > 0.5f * pp)
                             ? "以对齐偏差为主(之前 ENCODER_ELEC_OFFSET 的锅)"
                             : "以非线性为主(传感器 sin/cos 不正交/幅值不等)");
        (void)CDC_Printf("# 判据: 再发一次 cal, 看\"查表后残差\"是不是明显小于\"原始均|err|\" —— "
                         "小了就说明这张表修对了\r\n");
        (void)CDC_Printf("# 导出: lut -> tools/lut_capture.py 存盘并生成 Encoder_Lut.c\r\n");
    }
    else
    {
        (void)CDC_Printf("# 可能原因: 转子没被拖住(滑了)/转了但编码器丢点。"
                         "试试 calv 加大电压 或降低 APP_CAL_FREQ_HZ\r\n");
    }
}

static void App_CalReport(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t st = (uint8_t)cal_state;

    if (st == CAL_OFF) return;

    if (st == CAL_ABORT)
    {
        App_CalStop();
        (void)CDC_Printf("# cal 中止 (code=%u: 1=欠压 2=超时 3=编码器无效 0=手动)\r\n",
                         (unsigned)cal_fail_code);
        return;
    }

    if ((now - cal_progress_ms) < 500u) return;
    cal_progress_ms = now;
    (void)CDC_Printf("# cal %s: f=%.2fHz cycles=%.1f/%u 样本=%u\r\n",
                     (st == CAL_ALIGN) ? "对齐中" : ((st == CAL_RAMP) ? "升频中" : "采集中"),
                     (double)cal_freq, (double)cal_cycles,
                     (unsigned)APP_CAL_CYCLES, (unsigned)Encoder_CalCount());
}

/* --------------------------------------------------------------------------
 * 最小 USB 命令口(只为标定服务, 不做打点流)
 *   cal          开始标定(电机会自己慢慢转 20 秒左右)
 *   cal stop     中止
 *   lut          导出 2000 点表(idx,rad)
 *   enc          编码器诊断一行
 *   stat         状态一行
 *   help         帮助
 * ------------------------------------------------------------------------ */
/* 自己解析一个十进制数(可选符号 + 整数 + 小数)。
 *   ★不能用 sscanf("%f"): newlib-nano 默认不链接 scanf 的浮点支持
 *     (printf 那边有 -u _printf_float, scanf 这边没有 _scanf_float),
 *     结果就是 %f 静默失败 —— `vofa 50` 会被当成"参数非法"直接拒掉。
 *     自己解析既避开这个坑, 也省掉几 KB 的 scanf 浮点库。 */
static uint8_t App_ParseFloat(const char *s, float *out)
{
    float v = 0.0f;
    float scale = 0.1f;
    uint8_t neg = 0u;
    uint8_t digits = 0u;

    if ((s == NULL) || (out == NULL)) return 0u;

    while (*s == ' ') s++;
    if ((*s == '+') || (*s == '-'))
    {
        neg = (uint8_t)((*s == '-') ? 1u : 0u);
        s++;
    }
    while ((*s >= '0') && (*s <= '9'))
    {
        v = v * 10.0f + (float)(*s - '0');
        s++;
        digits++;
    }
    if (*s == '.')
    {
        s++;
        while ((*s >= '0') && (*s <= '9'))
        {
            v += (float)(*s - '0') * scale;
            scale *= 0.1f;
            s++;
            digits++;
        }
    }
    if (digits == 0u) return 0u;

    *out = (neg != 0u) ? -v : v;
    return 1u;
}

static void App_UsbCommand(const char *line)
{
    char cmd[16] = {0};
    char sub[16] = {0};
    int n = sscanf(line, "%15s %15s", cmd, sub);   /* 只取两个词, 数值自己解析 */

    if (n <= 0) return;

    if (strcmp(cmd, "cal") == 0)
    {
        if (strcmp(sub, "stop") == 0)
        {
            cal_stop_req = 1u;
            (void)CDC_Printf("# cal stop 请求已收到\r\n");
        }
        else
        {
            App_CalStart();
        }
    }
    else if (strcmp(cmd, "lut") == 0)
    {
        uint16_t i;
        uint8_t ok = 1u;

        (void)CDC_Printf("# lut %u 项, idx,rad (valid=%u)\r\n",
                         (unsigned)ENCODER_LUT_SIZE, (unsigned)Encoder_LutIsValid());
        for (i = 0u; (i < ENCODER_LUT_SIZE) && (ok != 0u); i++)
        {
            /* 2000 行必须用会等的版本, 否则大半会被 USB 忙丢掉。
             * 超时就立刻放弃(而不是每行白等) —— 上位机没在收的时候不能把主循环拖死 */
            ok = (CDC_PrintfBlocking(200u, "%u,%.8f\r\n", (unsigned)i,
                                     (double)Encoder_LutGet(i)) >= 0) ? 1u : 0u;
        }
        (void)CDC_Printf("# lut end%s\r\n", (ok != 0u) ? "" : " ★中断(上位机没在收?)");
    }
    else if (strcmp(cmd, "enc") == 0)
    {
        (void)CDC_Printf("# enc valid=%u raw=%+.4f elec=%+.4f sin[%d..%d] cos[%d..%d] "
                         "lut=%u\r\n",
                         (unsigned)Encoder_IsValid(),
                         (double)Encoder_GetRawElecAngle(),
                         (double)Encoder_Elec_Angle,
                         Encoder_GetSinMin(), Encoder_GetSinMax(),
                         Encoder_GetCosMin(), Encoder_GetCosMax(),
                         (unsigned)Encoder_LutIsValid());
    }
    else if (strcmp(cmd, "calv") == 0)
    {
        /* 用 sscanf 而不是 atof: atof 会把整个 strtod 解析器(约 7KB flash)拖进来,
         *   而 sscanf 这条路径解析命令时本来就要用 */
        float v = 0.0f;
        int got = (int)App_ParseFloat(sub, &v);

        if ((got == 1) && (v >= 0.20f) && (v <= 2.00f))
        {
            cal_voltage = v;
            (void)CDC_Printf("# calv = %.2fV (电流约 %.1fA, 越大磁场越硬、齿槽滞后越小)\r\n",
                             (double)cal_voltage, (double)(cal_voltage / 0.13f));
        }
        else
        {
            (void)CDC_Printf("# 用法 calv 0.20~2.00, 现在 %.2fV\r\n", (double)cal_voltage);
        }
    }
    else if (strcmp(cmd, "ang") == 0)
    {
        float hz = 0.0f;
        int got = (int)App_ParseFloat(sub, &hz);

        if ((got == 1) && (hz <= 0.0f))
        {
            ang_hz = 0u;
        }
        else if ((got == 1) && (hz >= 1.0f) && (hz <= 500.0f))
        {
            ang_hz = (uint16_t)hz;
        }
        else if (sub[0] == '\0')
        {
            ang_hz = (ang_hz != 0u) ? 0u : 100u;   /* 不带参数 = 开关 */
        }
        else
        {
            (void)CDC_Printf("# 用法 ang [1~500], 现在 %u Hz\r\n", (unsigned)ang_hz);
            return;
        }
        (void)CDC_Printf("# ang = %u Hz (0=关) raw=粗角度 corr=查表后 lut=表补的量\r\n",
                         (unsigned)ang_hz);
    }
    else if (strcmp(cmd, "vofa") == 0)
    {
        float hz = 0.0f;
        int got = (int)App_ParseFloat(sub, &hz);

        if ((got == 1) && (hz <= 0.0f))
        {
            vofa_div = 0u;
        }
        else if ((got == 1) && (hz >= 50.0f) && (hz <= 1000.0f))
        {
            vofa_hz = (uint16_t)hz;
            vofa_div = (uint16_t)(10000.0f / hz + 0.5f);
        }
        else if (sub[0] == '\0')
        {
            if (vofa_div != 0u) { vofa_div = 0u; }
            else { vofa_hz = 500u; vofa_div = 20u; }
        }
        else
        {
            (void)CDC_Printf("# 用法 vofa [50~1000], 现在 %u Hz\r\n", (unsigned)vofa_hz);
            return;
        }

        if (vofa_div != 0u)
        {
            ang_hz = 0u;                  /* 两种打点别混在一起 */
            vofa_head = 0u; vofa_tail = 0u; vofa_tick = 0u; vofa_drop = 0u;
            (void)CDC_Printf("# vofa %u Hz (实际 %u Hz, 每 %u 拍一帧)\r\n"
                             "# 通道: corr,raw,lut,mech,spd,out,iq,vdc,obs,err\r\n"
                             "# 单位: rad,rad,rad,rad,rpm,rpm,A,V,rad,rad\r\n"
                             "# obs=滑模观测角(只观测不驱动) err=obs−corr\r\n",
                             (unsigned)vofa_hz,
                             (unsigned)(10000u / (uint32_t)vofa_div),
                             (unsigned)vofa_div);
        }
        else
        {
            (void)CDC_Printf("# vofa 关 (累计丢帧 %u)\r\n", (unsigned)vofa_drop);
        }
    }
    else if (strcmp(cmd, "stat") == 0)
    {
        (void)CDC_Printf("# stat vdc=%.2f I=%.2f iq=%.2f | spd=%.0f rpm(电机) = %.1f rpm(输出轴) "
                         "fe=%.1fHz | th=%.4f mode=%u cal=%u lut=%u calv=%.2f ang=%u | "
                         "obs=%+.4f err=%+.3frad emf=%.3f off=%+.1f v=%u\r\n",
                         (double)Power, (double)cur_mag_filt, (double)iq_filt,
                         (double)spd_meas_rpm, (double)(spd_meas_rpm / APP_GEAR_RATIO),
                         (double)(spd_meas_rpm * APP_ELEC_PER_REV / 60.0f),
                         (double)Encoder_Elec_Angle,
                         (unsigned)Control_GetMode(), (unsigned)cal_state,
                         (unsigned)Encoder_LutIsValid(), (double)cal_voltage,
                         (unsigned)ang_hz,
                         (double)smo_theta, (double)smo_err,
                         (double)SMO.Output.Emf,
                         (double)(smo_off * 57.29578f), (unsigned)smo_valid);
    }
    else if (strcmp(cmd, "hfirot") == 0)
    {
        /* 手改解调轴旋转(deg); 一般由 hfiscan 自动设成 −atan2(C,K) */
        float v = 0.0f;

        if (App_ParseFloat(sub, &v) == 1u)
        {
            HFI_SetDemodRot(v * 0.0174532925f);
            (void)CDC_Printf("# hfirot = %+.1fdeg\r\n", (double)(hfi_demod_rot * 57.29578f));
        }
        else
        {
            (void)CDC_Printf("# 用法 hfirot <deg>, 现在 %+.1fdeg\r\n",
                             (double)(hfi_demod_rot * 57.29578f));
        }
    }
    else if (strcmp(cmd, "hfiscan") == 0)
    {
        /* 免锁相静态扫描: θ̂ 钉在 θenc+Δ 扫 12 档, 直接量 err(Δ) —— 不可能跑飞。
         *   PLL 一旦跑飞就分不清"没凸极"和"解调不对称/相位错", 这条命令就是用来定性。 */
        if (HFI_SpeedOk() == 0u)
        {
            (void)CDC_Printf("# 现在转速 %.0f 电机rpm, 超出 HFI 适用范围(≤%.0frad/s ≈86rpm):"
                             " 扫描必须静止, 先停住电机\r\n",
                             (double)(hfi_w_enc * 60.0f / (6.2831853f * 7.0f)),
                             (double)HFI_MAX_W);
        }
        else
        {
            HFI_ScanStart();
            (void)CDC_Printf("# hfiscan 开始: 12 档 × 200ms = 2.4s(θ̂=θenc+Δ, 0..165°, 环路不闭),"
                             " 扫完自动关注入并打印表格\r\n");
        }
    }
    else if (strcmp(cmd, "hfic2") == 0)
    {
        /* 手改 cos2Δ 不对称补偿量(一般由 hfiscan 自动填, 这里用于微调/回零) */
        float v = 0.0f;

        if (App_ParseFloat(sub, &v) == 1u)
        {
            HFI_SetB2(v);
            (void)CDC_Printf("# hfic2 = %+.4fA\r\n", (double)hfi_b2_comp);
        }
        else
        {
            (void)CDC_Printf("# 用法 hfic2 <A>(-0.5~0.5), 现在 %+.4fA\r\n",
                             (double)hfi_b2_comp);
        }
    }
    else if (strcmp(cmd, "hfi") == 0)
    {
        /* HFI 高频注入: 只观测不驱动(注入叠在 PLL 估计的 d 轴上)。
         *   ★它的用武之地是**静止/极低速**(补 SMO 在 478rpm 以下的空档): 转子一转,
         *     解调就多出与转速成正比的偏差(实测 10Hz 电≈86rpm 就有 −12°), 所以
         *     使能前先问一句转速 —— 超范围直接拒绝并说清楚, 而不是刚开就 flt=3。 */
        float v = 0.0f;
        int got = (int)App_ParseFloat(sub, &v);
        uint8_t want = (sub[0] == '\0') ? ((hfi_enable != 0u) ? 0u : 1u)
                                        : ((got == 1 && v != 0.0f) ? 1u : 0u);

        if ((want != 0u) && (HFI_SpeedOk() == 0u))
        {
            (void)CDC_Printf("# 现在转速 %.0f 电机rpm(电 %.0frad/s), 超出 HFI 适用范围"
                             "(≤%.0frad/s ≈86rpm): 请先把电机停住再 hfi 1\r\n",
                             (double)(hfi_w_enc * 60.0f / (6.2831853f * 7.0f)),
                             (double)hfi_w_enc, (double)HFI_MAX_W);
        }
        else
        {
            HFI_SetEnable(want);
            (void)CDC_Printf("# hfi en=%u U=%.2f/%.2fV kth=%.4fA | 头 32ms 是标定窗(cal=1),"
                             " 之后看 id/snr: snr<3 就是这点凸极不够用\r\n",
                             (unsigned)hfi_enable, (double)hfi_v_lim, (double)hfi_v_inj,
                             (double)hfi_kth);
        }
    }
    else if (strcmp(cmd, "hfiu") == 0)
    {
        float v = 0.0f;
        int got = (int)App_ParseFloat(sub, &v);

        if ((got == 1) && (v >= HFI_INJ_V_MIN) && (v <= HFI_INJ_V_MAX))
        {
            HFI_SetAmpl(v);
            (void)CDC_Printf("# hfiu = %.2fV (高频电流约 %.2fA)\r\n",
                             (double)hfi_v_inj, (double)(1.9f * hfi_v_inj));
        }
        else
        {
            (void)CDC_Printf("# 用法 hfiu %.2f~%.2f, 现在 %.2fV\r\n",
                             (double)HFI_INJ_V_MIN, (double)HFI_INJ_V_MAX,
                             (double)hfi_v_inj);
        }
    }
    else if (strcmp(cmd, "hfik") == 0)
    {
        /* 手动指定解调增益 Kth[A](err = Kth·sin2Δθ)。默认值只是"还没自标定完"时的兜底;
         *   固件一旦量出 kth_meas 就会自动用它, 所以这条命令一般只在调试时用。 */
        float v = 0.0f;
        int got = (int)App_ParseFloat(sub, &v);

        if ((got == 1) && (v >= HFI_KTH_MIN) && (v <= HFI_KTH_MAX))
        {
            HFI_SetKth(v);
            (void)CDC_Printf("# hfik = %.4fA (实测 %.4fA)\r\n",
                             (double)hfi_kth, (double)hfi_kth_meas);
        }
        else
        {
            (void)CDC_Printf("# 用法 hfik %.3f~%.3f, 现在 %.4fA\r\n",
                             (double)HFI_KTH_MIN, (double)HFI_KTH_MAX,
                             (double)hfi_kth);
        }
    }
    else if (strcmp(cmd, "hfikd") == 0)
    {
        /* 环路阻尼系数[A·s/rad]: 环路误差里减掉 kd·ω, 抵消"与注入轴转速同向的假误差"
         *   (现场实测的负阻尼 ⇒ 积分器在 ±200 之间自持振荡、注入轴在 ±120 之间扫)。
         *   现场用法: hfi 1 之后若 w 还在 ±限幅 来回跳/err 一直在扫, 就把它加大
         *   (0.0015 → 0.003 → 0.006), 直到 w 稳定在 0 附近; 加过头会变迟钝/不动。 */
        float v = 0.0f;
        int got = (int)App_ParseFloat(sub, &v);

        if (got == 1)
        {
            HFI_SetKd(v);
            (void)CDC_Printf("# hfikd = %.5f A·s/rad (默认 %.5f)\r\n",
                             (double)hfi_kd, (double)HFI_PLL_KD);
        }
        else
        {
            (void)CDC_Printf("# 用法 hfikd <A·s/rad> (0~0.05), 现在 %.5f; 默认 %.5f\r\n",
                             (double)hfi_kd, (double)HFI_PLL_KD);
        }
    }
    else if (strcmp(cmd, "hfin") == 0)
    {
        /* 2·ωe 自适应陷波开关。★默认关: 离线自检里静止时它落在 PLL 带宽内会振
         *   (开: 误差 RMS 19°; 关: 0.3°), 只有转起来(2fe 远高于环路带宽)才有意义 */
        float v = 0.0f;
        int got = (int)App_ParseFloat(sub, &v);

        if (sub[0] == '\0')
        {
            HFI_SetNotch((hfi_notch_en != 0u) ? 0u : 1u);
        }
        else if (got == 1)
        {
            HFI_SetNotch((v != 0.0f) ? 1u : 0u);
        }
        else
        {
            (void)CDC_Printf("# 用法 hfin [0|1], 现在 %u\r\n", (unsigned)hfi_notch_en);
        }
        (void)CDC_Printf("# hfi 陷波 = %u (默认关: 静止时它落在 PLL 带宽内会振, "
                         "见 docs/HFI诊断.md)\r\n", (unsigned)hfi_notch_en);
    }
    else if (strcmp(cmd, "help") == 0)
    {
        (void)CDC_Printf("# cal [stop] | calv <V> | vofa [Hz] | ang [Hz] | lut | enc | "
                         "stat | hfi [0|1] | hfiu <V> | hfik <A> | hfikd <A·s/rad> | hfin | hfiscan | hfic2 <A> | "
                         "hfirot <deg> | help\r\n"
                         "#   ★hfi 1 若立刻 flt=4: 固件会**自动**转入 hfiscan(2.4s, 环路不闭)"
                         "把响应相位量出来、把解调轴转正、用 |E| 定环路增益, 然后自动重标接管;"
                         " 详见 docs/HFI诊断.md\r\n");
    }
    else
    {
        (void)CDC_Printf("# 未知命令 '%s', 试 help\r\n", cmd);
    }
}

static void App_UsbPoll(void)
{
    static char line[64];
    static uint8_t len;
    static uint32_t last_ms;
    uint8_t chunk[32];
    uint32_t got = CDC_RxTake(chunk, (uint16_t)sizeof(chunk));

    if (got > 0u) last_ms = HAL_GetTick();

    for (uint32_t i = 0u; i < got; i++)
    {
        char c = (char)chunk[i];

        if (c == '\n' || c == '\r')
        {
            if (len > 0u)
            {
                line[len] = '\0';
                App_UsbCommand(line);
                len = 0u;
            }
        }
        else if (len < (uint8_t)(sizeof(line) - 1u))
        {
            line[len++] = c;
        }
        else
        {
            len = 0u;
        }
    }

    /* ★静默 300ms 就当成一行结束: 有些上位机/脚本发命令不带结尾符(或者只发 '\r'),
     *   那样命令会永远憋在缓冲里 —— 现象和"命令根本没收到"一模一样。 */
    if ((len > 0u) && ((HAL_GetTick() - last_ms) >= 300u))
    {
        line[len] = '\0';
        (void)CDC_Printf("# (命令没有结尾符, 按静默超时执行)\r\n");
        App_UsbCommand(line);
        len = 0u;
    }
}

void App_FOC_Init(void)
{
    Control_Init();
    /* 观测器: 只观测不驱动。Ts 必须与电流环周期一致, 且要在 SMO_Init 之前设 */
    SMO_SetTs(APP_CURRENT_PID_DT);
    SMO_Init();
    /* HFI 观测器: 同样是"只观测不驱动"(默认关, 要 hfi 1 才注入) */
    HFI_Init();
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
    if (BSP_CAN_Init() != HAL_OK)
    {
        Error_Handler();
    }
    MX_USB_DEVICE_Init();
    BSP_Timer_Init();
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
}

void App_FOC_MainLoop(void)
{
    static uint8_t banner_done;

    if (HAL_GetTick() - can_feedback_ms >= APP_CAN_FEEDBACK_PERIOD_MS)
    {
        can_feedback_ms = HAL_GetTick();
        App_SendCanFeedback();
    }

    App_UsbPoll();
    App_CalCollect();     /* 标定采完 -> 生成 2000 点表 */
    App_CalReport();      /* 标定进度/中止提示 */
    HFI_Report();
    HFI_ScanReport();          /* 静态扫描完成后打印 12 点表 + 定性结论(主循环里打, 不在中断) */         /* HFI 诊断(只在使能/报过故障时打, 20Hz) */

    /* 上电横幅延后 1.5s: USB 枚举完成之后再打招呼, 免得打在枚举前丢掉 */
    if (banner_done == 0u && HAL_GetTick() >= 1500u)
    {
        banner_done = 1u;
        (void)CDC_Printf("# ready. 命令: cal | cal stop | lut | enc | stat | help "
                         "(LUT %u 点, valid=%u)\r\n",
                         (unsigned)ENCODER_LUT_SIZE, (unsigned)Encoder_LutIsValid());
    }

    /* ★每 3 秒一行存活心跳 —— 专门用来切"到底是收不到还是发不出去":
     *     一行都没有        -> 发不出去(端口选错/USB线/固件没跑起来)
     *     有心跳, rx 不涨    -> 发得出去收不进来(上位机没真发/发到别的口去了)
     *     有心跳, rx 在涨    -> 收进来了, 那就是命令拼写/结尾符问题 */
    {
        static uint32_t alive_ms;
        if ((vofa_div == 0u) && (HAL_GetTick() - alive_ms >= 3000u))
        {
            alive_ms = HAL_GetTick();
            (void)CDC_Printf("# alive t=%u rx=%u vdc=%.2f I=%.2f spd=%.0f/%.1f out fe=%.0f "
                             "enc_vld=%u lut=%u cal=%u\r\n",
                             (unsigned)alive_ms, (unsigned)CDC_RxTotal(),
                             (double)Power, (double)cur_mag_filt,
                             (double)spd_meas_rpm,
                             (double)(spd_meas_rpm / APP_GEAR_RATIO),
                             (double)(spd_meas_rpm * APP_ELEC_PER_REV / 60.0f),
                             (unsigned)Encoder_IsValid(),
                             (unsigned)Encoder_LutIsValid(), (unsigned)cal_state);
        }
    }

    /* VOFA+ 打点: 主循环只负责把中断采好的帧打成一行文本(不阻塞, 忙就整帧丢) */
    if (vofa_div != 0u)
    {
        while (vofa_tail != vofa_head)
        {
            /* 先把整帧拷出来再推 tail: 避免打印期间中断把这一行覆盖掉
             * (缓冲满时中断是丢帧而不是覆盖 tail 行, 所以这里其实也安全, 但拷贝更省心) */
            float s[APP_VOFA_CH];
            uint8_t i;

            for (i = 0u; i < APP_VOFA_CH; i++)
            {
                s[i] = vofa_buf[vofa_tail][i];
            }
            vofa_tail = (uint8_t)((vofa_tail + 1u) % APP_VOFA_BUF);
            (void)CDC_Printf("%+.4f,%+.4f,%+.4f,%+.3f,%.0f,%.1f,%.2f,%.2f,"
                             "%+.4f,%+.4f,%+.4f,%+.4f\r\n",
                             (double)s[0], (double)s[1], (double)s[2], (double)s[3],
                             (double)s[4], (double)s[5], (double)s[6], (double)s[7],
                             (double)s[8], (double)s[9],
                             (double)s[10], (double)s[11]);
        }
    }

    /* 电角度打点(`ang <hz>` 开): 主循环里做, 不占中断时间 */
    if (ang_hz != 0u)
    {
        uint32_t now_ms = HAL_GetTick();

        if ((now_ms - ang_ms) >= (1000u / (uint32_t)ang_hz))
        {
            float raw = Encoder_GetRawElecAngle();

            ang_ms = now_ms;
            (void)CDC_Printf("# ang t=%u raw=%+.4f corr=%+.4f lut=%+.4f mech=%+.3f "
                             "spd=%.0f out=%.1f fe=%.1f | obs=%+.4f err=%+.4f "
                             "emf=%.3f off=%+.1f v=%u | hfi=%+.4f herr=%+.1f "
                             "hsnr=%.1f\r\n",
                             (unsigned)now_ms, (double)raw,
                             (double)Encoder_Elec_Angle,
                             (double)Encoder_LutGet(Encoder_LutIndex(raw)),
                             (double)Encoder_Mech_Position,
                             (double)spd_meas_rpm,
                             (double)(spd_meas_rpm / APP_GEAR_RATIO),
                             (double)(spd_meas_rpm * APP_ELEC_PER_REV / 60.0f),
                             (double)smo_theta, (double)smo_err,
                             (double)SMO.Output.Emf,
                             (double)(smo_off * 57.29578f), (unsigned)smo_valid,
                             (double)HFI_GetStreamTheta(),
                             (double)(HFI_GetStreamErr() * 57.29578f),
                             (double)hfi_snr);
        }
    }

    HAL_Delay(1u);
}
