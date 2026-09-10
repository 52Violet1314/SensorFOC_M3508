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
    if (HAL_GetTick() - can_feedback_ms >= APP_CAN_FEEDBACK_PERIOD_MS)
    {
        can_feedback_ms = HAL_GetTick();
        App_SendCanFeedback();
    }
    HAL_Delay(1u);
}
