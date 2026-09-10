#include "VF_App.h"

#include "FOC_Config.h"
#include "FOC_Param.h"
#include "Encoder_Map.h"
#include "VF.h"
#include "main.h"
#include "mt6701.h"
#include "tim.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>

#define VF_APP_TWO_PI 6.28318530718f

static volatile float vf_app_actual_elec_angle;
static volatile float vf_app_raw_elec_angle;
static volatile uint8_t vf_app_angle_valid;
static volatile mt6701_status_t vf_app_encoder_status = MT6701_STATUS_SPI_ERROR;
static volatile uint8_t vf_app_uart_tx_busy;
static uint32_t vf_app_uart_tx_started_ms;
static uint8_t vf_app_output_enabled;

static long VF_AppAngleErrorMrad(float measured, float reference)
{
    float error = measured - reference;

    error = fmodf(error, VF_APP_TWO_PI);
    if (error > 3.14159265359f)
    {
        error -= VF_APP_TWO_PI;
    }
    else if (error < -3.14159265359f)
    {
        error += VF_APP_TWO_PI;
    }
    return lroundf(error * 1000.0f);
}

static void VF_AppSetZeroVoltage(void)
{
    uint32_t midpoint = __HAL_TIM_GET_AUTORELOAD(&htim1) / 2u;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, midpoint);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, midpoint);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, midpoint);
}

static void VF_AppStartPwm(void)
{
    VF_AppSetZeroVoltage();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                          __HAL_TIM_GET_AUTORELOAD(&htim1) - 40u);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
}

void VF_AppSetOutputEnabled(uint8_t enable)
{
    enable = (enable != 0u) ? 1u : 0u;
    if (enable == vf_app_output_enabled)
    {
        return;
    }
    if (enable != 0u)
    {
        VF_AppStartPwm();
    }
    else
    {
        VF_SetTargetRPM(0.0f);
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
        (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
        (void)HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4);
        VF_AppSetZeroVoltage();
    }
    vf_app_output_enabled = enable;
}

static void VF_AppUpdateEncoder(void)
{
    float raw_electrical_angle;
    float electrical_angle;
    float encoder_offset;

    vf_app_encoder_status = mt6701_read(&mt6701_data);
    if (vf_app_encoder_status == MT6701_STATUS_OK)
    {
        /* Keep the raw diagnostic angle independent of the startup offset. */
        raw_electrical_angle = FOC_EncoderCountToElectricalAngleLinear(
                                   mt6701_data.angle_count);
        raw_electrical_angle = fmodf(raw_electrical_angle, VF_APP_TWO_PI);
        if (raw_electrical_angle < 0.0f)
        {
            raw_electrical_angle += VF_APP_TWO_PI;
        }

        electrical_angle = FOC_EncoderCountToElectricalAngle(
                               mt6701_data.angle_count);
        /* Sample the mapped angle before applying the alignment offset. */
        VF_CalibrationSample(electrical_angle);
        encoder_offset = VF_GetEncoderOffset();
        electrical_angle += encoder_offset;
        electrical_angle = fmodf(electrical_angle, VF_APP_TWO_PI);
        if (electrical_angle < 0.0f)
        {
            electrical_angle += VF_APP_TWO_PI;
        }

        vf_app_raw_elec_angle = raw_electrical_angle;
        vf_app_actual_elec_angle = electrical_angle;
        vf_app_angle_valid = 1u;
    }
}

static void VF_AppCdcPrint(void)
{
    static uint32_t last_print_ms;
    static char message[128];
    long transformed_mrad;
    long raw_mrad;
    long simulated_mrad;
    long transformed_error_mrad;
    int length;

    if (vf_app_uart_tx_busy != 0u)
    {
        /* Recover from a missed TC interrupt instead of blocking all future
         * diagnostics forever. */
        if ((HAL_GetTick() - vf_app_uart_tx_started_ms) > 100u)
        {
            (void)HAL_UART_AbortTransmit(&huart3);
            vf_app_uart_tx_busy = 0u;
        }
        else
        {
            return;
        }
    }

    if ((HAL_GetTick() - last_print_ms) < FOC_PRINT_PERIOD_MS)
    {
        return;
    }

    raw_mrad = lroundf(vf_app_raw_elec_angle * 1000.0f);
    transformed_mrad = lroundf(vf_app_actual_elec_angle * 1000.0f);
    simulated_mrad = lroundf(vf_sim_elec_angle * 1000.0f);
    transformed_error_mrad = VF_AppAngleErrorMrad(vf_app_actual_elec_angle,
                                                  vf_sim_elec_angle);
    length = snprintf(message, sizeof(message),
                      "%ld,%ld,%ld,%ld\r\n",
                      raw_mrad, transformed_mrad, simulated_mrad,
                      transformed_error_mrad);
    if (length > 0 && length < (int)sizeof(message) &&
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)message, (uint16_t)length) == HAL_OK)
    {
        vf_app_uart_tx_busy = 1u;
        vf_app_uart_tx_started_ms = HAL_GetTick();
        last_print_ms = HAL_GetTick();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        vf_app_uart_tx_busy = 0u;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        vf_app_uart_tx_busy = 0u;
    }
}

void VF_AppInit(void)
{
    VF_Init();
#if VF_ENABLE_STARTUP_CALIBRATION
    VF_CalibrationStart();
#endif
    VF_SetTargetRPM(FOC_VF_TARGET_RPM);
    vf_app_output_enabled = 0u;
    VF_AppSetZeroVoltage();
    (void)HAL_ADCEx_InjectedStart_IT(&hadc1);
    (void)HAL_ADCEx_InjectedStart_IT(&hadc2);
    (void)HAL_TIM_Base_Start_IT(&htim3);

#if FOC_VF_STANDALONE_OPEN_LOOP
    /* Start the open-loop drive immediately; the CiA402 stack is bypassed so
     * it cannot turn the output back off. */
    VF_AppSetOutputEnabled(1u);
#endif
}

void VF_AppTask(void)
{
    VF_AppCdcPrint();
}

void VF_AppAdcTick(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc2)
    {
        return;
    }

    FOC_ParamUpdateAdc2();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        /* TIM3 is configured for 10 kHz. Consume the previous DMA frame and
         * launch the next one from this deterministic interrupt. */
        VF_AppUpdateEncoder();
        /* Open-loop control is independent of encoder validity. */
        VF_Tick(foc_param.bus_voltage, vf_app_actual_elec_angle);
    }
}
