#include "BSP_CAN.h"

static volatile uint8_t bsp_can_ready = 0u;

HAL_StatusTypeDef BSP_CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0u;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0u;
    filter.FilterIdLow = 0u;
    filter.FilterMaskIdHigh = 0u;
    filter.FilterMaskIdLow = 0u;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14u;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(&hcan1,
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_BUSOFF |
                                     CAN_IT_ERROR |
                                     CAN_IT_ERROR_PASSIVE |
                                     CAN_IT_ERROR_WARNING) != HAL_OK)
    {
        HAL_CAN_Stop(&hcan1);
        return HAL_ERROR;
    }

    bsp_can_ready = 1u;
    return HAL_OK;
}

static HAL_StatusTypeDef BSP_CAN_Send(uint32_t id,
                                      uint32_t ide,
                                      const uint8_t *data,
                                      uint8_t dlc)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint8_t payload[8] = {0};

    if (bsp_can_ready == 0u || data == 0 || dlc > 8u)
    {
        return HAL_ERROR;
    }

    header.StdId = (ide == CAN_ID_STD) ? id : 0u;
    header.ExtId = (ide == CAN_ID_EXT) ? id : 0u;
    header.IDE = ide;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;
    header.TransmitGlobalTime = DISABLE;

    for (uint8_t i = 0u; i < dlc; i++)
    {
        payload[i] = data[i];
    }

    return HAL_CAN_AddTxMessage(&hcan1, &header, payload, &mailbox);
}

HAL_StatusTypeDef BSP_CAN_SendStd(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    if (id > 0x7FFu)
    {
        return HAL_ERROR;
    }

    return BSP_CAN_Send(id, CAN_ID_STD, data, dlc);
}

HAL_StatusTypeDef BSP_CAN_SendExt(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (id > 0x1FFFFFFFu)
    {
        return HAL_ERROR;
    }

    return BSP_CAN_Send(id, CAN_ID_EXT, data, dlc);
}

uint8_t BSP_CAN_IsReady(void)
{
    return bsp_can_ready;
}

__weak void BSP_CAN_RxCallback(const CAN_RxHeaderTypeDef *header,
                               const uint8_t *data)
{
    (void)header;
    (void)data;
}

__weak void BSP_CAN_ErrorCallback(uint32_t error_code)
{
    (void)error_code;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (hcan != &hcan1)
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }

        BSP_CAN_RxCallback(&header, data);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        BSP_CAN_ErrorCallback(HAL_CAN_GetError(hcan));
    }
}
