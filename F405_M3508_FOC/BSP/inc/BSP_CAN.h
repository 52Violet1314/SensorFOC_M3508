#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h"

HAL_StatusTypeDef BSP_CAN_Init(void);
HAL_StatusTypeDef BSP_CAN_SendStd(uint16_t id, const uint8_t *data, uint8_t dlc);
HAL_StatusTypeDef BSP_CAN_SendExt(uint32_t id, const uint8_t *data, uint8_t dlc);
uint8_t BSP_CAN_IsReady(void);

void BSP_CAN_RxCallback(const CAN_RxHeaderTypeDef *header, const uint8_t *data);
void BSP_CAN_ErrorCallback(uint32_t error_code);

#endif
