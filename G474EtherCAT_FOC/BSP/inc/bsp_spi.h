#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    SPI_HandleTypeDef *handle;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} bsp_spi_device_t;

void bsp_spi_select(const bsp_spi_device_t *device);
void bsp_spi_deselect(const bsp_spi_device_t *device);
HAL_StatusTypeDef bsp_spi_receive(const bsp_spi_device_t *device, uint8_t *data,
                                   uint16_t size, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
