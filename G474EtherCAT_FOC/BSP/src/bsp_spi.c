#include "bsp_spi.h"

void bsp_spi_select(const bsp_spi_device_t *device)
{
    device->cs_port->BSRR = (uint32_t)device->cs_pin << 16U;
}

void bsp_spi_deselect(const bsp_spi_device_t *device)
{
    device->cs_port->BSRR = device->cs_pin;
}

HAL_StatusTypeDef bsp_spi_receive(const bsp_spi_device_t *device, uint8_t *data,
                                   uint16_t size, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if ((device == NULL) || (device->handle == NULL) || (device->cs_port == NULL) ||
        (data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }

    bsp_spi_select(device);
    status = HAL_SPI_Receive(device->handle, data, size, timeout_ms);
    bsp_spi_deselect(device);

    return status;
}
