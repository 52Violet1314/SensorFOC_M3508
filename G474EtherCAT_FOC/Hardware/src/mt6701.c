#include "mt6701.h"

#include "bsp_spi.h"
#include "spi.h"

#define MT6701_CRC_MASK              0x3FU
#define MT6701_MAGNET_STRENGTH_MASK  0x03U
#define MT6701_ANGLE_MASK            0x3FFFU

static const bsp_spi_device_t mt6701_spi_device = {
    .handle = &hspi1,
    .cs_port = MT6701_CS_GPIO_Port,
    .cs_pin = MT6701_CS_Pin,
};

mt6701_data_t mt6701_data;

static uint8_t mt6701_dma_frame[MT6701_READ_FRAME_SIZE];
static volatile uint8_t mt6701_dma_is_busy;
static volatile uint8_t mt6701_dma_result_ready;
static volatile mt6701_status_t mt6701_dma_status = MT6701_STATUS_SPI_ERROR;

uint8_t mt6701_calculate_crc(uint32_t data)
{
    uint8_t crc = 0U;

    for (int32_t bit = 17; bit >= 0; --bit)
    {
        uint8_t input_bit = (uint8_t)((data >> (uint32_t)bit) & 1U);

        crc <<= 1U;
        if (((crc >> 6U) ^ input_bit) != 0U)
        {
            crc ^= 0x43U;
        }
        crc &= MT6701_CRC_MASK;
    }

    return crc;
}

static mt6701_status_t mt6701_decode_frame(const uint8_t *frame,
                                            mt6701_data_t *data)
{
    uint32_t raw;
    uint8_t received_crc;

    raw = ((uint32_t)frame[0] << 16U) |
          ((uint32_t)frame[1] << 8U) | frame[2];
    received_crc = (uint8_t)(raw & MT6701_CRC_MASK);
    if (raw == 0U || mt6701_calculate_crc(raw >> 6U) != received_crc)
    {
        return MT6701_STATUS_CRC_ERROR;
    }

    data->angle_count = (uint16_t)((raw >> 10U) & MT6701_ANGLE_MASK);
    data->magnet_strength = (uint8_t)((raw >> 6U) & MT6701_MAGNET_STRENGTH_MASK);
    data->magnet_too_weak = ((raw >> 8U) & 1U) != 0U;
    data->overspeed = ((raw >> 9U) & 1U) != 0U;
    data->crc = received_crc;
    return MT6701_STATUS_OK;
}

mt6701_status_t mt6701_read(mt6701_data_t *data)
{
    uint8_t frame[MT6701_READ_FRAME_SIZE];
    uint8_t attempt;

    if (data == NULL)
    {
        return MT6701_STATUS_SPI_ERROR;
    }

    for (attempt = 0u; attempt < 3u; attempt++)
    {
        if (bsp_spi_receive(&mt6701_spi_device, frame, sizeof(frame),
                            MT6701_SPI_TIMEOUT_MS) != HAL_OK)
        {
            return MT6701_STATUS_SPI_ERROR;
        }

        if (mt6701_decode_frame(frame, data) == MT6701_STATUS_OK)
        {
            return MT6701_STATUS_OK;
        }
    }

    return MT6701_STATUS_CRC_ERROR;
}

HAL_StatusTypeDef mt6701_dma_start(void)
{
    HAL_StatusTypeDef status;

    if (mt6701_dma_is_busy != 0U)
    {
        return HAL_BUSY;
    }

    mt6701_dma_result_ready = 0U;
    mt6701_dma_is_busy = 1U;
    bsp_spi_select(&mt6701_spi_device);
    status = HAL_SPI_Receive_DMA(mt6701_spi_device.handle,
                                 mt6701_dma_frame,
                                 MT6701_READ_FRAME_SIZE);
    if (status != HAL_OK)
    {
        bsp_spi_deselect(&mt6701_spi_device);
        mt6701_dma_is_busy = 0U;
        mt6701_dma_status = MT6701_STATUS_SPI_ERROR;
        mt6701_dma_result_ready = 1U;
    }

    return status;
}

uint8_t mt6701_dma_fetch(mt6701_data_t *data, mt6701_status_t *status)
{
    if (mt6701_dma_result_ready == 0U)
    {
        return 0U;
    }

    if (data != NULL)
    {
        *data = mt6701_data;
    }
    if (status != NULL)
    {
        *status = mt6701_dma_status;
    }
    mt6701_dma_result_ready = 0U;
    return 1U;
}

uint8_t mt6701_dma_busy(void)
{
    return mt6701_dma_is_busy;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != mt6701_spi_device.handle)
    {
        return;
    }

    bsp_spi_deselect(&mt6701_spi_device);
    mt6701_dma_status = mt6701_decode_frame(mt6701_dma_frame, &mt6701_data);
    mt6701_dma_is_busy = 0U;
    mt6701_dma_result_ready = 1U;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != mt6701_spi_device.handle)
    {
        return;
    }

    bsp_spi_deselect(&mt6701_spi_device);
    mt6701_dma_status = MT6701_STATUS_SPI_ERROR;
    mt6701_dma_is_busy = 0U;
    mt6701_dma_result_ready = 1U;
}

float mt6701_count_to_degrees(uint16_t angle_count)
{
    return (float)angle_count * (360.0f / (float)MT6701_COUNTS_PER_TURN);
}
