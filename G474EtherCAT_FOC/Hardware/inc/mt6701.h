#ifndef MT6701_H
#define MT6701_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MT6701_RESOLUTION_BITS     14U
#define MT6701_COUNTS_PER_TURN     (1U << MT6701_RESOLUTION_BITS)
#define MT6701_READ_FRAME_SIZE     3U
#define MT6701_SPI_TIMEOUT_MS      2U

typedef enum
{
    MT6701_STATUS_OK = 0,
    MT6701_STATUS_SPI_ERROR,
    MT6701_STATUS_CRC_ERROR
} mt6701_status_t;

typedef struct
{
    uint16_t angle_count;
    uint8_t magnet_strength;
    bool magnet_too_weak;
    bool overspeed;
    uint8_t crc;
} mt6701_data_t;

extern mt6701_data_t mt6701_data;

mt6701_status_t mt6701_read(mt6701_data_t *data);
HAL_StatusTypeDef mt6701_dma_start(void);
uint8_t mt6701_dma_fetch(mt6701_data_t *data, mt6701_status_t *status);
uint8_t mt6701_dma_busy(void);
uint8_t mt6701_calculate_crc(uint32_t data);
float mt6701_count_to_degrees(uint16_t angle_count);

#ifdef __cplusplus
}
#endif

#endif
