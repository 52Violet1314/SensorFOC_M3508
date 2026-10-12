/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include <stdio.h>
#include <stdarg.h>
#include "main.h"      /* HAL_GetTick (CDC_PrintfBlocking 里等端点用) */
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* ---------------------------------------------------------------------------
 * 接收环形缓冲: USB 中断(CDC_Receive_FS)写, 主循环用 CDC_RxTake() 取。
 *   只服务于"标定命令"这一件事(cal / lut / stat), 所以做得尽可能小:
 *   没有优先级、没有多级队列 —— 命令量级是"人敲几个字符", 128 字节足够。
 * ------------------------------------------------------------------------- */
#define CDC_RX_RING_SIZE 128u
#define CDC_RX_RING_MASK (CDC_RX_RING_SIZE - 1u)

static volatile uint8_t  cdc_rx_ring[CDC_RX_RING_SIZE];
static volatile uint16_t cdc_rx_head;
static volatile uint16_t cdc_rx_tail;
static volatile uint32_t cdc_rx_total;   /* 累计收到的字节数 */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  for (uint32_t i = 0u; i < *Len; i++)
  {
    uint16_t next = (uint16_t)((cdc_rx_head + 1u) & CDC_RX_RING_MASK);
    if (next == cdc_rx_tail)
    {
      break;                     /* 满了就丢, 主循环没跟上(正常不会) */
    }
    cdc_rx_ring[cdc_rx_head] = Buf[i];
    cdc_rx_head = next;
  }
  cdc_rx_total += *Len;
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  /* ★没枚举完(没插 USB / 主机没打开串口)时 pClassData 是 NULL, 直接解引用会 HardFault
   *   —— 上电横幅就踩过这个: 板子独立上电(不接电脑)时一开机就死在打印上。 */
  if (hcdc == NULL || Len == 0u){
    return USBD_FAIL;
  }
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* 发送暂存必须是静态的: CDC_Transmit_FS 只是把指针交给 USB 栈,
 *   一包 64 字节发不完的尾巴由中断后续搬运 —— 用栈上的局部数组的话,
 *   函数返回后那块栈随时会被别的调用(中断/下一次 printf)覆盖, 后半行就是垃圾。 */
static char cdc_tx_buf[512];

static int Cdc_Format(char *dst, size_t n, const char *format, va_list args)
{
  int len = vsnprintf(dst, n, format, args);

  if (len < 0) return -1;
  if (len > (int)(n - 1u)) len = (int)(n - 1u);
  return len;
}

/**
  * @brief  CDC_Printf
  *         USB 虚拟串口格式化打印, 用法与 printf 一致
  *         ★端点忙时直接丢这一行(不阻塞)。只用于状态/诊断行, 不要用于打点流。
  * @retval 已格式化字符数, 发送失败(总线忙)返回 -1
  */
int CDC_Printf(const char *format, ...)
{
  va_list args;
  int len;

  if (format == NULL)
  {
    return -1;
  }

  va_start(args, format);
  len = Cdc_Format(cdc_tx_buf, sizeof(cdc_tx_buf), format, args);
  va_end(args);

  if (len > 0)
  {
    if (CDC_Transmit_FS((uint8_t *)cdc_tx_buf, (uint16_t)len) != USBD_OK)
    {
      return -1;
    }
  }

  return len;
}

/**
  * @brief  CDC_PrintfBlocking
  *         会等端点空出来的版本。只给"一次性大批量输出"用(例如导出 2000 点表),
  *         它会占住主循环; 打点/状态行绝对不要用。
  * @retval 已发送字符数, 超时返回 -1
  */
int CDC_PrintfBlocking(uint32_t timeout_ms, const char *format, ...)
{
  va_list args;
  int len;
  uint32_t t0 = HAL_GetTick();

  if (format == NULL)
  {
    return -1;
  }

  va_start(args, format);
  len = Cdc_Format(cdc_tx_buf, sizeof(cdc_tx_buf), format, args);
  va_end(args);
  if (len <= 0)
  {
    return len;
  }

  while (CDC_Transmit_FS((uint8_t *)cdc_tx_buf, (uint16_t)len) != USBD_OK)
  {
    if ((HAL_GetTick() - t0) > timeout_ms)
    {
      return -1;      /* 上位机没开串口 / 拔线了 */
    }
  }

  return len;
}

/**
  * @brief  CDC_RxTake
  *         从接收环形缓冲取字节(主循环调用), 返回实际取到的字节数
  */
uint32_t CDC_RxTake(uint8_t *dst, uint16_t max)
{
  uint32_t n = 0u;

  if (dst == NULL) return 0u;

  while ((n < (uint32_t)max) && (cdc_rx_tail != cdc_rx_head))
  {
    dst[n++] = cdc_rx_ring[cdc_rx_tail];
    cdc_rx_tail = (uint16_t)((cdc_rx_tail + 1u) & CDC_RX_RING_MASK);
  }

  return n;
}

/* 累计收到的字节数: 恒 0 = 上位机的命令根本没进 USB(端口/驱动/接收方向问题),
 *   和命令解析、编码器、标定逻辑都无关 —— 排查"发了命令没反应"先看这个数。 */
uint32_t CDC_RxTotal(void)
{
  return cdc_rx_total;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
