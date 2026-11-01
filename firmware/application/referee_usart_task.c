/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

/**
 * Modified the "RM referee system data solve" into CV_data receiving and processing
 * Commented out the original referee_usart_task function and added the UART7_CommandRoute function
 */

#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "INS_task.h"

#include "bsp_usart.h"
#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"

#include <stdbool.h> // Include this header to use bool type

/**
 * @brief          single byte upacked
 * @param[in]      void
 * @retval         none
 */
/**
 * @brief          单字节解包
 * @param[in]      void
 * @retval         none
 */
static void referee_unpack_fifo_data(void);

extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
char dma_buf[50];

// DMA data buffer for reception
unsigned char rx_data[50];

/* Below is the format of a complete received packet */
/* Header (a55a): 2 bytes */
/* Packet length: 1 byte */
/* ---Below is the payload, subject to change as per requirements--- */
/* hex_Yaw: 4 bytes (to be converted to IEEE 754 standard single precision floating point) */
/* hex_Pitch: 4 bytes (to be converted to IEEE 754 standard single precision floating point) */
/* ---Above is the payload--- */
/* Checksum: 1 byte */
/* Detect flag: 1 byte */
/* Footer (ff): 1 byte */
/* (full byte layout is documented above find_CpltPacket) */

// define a complete packet
uint8_t packet[CV_PACKET_LENGTH];

/**
 * @brief          referee task
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
/**
 * @brief          CV 通讯任务（原为裁判系统任务）
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void referee_usart_task(void const *argument)
{

  memset(dma_buf, 0, sizeof(dma_buf)); // was 200: dma_buf is only 50 bytes
  memset(rx_data, 0, sizeof(rx_data));
  const fp32 *imu = get_INS_angle_point();
  while (1)
  {
    sprintf((char *)dma_buf, "A5%f,%f,%f", imu[0], imu[1], imu[2]);
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)dma_buf, strlen((const char *)dma_buf));
    HAL_UART_Receive_DMA(&huart6, (uint8_t *)rx_data, sizeof(rx_data));
    osDelay(1); // yield: without this the loop starved the OLED and idle tasks
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    // HAL_UART_Receive_DMA(&huart6, (uint8_t*)rx_rdata, 9);
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)dma_buf, strlen((const char *)dma_buf));
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    UART7_CommandRoute();
    HAL_UART_Receive_DMA(&huart6, (uint8_t *)rx_data, sizeof(rx_data));
  }
}

/*
 * Packet sent by the CV computer (standard-cv / Robox2025 UART_UTIL.send_data), 14 bytes:
 *   [0]  0xA5          header
 *   [1]  0x5A          header
 *   [2]  0x0D          length byte (CV side writes 0x0D even though the packet is 14 bytes, so it is not checked)
 *   [3..6]  yaw        float32, big-endian, radians
 *   [7..10] pitch      float32, big-endian, radians
 *   [11] checksum      (sum of bytes [3..10]) & 0xFF
 *   [12] detect flag   0x01 = target found, 0x00 = no target
 *   [13] 0xFF          footer
 */
#define CV_PKT_CHECKSUM_INDEX 11

static bool cv_packet_valid(const uint8_t *p)
{
  if (p[0] != 0xA5 || p[1] != 0x5A || p[CV_PACKET_LENGTH - 1] != 0xFF)
  {
    return false;
  }

  uint8_t sum = 0;
  for (size_t k = 3; k < CV_PKT_CHECKSUM_INDEX; k++)
  {
    sum += p[k];
  }
  return sum == p[CV_PKT_CHECKSUM_INDEX];
}

// Find the NEWEST valid packet in rx_data and copy it into packet[].
// The DMA fills rx_data front to back, so the newest data is at the end:
// search backwards so the gimbal gets the latest angles instead of the oldest.
// The loop bound keeps the whole packet inside rx_data (the old loop read past the end).
bool find_CpltPacket(void)
{
  for (size_t i = sizeof(rx_data) - sizeof(packet) + 1; i-- > 0;)
  {
    if (cv_packet_valid(&rx_data[i]))
    {
      memcpy(packet, &rx_data[i], sizeof(packet));
      return true;
    }
  }
  return false;
}

cv_Data_TypeDef cv_Data;
void UART7_CommandRoute(void)
{
  // get a complete packet by checking the rx_data
  bool packetFound = find_CpltPacket();

  // If no complete packet was found, return early and skip the rest
  if (!packetFound)
  {
    return;
  }

  HexToFloat yaw;
  HexToFloat pitch;

  // get Yaw
  uint32_t temp_Yaw = (uint32_t)packet[3] << 24 |
                      (uint32_t)packet[4] << 16 |
                      (uint32_t)packet[5] << 8 |
                      (uint32_t)packet[6];
  yaw.hex = temp_Yaw;
  cv_Data.yaw = yaw.floatValue;

  // get Pitch
  uint32_t temp_Pitch = (uint32_t)packet[7] << 24 |
                        (uint32_t)packet[8] << 16 |
                        (uint32_t)packet[9] << 8 |
                        (uint32_t)packet[10];
  pitch.hex = temp_Pitch;
  cv_Data.pitch = pitch.floatValue;

  // get Find_Target status
  cv_Data.find_target = (char)packet[12];

  // get the new cv data flag
  detect_hook(USER_USART_DATA_TOE);
  cv_Data.new_cv_data_flag = 1;
}

/**
 * @brief          single byte upacked
 * @param[in]      void
 * @retval         none
 */
/**
 * @brief          单字节解包
 * @param[in]      void
 * @retval         none
 */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while (fifo_s_used(&referee_fifo))
  {
    byte = fifo_s_get(&referee_fifo);
    switch (p_obj->unpack_step)
    {
    case STEP_HEADER_SOF:
    {
      if (byte == sof)
      {
        p_obj->unpack_step = STEP_LENGTH_LOW;
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      else
      {
        p_obj->index = 0;
      }
    }
    break;

    case STEP_LENGTH_LOW:
    {
      p_obj->data_len = byte;
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_LENGTH_HIGH;
    }
    break;

    case STEP_LENGTH_HIGH:
    {
      p_obj->data_len |= (byte << 8);
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
      {
        p_obj->unpack_step = STEP_FRAME_SEQ;
      }
      else
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }
    }
    break;
    case STEP_FRAME_SEQ:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_HEADER_CRC8;
    }
    break;

    case STEP_HEADER_CRC8:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
      {
        if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
        {
          p_obj->unpack_step = STEP_DATA_CRC16;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }
    }
    break;

    case STEP_DATA_CRC16:
    {
      if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;

        if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          referee_data_solve(p_obj->protocol_packet);
        }
      }
    }
    break;

    default:
    {
      p_obj->unpack_step = STEP_HEADER_SOF;
      p_obj->index = 0;
    }
    break;
    }
  }
}

void USART6_IRQHandler(void)
{
  static volatile uint8_t res;
  if (USART6->SR & UART_FLAG_IDLE)
  {
    __HAL_UART_CLEAR_PEFLAG(&huart6);

    static uint16_t this_time_rx_len = 0;

    if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
    {
      __HAL_DMA_DISABLE(huart6.hdmarx);
      this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
      huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
      __HAL_DMA_ENABLE(huart6.hdmarx);
      fifo_s_puts(&referee_fifo, (char *)usart6_buf[0], this_time_rx_len);
      detect_hook(REFEREE_TOE);
    }
    else
    {
      __HAL_DMA_DISABLE(huart6.hdmarx);
      this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
      huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
      __HAL_DMA_ENABLE(huart6.hdmarx);
      fifo_s_puts(&referee_fifo, (char *)usart6_buf[1], this_time_rx_len);
      detect_hook(REFEREE_TOE);
    }
  }
}
