/*
 * @Author: TJSP_2022_TY
 * @Date:   2021-11-28
 * @Last Modified by: XiaoYoung
 * @Last Modified time: 2023-01-13
 * @brief: port from RM_standard_robot remote_control. uart1_dma_rx has been reconfigured in Cube. use double dma buffer. Added Kalmanfilter for prediction.
 */

#include "autoaim.h"
#include "stm32f4xx_hal.h"
#include "CRC8_CRC16.h"
#include "arm_math.h"
#include "AHRS_middleware.h"
#include "bsp_usart.h"

#define AUTOAIM_FRAME_LEN 22
#define AUTOAIM_FRAME_BUF AUTOAIM_FRAME_LEN * 2
#define AUTO_FRAME_HEAD 0xf1
#define AUTO_FRAME_END 0xf2

#pragma pack(1)
typedef struct
{
    uint8_t head;         // 1 byte
    uint16_t timestamp;   // 2 byte
    float yaw_in_world;   // 4 byte degree
    float pitch_in_world; // 4 byte degree
    float extra_float;    // 4 byte
    uint8_t state;        // 1 byte
    uint16_t time;        // 2 byte
    uint8_t extra[2];     // 2 byte
    uint8_t crc8_check;   // 1 byte
    uint8_t end;          // 1 byte
} frame_t;
#pragma pack()

typedef struct
{
    fp32 yaw_in_world;   // rad
    fp32 pitch_in_world; // rad
    bool_t is_latest;
} autoaim_target_t;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t autoaim_frame_tx_buf[AUTOAIM_FRAME_LEN];
uint8_t autoaim_frame_rx_buf[2][AUTOAIM_FRAME_BUF];
frame_t frame_rx;
frame_t frame_tx;
autoaim_target_t autoaim_target;

void usart1_rx_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
uint8_t unpack_frame(uint8_t *autoaim_buf);
void pack_frame(uint8_t *buff, frame_t *frame);

void autoaim_init(void)
{
    autoaim_target.yaw_in_world = 0.0;
    autoaim_target.pitch_in_world = 0.0;
    autoaim_target.is_latest = 0;
    usart1_rx_dma_init(autoaim_frame_rx_buf[0], autoaim_frame_rx_buf[1], AUTOAIM_FRAME_BUF);
}

void set_autoaim_angle(fp32 *add_yaw_set, fp32 *add_pitch_set, fp32 absolute_yaw_set, fp32 absolute_pitch_set)
{
    *add_yaw_set = autoaim_target.yaw_in_world - absolute_yaw_set;
    *add_pitch_set = 0.0f; // 机械没装平衡补偿，先禁掉
    // *add_pitch_set = autoaim_target.pitch_in_world - absolute_pitch_set;
}

void usart1_rx_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    // 失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);

    // 缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);

    // 缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);

    // 数据长度
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;

    // 使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    // 使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

// 中断服务函数
void USART1_IRQHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE) // 接收数据中
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE) // 数据接收完毕
    {
        static uint16_t this_time_fram_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            // 缓冲区1
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // 获取接收数据长度，长度 = 设定长度 - 剩余长度
            this_time_fram_len = AUTOAIM_FRAME_BUF - hdma_usart1_rx.Instance->NDTR;

            // 重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = AUTOAIM_FRAME_BUF;

            // 设定缓冲区2
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_fram_len == AUTOAIM_FRAME_LEN)
            {
                unpack_frame(autoaim_frame_rx_buf[0]);
            }
        }
        else
        {
            // 缓冲区2
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // 获取接收数据长度，长度 = 设定长度 - 剩余长度
            this_time_fram_len = AUTOAIM_FRAME_BUF - hdma_usart1_rx.Instance->NDTR;

            // 重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = AUTOAIM_FRAME_BUF;

            // 设定缓冲区1
            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);

            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_fram_len == AUTOAIM_FRAME_LEN)
            {
                unpack_frame(autoaim_frame_rx_buf[1]);
            }
        }
    }
}

uint8_t unpack_frame(uint8_t *autoaim_buf)
{
    memcpy((uint8_t *)&frame_rx, autoaim_buf, AUTOAIM_FRAME_LEN);
    uint8_t crc8_check = get_CRC8_check_sum((uint8_t *)&frame_rx, AUTOAIM_FRAME_LEN - 2, 0xff);
    if (frame_rx.head != AUTO_FRAME_HEAD || frame_rx.end != AUTO_FRAME_END || frame_rx.crc8_check != crc8_check)
    {
        return 0;
    }

    autoaim_target.yaw_in_world = frame_rx.yaw_in_world / 57.3f;
    autoaim_target.pitch_in_world = frame_rx.pitch_in_world / 57.3f;
    autoaim_target.is_latest = 1;
    return 1;
}

void send_to_computer(fp32 absolute_yaw, fp32 absolute_pitch)
{
    frame_tx.head = AUTO_FRAME_HEAD;
    frame_tx.end = AUTO_FRAME_END;

    frame_tx.yaw_in_world = absolute_yaw * 57.3f;
    frame_tx.pitch_in_world = absolute_pitch * 57.3f;

    pack_frame(autoaim_frame_tx_buf, &frame_tx);
    usart1_tx_dma_enable(autoaim_frame_tx_buf, AUTOAIM_FRAME_LEN);
}

void pack_frame(uint8_t *buff, frame_t *frame)
{
    memcpy(buff, frame, sizeof(frame_t));
    buff[sizeof(frame_t) - 2] = get_CRC8_check_sum((uint8_t *)frame, sizeof(frame_t) - 2, 0xff);
}
