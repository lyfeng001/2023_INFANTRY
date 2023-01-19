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

#define AUTOAIM_FRAME_LEN 22
#define AUTOAIM_FRAME_BUF 44
#define AUTO_FRAME_HEAD 0xf1
#define AUTO_FRAME_END 0xf2

#define mat arm_matrix_instance_f32
#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32
#define mat_inv arm_mat_inverse_f32
#define sin arm_sin_f32
#define cos arm_cos_f32
#define atan2 AHRS_atan2f

#pragma pack(1)
typedef struct
{
    uint8_t head;       // 1 byte
    uint16_t timestamp; // 2 byte
    float x_in_gimbal;  // 4 byte
    float y_in_gimbal;  // 4 byte
    float z_in_gimbal;  // 4 byte
    uint8_t state;      // 1 byte
    uint16_t time;      // 2 byte
    uint8_t extra[2];   // 2 byte
    uint8_t crc8_check; // 1 byte
    uint8_t end;        // 1 byte
} frame_t;
#pragma pack()

typedef struct
{
    fp32 x_in_gimbal;
    fp32 y_in_gimbal;
    fp32 z_in_gimbal;
} autoaim_target_t;

typedef struct
{
    float raw_value;
    float filtered_value[2];
    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
    float raw_value;
    float filtered_value[2];
    float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];
    float A_data[4];
    float H_data[4];
    float Q_data[4];
    float R_data[4];
} kalman_filter_init_t;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t autoaim_frame_tx_buf[AUTOAIM_FRAME_LEN];
uint8_t autoaim_frame_rx_buf[2][AUTOAIM_FRAME_BUF];
frame_t frame;
kalman_filter_init_t kalman_para_pitch;
kalman_filter_init_t kalman_para_yaw;
kalman_filter_t kalman_filter_pitch;
kalman_filter_t kalman_filter_yaw;
autoaim_target_t autoaim_target;

void usart1_rx_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
uint8_t unpack_frame(uint8_t *autoaim_buf);

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
void kalman_para_init(void);
float kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);

void autoaim_init(void)
{
    usart1_rx_dma_init(autoaim_frame_rx_buf[0], autoaim_frame_rx_buf[1], AUTOAIM_FRAME_BUF);
    autoaim_target.x_in_gimbal = 0.0;
    autoaim_target.y_in_gimbal = 0.0;
    autoaim_target.z_in_gimbal = 0.0;
}

void set_autoaim_angle(fp32 *yaw_set, fp32 *pitch_set, fp32 gimbal_absolute_yaw, fp32 gimbal_absolute_pitch)
{
    // 云台坐标系：
    // 右手直角坐标系。
    // 原点为pitch轴线与yaw轴线交点。
    // z轴正方向为子弹射出方向，
    // y轴正方向为垂直子弹射出方向向下，
    // x轴正方向为面向枪口时的左侧。
    fp32 x_in_gimbal = autoaim_target.x_in_gimbal;
    fp32 y_in_gimbal = autoaim_target.y_in_gimbal;
    fp32 z_in_gimbal = autoaim_target.z_in_gimbal;

    // 世界坐标系:
    // 右手直角坐标系。
    // 原点与云台坐标系重合。
    // y轴正方向为重力加速度方向。
    // 由于是绝对坐标系，x轴、z轴不好描述，
    // 可以类比为x轴始终指向北。
    // 当底盘静止或小陀螺时，可以作为惯性参考系。
    fp32 x_in_world;
    fp32 y_in_world;
    fp32 z_in_world;

    // 将云台坐标系转换为世界坐标系：
    //
    // 云台坐标系是如何从世界坐标系变换而来的：
    //     初始：云台坐标系和世界坐标系（0系）重合。
    //     yaw轴电机旋转：云台坐标系绕世界坐标系y轴旋转，
    //                   角度为gimbal_absolute_yaw(rad)，
    //                   符号符合右手螺旋。
    //                   该云台坐标系记为1系。
    //     pitch轴电机旋转：云台坐标系绕1系x轴旋转，
    //                     角度为gimbal_absolute_pitch(rad)，
    //                     符号符合右手螺旋。
    //                     获得2系。
    //     这里可以看出云台使用的欧拉角旋转为内旋。
    //
    // 可得公式：p0 = Ry * Rx * p2
    // R_y为绕y轴旋转矩阵，R_x为绕x轴旋转矩阵，
    // 通过该公式，可以将击打目标在云台坐标系下的坐标p2，
    // 转变为目标在世界坐标系下的坐标p0.
    fp32 sin_y = sin(gimbal_absolute_yaw);
    fp32 cos_y = cos(gimbal_absolute_yaw);
    fp32 sin_x = sin(gimbal_absolute_pitch);
    fp32 cos_x = cos(gimbal_absolute_pitch);
    x_in_world = x_in_gimbal * cos_y + y_in_gimbal * sin_y * sin_x + z_in_gimbal * sin_y * cos_x;
    y_in_world = x_in_gimbal * 0.0 + y_in_gimbal * cos_x + z_in_gimbal * (-sin_x);
    z_in_world = x_in_gimbal * (-sin_y) + y_in_gimbal * cos_y * sin_x + z_in_gimbal * cos_y * cos_x;

    fp32 xz_length;
    arm_sqrt_f32(x_in_world * x_in_world + z_in_world * z_in_world, &xz_length);
    *yaw_set = atan2(x_in_world, z_in_world);
    *pitch_set = atan2(y_in_world, xz_length);
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
    memcpy((uint8_t *)&frame, autoaim_buf, AUTOAIM_FRAME_LEN);
    uint8_t crc8_check = get_CRC8_check_sum((uint8_t *)&frame, AUTOAIM_FRAME_LEN - 2, 0xff);
    if (frame.head != AUTO_FRAME_HEAD || frame.end != AUTO_FRAME_END || frame.crc8_check != crc8_check)
    {
        return 0;
    }

    autoaim_target.x_in_gimbal = frame.x_in_gimbal;
    autoaim_target.y_in_gimbal = frame.y_in_gimbal;
    autoaim_target.z_in_gimbal = frame.z_in_gimbal;
    return 1;
}

void kalman_para_init(void)
{
    /***pitch**/
    kalman_para_pitch.A_data[0] = 1;
    kalman_para_pitch.A_data[1] = 0.001;
    kalman_para_pitch.A_data[2] = 0;
    kalman_para_pitch.A_data[3] = 1;

    kalman_para_pitch.H_data[0] = 1;
    kalman_para_pitch.H_data[1] = 0;
    kalman_para_pitch.H_data[2] = 0;
    kalman_para_pitch.H_data[3] = 1;

    kalman_para_pitch.Q_data[0] = 0.01;
    kalman_para_pitch.Q_data[1] = 0;
    kalman_para_pitch.Q_data[2] = 0;
    kalman_para_pitch.Q_data[3] = 0.01;

    kalman_para_pitch.R_data[0] = 0.1;
    kalman_para_pitch.R_data[1] = 0;
    kalman_para_pitch.R_data[2] = 0;
    kalman_para_pitch.R_data[3] = 0.1;
    /***yaw***/
    kalman_para_yaw.A_data[0] = 1;
    kalman_para_yaw.A_data[1] = 0.001;
    kalman_para_yaw.A_data[2] = 0;
    kalman_para_yaw.A_data[3] = 1;

    kalman_para_yaw.H_data[0] = 1;
    kalman_para_yaw.H_data[1] = 0;
    kalman_para_yaw.H_data[2] = 0;
    kalman_para_yaw.H_data[3] = 1;

    kalman_para_yaw.Q_data[0] = 0.02;
    kalman_para_yaw.Q_data[1] = 0;
    kalman_para_yaw.Q_data[2] = 0;
    kalman_para_yaw.Q_data[3] = 0.02;

    kalman_para_yaw.R_data[0] = 0.02;
    kalman_para_yaw.R_data[1] = 0;
    kalman_para_yaw.R_data[2] = 0;
    kalman_para_yaw.R_data[3] = 0.02;

    kalman_filter_init(&kalman_filter_yaw, &kalman_para_yaw);
}
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
    mat_init(&F->A, 2, 2, (float *)I->A_data);
    mat_init(&F->AT, 2, 2, (float *)I->AT_data);
    mat_trans(&F->A, &F->AT);
    mat_init(&F->H, 2, 2, (float *)I->H_data);
    mat_init(&F->HT, 2, 2, (float *)I->HT_data);
    mat_init(&F->K, 2, 2, (float *)I->K_data);
    mat_trans(&F->H, &F->HT);
    mat_init(&F->P, 2, 2, (float *)I->P_data);
    mat_init(&F->Pminus, 2, 2, (float *)I->Pminus_data);
    mat_init(&F->Q, 2, 2, (float *)I->Q_data);
    mat_init(&F->R, 2, 2, (float *)I->R_data);
    mat_init(&F->z, 2, 1, (float *)I->z_data);
    mat_init(&F->xhat, 2, 1, (float *)I->xhat_data);
    mat_init(&F->xhatminus, 2, 1, (float *)I->xhatminus_data);
}

float kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
    float TEMP_data[4] = {0, 0, 0, 0};
    float TEMP_data21[2] = {0, 0};
    mat TEMP, TEMP21;

    mat_init(&TEMP, 2, 2, (float *)TEMP_data);
    mat_init(&TEMP21, 2, 1, (float *)TEMP_data21);

    F->z.pData[0] = signal1;
    F->z.pData[1] = signal2;

    // 1. xhat'(k)= A xhat(k-1)
    mat_mult(&F->A, &F->xhat, &F->xhatminus);

    // 2. P'(k) = A P(k-1) AT + Q
    mat_mult(&F->A, &F->P, &F->Pminus);
    mat_mult(&F->Pminus, &F->AT, &TEMP);
    mat_add(&TEMP, &F->Q, &F->Pminus);

    // 3. K(k) = P'(k) HT / (H P'(k) HT + R)
    mat_mult(&F->H, &F->Pminus, &F->K);
    mat_mult(&F->K, &F->HT, &TEMP);
    mat_add(&TEMP, &F->R, &F->K);

    mat_inv(&F->K, &F->P);
    mat_mult(&F->Pminus, &F->HT, &TEMP);
    mat_mult(&TEMP, &F->P, &F->K);

    // 4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
    mat_mult(&F->H, &F->xhatminus, &TEMP21);
    mat_sub(&F->z, &TEMP21, &F->xhat);
    mat_mult(&F->K, &F->xhat, &TEMP21);
    mat_add(&F->xhatminus, &TEMP21, &F->xhat);

    // 5. P(k) = (1-K(k)H)P'(k)
    mat_mult(&F->K, &F->H, &F->P);
    mat_sub(&F->Q, &F->P, &TEMP);
    mat_mult(&TEMP, &F->Pminus, &F->P);

    F->filtered_value[0] = F->xhat.pData[0];
    F->filtered_value[1] = F->xhat.pData[1];

    return F->filtered_value[0];
}
