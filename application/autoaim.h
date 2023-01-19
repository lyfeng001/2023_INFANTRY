/*
 * @Author: TJSP_2022_TY
 * @Date:   2021-11-28
 * @Last Modified by: XiaoYoung
 * @Last Modified time: 2023-01-13
 * @brief: port from RM_standard_robot remote_control. uart1_dma_rx has been reconfigured in Cube. use double dma buffer. Added Kalmanfilter for prediction.
 */

#ifndef __AUTOAIM_H__
#define __AUTOAIM_H__

#include "struct_typedef.h"

void autoaim_init(void);
void set_autoaim_angle(fp32 *yaw_set, fp32 *pitch_set, fp32 gimbal_absolute_yaw, fp32 gimbal_absolute_pitch);

#endif
