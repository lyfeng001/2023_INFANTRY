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

typedef struct
{
	fp32 yaw_target;
	fp32 my_yaw_overloop;
	fp32 pitch_target;
	fp32 yaw_speed;
} autoaim_data_t;

void autoaim_init(void);
const autoaim_data_t *get_autoaim_data(void);

#endif
