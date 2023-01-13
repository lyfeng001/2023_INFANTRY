#ifndef __AUTOAIM_H__
#define __AUTOAIM_H__

#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#define AUTOAIM_FRAME_LEN 22 
#define AUTOAIM_FRAME_BUF 44 
#define AUTO_FRAME_HEAD 0xf1

#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

#pragma  pack(1) 
typedef struct _frame				
{
	uint8_t head;		//0xf1 0xf1
	uint16_t timestamp;
	float yaw;	
	float pitch;
	float speed;
	uint8_t state;
	uint16_t time;
	
	uint8_t extra[2]; //additional imformation	
	uint8_t crc8check;
	uint8_t end;
}frame;			
#pragma  pack() 

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
	const frame *autoaim_rx;
	const frame *autoaim_tx;
	fp32 yaw_target;
	fp32 my_yaw_overloop;
	fp32 pitch_target;
	fp32 yaw_speed;
//	const fp32 *gimbal_INT_gyro_point_auto;	
} auto_aim_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;

#endif
