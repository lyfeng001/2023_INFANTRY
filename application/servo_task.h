#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"
#include "gimbal_task.h"
//µ¯²Ö¸Ç¶æ»ú½Ç¶ÈÖµ
#ifdef INFANTRY_3

#define LIP_OPEN_ECD 1500
#define LIP_CLOSED_ECD 500
#endif

#ifdef INFANTRY_4

#define LIP_OPEN_ECD 1800
#define LIP_CLOSED_ECD 550
#endif

#define LIP_CLOSED_STATE 0
#define LIP_OPEN_STATE 1

#define LIP_TCH TIM_CHANNEL_1
#define LIP_TIM &htim1

typedef struct
{
	bool_t ammo_lip_state;
	uint16_t ammo_lip_angle;
}gimbal_servo_t;

gimbal_servo_t *get_gimbal_servo_t(void);

extern void servo_task(void const * argument);
#endif
