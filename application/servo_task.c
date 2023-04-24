/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"
#include "tim.h"

gimbal_servo_t gimbal_servo;

gimbal_servo_t *get_gimbal_servo_t(void)
{
	return &gimbal_servo;
}

const RC_ctrl_t *servo_rc;
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
  
void ammo_lip_invert(gimbal_servo_t *servo)
{
	if(servo->ammo_lip_state == LIP_OPEN_STATE)
	{
		servo->ammo_lip_angle = LIP_CLOSED_ECD;
		servo->ammo_lip_state = LIP_CLOSED_STATE;
		__HAL_TIM_SetCompare(LIP_TIM, LIP_TCH, LIP_CLOSED_ECD);
		HAL_Delay(300);
	}		
	else
	{
		servo->ammo_lip_angle = LIP_OPEN_ECD;
		servo->ammo_lip_state = LIP_OPEN_STATE;
		__HAL_TIM_SetCompare(LIP_TIM, LIP_TCH, LIP_OPEN_ECD);
		HAL_Delay(300);
	}
  
	
}
/**
  * @brief          ¶æ»úÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    //servo_rc = get_remote_control_point();
	
    HAL_TIM_Base_Start(LIP_TIM);
    HAL_TIM_PWM_Start(LIP_TIM, LIP_TCH);
	
	servo_rc = get_remote_control_point();
	
	gimbal_servo.ammo_lip_state = LIP_OPEN_STATE;
	gimbal_servo.ammo_lip_angle = LIP_OPEN_ECD;
	__HAL_TIM_SetCompare(LIP_TIM, LIP_TCH, LIP_OPEN_ECD);
	HAL_Delay(300);
	
       while(1)
    {
		if((servo_rc->key.v & KEY_PRESSED_OFFSET_CTRL) == KEY_PRESSED_OFFSET_CTRL)
		{
			ammo_lip_invert(&gimbal_servo);
		}

        osDelay(10);
    }
}


