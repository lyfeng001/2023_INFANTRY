/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "usart_debug.h"

//#define POWER_LIMIT         80.0f

//#define WARNING_POWER_BUFF  55.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 无判断总电流限制
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f    //缓冲器总电流限制
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f    //功率电流限制
fp32 current_scale;
/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */

		
void chassis_power_control(chassis_move_t *chassis_power_control)
{
	float power_limit,warning_power,warning_power_buff;
	
	if(ext_game_robot_status.robot_level==1)
	{
		warning_power = 40.0;
	}
	if(ext_game_robot_status.robot_level==2)
	{
		warning_power = 50.0;
	}
	if(ext_game_robot_status.robot_level==3)
	{
		warning_power = 60.0;
	}
	
	power_limit=ext_game_robot_status.chassis_power_limit;
	
	warning_power_buff=55.0f;
	
	fp32 chassis_power = 0.0f;
	fp32 chassis_power_buffer = 0.0f;
	fp32 total_current_limit = 0.0f;
	fp32 total_current = 0.0f;
	
    uint8_t robot_id = get_robot_id();
	
    if(toe_is_error(REFEREE_TOE))
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    else
    {
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
      
        if(chassis_power_buffer < warning_power_buff)
        {
            fp32 power_scale;
            if(chassis_power_buffer > 5.0f)//缩小WARNING_POWER_BUFF
            {
                power_scale = chassis_power_buffer / warning_power_buff;
            }
            else //only left 10% of WARNING_POWER_BUFF
            {
                power_scale = 5.0f / warning_power_buff;
            }
            total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;//缩小
        }
        else//缓冲能量大于
        {
            if(chassis_power > warning_power)//功率大于warning_power
            {
                fp32 power_scale;
                //功率小于80w
                if(chassis_power < power_limit)
                {
                    //缩小
                    power_scale = (power_limit - chassis_power) / (power_limit - warning_power);
                }
                //功率大于80w
                else
                {
                    power_scale = 0.0f;
                }
                
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
            }
            //功率小于warning_power
            else
            {
                total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
            }
        } 
    }

    
    total_current = 0.0f;
   
    for(uint8_t i = 0; i < 4; i++) //计算电机电流设定
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
    }
    
    if(total_current > total_current_limit)
    {
        current_scale = total_current_limit / total_current;
        chassis_power_control->motor_speed_pid[0].out*=current_scale;
        chassis_power_control->motor_speed_pid[1].out*=current_scale;
        chassis_power_control->motor_speed_pid[2].out*=current_scale;
        chassis_power_control->motor_speed_pid[3].out*=current_scale;
    }
	//UART_DMA_SEND(ext_power_heat_data.chassis_power_buffer);
}
