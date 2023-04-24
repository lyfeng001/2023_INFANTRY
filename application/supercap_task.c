/**
  ****************(C) COPYRIGHT 2021 TJRM_SUPERPOWER****************************
  * @file       supercap_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     SEPT-2-2021     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
	****************(C) COPYRIGHT 2021 TJRM_SUPERPOWER****************************
	*/
#include "string.h"
#include "cmsis_os.h"

#include "supercap_task.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "referee.h"
#include "chassis_behaviour.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t supercap_high_water;
#endif

int32_t Power_Limitation_Num=55000;
int32_t Residue_Power=55000/25;

int8_t CAN2_Cap_flag=0;
int8_t Cap_Switch_Flag=0;
int8_t cap_on=1;

int32_t Lost_Connection_Count=0;
uint8_t Cap_Switch_Count=0;

int32_t Cap_Toutuous_Uppest=25000;
int32_t Cap_Toutuous_Up=24000;
int32_t Cap_Toutuous_Down=20000;

int32_t Chassis_Power=0;

cap_state cap_FSM;
cap_state cap_FSM_ex;
cap_control_t cap_control;

/**
  * @brief          超级电容初始化函数
  * @param[in]      pvParameters: 空
  * @retval         none
  */
static void Cap_init(cap_control_t *cap_control_init);

/**
  * @brief          超级电容控制函数，间隔 SUPERCAP_CONTROL_TIME 1ms
  * @param[in]      cap_control_init: cap_control_t变量指针
  * @retval         none
  */
static void Cap_Contorl(cap_control_t *cap_control);

/**
  * @brief          超级电容任务，间隔 SUPERCAP_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void supercap_task(void const * argument)
{
	vTaskDelay(SUPERCAP_TASK_INIT_TIME);
	Cap_init(&cap_control);
	
	while(1)
	{
	
#if SUPER_CAP_ON
			Cap_Contorl(&cap_control);	//选择运行模式
			CAN_cmd_supercap(cap_FSM);	//发送运行模式通道
			cap_FSM_ex = cap_FSM;	//状态机更新
#endif


			vTaskDelay(SUPERCAP_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark		//1
        supercap_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

/**
  * @brief          超级电容初始化函数
  * @param[in]      cap_control_init: cap_control_t变量指针
  * @retval         none
  */
static void Cap_init(cap_control_t *cap_control_init)
{
	if (cap_control_init == NULL)
    {
        return;
    }
		
	//获取遥控器指针
    cap_control_init->cap_rc = get_remote_control_point();
	cap_control_init->cap_message = get_cap_measure_point();
}


/**
  * @brief          超级电容控制函数，间隔 SUPERCAP_CONTROL_TIME 1ms
  * @param[in]      cap_control_init: cap_control_t变量指针
  * @retval         none
  */
static void Cap_Contorl(cap_control_t *cap_control)
{
	fp32 chassis_power = 0.0f;
	fp32 chassis_power_buffer = 0.0f;
	
	Lost_Connection_Count+=1;
	if(Lost_Connection_Count>60000) 
		Lost_Connection_Count=60001;
	
#if Cap_Offline_Debug	//离线调试 0
	
	int8_t Debug_Flag=0;
	
	extern supercap_module_receive SCM_rx_message;
	const supercap_module_receive *cap_message	= &SCM_rx_message;
	
	if(Debug_Flag==0 && CAN2_Cap_flag!=4){ //放电
	
		if(cap_message->cap_vol<Cap_Toutuous_Up){//回环判断
				CAN2_Cap_flag=2;
		}
		else if (CAN2_Cap_flag==2 && cap_message->cap_vol<Cap_Toutuous_Uppest){
				Residue_Power=Power_Limitation_Num/25;
				CAN_cmd_supercap(2);//边充边放
		}
		else{
				CAN_cmd_supercap(3);//纯电容放电
				CAN2_Cap_flag=0;
		}
		if(cap_message->cap_vol<Cap_Toutuous_Up){
				CAN2_Cap_flag=4;
		}
	}
	else{
		if(cap_message->cap_vol<Cap_Toutuous_Up){//回环判断
				CAN2_Cap_flag=1;
		}
		if(CAN2_Cap_flag==1 && cap_message->cap_vol<Cap_Toutuous_Uppest && cap_message->chassis_power<Power_Limitation_Num){
				Residue_Power=(Power_Limitation_Num-cap_message->chassis_power)/25;
				CAN_cmd_supercap(1);//边充边放
		}
		else{
				CAN_cmd_supercap(0);//纯电池放电
				CAN2_Cap_flag=0;
		}
	}

#else
	
	#if Referee_System //裁判系统

	get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
	Chassis_Power = chassis_power*1000;		
	
	#else
	if(Lost_Connection_Count<100)
			Chassis_Power = SCM_rx_message.chassis_power;
	else
			Chassis_Power = current_sum*24;
	
	
	if(ext_game_robot_status.mains_power_chassis_output==0)//如果底盘被裁判系统断电，电容不放电
	{
		if(cap_control->cap_message->cap_vol<Cap_Toutuous_Uppest)
			cap_FSM=cap_dis_charge;//只充不放
		else
			cap_FSM=cap_dis_ucharge;//不充不放
		cap_on=0;
	}
	
	#endif

	if(cap_on==1)
	{
	if(switch_is_mid(cap_control->cap_rc->rc.s[CHASSIS_MODE_CHANNEL])||switch_is_up(cap_control->cap_rc->rc.s[CHASSIS_MODE_CHANNEL]))  //右边拨杆处于中间或者上面
	{
		if(cap_control->cap_message->cap_vol>26000)//电容电量26000
		{
			cap_FSM=cap_en_ucharge;//不充但放
		}
		else
		{
			if((cap_control->cap_rc->key.v & KEY_PRESSED_OFFSET_F ) == KEY_PRESSED_OFFSET_F ||(cap_control->cap_rc->rc.ch[4]>4500))
			{
				cap_FSM=cap_en_ucharge;
			}
			else{
				cap_FSM=cap_dis_charge;//不放但充
			}
		}
	}
	
	else if(switch_is_down(cap_control->cap_rc->rc.s[CHASSIS_MODE_CHANNEL]))//down
	{
		if(cap_control->cap_message->cap_vol<Cap_Toutuous_Uppest)
		{
			cap_FSM = cap_dis_charge;//电容电量小于25000 充
		}
		else//电容电量大于25000
		{
			cap_FSM = cap_dis_ucharge;//不充电
			CAN2_Cap_flag=0;
		}
	}
	
	
	else//读不到遥控器值？
	{
		cap_FSM = cap_dis_ucharge;//不充电且用电池放电
	}
}
#endif
}

