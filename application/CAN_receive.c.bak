/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "detect_task.h"
#include "supercap_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
//cap data read
static void get_cap_data(supercap_module_receive *ptr, uint8_t *data)
{
		(ptr)->charge_enabled_state = (data)[0] & 0x0001;
		(ptr)->residue_power = (data)[1];
		(ptr)->charge_power  = (uint16_t)((data)[2]<<8 | (data)[3])*25;
		(ptr)->chassis_power = (int16_t) ((data)[4]<<8 | (data)[5])*25;
		(ptr)->cap_vol 			= (int16_t) ((data)[6]<<8 | (data)[7])*1.25;
}
		
//				SCM_rx_message.charge_enabled_state = rx_data[0] & 0x0001;
//				SCM_rx_message.residue_power = rx_data[1];
//				SCM_rx_message.charge_power  = (uint16_t)(rx_data[2]<<8 | rx_data[3])*25;
//				SCM_rx_message.chassis_power = (int16_t) (rx_data[4]<<8 | rx_data[5])*25;
//				SCM_rx_message.cap_vol 			 = (int16_t) (rx_data[6]<<8 | rx_data[7])*1.25;
		
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
						4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
					4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006/3508电机 7:左摩擦轮 3508电机（去减速箱） 8: 右摩擦轮 3508电机（去减速箱） */
static motor_measure_t motor_chassis[9];
//supercap data
supercap_module_receive SCM_rx_message;	

static CAN_TxHeaderTypeDef  fric_tx_message;
static uint8_t              fric_can_send_data[8];
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  supercap_tx_message;
static uint8_t              supercap_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if (hcan == &hcan2)
	{
		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_YAW_MOTOR_ID:
			case CAN_PIT_MOTOR_ID:
			case CAN_TRIGGER_MOTOR_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				get_motor_measure(&motor_chassis[i], rx_data);
				detect_hook(CHASSIS_MOTOR1_TOE + i);
				break;
			}
			case CAN_CAP_ID:
			{
				get_cap_data(&SCM_rx_message, rx_data);
				Lost_Connection_Count = 0;
				break;
			}
			default:
				break;
		}
	}
	else if (hcan == &hcan1)
	{
		switch (rx_header.StdId)
		{
			case CAN_3508_FRICL_ID:
			{
				static uint8_t i = 7;
				get_motor_measure(&motor_chassis[i], rx_data);
				break;
			}
			case CAN_3508_FRICR_ID:
			{
				static uint8_t i = 8;
				get_motor_measure(&motor_chassis[i], rx_data);
				break;
			}
			default:
				break;
		}
	}
}

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = 0x00;
    gimbal_can_send_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流(0x201,0x202)
  * @param[in]      fricl: (0x201) 
  * @param[in]      fricr: (0x202) 
  * @retval         none
  */
void CAN_cmd_fric(int16_t fricl, int16_t fricr)
{
    uint32_t send_mail_box;
    fric_tx_message.StdId = 0x200;
    fric_tx_message.IDE = CAN_ID_STD;
    fric_tx_message.RTR = CAN_RTR_DATA;
    fric_tx_message.DLC = 0x08;
    fric_can_send_data[0] = (fricl >> 8);
    fric_can_send_data[1] = fricl;
    fric_can_send_data[2] = (fricr >> 8);
    fric_can_send_data[3] = fricr;
    fric_can_send_data[4] = 0;
    fric_can_send_data[5] = 0;
    fric_can_send_data[6] = 0;
    fric_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &fric_tx_message, fric_can_send_data, &send_mail_box);
}


/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          根据通道选择电容工作模式(0-1-2-3)
  * @param[in]      cap_flag : (0-1-2-3)
  * @retval         none
  */
void CAN_cmd_supercap(int16_t flag)
{
    uint32_t send_mail_box;
	
	uint8_t Powersourses_Charge=0x0001;
	uint8_t Powersourses_Uncharge=0x0000;
	uint8_t Capsourses_Charge=0x0003;
	uint8_t Capsourses_Uncharge=0x0002;
		
    supercap_tx_message.StdId = 0x300;
    supercap_tx_message.IDE = CAN_ID_STD;
    supercap_tx_message.RTR = CAN_RTR_DATA;
    supercap_tx_message.DLC = 0x08;
		
	if (flag==1){ //电容没充满，有剩余功率
    supercap_can_send_data[0] = (unsigned char)(Powersourses_Charge);
		supercap_can_send_data[1] = (unsigned char)0;
		supercap_can_send_data[2] = (unsigned char)(Residue_Power >> 8);
		supercap_can_send_data[3] = (unsigned char)Residue_Power;
		supercap_can_send_data[4] = (unsigned char)0;
		supercap_can_send_data[5] = (unsigned char)0;
		supercap_can_send_data[6] = (unsigned char)0;
		supercap_can_send_data[7] = (unsigned char)0;
	}
	else if(flag==0){	//电池供电
		supercap_can_send_data[0] = (unsigned char)(Powersourses_Uncharge);
		supercap_can_send_data[1] = (unsigned char)0;
		supercap_can_send_data[2] = (unsigned char)0;
		supercap_can_send_data[3] = (unsigned char)0;
		supercap_can_send_data[4] = (unsigned char)0;
		supercap_can_send_data[5] = (unsigned char)0;
		supercap_can_send_data[6] = (unsigned char)0;
		supercap_can_send_data[7] = (unsigned char)0;
	}
	else if(flag==2){	//用电容供电,电池同时供电
		supercap_can_send_data[0] = (unsigned char)(Capsourses_Charge);
		supercap_can_send_data[1] = (unsigned char)0;
		supercap_can_send_data[2] = (unsigned char)(Residue_Power>>8);
		supercap_can_send_data[3] = (unsigned char)(Residue_Power);
		supercap_can_send_data[4] = (unsigned char)0;
		supercap_can_send_data[5] = (unsigned char)0;
		supercap_can_send_data[6] = (unsigned char)0;
		supercap_can_send_data[7] = (unsigned char)0;
	}
	else if(flag==3){	//用电容供电
		supercap_can_send_data[0] = (unsigned char)(Capsourses_Uncharge);
		supercap_can_send_data[1] = (unsigned char)0;
		supercap_can_send_data[2] = (unsigned char)0;
		supercap_can_send_data[3] = (unsigned char)0;
		supercap_can_send_data[4] = (unsigned char)0;
		supercap_can_send_data[5] = (unsigned char)0;
		supercap_can_send_data[6] = (unsigned char)0;
		supercap_can_send_data[7] = (unsigned char)0;
	}
	
	  HAL_CAN_AddTxMessage(&hcan2, &supercap_tx_message, supercap_can_send_data, &send_mail_box);
}




/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

const motor_measure_t *get_fricl_motor_measure_point(void)
{
    return &motor_chassis[7];
}

const motor_measure_t *get_fricr_motor_measure_point(void)
{
    return &motor_chassis[8];
}

/**
  * @brief          返回超级电容数据指针
  * @param[in]      null
  * @retval         电机数据指针
  */
const supercap_module_receive *get_cap_measure_point(void)
{
	return &SCM_rx_message;
}
