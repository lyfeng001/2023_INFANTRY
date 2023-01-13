/*
* @Author: TJSP_2022_TY
* @Date:   2021-11-28
* @Last Modified by:   
* @Last Modified time:  
* @brief: port from RM_standard_robot remote_control. uart1_dma_rx has been reconfigured in Cube. use double dma buffer. Added Kalmanfilter for prediction.
*/

#include "autoaim.h"
#include "struct_typedef.h"
#include "usart.h"
#include "bsp_usart.h"
#include "fifo.h"
#include "bsp_crc8.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "INS_task.h"



extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
uint8_t Auto_Update_Flag; 
uint8_t autoaim_frame_rx_buf[2][AUTOAIM_FRAME_BUF];
static uint8_t autoaim_frame_tx_buf[AUTOAIM_FRAME_LEN];
frame fram;
frame Auto_Frame_Large_Raw;
frame sendtoCom_frame;
uint16_t autoaim_watch;
uint16_t autoaim_watch2;

void usart1_rx_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern uint8_t mycrc8Check(uint8_t *buff,uint16_t len);
uint8_t Auto_Frame_Unpack(uint8_t *autoaim_buf);
void packFrame(uint8_t *buff,frame *fram);
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
void kalman_para_init (void);
float kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
kalman_filter_init_t kalman_para_pitch;
kalman_filter_init_t kalman_para_yaw;
kalman_filter_t kalman_filter_pitch;
kalman_filter_t kalman_filter_yaw;
auto_aim_t auto_aim_data;
/*static fp32 yaw_calc;
static fp32 pitch_calc;
fp32 yaw_velo_clac;
fp32 lastyaw;
fp32 lasttarget;
fp32 nowtarget;
fp32 lastspeed;
fp32 nowmyyaw;
fp32 lastmyyaw;
uint16_t yaw_calc1000;
uint16_t yaw_raw1000;

uint32_t last_timestamp;
uint32_t this_timestamp;
uint16_t delta_timestamp;
*/
fp32 lastyaw_rx;
fp32 thisyaw_rx;
fp32 myyaw;
uint32_t last_timestamp_rx;
uint32_t this_timestamp_rx;
uint16_t delta_timestamp_rx;
extern fp32 para_pass_yaw_angle(void);	
const auto_aim_t *get_autoaim_point(void)
{
		return &auto_aim_data;
}
void autoaim_init(void)
{
	usart1_rx_dma_init(autoaim_frame_rx_buf[0],autoaim_frame_rx_buf[1], AUTOAIM_FRAME_BUF);
	auto_aim_data.autoaim_rx = &fram;
	auto_aim_data.autoaim_tx = &sendtoCom_frame;
	auto_aim_data.pitch_target = 0; 
	auto_aim_data.yaw_target = 0; 	
//	auto_aim_data.gimbal_INT_gyro_point_auto = get_INS_angle_point();
}



	
void usart1_rx_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

}


//中断服务函数
void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
				
        static uint16_t this_time_fram_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_fram_len = AUTOAIM_FRAME_BUF - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = AUTOAIM_FRAME_BUF;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_fram_len == AUTOAIM_FRAME_LEN)
            {
								autoaim_watch++;
                Auto_Frame_Unpack(autoaim_frame_rx_buf[0]);
            }
        }
        else
        { 
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_fram_len = AUTOAIM_FRAME_BUF - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = AUTOAIM_FRAME_BUF;

            //set memory buffer 0
            //设定缓冲区0
            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_fram_len == AUTOAIM_FRAME_LEN)
            {		
				autoaim_watch2++;
                Auto_Frame_Unpack(autoaim_frame_rx_buf[1]);
            }
				
        }
    }
    
}

uint8_t Auto_Frame_Unpack(/*volatile const frame*/uint8_t *autoaim_buf)
{
	static int count;
	static fp32 myyaw_overloop;
	static fp32 lastmyyaw_overloop;
	lastmyyaw_overloop = myyaw_overloop;
	last_timestamp_rx =this_timestamp_rx;
	lastyaw_rx = thisyaw_rx;
	this_timestamp_rx = HAL_GetTick();
	delta_timestamp_rx = this_timestamp_rx - last_timestamp_rx;
	memcpy ( (uint8_t *)&Auto_Frame_Large_Raw, autoaim_buf, AUTOAIM_FRAME_LEN);

	if(Auto_Frame_Large_Raw.head != AUTO_FRAME_HEAD)
	{
		return 0;
	}
	uint8_t crc_check_auto = mycrc8Check((uint8_t *)&Auto_Frame_Large_Raw,AUTOAIM_FRAME_LEN-2);
	if(Auto_Frame_Large_Raw.crc8check != crc_check_auto)
	{
		return 0;
	}
	memcpy((uint8_t *)&fram, autoaim_buf, AUTOAIM_FRAME_LEN);

	Auto_Frame_Large_Raw.head = 0;
	Auto_Frame_Large_Raw.crc8check = 0;
	Auto_Update_Flag = 1;
	myyaw_overloop = para_pass_yaw_angle();
	if (myyaw_overloop - lastmyyaw_overloop > PI){
		count++;
	}
	else if (myyaw_overloop - lastmyyaw_overloop < -PI){
		count--;
	}
	auto_aim_data.my_yaw_overloop = myyaw_overloop + count * 2 * PI;
	thisyaw_rx = -fram.yaw / 57.3f ;
	auto_aim_data.yaw_target = thisyaw_rx + auto_aim_data.my_yaw_overloop;		
	auto_aim_data.yaw_speed = (	thisyaw_rx-lastyaw_rx)/delta_timestamp_rx/1000;
	auto_aim_data.pitch_target = fram.pitch / 57.3f;
	return 1;
}


void sendtoComputer(int timestamp_doing, int auto_aim, int big_buff,int entering_auto_aim, float yaw, float pitch)
{
	sendtoCom_frame.head=0xf1;
	sendtoCom_frame.end=0xf2;
	
	sendtoCom_frame.timestamp = timestamp_doing;
	sendtoCom_frame.yaw 	= yaw;
	sendtoCom_frame.pitch = pitch;
//	sendtoCom_frame.yaw = Yaw;
//	sendtoCom_frame.pitch = Pitch;
/*
	int16_t a;
	a = ext_game_robot_state.robot_level;
	switch(a){
		case 1: sendtoCom_frame.speed = 14.0; break;
		case 2: sendtoCom_frame.speed = 17.0; break;
		case 3: sendtoCom_frame.speed = 17.0; break;
	}
	*/
	sendtoCom_frame.speed = 14.0;
	if(auto_aim == 1)
		sendtoCom_frame.extra[0] = 0x11;
	if(big_buff == 1)
		sendtoCom_frame.extra[0] = 0x21;// Smallbuff Clockwise
	if(big_buff == 2)
		sendtoCom_frame.extra[0] = 0x22;// Smallbuff Counterclockwise
	if(big_buff == 3)
		sendtoCom_frame.extra[0] = 0x23;// Bigbuff Clockwise
	if(big_buff == 4)
		sendtoCom_frame.extra[0] = 0x24;// Bigbuff Counterclockwise
	
	if(entering_auto_aim==1)
		sendtoCom_frame.extra[1] = 0x01;
	else
		sendtoCom_frame.extra[1] = 0x00;
	
	packFrame(autoaim_frame_tx_buf,&sendtoCom_frame);//减少每次搬运内存时间
	usart1_tx_dma_enable(autoaim_frame_tx_buf,AUTOAIM_FRAME_LEN);

}


void packFrame(uint8_t *buff,frame *fram)
{	
	memcpy(buff,fram,sizeof(frame));
	buff[sizeof(frame)-2] = mycrc8Check((uint8_t *)fram,sizeof(frame)-2);
}

void kalman_para_init (void){
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
	
	kalman_filter_init (&kalman_filter_yaw , &kalman_para_yaw);
}
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->A,2,2,(float *)I->A_data);	
  mat_init(&F->AT,2,2,(float *)I->AT_data);	
  mat_trans(&F->A, &F->AT);	
  mat_init(&F->H,2,2,(float *)I->H_data);
  mat_init(&F->HT,2,2,(float *)I->HT_data);	
  mat_init(&F->K,2,2,(float *)I->K_data);	
  mat_trans(&F->H, &F->HT);
	mat_init(&F->P,2,2,(float *)I->P_data);
	mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);  	
	mat_init(&F->Q,2,2,(float *)I->Q_data);	
	mat_init(&F->R,2,2,(float *)I->R_data);	
	mat_init(&F->z,2,1,(float *)I->z_data);	
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);	

	
}

float kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);

  F->z.pData[0] = signal1;
  F->z.pData[1] = signal2;

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);

  mat_inv(&F->K, &F->P);
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);
  mat_sub(&F->z, &TEMP21, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP21);
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);
  mat_sub(&F->Q, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];

  return F->filtered_value[0];
}
