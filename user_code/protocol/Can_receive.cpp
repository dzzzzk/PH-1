#include "can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_can.h"
#include "struct_typedef.h"
#include "string.h"
#include "usart.h"


void Can_receive::init()
{
    can_filter_init();
}
/*----------------------------数据处理函数------------------------------*/
//驱动轮电机
void Can_receive::get_balance_motive_motor_measure(uint8_t num, uint8_t data[8])
{
    chassis_blance_motive_motor[num].last_ecd = chassis_blance_motive_motor[num].ecd;
    chassis_blance_motive_motor[num].command = data[0];
    chassis_blance_motive_motor[num].temperate = data[1];
    chassis_blance_motive_motor[num].given_current = (uint16_t)(data[3] << 8 | data[2]);
    chassis_blance_motive_motor[num].speed_rpm = (uint16_t)(data[5] << 8 | data[4]);
    chassis_blance_motive_motor[num].ecd = (uint16_t)(data[7] << 8 | data[6]);
}
//关节电机
uint8_t motor_receive[80]={0};
uint32_t crc32_2 = 0;
uint32_t crc32_1 = 0;
fp32 r_T = 0;
fp32 W_T = 0;
fp32 A_T = 0;
fp32 p_T = 0;
void Can_receive::get_joint_motive_motor_measure(int n, uint8_t *ReadFormUart)
{
 if(ReadFormUart[0] == 0xFE && ReadFormUart[1] == 0xEE)
	{
		crc32_1 = (uint32_t)(ReadFormUart[77] << 24 | ReadFormUart[76] << 16 | ReadFormUart[75] << 8 | ReadFormUart[74] );
		for(int i = 0; i <78;i++)
			{
				motor_receive[i] = ReadFormUart[i];
			}
			crc32_2 = crc32_core((uint32_t*)(&(motor_receive)), 18);
			if(crc32_1 == crc32_2)
			{
                int motor_id = 0;
                motor_id = motor_receive[2];
                if(n == 0){
                    if(motor_id == 0)motor_id =1;
                    else if(motor_id == 2)motor_id =0;                    
                }
                if(n == 1){
                    if(motor_id == 0)motor_id =3;
                    else if(motor_id == 2)motor_id =2;                    
                }
					//接收数据拷贝     
					chassis_joint_motor[motor_id].Temp= motor_receive[6];
					
					r_T = (fp32)(motor_receive[13] << 8 | motor_receive[12] )/256;
					W_T = (fp32)(motor_receive[15] << 8 | motor_receive[14] ) / 128/9.1;
					A_T = (fp32)(motor_receive[27] << 8 | motor_receive[26] ) ;
					p_T = (fp32)(motor_receive[33] << 24 | motor_receive[32] << 16 |motor_receive[31] << 8 | motor_receive[30]) / 16384*2*3.14/9.1;
					if(r_T<128){
					chassis_joint_motor[motor_id].T = r_T;
					chassis_joint_motor[motor_id].W = W_T;
					chassis_joint_motor[motor_id].Acc = A_T;
					chassis_joint_motor[motor_id].Pos = p_T;	
					}						
				}
			}
}

/*----------------------------数据发送函数------------------------------*/
/*  驱动轮电机
		id：0x141,0x142
		电流范围 [-2000,2000]
		待修改
*/
void Can_receive::can_cmd_balahnce_chassis_motive_motor(int16_t motor1, int16_t motor2)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x280;    
    chassis_tx_message.IDE = CAN_ID_STD;                     
    chassis_tx_message.RTR = CAN_RTR_DATA;                   
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1 ;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = 0x00;
    chassis_can_send_data[5] = 0x00;
    chassis_can_send_data[6] = 0x00;
    chassis_can_send_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	
}
//A1电机数据填充
uint8_t motor_s1[34] = {0};
extern Can_receive can_receive;
void uart_cmd_joint_motor(int uart_num,uint16_t id,fp32 torque)
{
    can_receive.motor_A1_sendmeasure[id].start[0] = 0xFE;
    can_receive.motor_A1_sendmeasure[id].start[1] = 0xEE;
    if(id == 0 || id == 2)
    {
        can_receive.motor_A1_sendmeasure[id].motorID =2;
    }else{
        can_receive.motor_A1_sendmeasure[id].motorID =0;
    }     
    can_receive.motor_A1_sendmeasure[id].reserved_1 = 0x0;
	can_receive.motor_A1_sendmeasure[id].mode = 10;     //控制模式，混合为10
    can_receive.motor_A1_sendmeasure[id].ModifyBit = 0xFF;
    can_receive.motor_A1_sendmeasure[id].ReadBit = 0x0;
    can_receive.motor_A1_sendmeasure[id].reserved_2 = 0x0;
    can_receive.motor_A1_sendmeasure[id].Modify = 0;
	//控制部分
    can_receive.motor_A1_sendmeasure[id].T = torque*256;//0.16*256;
    can_receive.motor_A1_sendmeasure[id].W = 0*5.0*9.1*128;
	can_receive.motor_A1_sendmeasure[id].Pos =(int)((0*3.14*9.1/6.2832)*16384.0);
    can_receive.motor_A1_sendmeasure[id].K_P = 0*2048;
    can_receive.motor_A1_sendmeasure[id].K_W = 0*3.0*1024;
    can_receive.motor_A1_sendmeasure[id].LowHzMotorCmdIndex = 0;
    can_receive.motor_A1_sendmeasure[id].LowHzMotorCmdByte = 0;
    can_receive.motor_A1_sendmeasure[id].Res[0] = 0; 
		
	memcpy(motor_s1,&can_receive.motor_A1_sendmeasure[id],30);
	
	can_receive.motor_A1_sendmeasure[id].CRC32 = crc32_core((uint32_t*)(&(motor_s1)), 7);
		
	memcpy(motor_s1,&can_receive.motor_A1_sendmeasure[id],34);
		

	if( uart_num == 1)
		{
			for (int i = 0; i < 34; i++)
				{
					HAL_UART_Transmit(&huart1, &motor_s1[i], 1 , 0xFFF);
				}
		}else
        {
			for (int i = 0; i < 34; i++)
				{
					HAL_UART_Transmit(&huart6, &motor_s1[i], 1 , 0xFFF);
				}
		}			
}

//数据指针
 const motor_9025_measure_t *Can_receive::get_chassis_motive_motor_balance_measure_point(uint8_t i)
{
    return &chassis_blance_motive_motor[i];
} 

const motor_A1_measure_t *Can_receive::get_joint_motor_measure_point(uint8_t i)
{
    return &chassis_joint_motor[i];
}

//接收遥控器数据
void Can_receive::receive_rc_board_com(uint8_t data[8])
{
    chassis_receive.ch_0 = (int16_t)(data[0] << 8 | data[1]);
    chassis_receive.ch_2 = (int16_t)(data[2] << 8 | data[3]);
    chassis_receive.ch_3 = (int16_t)(data[4] << 8 | data[5]);
    chassis_receive.v = (uint16_t)(data[6] << 8 | data[7]);
}

//接收云台状态
void Can_receive::receive_gimbal_board_com(uint8_t data[8])
{
    chassis_receive.s1 = data[0];
    chassis_receive.gimbal_behaviour = data[1];
    chassis_receive.gimbal_yaw_angle = (fp32)(int32_t)(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000;
}

//接收ui标识符
void Can_receive::receive_ui_board_com(uint8_t data[8])
{
    chassis_receive.auto_state = data[0];
    chassis_receive.aim_state = data[1];
    chassis_receive.fric_state = data[2];
    chassis_receive.gimbal_pitch_angle = (fp32)(int16_t)(data[3] << 8 | data[4]);;
    chassis_receive.v = (uint16_t)(data[5] << 8 | data[6]);
}

//发送冷却数据
void Can_receive::send_cooling_and_id_board_com(uint16_t id1_17mm_cooling_limit, uint16_t id1_17mm_cooling_rate, uint16_t id1_17mm_cooling_heat, uint8_t color, uint8_t robot_id)
{
    //数据填充
    chassis_send.id1_17mm_cooling_limit = id1_17mm_cooling_limit;
    chassis_send.id1_17mm_cooling_rate = id1_17mm_cooling_rate;
    chassis_send.id1_17mm_cooling_heat = id1_17mm_cooling_heat;
    chassis_send.color = color;
    chassis_send.robot_id = robot_id;


    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_COOLING_BOARM_COM_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = id1_17mm_cooling_limit >> 8;
    chassis_can_send_data[1] = id1_17mm_cooling_limit;
    chassis_can_send_data[2] = id1_17mm_cooling_rate >> 8;
    chassis_can_send_data[3] = id1_17mm_cooling_rate;
    chassis_can_send_data[4] = id1_17mm_cooling_heat >> 8;
    chassis_can_send_data[5] = id1_17mm_cooling_heat;
    chassis_can_send_data[6] = color;
    chassis_can_send_data[7] = robot_id;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

//发送17mm射速
void Can_receive::send_17mm_speed_and_mode_board_com(uint16_t id1_17mm_speed_limit, uint16_t bullet_speed, uint8_t chassis_behaviour,uint8_t game_progress)
{
    //数据填充
    chassis_send.id1_17mm_speed_limi = id1_17mm_speed_limit;
    chassis_send.bullet_speed = bullet_speed;
    chassis_send.chassis_behaviour = chassis_behaviour;
    chassis_send.game_progress = game_progress;
	
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_17MM_SPEED_BOARD_COM_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = id1_17mm_speed_limit >> 8;
    chassis_can_send_data[1] = id1_17mm_speed_limit;
    chassis_can_send_data[2] = bullet_speed >> 8;
    chassis_can_send_data[3] = bullet_speed;
    chassis_can_send_data[4] = chassis_behaviour;
    chassis_can_send_data[5] = game_progress;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

//接收超电数据
void Can_receive::get_super_cap_data(uint8_t data[8])
{
    cap_receive.input_vot = ((float)((int16_t)(data[1] << 8 | data[0])) / 100.0f);     //输入电压
    cap_receive.cap_vot = ((float)((int16_t)(data[3] << 8 | data[2])) / 100.0f);       //电容电压
    cap_receive.input_current = ((float)((int16_t)(data[5] << 8 | data[4])) / 100.0f); //输入电流
    cap_receive.target_power = ((float)((int16_t)(data[7] << 8 | data[6])) / 100.0f);  //输入功率
}

//发送超电数据
void Can_receive::can_cmd_super_cap_power(uint16_t set_power)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x210;
    chassis_tx_message.IDE = CAN_ID_STD;   // 0x0000
    chassis_tx_message.RTR = CAN_RTR_DATA; // 0x0000
    chassis_tx_message.DLC = 0x08;

    chassis_can_send_data[0] = (set_power >> 8);
    chassis_can_send_data[1] = (set_power);
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
