#include "communicate.h"
#include "string.h"
#include "bsp_led.h"
#include "Chassis.h"
#include "detect_task.h"
#include "Remote_control.h"
#include "Can_receive.h"
#include "usart.h"
#ifdef __cplusplus
extern "C"
{
#endif

#include "CRC8_CRC16.h"
#include "fifo.h"

#ifdef __cplusplus
}
#endif

//模块
Remote_control remote_control;
Can_receive can_receive;
Referee referee;
Communicate communicate;

uint8_t UART1_Buffer[2][BUFFER_LEN]; //串口1缓存
uint8_t UART6_Buffer[2][BUFFER_LEN]; //串口6缓存

void Communicate::init()
{

// TODO 这里最好使用指针赋值,减少计算量,后续需修改
//#if CHASSIS_REMOTE_OPEN
//    ;
//#else
    remote_control.init();
//#endif

    can_receive.init();

    usart1_init(UART1_Buffer[0], UART1_Buffer[1], BUFFER_LEN);//缓存区间初始化
		usart6_init(UART6_Buffer[0], UART6_Buffer[1], BUFFER_LEN);
	
}
void Communicate::run()
{
			//referee.unpack();
			//referee.determine_ID();
		/*测试电机通讯*/
			//uart_cmd_joint_motor(6, 2,1);
		//	uart_cmd_joint_motor(1, 2,1);
		//TODO _data这里最好使用指针赋值,减少计算量,后续需修改
#if CHASSIS_REMOTE_OPEN
    remote_control.rc_ctrl.rc.ch[0] = can_receive.chassis_receive.ch_0;
    remote_control.rc_ctrl.rc.ch[2] = can_receive.chassis_receive.ch_2;
    remote_control.rc_ctrl.rc.ch[3] = can_receive.chassis_receive.ch_3;
    remote_control.rc_ctrl.key.v = can_receive.chassis_receive.v;
    remote_control.rc_ctrl.rc.s[1] = can_receive.chassis_receive.s1;
#else

#endif
}

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{

    // TODO 设备检测未更新
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (hcan == &CHASSIS_CAN) //接底盘CAN 信息  can2
        {

            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {
            case CAN_RC_BOARM_COM_ID:
                can_receive.receive_rc_board_com(rx_data);
                break;								 
							default:
							{
									break;
							}
            }
        }
        else if (hcan == &BOARD_COM_CAN) //接底盘CAN 信息 can1
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {
							/*----------------------------平衡步兵底盘电机接收-------------------------------------*/
            case CAN_MOTIVE_BCR_MOTOR_ID:
                can_receive.get_balance_motive_motor_measure(MOTIVE_BCR_MOTOR, rx_data);
                break;
            case CAN_MOTIVE_BCL_MOTOR_ID:
                can_receive.get_balance_motive_motor_measure(MOTIVE_BCL_MOTOR, rx_data);
                // detect_hook(CHASSIS_MOTIVE_BR_MOTOR_TOE);
                break;
            case CAN_RC_BOARM_COM_ID:
                can_receive.receive_rc_board_com(rx_data);
                break;

            case CAN_GIMBAL_BOARD_COM_ID:
                can_receive.receive_gimbal_board_com(rx_data);
                break;
            case CAN_UI_COM_ID:
                can_receive.receive_ui_board_com(rx_data);
                break;

            default:
            {
                break;
            }
            }
        }
    }
		
    //遥控器串口
    void USART3_IRQHandler(void)
    {
        if (huart3.Instance->SR & UART_FLAG_RXNE) //接收到数据
        {
            __HAL_UART_CLEAR_PEFLAG(&huart3);
        }
        else if (USART3->SR & UART_FLAG_IDLE)
        {
            static uint16_t this_time_rx_len = 0;

            __HAL_UART_CLEAR_PEFLAG(&huart3);

            if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
            {
                /* Current memory buffer used is Memory 0 */

                // disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                // get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                // reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                // set memory buffer 1
                //设定缓冲区1
                hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

                // enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    remote_control.unpack(0);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                    remote_control.sbus_to_usart1(0);
                }
            }
            else
            {
                /* Current memory buffer used is Memory 1 */
                // disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                // get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                // reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                // set memory buffer 0
                //设定缓冲区0
                DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

                // enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    //处理遥控器数据
                    remote_control.unpack(1);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                    remote_control.sbus_to_usart1(1);
                }
            }
        }
    }

/*------------------------------------关节电机串口中断-----------------------------------------------*/

    void USART1_IRQHandler(void)
    {
        static volatile uint8_t res;
        if (USART1->SR & UART_FLAG_IDLE)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart1);

            static uint16_t this_time_rx_len = 0;

            if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
            {
                __HAL_DMA_DISABLE(huart1.hdmarx);
                this_time_rx_len = BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
                __HAL_DMA_SET_COUNTER(huart1.hdmarx, BUFFER_LEN);
                huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
                __HAL_DMA_ENABLE(huart1.hdmarx);
                //数据处理函数
				can_receive.get_joint_motive_motor_measure(0,UART1_Buffer[0]);                
                memset(UART1_Buffer[0], 0, BUFFER_LEN);
                detect_hook(VISION_TOE);
            }
            else
            {
                __HAL_DMA_DISABLE(huart1.hdmarx);
                this_time_rx_len = BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
                __HAL_DMA_SET_COUNTER(huart1.hdmarx, BUFFER_LEN);
                huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
                __HAL_DMA_ENABLE(huart1.hdmarx);
				//数据处理函数
                can_receive.get_joint_motive_motor_measure(0,UART1_Buffer[1]);
                memset(UART1_Buffer[1], 0, BUFFER_LEN);   
                detect_hook(VISION_TOE);
            }
        }
    }
    //左侧电机  通过UART6通讯
		    void USART6_IRQHandler(void)
    {
        static volatile uint8_t res;
        if (USART6->SR & UART_FLAG_IDLE)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart6);

            static uint16_t this_time_rx_len = 0;

            if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, BUFFER_LEN);
                huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
                __HAL_DMA_ENABLE(huart6.hdmarx);
								//数据处理函数
                can_receive.get_joint_motive_motor_measure(1,UART6_Buffer[0]);
                memset(UART6_Buffer[0], 0, BUFFER_LEN);
                detect_hook(VISION_TOE);
            }
            else
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, BUFFER_LEN);
                huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
                __HAL_DMA_ENABLE(huart6.hdmarx);
								//数据处理函数
                can_receive.get_joint_motive_motor_measure(1,UART6_Buffer[1]);
                memset(UART6_Buffer[1], 0, BUFFER_LEN);   
                detect_hook(VISION_TOE);
            }
        }
    }
    /*串口发送电机函数*/

}
#endif
