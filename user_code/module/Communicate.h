#ifndef COMMUNICATe_H
#define COMMUNICATe_H

#include "cmsis_os.h"
#include "main.h"
#include "Remote_control.h"
#include "Can_receive.h"
#include "Referee.h"
#include "Config.h"
#include "bsp_usart.h"

class Communicate
{
public:
    void init();
    void run();
};

#define BUFFER_LEN 200
extern Remote_control remote_control;
extern Can_receive can_receive;
extern Communicate communicate;

extern uint8_t CmdID;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
#endif
