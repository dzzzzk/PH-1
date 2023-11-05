#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "struct_typedef.h"
#include "motor.h"

#define CHASSIS_CAN hcan2
#define BOARD_COM_CAN hcan1

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


/*--------------------------------底盘电机编号-----------------------------------------*/
enum motive_balance_chassis_motor_id_e
{
  //平衡底盘动力电机接收
  MOTIVE_BCR_MOTOR = 0,
  MOTIVE_BCL_MOTOR,
};

/* --------------------------------CAN send and receive ID---------------------------------- */
typedef enum
{
  //平衡底盘驱动轮电机编号  CAN2
	CAN_MOTIVE_BCR_MOTOR_ID = 0x141,
  CAN_MOTIVE_BCL_MOTOR_ID = 0x142,
  CAN_CHASSIS_B_MOTIVE_ALL_ID = 0x280,
  //板间通信ID
  CAN_RC_BOARM_COM_ID = 0x101,
  CAN_GIMBAL_BOARD_COM_ID = 0x102,
  CAN_COOLING_BOARM_COM_ID = 0x303 ,
  CAN_17MM_SPEED_BOARD_COM_ID = 0x304,
  CAN_UI_COM_ID = 0x305,

  //超级电容接收ID
  CAN_SUPER_CAP_ID = 0x211

} can_msg_id_e;

//超级电容数据结构体
typedef struct
{
  float input_vot;     //输入电压
  float cap_vot;       //超级电容电压
  float input_current; //输入电流
  float target_power;  //目标功率
} cap_receive_t;

//底盘接收数据结构体
typedef struct
{
  //遥控器数据
  int16_t ch_0;
  int16_t ch_2;
  int16_t ch_3;
  uint16_t v;

  //云台状态
  uint8_t s1;
  uint8_t gimbal_behaviour;
  fp32 gimbal_yaw_angle;

  // UI状态
  fp32 gimbal_pitch_angle;
  bool_t auto_state;
  bool_t aim_state;
  bool_t fric_state;
} chassis_receive_t;

//底盘发送数据结构体
typedef struct
{
  //测试热量及ID
  uint16_t id1_17mm_cooling_limit;//17mm测速热量上限
  uint16_t id1_17mm_cooling_rate;//17mm测速热量冷却
  uint16_t id1_17mm_cooling_heat; //17mm测速实时热量
  uint8_t color;               //判断红蓝方
  uint8_t robot_id;            //机器人编号

  //测速速度及底盘模式
  uint16_t id1_17mm_speed_limi; // 17mm测速射速上限
  uint16_t bullet_speed;        // 17mm测速实时射速

  uint8_t chassis_behaviour;

  uint8_t game_progress;

} chassis_send_t;
//A1电机数据发送格式
#pragma pack(1)
// STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef __packed struct
{
  // 定义 数据包头
		unsigned char  start[2];     // 包头
		unsigned char  motorID;      // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
		unsigned char  reserved_1;
	// 定义 数据
    uint8_t  mode;        // 关节模式选择
    uint8_t  ModifyBit;   // 电机控制参数修改位
    uint8_t  ReadBit;     // 电机控制参数发送位
    uint8_t  reserved_2;

    uint32_t  Modify;     // 电机参数修改 的数据 
  //实际给FOC的指令力矩为：
  //K_P*delta_Pos + K_W*delta_W + T
    int16_t     T;      // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
    int16_t     W;      // 期望关节速度 （电机本身的速度） x128,       8 + 7描述	
    int32_t   Pos;      // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）
    int16_t    K_P;      // 关节刚度系数 x2048  4+11 描述
    int16_t    K_W;      // 关节速度系数 x1024  5+10 描述
	
    uint8_t LowHzMotorCmdIndex;     // 电机低频率控制命令的索引, 0-7, 分别代表LowHzMotorCmd中的8个字节
    uint8_t LowHzMotorCmdByte;      // 电机低频率控制命令的字节
    uint32_t  Res[1];    // 通讯 保留字节  用于实现别的一些通讯内容
		
		uint32_t		CRC32 ;
} motor_A1_sendmeasure_t;
#pragma pack()



class Can_receive
{
public: 
  //电机反馈数据结构体
  motor_9025_measure_t chassis_blance_motive_motor[2];//9025

  motor_A1_measure_t  chassis_joint_motor[4];//关节电机

  //发送数据结构体
  CAN_TxHeaderTypeDef chassis_tx_message;
  uint8_t chassis_can_send_data[8];

  
  //底盘接收,发送信息
  chassis_receive_t chassis_receive;
  chassis_send_t chassis_send;
  motor_A1_sendmeasure_t motor_A1_sendmeasure[4];
  //超电数据
  cap_receive_t cap_receive;

  //初始化
  void init();

  /*----------------------------------电机数据接收---------------------*/
	
  //平衡底盘数据接收函数
  void get_balance_motive_motor_measure(uint8_t num, uint8_t data[8]);
  //平衡驱动轮电机数据
  void can_cmd_balahnce_chassis_motive_motor(int16_t motor1, int16_t motor2);

  //返回底盘动力电机 
	
  //9025电机数据
  const motor_9025_measure_t *get_chassis_motive_motor_balance_measure_point(uint8_t i);
  const motor_A1_measure_t *get_joint_motor_measure_point(uint8_t i);
  
  void get_joint_motive_motor_measure(int n, uint8_t *ReadFormUart);
  //板间通信函数
  void receive_rc_board_com(uint8_t data[8]);

  void receive_gimbal_board_com(uint8_t data[8]);

  void receive_ui_board_com(uint8_t data[8]);

  // 发送枪口热量及ID
  void send_cooling_and_id_board_com(uint16_t id1_17mm_cooling_limit, uint16_t id1_17mm_cooling_rate, uint16_t id1_17mm_cooling_heat, uint8_t color, uint8_t robot_id);
  //发送枪口速度及底盘模式
  void send_17mm_speed_and_mode_board_com(uint16_t id1_17mm_speed_limi, uint16_t bullet_speed, uint8_t chassis_behaviour,uint8_t temp_game_progress);

  //发送超级电容设定功率
  void can_cmd_super_cap_power(uint16_t set_power);

  // 获取超电输入电压、电容电压、输入电流、设定功率
  void get_super_cap_data(uint8_t data[8]);
};
/*-------------------串口处理函数-------------------------------*/
//A1电机
void get_joint_motive_motor_measure(int n, uint8_t *ReadFormUart);

#ifdef __cplusplus
extern "C" {
#endif
//A1电机发送数据   串口号+电机id+力矩大小
void uart_cmd_joint_motor(int uart_num,uint16_t id,fp32 torque);

#ifdef __cplusplus
};
#endif

#endif
