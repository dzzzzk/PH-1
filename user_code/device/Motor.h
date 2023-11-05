#ifndef MOTOR_H
#define MOTOR_H

#include "Pid.h"
#include "crc32.h"


/* --------------------------------底盘电机数据结构体---------------------------------- */
//驱动轮电机 9025
typedef struct
{
  uint8_t command;//电机命令
  uint16_t ecd;    //0-16383   实测下来数值不准确
  int16_t speed_rpm;
  int16_t given_current;//电流值-33-33范围-2048~2048
  int8_t temperate;
  uint16_t last_ecd;
} motor_9025_measure_t;

//dji电机 m3508
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

//关节电机A1
typedef struct {  							// 以 4个字节一组排列 ，不然编译器会凑整
    int8_t  		Temp;        		// 电机当前平均温度   
    fp32     T;      				// 当前实际电机输出力矩       7 + 8 描述
    fp32     W;      				// 当前实际电机速度（高速）   8 + 7 描述 
    fp32   	Acc;           	// 电机转子加速度       15+0 描述  惯量较小
    fp32   	Pos;      			// 当前电机位置（主控0点修正，电机关节还是以编码器0点为准） 
}motor_A1_measure_t;

class M3508_motor
{
		public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;


    fp32 accel;
    fp32 speed;
    fp32 speed_set;

    fp32 current_set;
    int16_t current_give;

    void init(const motor_measure_t *motor_measure_);
} ;

//mf9025
class MF9025_motor
{
		public:
    const motor_9025_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    fp32 accel;
    fp32 speed;
    fp32 speed_set;
		fp32 torque;
		fp32 torque_set;
		fp32 omg;

    fp32 current_set;
    int16_t current_give;

    void init(const motor_9025_measure_t *motor_measure_);
};

//gm6020电机
class G6020_motor
{
		public:
    const motor_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;

    uint16_t offset_ecd;  //用户定义的初始中值

    fp32 max_angle; //rad   角度限幅
    fp32 mid_angle; //rad
    fp32 min_angle; //rad

    fp32 angle;
    fp32 angle_set;
    fp32 speed;
    fp32 speed_set;
    fp32 current_set;
    int16_t current_give;

    void init(const motor_measure_t *motor_measure_);
};

//unitree_A1  关节电机
class UnitreeA1_motor_t
{
		public:
    const motor_A1_measure_t *motor_measure;
    //速度环pid和角度环pid, 用户可以选择性开启
    Pid speed_pid;
    Pid angle_pid;
		
    fp32 max_angle; //rad   角度限幅
    fp32 mid_angle; //rad
    fp32 min_angle; //rad
    fp32 angle;
    fp32 torque;   
    fp32 torque_set;
    fp32 angular_velocity;//角速度
    fp32 angular_velocity_set;
    fp32 postion;//位置
    fp32 postion_set;
    fp32 Stiffness_Kd;//位置刚度kp
    fp32 Stiffness_Kd_set;
    fp32 Stiffness_Kw;//位置刚度kd
    fp32 Stiffness_Kw_set;

    void init(const motor_A1_measure_t *motor_measure_);
};
#endif
