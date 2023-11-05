#include "joint_motor.h"
#include "motor.h"
#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"
#ifdef __cplusplus 
extern "C"
{
	
#include "user_lib.h"
	
}
#endif

//关节模块
Joint joint;
void Joint::init()
{
	//todo 初始化null判断

	//发送0数据

	//初始化电机
	for (uint8_t i = 0; i < 4; ++i)
    {
		joint_motor[i].init(can_receive.get_joint_motor_measure_point(i));
	}
	uart_cmd_joint_motor(1,0,0);
	uart_cmd_joint_motor(1,2,0);
	uart_cmd_joint_motor(6,0,0);	
	uart_cmd_joint_motor(6,2,0);

	//将机体y方向的速度初始化为0
	joint.y = 0;

	//数据更新
	feedback_update();
}
void Joint::set_mode()
{
	//暂时为空
}
void Joint::feedback_update()
{
//		//更新电机数据
//		for (uint8_t i = 0; i < 4; ++i)
//    {
//        joint_motor[i].angle = joint_motor[i].postion+INIT_ANGLE;        
//    } 
//		//腿部高度以及整车高度
//		joint.yL = sqrt(pow(l3,2)-pow(l1-l2*cos(joint_motor[3].angle),2))  -l2*sin(joint_motor[3].angle);
//		joint.yR = sqrt(pow(l3,2)-pow(l1-l2*cos(joint_motor[0].angle),2))  -l2*sin(joint_motor[0].angle);
//		joint.y = (joint.yL + joint.yR)/ 2;
//		
//		//y方向加速度
//		joint.y_accel =*(chassis.accel_fliter + INS_ACCEL_Z_ADDRESS_OFFSET);
//		
//		//计算左右末端执行力转换到关节电机的力矩系数
//		vyR = -(l1-l2*cos(joint_motor[0].angle))*l2*sin(joint_motor[0].angle)/sqrt(pow(l3,2)-pow(l1-l2*cos(joint_motor[0].angle),2))-l2*cos(joint_motor[0].angle);
//		vyL = -(l1-l2*cos(joint_motor[3].angle))*l2*sin(joint_motor[3].angle)/sqrt(pow(l3,2)-pow(l1-l2*cos(joint_motor[3].angle),2))-l2*cos(joint_motor[3].angle);
//		//计算y方向速度
//		joint.y = joint.vy + joint.y_accel *JOINTMOTOR_CONTROL_TIME_S;
//		
//		//底盘姿态角度更新
//		joint.chassis_yaw = imu.INS_angle[0];
//		joint.chassis_pitch = imu.INS_angle[2];
//		joint.chassis_roll = imu.INS_angle[1];	
//		//底盘姿态角速度更新
//		joint.chassis_yaw_speed = imu.INS_gyro[0];
//		joint.chassis_pitch_speed = imu.INS_gyro[2];
//		joint.chassis_roll_speed = imu.INS_gyro[1];
}
void Joint::set_contorl()
{
	//先写一个保持固定机体高度的
	joint.y_set = LEY_Y_MID_HEIGHT;
	joint.roll_set = 0;
}
void Joint::solve()
{
//计算y方向和ROLL方向设定值与反馈值的偏差
 	joint.delta_y = joint.y_set - joint.y;
	joint.delta_roll = joint.roll_set - joint.chassis_roll;
	
	//计算左腿和右腿的末端执行力
	joint.FL = (KP_Y * joint.delta_y - KD_Y * joint.vy + G)/2 + (KP_ROLL * joint.delta_roll - KD_ROLL * joint.chassis_roll_speed)/D;
	joint.FR = (KP_Y * joint.delta_y - KD_Y * joint.vy + G)/2 - (KP_ROLL * joint.delta_roll - KD_ROLL * joint.chassis_roll_speed)/D;
	
	//计算每个关节电机的控制力矩
	uint8_t i;
		joint.joint_motor[0].torque_set = 0.5*joint.FR*joint.vyR;
		joint.joint_motor[1].torque_set = -0.5*joint.FL*joint.vyR;
	  joint.joint_motor[2].torque_set = 0.5*joint.FR*joint.vyL;
		joint.joint_motor[3].torque_set = -0.5*joint.FL*joint.vyL;

	//如果DOWN掉，则控制力矩为0

   if (chassis.chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
  {
    for(i=0;i<4;i++)
	  {
		  joint.joint_motor[i].torque_set = 0;
	  }  
	}
}
void Joint::power_ctrl()
{
	//功率控制

}
void Joint::output()
{
	//掉线检测
 	uart_cmd_joint_motor(6,3,0);
	uart_cmd_joint_motor(1,1,0);
	uart_cmd_joint_motor(6,2,0);
	uart_cmd_joint_motor(1,0,0);	
	//发送电机控制扭矩参数（缩腿）
// 	uart_cmd_joint_motor(6,3,0.2);
//	uart_cmd_joint_motor(1,1,0.2);
//	uart_cmd_joint_motor(6,2,-0.2);
//	uart_cmd_joint_motor(1,0,-0.2);

  
//	uart_cmd_joint_motor(6,3,-0.15);
//	uart_cmd_joint_motor(1,1,-0.15);
//	uart_cmd_joint_motor(6,2,0.15);
//	uart_cmd_joint_motor(1,0,0.15); 
/*
	前进方向
	左 ID   2+         1 -         右
          3-	        0 +
	
	*/
// 	uart_cmd_joint_motor(1,0,joint.joint_motor[0].torque_set);//右后
//	uart_cmd_joint_motor(1,1,joint.joint_motor[1].torque_set);//右前
//	uart_cmd_joint_motor(6,2,joint.joint_motor[2].torque_set);//左前
//	uart_cmd_joint_motor(6,3,joint.joint_motor[3].torque_set); //左后 
	vTaskDelay(201);
}
