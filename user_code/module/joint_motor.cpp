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

//�ؽ�ģ��
Joint joint;
void Joint::init()
{
	//todo ��ʼ��null�ж�

	//����0����

	//��ʼ�����
	for (uint8_t i = 0; i < 4; ++i)
    {
		joint_motor[i].init(can_receive.get_joint_motor_measure_point(i));
	}
	uart_cmd_joint_motor(1,0,0);
	uart_cmd_joint_motor(1,2,0);
	uart_cmd_joint_motor(6,0,0);	
	uart_cmd_joint_motor(6,2,0);

	//������y������ٶȳ�ʼ��Ϊ0
	joint.y = 0;

	//���ݸ���
	feedback_update();
}
void Joint::set_mode()
{
	//��ʱΪ��
}
void Joint::feedback_update()
{
//		//���µ������
//		for (uint8_t i = 0; i < 4; ++i)
//    {
//        joint_motor[i].angle = joint_motor[i].postion+INIT_ANGLE;        
//    } 
//		//�Ȳ��߶��Լ������߶�
//		joint.yL = sqrt(pow(l3,2)-pow(l1-l2*cos(joint_motor[3].angle),2))  -l2*sin(joint_motor[3].angle);
//		joint.yR = sqrt(pow(l3,2)-pow(l1-l2*cos(joint_motor[0].angle),2))  -l2*sin(joint_motor[0].angle);
//		joint.y = (joint.yL + joint.yR)/ 2;
//		
//		//y������ٶ�
//		joint.y_accel =*(chassis.accel_fliter + INS_ACCEL_Z_ADDRESS_OFFSET);
//		
//		//��������ĩ��ִ����ת�����ؽڵ��������ϵ��
//		vyR = -(l1-l2*cos(joint_motor[0].angle))*l2*sin(joint_motor[0].angle)/sqrt(pow(l3,2)-pow(l1-l2*cos(joint_motor[0].angle),2))-l2*cos(joint_motor[0].angle);
//		vyL = -(l1-l2*cos(joint_motor[3].angle))*l2*sin(joint_motor[3].angle)/sqrt(pow(l3,2)-pow(l1-l2*cos(joint_motor[3].angle),2))-l2*cos(joint_motor[3].angle);
//		//����y�����ٶ�
//		joint.y = joint.vy + joint.y_accel *JOINTMOTOR_CONTROL_TIME_S;
//		
//		//������̬�Ƕȸ���
//		joint.chassis_yaw = imu.INS_angle[0];
//		joint.chassis_pitch = imu.INS_angle[2];
//		joint.chassis_roll = imu.INS_angle[1];	
//		//������̬���ٶȸ���
//		joint.chassis_yaw_speed = imu.INS_gyro[0];
//		joint.chassis_pitch_speed = imu.INS_gyro[2];
//		joint.chassis_roll_speed = imu.INS_gyro[1];
}
void Joint::set_contorl()
{
	//��дһ�����̶ֹ�����߶ȵ�
	joint.y_set = LEY_Y_MID_HEIGHT;
	joint.roll_set = 0;
}
void Joint::solve()
{
//����y�����ROLL�����趨ֵ�뷴��ֵ��ƫ��
 	joint.delta_y = joint.y_set - joint.y;
	joint.delta_roll = joint.roll_set - joint.chassis_roll;
	
	//�������Ⱥ����ȵ�ĩ��ִ����
	joint.FL = (KP_Y * joint.delta_y - KD_Y * joint.vy + G)/2 + (KP_ROLL * joint.delta_roll - KD_ROLL * joint.chassis_roll_speed)/D;
	joint.FR = (KP_Y * joint.delta_y - KD_Y * joint.vy + G)/2 - (KP_ROLL * joint.delta_roll - KD_ROLL * joint.chassis_roll_speed)/D;
	
	//����ÿ���ؽڵ���Ŀ�������
	uint8_t i;
		joint.joint_motor[0].torque_set = 0.5*joint.FR*joint.vyR;
		joint.joint_motor[1].torque_set = -0.5*joint.FL*joint.vyR;
	  joint.joint_motor[2].torque_set = 0.5*joint.FR*joint.vyL;
		joint.joint_motor[3].torque_set = -0.5*joint.FL*joint.vyL;

	//���DOWN�������������Ϊ0

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
	//���ʿ���

}
void Joint::output()
{
	//���߼��
 	uart_cmd_joint_motor(6,3,0);
	uart_cmd_joint_motor(1,1,0);
	uart_cmd_joint_motor(6,2,0);
	uart_cmd_joint_motor(1,0,0);	
	//���͵������Ť�ز��������ȣ�
// 	uart_cmd_joint_motor(6,3,0.2);
//	uart_cmd_joint_motor(1,1,0.2);
//	uart_cmd_joint_motor(6,2,-0.2);
//	uart_cmd_joint_motor(1,0,-0.2);

  
//	uart_cmd_joint_motor(6,3,-0.15);
//	uart_cmd_joint_motor(1,1,-0.15);
//	uart_cmd_joint_motor(6,2,0.15);
//	uart_cmd_joint_motor(1,0,0.15); 
/*
	ǰ������
	�� ID   2+         1 -         ��
          3-	        0 +
	
	*/
// 	uart_cmd_joint_motor(1,0,joint.joint_motor[0].torque_set);//�Һ�
//	uart_cmd_joint_motor(1,1,joint.joint_motor[1].torque_set);//��ǰ
//	uart_cmd_joint_motor(6,2,joint.joint_motor[2].torque_set);//��ǰ
//	uart_cmd_joint_motor(6,3,joint.joint_motor[3].torque_set); //��� 
	vTaskDelay(201);
}
