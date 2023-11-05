/* �ؽڵ�� */
#ifndef JOINT_MOTOR_H
#define JOINT_MOTOR_H

#include "struct_typedef.h"
#include "First_order_filter.h"
#include "Remote_control.h"
#include "Motor.h"
#include "Pid.h"
#include "Config.h"
#include "chassis.h"
/* --------------------�ؽڵ������------------- */
//Y�����˶���KP��KDֵ
#define KP_Y 100.0f
#define KD_Y 40.0f

//ROLL�����˶���KP��KDֵ
#define KP_ROLL 20.0f
#define KD_ROLL 5.0f

//�ؽڵ������λ�Ƕȣ�150�㻻��Ϊ������
#define INIT_ANGLE 2.618
//�������������ʱ�䣬s
#define JOINTMOTOR_CONTROL_TIME_S 0.005
//�Ȳ����˵ĳ��Ȳ���
#define l1 0.0504
#define l2 0.15
#define l3 0.25

//����y�����Roll�����ҡ��ͨ����
#define LEG_Y_CHANNEL 1
#define LEG_ROLL_CHANNEL 0

//ҡ��λ������ӳ�䵽y����߶Ⱥ�Roll����Ƕȵı���ϵ��
#define LEG_Y_RC_SEN 0.0002
#define LEG_ROLL_RC_SEN 0.0002

//y������м�߶�
#define LEY_Y_MID_HEIGHT 0.27

//��߻���������6kg��9.8=58.8��
#define G 40     //

//���ּ��
#define D 0.493

//Y�����˶���KP��KDֵ
#define KP_Y 100.0f
#define KD_Y 40.0f

//ROLL�����˶���KP��KDֵ
#define KP_ROLL 20.0f
#define KD_ROLL 5.0f
class Joint{
	public:
	UnitreeA1_motor_t joint_motor[4];              //�ؽڵ������
	fp32 y;                                    //����߶�
	fp32 yL;                                   //����Ȳ��߶�
	fp32 yR;                                   //�ұ��Ȳ��߶�
	fp32 delta_y;                              //����߶��趨ֵ�뷴��ֵ�Ĳ�ֵ
	fp32 delta_roll;                           //����roll�Ƕ��趨ֵ�뷴��ֵ�Ĳ�ֵ
	fp32 y_set;                                //����߶��趨ֵ
	fp32 roll_set;                             //ROLL��Ƕ��趨ֵ
	fp32 vy;                                   //����y������ٶ�
	fp32 vyL;                                  //����Ȳ���λ��΢��ֵ����������ĩ��ִ����ת���ɹؽڵ������
	fp32 vyR;                                  //�ұ��Ȳ���λ��΢��ֵ����������ĩ��ִ����ת���ɹؽڵ������
	fp32 FL;                                   //����Ȳ���ĩ��ִ����
	fp32 FR;                                   //�ұ��Ȳ���ĩ��ִ����
	fp32 y_accel;                              //�˲����y������ٶ�
	
	fp32 chassis_yaw;                          //���������Ƿ����ĵ�ǰyaw�Ƕ�
  	fp32 chassis_pitch;                        //���������Ƿ����ĵ�ǰpitch�Ƕ�
 	fp32 chassis_roll;                         //���������Ƿ����ĵ�ǰroll�Ƕ�
 	fp32 chassis_yaw_speed;                    //���������Ƿ����ĵ�ǰyaw���ٶ�
	fp32 chassis_pitch_speed;                  //���������Ƿ����ĵ�ǰpitch���ٶ�
	fp32 chassis_roll_speed;                   //���������Ƿ����ĵ�ǰroll���ٶ�
	//��������
    void init();

    void set_mode();

    void feedback_update();

    void set_contorl();

    void solve();

    void power_ctrl();

    void output();
};

extern Joint joint;
extern INS imu;
extern Can_receive can_receive;
extern Chassis chassis;
#endif
