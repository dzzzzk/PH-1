#include "Chassis.h"
#include "Communicate.h"
#include "cmsis_os.h"
#include "arm_math.h"
#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{
	
#include "user_lib.h"
	
}
#endif

//底盘模块
Chassis chassis;

/*-------------------控制变量数据----------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------*/


/**
 * @brief          初始化变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]
 * @retval         none
 */
void Chassis::init()
{
    //获取遥控器指针
    chassis_RC = remote_control.get_remote_control_point();
    last_chassis_RC = remote_control.get_last_remote_control_point();
    chassis_last_key_v = 0;  

    //设置初试状态机
    chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    last_chassis_behaviour_mode = chassis_behaviour_mode;
    chassis_mode = CHASSIS_VECTOR_RAW;
    last_chassis_mode = chassis_mode;

    //初始化底盘电机
    for (uint8_t i = 0; i < 2; ++i)
    {
			//电机数据
			chassis_balance_motive_motor[i].init(can_receive.get_chassis_motive_motor_balance_measure_point(i));
			//初始化pid
			fp32 motive_speed_pid_parm[5] = {MOTIVE_MOTOR_SPEED_PID_KP, MOTIVE_MOTOR_SPEED_PID_KI, MOTIVE_MOTOR_SPEED_PID_KD, MOTIVE_MOTOR_SPEED_PID_MAX_IOUT, MOTIVE_MOTOR_SPEED_PID_MAX_OUT};
			chassis_balance_motive_motor[i].speed_pid.init(PID_SPEED, motive_speed_pid_parm, &chassis_balance_motive_motor[i].speed, &chassis_balance_motive_motor[i].speed_set, NULL);
			chassis_balance_motive_motor[i].speed_pid.pid_clear();
    }
		
		//加速度计的滤波数据
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //用一阶滤波代替斜波函数生成
    chassis_cmd_slow_set_vx.init(CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    chassis_cmd_slow_set_vy.init(CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    /*----------------------------陀螺仪指针-----------------------------*/
    //陀螺仪数据指针获取   
		chassis_INS_angle = imu.get_INS_angle_point();//姿态角	
		accel_fliter = imu.get_gyro_data_point();//角速度指针

    //初始化角度Z轴PID
    fp32 z_angle_pid_parm[5] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT};
    chassis_wz_angle_pid.init(PID_ANGLE, z_angle_pid_parm, &chassis_relative_angle, &chassis_relative_angle_set, NULL);
    chassis_wz_angle_pid.pid_clear();
		
    //速度限幅设置
    Vx.min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    Vx.max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    Vy.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    Vy.max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;    
    Vz.min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z;
    Vz.max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;

    //更新一下数据
    feedback_update();
}

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]
 * @retval         none
 */
void Chassis::set_mode()
{
    chassis_behaviour_mode_set();
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]
 * @retval         none
 */
void Chassis::feedback_update()
{
	//记录上一次遥控器值
	chassis_last_key_v = chassis_RC->key.v;

	//切入跟随云台模式
	if ((last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && (chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW))
	{
		chassis_relative_angle_set = INIT_YAW_SET;
	}
	//切入不跟随云台模式
	else if ((last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && (chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW))
	{
		yaw_set = 0;
	}
	//切入不跟随云台模式
	else if ((last_chassis_mode != CHASSIS_VECTOR_RAW) && (chassis_mode == CHASSIS_VECTOR_RAW))
	{
		yaw_set = 0;
	}

	//更新电机数据
	for (uint8_t i = 0; i < 2; ++i)
	{
	//更新电机的速度、角速度、转矩
    //待修改
        // chassis_balance_motive_motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_balance_motive_motor[i].motor_measure->speed_rpm;
        // chassis_balance_motive_motor[i].omg = CHASSIS_MOTOR_RPM_TO_OMG_SEN * chassis_balance_motive_motor[i].motor_measure->speed_rpm;
        // chassis_balance_motive_motor[i].torque = CHASSIS_MOTOR_9025_CURRENT_TO_TORQUE_SEN * chassis_balance_motive_motor[i].current_give ;
	}
	//更新底盘.右手坐标系
	// x.speed = (chassis_balance_motive_motor[0].speed - chassis_balance_motive_motor[1].speed) /2;
	// omg.speed = (chassis_balance_motive_motor[0].omg - chassis_balance_motive_motor[1].omg) /2;
	//底盘相对于云台的角度,由云台发送过来 编码器中的角度
	//chassis_relative_angle = can_receive.chassis_receive.gimbal_yaw_angle;
}

fp32 move_top_xyz_parm[3] = {1.0, 1.0, 1.3};

/**
 * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @param[out]
 * @retval         none
 */
void Chassis::set_contorl()
{
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;

    //获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set);
	
    //跟随云台模式
    if (chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = sin(-chassis_relative_angle);
        cos_yaw = cos(-chassis_relative_angle);

        x.speed_set = cos_yaw * vx_set + sin_yaw * vy_set;
        y.speed_set = -sin_yaw * vx_set + cos_yaw * vy_set;

        //设置控制相对云台角度
        chassis_relative_angle_set = rad_format(angle_set);

        x.min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
        x.max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
        //速度限幅
        x.speed_set = fp32_constrain(x.speed_set, x.min_speed, x.max_speed);
        y.speed_set = fp32_constrain(y.speed_set, y.min_speed, y.max_speed);
        z.speed_set = fp32_constrain(z.speed_set, z.min_speed, z.max_speed);
    }
    else if (chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //“angle_set” 是旋转速度控制
        z.speed_set = angle_set;
        //速度限幅
        x.speed_set = fp32_constrain(vx_set, x.min_speed, x.max_speed);
        y.speed_set = fp32_constrain(vy_set, y.min_speed, y.max_speed);
    }
    else if (chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //在原始模式，设置值是发送到CAN总线
        Vx.speed_set = vx_set;
        Vy.speed_set = vy_set;
        Vz.speed_set = angle_set;
        chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_cmd_slow_set_vy.out = 0.0f;
    }
    else if (chassis_mode == CHASSIS_BLANCE_NO_MOVE)
    {
        Vx.speed_set = 0.0f;
        angle_set = 0.0f;
    }
}

/*
    平衡步兵解算

*/
void Chassis::chassis_vector_to_mecanum_wheel_speed(fp32 wheel_torque[2])
{
	//解算


}
/**
 * @brief          解算数据,并进行pid计算
 * @brief          平衡底盘对扭矩进行控制
 * @param[out]      
 * @retval         none
 */
void Chassis::solve()
{
    fp32 max_torque = 0.0f, torque_rate = 0.0f;
    fp32 temp = 0.0f;
    uint8_t i = 0;


}


//缓冲能量 单位为J
fp32 chassis_power_buffer = 0.0f;     //裁判剩余缓冲能量
fp32 chassis_power_cap_buffer = 0.0f; //电容剩余能量
fp32 chassis_power = 0.0f;
fp32 chassis_power_limit = 0.0f;
/**
 * @brief          底盘功率控制
 * @param[in]				
 * @retval         none
 */
void Chassis::power_ctrl()
{

}

/**
 * @brief         输出电流
 * @param[in]
 * @retval         none
 */

 
void Chassis::output()
{
	can_receive.can_cmd_balahnce_chassis_motive_motor(0,0);
//平衡发送电流函数
//can_receive.can_cmd_balahnce_chassis_motive_motor(chassis.chassis_balance_motive_motor[0].current_give, chassis.chassis_balance_motive_motor[1].current_give);	
}

/**
 * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种行为模式
 * @param[in]
 * @retval         none
 */
void Chassis::chassis_behaviour_mode_set()
{
    last_chassis_behaviour_mode = chassis_behaviour_mode;
    last_chassis_mode = chassis_mode;

    //遥控器设置模式
    if (switch_is_up(chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])) //右拨杆上 底盘行为 跟随云台
    {
				chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }
    else if (switch_is_mid(chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])) //右拨杆中 底盘行为 自主运动
    {
        chassis_behaviour_mode = CHASSIS_BLANCE;      //保持平衡
    }
    else if (switch_is_down(chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])) //右拨杆下 底盘行为 无力
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }

    //根据行为模式选择一个底盘控制模式
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) // 底盘控制 开环 直接将遥控器杆量转化为电流值 当前逻辑表现为无力
    {
        chassis_mode = CHASSIS_VECTOR_RAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) // 底盘控制 闭环 跟随云台
    {
        chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW) // 底盘控制 闭环 自主运动
    {
        chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_BLANCE)          // 倒立摆形态
    {
        chassis_mode = CHASSIS_BLANCE_NO_MOVE;
    }
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]       包括底盘所有信息.
 * @retval         none
 */
void Chassis::chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN) //底盘控制 测试用 直接将遥控器杆量转化为电流值
    {
        chassis_open_set_control(vx_set, vy_set, angle_set);
    }
    else if (chassis_behaviour_mode == CHASSIS_BLANCE) 
    {
        chassis_no_move_balance_control(vx_set, angle_set);
    }
    last_chassis_RC->key.v = chassis_RC->key.v;
}

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @retval         返回空
 */
void Chassis::chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @retval         返回空
 */
void Chassis::chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @retval         返回空
 */
void Chassis::chassis_no_move_balance_control(fp32 *vx_set,  fp32 *angle_set)
{
    if (vx_set == NULL || angle_set == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *angle_set = 0.0f;
}
/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘与云台控制到的相对角度
 * @retval         返回空
 */
void Chassis::chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
    {
        return;
    }
    chassis_rc_to_control_vector(vx_set, vy_set);
		
    *angle_set = 0;
}

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      数据
 * @retval         返回空
 */
void Chassis::chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{

    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      数据
 * @retval         none
 */
void Chassis::chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }
    *vx_set = chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @retval         none
 */
void Chassis::chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set)
{
    if (vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    //键盘控制
    if (KEY_CHASSIS_FRONT)
    {
        vx_set_channel = x.max_speed;
    }
    else if (KEY_CHASSIS_BACK)
    {
        vx_set_channel = x.min_speed;
    }

    if (KEY_CHASSIS_LEFT)
    {
        vy_set_channel = y.max_speed;
    }
    else if (KEY_CHASSIS_RIGHT)
    {
        vy_set_channel = y.min_speed;
    }

    //一阶低通滤波代替斜波作为底盘速度输入
    chassis_cmd_slow_set_vx.first_order_filter_cali(vx_set_channel);
    chassis_cmd_slow_set_vy.first_order_filter_cali(vy_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_cmd_slow_set_vy.out;
}



/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
  */
fp32 Chassis::motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}
