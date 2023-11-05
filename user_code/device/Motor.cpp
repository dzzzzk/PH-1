#include "Motor.h"

void M3508_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}

void G6020_motor::init(const motor_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
}

void MF9025_motor::init(const motor_9025_measure_t *motor_measure_)
{
    motor_measure = motor_measure_;
} 

void UnitreeA1_motor_t::init(const motor_A1_measure_t *motor_measure_)
{
    //宇树电机返回数据的原则是，发送一次数据，返回一次，因此初始化前需要发送一个0数据。
	  motor_measure = motor_measure_;
}
