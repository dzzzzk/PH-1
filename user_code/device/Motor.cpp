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
    //��������������ݵ�ԭ���ǣ�����һ�����ݣ�����һ�Σ���˳�ʼ��ǰ��Ҫ����һ��0���ݡ�
	  motor_measure = motor_measure_;
}
