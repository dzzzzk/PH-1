#include "chassis_task.h"
#include "system_config.h" 
#include "chassis.h"
#include "joint_motor.h"
// uint8_t chassis_flag = 0; 
void chassis_task(void *pvParameters) {

    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //初始化
    chassis.init();
    //主任务循环
    while(true) {
      //设置模式
      chassis.set_mode();
      //反馈数据
      chassis.feedback_update();
      //设置控制量
      chassis.set_contorl();
      //解算
      chassis.solve();
      //功率控制
      chassis.power_ctrl();
      //电流输出
      chassis.output();
      //系统延时
      vTaskDelay(CHASSIS_CONTROL_TIME_MS);
			

    }
}
void joint_task(void *pvParameters) {

    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //初始化
    joint.init();
    //判断电机是否在线
    
    //主任务循环
    while(true) {

      //设置模式
      joint.set_mode();
      //反馈数据
      joint.feedback_update();
      //设置控制量
      joint.set_contorl();
      //解算
      joint.solve();
      //功率控制
      joint.power_ctrl();
      //电流输出
      joint.output();
      //系统延时
      vTaskDelay(2);
    }
}