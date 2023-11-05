#include "INS_task.h"

#define IMU_TASK_INIT_TIME 357

void ins_task(void *pvParameters)
{
  vTaskDelay(IMU_TASK_INIT_TIME);
  imu.init();
  while (1)
  {
    imu.INS_Info_Get();
  }
}

