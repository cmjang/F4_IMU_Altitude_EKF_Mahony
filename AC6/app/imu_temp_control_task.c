#include "imu_temp_control_task.h"
#include "BMI088driver.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"
#include "bsp_imu_pwm.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "MahonyAHRS.h"
#include "tim.h"
#define cheat TRUE  //作弊模式 去掉较小的gyro值
#define correct_Time_define 1000    //上电去0飘 1000次取平均
#define temp_times 300       //探测温度阈值


#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定

#define TEMPERATURE_PID_KP 800.0f //kp of temperature control PID 
#define TEMPERATURE_PID_KI 0.02f    //ki of temperature control PID 
#define TEMPERATURE_PID_KD 5.0f    //kd of temperature control PID 

#define TEMPERATURE_PID_MAX_OUT 600.0f  //max out of temperature control PID 
#define TEMPERATURE_PID_MAX_IOUT 600.0f //max iout of temperature control PID 
#define Destination_TEMPERATURE 40.f

extern SPI_HandleTypeDef hspi2;

//task handler 任务句柄
TaskHandle_t INS_task_local_handler;

fp32 gyro[3], accel[3], temp;
float gyro_correct[3]={0};
float roll,pitch,yaw=0;
uint8_t attitude_flag=0;
uint32_t correct_times=0;

//kp, ki,kd three params
const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
//pid struct 
pid_type_def imu_temp_pid;

void Imu_Init()
{
  IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f,0); //ekf初始化
	PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
  Mahony_Init(1000);  //mahony姿态解算初始化
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	while(BMI088_init()){}
    //set spi frequency
//  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
//  if (HAL_SPI_Init(&hspi2) != HAL_OK)
//  {
//    Error_Handler();
//  }
}
/**
  * @brief          bmi088 temperature control 
  * @param[in]      argument: NULL
  * @retval         none
  */
/**
  * @brief          bmi088温度控制
  * @param[in]      argument: NULL
  * @retval         none 
  */
void IMU_Temperature_Ctrl()
{

  uint16_t tempPWM;
  //pid calculate. PID计算
  PID_calc(&imu_temp_pid, temp, Destination_TEMPERATURE);
  if (imu_temp_pid.out < 0.0f)
  {
      imu_temp_pid.out = 0.0f;
  }
  tempPWM = (uint16_t)imu_temp_pid.out;
  htim3.Instance->CCR2=tempPWM;
}

static uint8_t first_mahony=0; 
void INS_Task()
{
    static uint32_t count = 0;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_read(gyro, accel, &temp);
        if(first_mahony==0)
        {
          first_mahony++;
          MahonyAHRSinit(accel[0],accel[1],accel[2],0,0,0);  
        }
        if(attitude_flag==2)  //ekf的姿态解算
        {
          gyro[0]-=gyro_correct[0];   //减去陀螺仪0飘
          gyro[1]-=gyro_correct[1];
          gyro[2]-=gyro_correct[2];
          
          //===========================================================================
            //ekf姿态解算部分
          //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
          IMU_QuaternionEKF_Update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2]);
          //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
          //=============================================================================== 
            
          //=================================================================================
          //mahony姿态解算部分
          //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
          Mahony_update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],0,0,0);
          Mahony_computeAngles(); //角度计算   移植到别的平台需要替换掉对应的arm_atan2_f32 和 arm_asin
          //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
          //=============================================================================
          //ekf获取姿态角度函数
          pitch=Get_Pitch(); //获得pitch
          roll=Get_Roll();//获得roll
          yaw=Get_Yaw();//获得yaw
          //==============================================================================
        }
        else if(attitude_flag==1)   //状态1 开始1000次的陀螺仪0飘初始化
        {
            //gyro correct
            gyro_correct[0]+= gyro[0];
            gyro_correct[1]+= gyro[1];
            gyro_correct[2]+= gyro[2];
            correct_times++;
            if(correct_times>=correct_Time_define)
            {
              gyro_correct[0]/=correct_Time_define;
              gyro_correct[1]/=correct_Time_define;
              gyro_correct[2]/=correct_Time_define;
              attitude_flag=2; //go to 2 state
            }
        }
    }
// temperature control
    if ((count % 10) == 0)
    {
        // 100hz 的温度控制pid
        IMU_Temperature_Ctrl();
        
        static uint32_t temp_Ticks=0;
        if((fabsf(temp-Destination_TEMPERATURE)<0.5f)&&attitude_flag==0) //接近额定温度之差小于0.5° 开始计数
        {
          temp_Ticks++;
          if(temp_Ticks>temp_times)   //计数达到一定次数后 才进入0飘初始化 说明温度已经达到目标
          {
            attitude_flag=1;  //go to correct state
          }
        }
    }
    count++;
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == Accel_INT_Pin)
    {
    }
    else if(GPIO_Pin == Gryo_INT_Pin)
    {
    }
}
