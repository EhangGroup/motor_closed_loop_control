/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//编码器方向规定：向前为正
//直流电机旋转方向规定：向前为正

int flag=3;

int Direction_motor_A=0,Read_Encoder_A=0,Last_Read_Encoder_A=0,Encoder_A=0;  //电机编码器相关
int Direction_motor_B=0,Read_Encoder_B=0,Last_Read_Encoder_B=0,Encoder_B=0;
int Direction_motor_C=0,Read_Encoder_C=0,Last_Read_Encoder_C=0,Encoder_C=0;
int Direction_motor_D=0,Read_Encoder_D=0,Last_Read_Encoder_D=0,Encoder_D=0;
short Read_Encoder_short_A=0,Read_Encoder_short_B=0,Read_Encoder_short_C=0,Read_Encoder_short_D=0;



int motor_a=0,motor_b=0,motor_c=0,motor_d=0;  //各电机的PWM控制输出

int Target_velocity=20;  //设定速度控制的目标速度为30个脉冲每10ms
int Target_position=8000;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* 求绝对值函数 */
int myabs(long int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


/* 定时器输出PWM函数，控制电机进行旋转 */
void Set_Pwm(int motor_pwm_a,int motor_pwm_b,int motor_pwm_c,int motor_pwm_d)
{	
  if(motor_pwm_a<0)  //MotorA
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, myabs(motor_pwm_a));
	
  if(motor_pwm_b<0)  //MotorB
	{
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	}
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, myabs(motor_pwm_b));
	
  if(motor_pwm_c<0)  //MotorC
	{
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, myabs(motor_pwm_c));
	
  if(motor_pwm_d<0)  //MotorD
	{
		HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, myabs(motor_pwm_d));
}


/* 限幅函数 */
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}


/* PI速度环控制器 */
int Incremental_PI (int Encoder,int Target)
{ 	
   float Kp=200,Ki=1;	
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差=目标值-测量值
	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}


/* PID位置环控制器 */
int Position_PID (int Encoder,int Target)
{ 	
	 float Position_KP=20,Position_KI=0,Position_KD=0;
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}


/* 读取10ms内各编码器的变化量 */
int Read_Encoder(int Encode_x)
{
	short Encoder_NUM;
	
	switch(Encode_x)
	{
		case 1:
		{
			Direction_motor_A = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);	//方向相关，该变量暂未使用
			Read_Encoder_A=__HAL_TIM_GET_COUNTER(&htim2);	
			Read_Encoder_short_A = 	(short)Read_Encoder_A;
			TIM2 -> CNT=0;
			Encoder_NUM = -Read_Encoder_short_A;
			break;
		}
		case 2:
		{
			Direction_motor_B = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);	//方向相关，该变量暂未使用
			Read_Encoder_B=__HAL_TIM_GET_COUNTER(&htim3);	
			Read_Encoder_short_B = 	(short)Read_Encoder_B;
			TIM3 -> CNT=0;
			Encoder_NUM = -Read_Encoder_short_B;
			break;
		}
		case 3:
		{
			Direction_motor_C = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);	//方向相关，该变量暂未使用
			Read_Encoder_C=__HAL_TIM_GET_COUNTER(&htim4);	
			Read_Encoder_short_C = 	(short)Read_Encoder_C;
			TIM4 -> CNT=0;
			Encoder_NUM = Read_Encoder_short_C;
			break;
		}
		case 4:
		{
			Direction_motor_D = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5);	//方向相关，该变量暂未使用
			Read_Encoder_D=__HAL_TIM_GET_COUNTER(&htim5);
			Read_Encoder_short_D = 	(short)Read_Encoder_D;
			TIM5 -> CNT=0;
			Encoder_NUM = Read_Encoder_short_D;
			break;
		}
		default: Encoder_NUM=0;
	}
	return Encoder_NUM;
}




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);	//一定要开启TIM1通道1的捕获中断
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);	//一定要开启TIM1通道2的捕获中断
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);	//一定要开启TIM1通道3的捕获中断
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);	//一定要开启TIM1通道4的捕获中断
	
  __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);	//一定要开启TIM1的更新中断
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);  //MotorA  编码器输入
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);  //MotorA  编码器输入
	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);  //MotorB  编码器输入
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);  //MotorB  编码器输入
	
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);  //MotorC  编码器输入
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);  //MotorC  编码器输入
	
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);  //MotorD  编码器输入
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);  //MotorD  编码器输入
	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); 	 //MotorA\B\C\D  PWM输出
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); 	 //MotorA\B\C\D  PWM输出
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); 	 //MotorA\B\C\D  PWM输出
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); 	 //MotorA\B\C\D  PWM输出
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//		while(flag)
//		{
//			for(int i=0;i<3;i++)
//			{
//				Set_Pwm(1000);
//				HAL_Delay(1000); 
//				flag--;
//			}
//		}

		
		Encoder_A = Read_Encoder(1);
		Encoder_B = Read_Encoder(2);
		Encoder_C = Read_Encoder(3);
		Encoder_D = Read_Encoder(4);

		motor_a=Incremental_PI(Encoder_A,Target_velocity); // PI速度环控制器
    motor_b=Incremental_PI(Encoder_B,Target_velocity); // PI速度环控制器
		motor_c=Incremental_PI(Encoder_C,Target_velocity); // PI速度环控制器
		motor_d=Incremental_PI(Encoder_D,Target_velocity); // PI速度环控制器
		
		
//		motor_a=Position_PID(Read_Encoder_D,Target_position);  //PID位置环控制器

		motor_a=target_limit_int(motor_a,-2000,2000);  //限幅函数
		motor_b=target_limit_int(motor_b,-2000,2000);  //限幅函数
		motor_c=target_limit_int(motor_c,-2000,2000);  //限幅函数
		motor_d=target_limit_int(motor_d,-2000,2000);  //限幅函数
		
		
		//Set_Pwm(motor_a,motor_b,motor_c,motor_d);
		
		HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
