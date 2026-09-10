/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "global.h"
#include "stdio.h"
#include "stdlib.h"
#include "robot_arm.h"
#include "string.h"
#include "usart.h"
#include "arm_auto.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 100  
uint8_t Serial_RxPacket[RX_BUFFER_SIZE] = {0};  
uint8_t Serial_RxFlag = 0;                      
uint8_t Serial_RxIndex = 0;
uint8_t View_RxByte;   
#define M_PI 3.14159
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static float robot_param1 = 20.0f;  
static float robot_param2 = 0.0f;   
static float robot_param3 = 15.0f;  
static float robot_param4 = 0.0f;   
static float robot_param5 = 0.0f;   
static float robot_param6 = 0.0f;   
static float robot_param7 = 0.0f;   
static float robot_param8 = 0.0f;   
static float robot_param9 = 0.0f;   
static float robot_param10 = 0.0f;  


static uint8_t slider_ctrl_group = 0;

static float robot_param1_temp = 20.0f; 
static float robot_param2_temp = 0.0f;

static uint8_t auto_param1 = 0;  
static uint8_t auto_param2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void remove_brackets(uint8_t *buf);
void Process_Received_Command(void);  
void Process_Key_Command(char *data); 
void Process_Slider_Command(char *data);  
void Process_Joystick_Command(char *data);
void Process_Location_Command(char *data);
void Process_Auto_Command(char *data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch,FILE *f)
{
    HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,100);
    return ch;
}

void remove_brackets(uint8_t *buf)
{
  uint32_t len = strlen((char *)buf);
  if (len >= 2 && buf[0] == '[' && buf[len-1] == ']')
  {
    buf[len-1] = '\0';
    memmove(buf, buf+1, len-1);
  }
}

void Process_Auto_Command(char *data)
{
  
  char *num1_str = strtok(data, ",");
  char *num2_str = strtok(NULL, ",");
  
  
  if (num1_str == NULL || num2_str == NULL)
  {
		uint8_t err_msg1[] = "error\r\n";
    HAL_UART_Transmit(&huart3, err_msg1, sizeof(err_msg1)-1, HAL_MAX_DELAY);
    return;
  }
  
  
  auto_param1 = atoi(num1_str);
  auto_param2 = atoi(num2_str);
	
	uint8_t send_buf[16] = {0};
	
	sprintf((char*)send_buf, "%d,%d", auto_param1, auto_param2);
	HAL_UART_Transmit(&huart3, send_buf, strlen((char*)send_buf), HAL_MAX_DELAY);
	
	
	printf( "%d,%d\r\n", auto_param1, auto_param2);
	
	

	
	
}
void Process_Key_Command(char *data)
{
  char *Name = strtok(data, ",");
  char *Action = strtok(NULL, ",");
  
  if (Name != NULL && Action != NULL)
  {
    
    if (strcmp(Name, "1") == 0 && strcmp(Action, "up") == 0)
    {
      slider_ctrl_group = 1;
    }
    
    else if (strcmp(Name, "2") == 0 && strcmp(Action, "down") == 0)
    {
      slider_ctrl_group = 2;
    }

  }
}

void polar_to_xy(float length, float angle_deg)
{
    float angle_rad = angle_deg * (float)M_PI / 180.0f;

    robot_param1 = length * cosf(angle_rad);  
    robot_param2 = length * sinf(angle_rad);  
}

void Process_Slider_Command(char *data)
{
  char *Axis = strtok(data, ",");   
  char *Value = strtok(NULL, ",");   
  
  if (Axis != NULL && Value != NULL)
  {
    float slider_val = atof(Value);
    float coordinate_val = slider_val;
    uint8_t is_valid = 1;  
    
   
    if (slider_ctrl_group == 1)
    {
      
      if (strcmp(Axis, "6") == 0 || strcmp(Axis, "7") == 0 || 
          strcmp(Axis, "8") == 0 || strcmp(Axis, "9") == 0)
      {
        is_valid = 0;
      }
    }
    else if (slider_ctrl_group == 2)
    {
      
      if (strcmp(Axis, "2") == 0 || strcmp(Axis, "3") == 0 || 
          strcmp(Axis, "4") == 0 || strcmp(Axis, "5") == 0)
      {
        is_valid = 0;
       }
    }
    
    
  
    if (is_valid)
    {
      if (strcmp(Axis, "2") == 0)
      {
				float polar_length = coordinate_val;
				robot_param1_temp= polar_length;
			        
        float polar_angle = robot_param2_temp; 
				
				polar_to_xy(polar_length, polar_angle);
        
       }
      else if (strcmp(Axis, "3") == 0)
      {
        float polar_angle = coordinate_val;
				robot_param2_temp= polar_angle;
				float polar_length = robot_param1_temp;
				polar_to_xy(polar_length, polar_angle);
      }
      else if (strcmp(Axis, "4") == 0)
      {
        robot_param3 = coordinate_val;
      }
      else if (strcmp(Axis, "5") == 0)
      {
        robot_param4 = coordinate_val;
      }
      else if (strcmp(Axis, "6") == 0)
      {
        robot_param5 = coordinate_val;
        pwm_servo_angle_set(&pwm_servos[0],robot_param5,1000);
      }
      else if (strcmp(Axis, "7") == 0)
      {
        robot_param6 = coordinate_val;
        pwm_servo_angle_set(&pwm_servos[1],robot_param6,1000);
      }
      else if (strcmp(Axis, "8") == 0)
      {
        robot_param7 = coordinate_val;
        pwm_servo_angle_set(&pwm_servos[2],robot_param7,1000);
        printf("[slider,8,%.1f] -> Robot Arm Param7 : %.1f\r\n", slider_val, robot_param7);
      }
      else if (strcmp(Axis, "9") == 0)
      {
        robot_param8 = coordinate_val;
        pwm_servo_angle_set(&pwm_servos[3],robot_param8,1000);
      }
      else if (strcmp(Axis, "10") == 0)
      {
        robot_param9 = coordinate_val;
        pwm_servo_angle_set(&pwm_servos[4],robot_param9,1000);
      }
      else if (strcmp(Axis, "11") == 0)
      {
        robot_param10 = coordinate_val;
        pwm_servo_angle_set(&pwm_servos[5],robot_param10,1000);
      }
       if (strcmp(Axis, "2") == 0 || strcmp(Axis, "3") == 0 || 
          strcmp(Axis, "4") == 0 || strcmp(Axis, "5") == 0)
      {
        uint8_t ret = robot_arm_angle_set(
          robot_param1,   
          robot_param2,   
          robot_param3,   
          0,   
          -5.0f,         
          40.0f,           
          1000U           
        );
      }
    }
  }
}


void Process_Joystick_Command(char *data)
{
  char *LH_str = strtok(data, ",");
  char *LV_str = strtok(NULL, ",");
  char *RH_str = strtok(NULL, ",");
  char *RV_str = strtok(NULL, ",");
  
  if (LH_str != NULL && LV_str != NULL && RH_str != NULL && RV_str != NULL)
  {
    float LH = atof(LH_str);
    float LV = atof(LV_str);
    float RH = atof(RH_str);
    float RV = atof(RV_str);
  }
}


void Process_Location_Command(char *data)
{
  char *a_str = strtok(data, ",");
  char *b_str = strtok(NULL, ",");
  char *c_str = strtok(NULL, ",");
  char *d_str = strtok(NULL, ",");
  
  if (a_str == NULL || b_str == NULL || c_str == NULL || d_str == NULL)
  {
    return;
  }
  
  float a = atof(a_str);
  float b = atof(b_str);
  float c = atof(c_str);
  float d = atof(d_str);
  
  robot_param1 = a;
  robot_param2 = b;
  robot_param3 = c;
  robot_param4 = d;
  
   uint8_t ret = robot_arm_angle_set(
    robot_param1,   
    robot_param2,   
    robot_param3,   
    0,   
    -5.0f,         
    40.0f,           
    1000U           
  );

}


void Process_Received_Command(void)
{
  remove_brackets(Serial_RxPacket);
  
  char *Tag = strtok((char *)Serial_RxPacket, ",");
  char *Data = strtok(NULL, "\0");
  
  if (Tag == NULL)
  {
    goto cmd_reset;
  }
  	
	if (strcmp(Tag, "auto") == 0)
  {
    Process_Auto_Command(Data);
  }
	
	
  else if (strcmp(Tag, "key") == 0)
  {
    Process_Key_Command(Data);
  }
  else if (strcmp(Tag, "slider") == 0)
  {
    Process_Slider_Command(Data);
  }
  else if (strcmp(Tag, "joystick") == 0)
  {
    Process_Joystick_Command(Data);
  }
  else if (strcmp(Tag, "location") == 0)
  {
    Process_Location_Command(Data);
  }
  
cmd_reset:
  Serial_RxFlag = 0;
  Serial_RxIndex = 0;
  memset(Serial_RxPacket, 0, RX_BUFFER_SIZE);
  HAL_UART_Receive_IT(&huart3, &Serial_RxPacket[Serial_RxIndex], 1);
}




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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  
	uint8_t connect_msg[] = "connect\r\n"; 
  HAL_UART_Transmit(&huart3, connect_msg, sizeof(connect_msg)-1, HAL_MAX_DELAY);
	robot_arm_init();
  VisionArm_Init();
	pwm_servo_angle_set(&pwm_servos[3], 85.0f, 200);
  pwm_servo_angle_set(&pwm_servos[4], 90.0f, 200);
	pwm_servo_angle_set(&pwm_servos[5], 150.0f, 2000);
	HAL_UART_Receive_IT(&huart3, &Serial_RxPacket[Serial_RxIndex], 1);
  HAL_UART_Receive_IT(&huart2, &View_RxByte, 1);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		
	if (Serial_RxFlag == 1)
    {
      Process_Received_Command();  
    }
    
		  VisionArm_Cmd2_Task(); 
		  VisionArm_ScanTask();
		
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)  
  {
    // Check if reception is complete (newline character or buffer full)
    if (Serial_RxPacket[Serial_RxIndex] == '\n' || Serial_RxIndex >= RX_BUFFER_SIZE - 1)
    {
      // Handle \r\n newline characters
      if (Serial_RxIndex >= 1 && Serial_RxPacket[Serial_RxIndex - 1] == '\r')
      {
        Serial_RxPacket[Serial_RxIndex - 1] = '\0';  // Replace \r with terminator
      }
      else
      {
        Serial_RxPacket[Serial_RxIndex] = '\0';  // Replace \n with terminator
      }
      Serial_RxFlag = 1;  // Set reception complete flag
    }
    else
    {
      Serial_RxIndex++;  // Increment index
      // Continue receiving next byte
      HAL_UART_Receive_IT(&huart3, &Serial_RxPacket[Serial_RxIndex], 1);
    }
  }
	    if (huart == &huart2)
    {
        uint8_t ch = View_RxByte;
            VisionArm_OnUartByte(ch);

        // ��ס��ֱ�Ӷ�������ֽڣ��൱�ڡ��رմ���2��
			HAL_UART_Receive_IT(&huart2, &View_RxByte, 1);
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
