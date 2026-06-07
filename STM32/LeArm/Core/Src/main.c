/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "global.h"
#include "stdio.h"
#include "stdlib.h"
#include "led.h"
#include "buzzer.h"
#include "ultrasound.h"
#include "robot_arm.h"
#include "wonder_mv.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
RecognitionHanleTypeDef color;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ULTRASOUND_OFFSET 7.6f

/* ===================== Vision calibration TODOs =====================
 * K230 sends display pixel coordinates: x in [0, 320], y in [0, 240].
 * For eye-in-hand, the arm must move to a fixed capture pose first.
 *
 * Tune these values on the real arm:
 * 1. VISION_X_CM_PER_PIXEL / VISION_Y_CM_PER_PIXEL:
 *    Move a block by a known distance on the table and compare pixel change.
 *    If motion direction is reversed, make the value negative.
 * 2. CAMERA_TO_CLAW_*_OFFSET_CM:
 *    When the block is centered in the K230 image, measure how far the claw
 *    center is from the block center on the table.
 * 3. VISION_GRAB_Z_CM and claw angles:
 *    Tune until the claw closes around the 5 cm block without scraping.
 */
#define VISION_IMAGE_WIDTH             320.0f
#define VISION_IMAGE_HEIGHT            240.0f
#define VISION_CENTER_X                (VISION_IMAGE_WIDTH / 2.0f)
#define VISION_CENTER_Y                (VISION_IMAGE_HEIGHT / 2.0f)

#define VISION_CAPTURE_X_CM            15.0f//相机初始位置，一般不用改
#define VISION_CAPTURE_Y_CM             0.0f
#define VISION_CAPTURE_Z_CM            -3.0f

#define VISION_X_CM_PER_PIXEL           0.050f
#define VISION_Y_CM_PER_PIXEL           0.050f
#define CAMERA_TO_CLAW_X_OFFSET_CM      0.0f
#define CAMERA_TO_CLAW_Y_OFFSET_CM      0.0f

#define VISION_APPROACH_Z_CM            4.0f
#define VISION_GRAB_Z_CM               -7.0f
#define VISION_TARGET_MIN_X_CM         10.0f
#define VISION_TARGET_MAX_X_CM         20.0f
#define VISION_TARGET_MIN_Y_CM        -10.0f
#define VISION_TARGET_MAX_Y_CM         10.0f

#define CLAW_OPEN_ANGLE                90.0f
#define CLAW_CLOSE_ANGLE                0.0f

#define PLACE_RED_X_CM                  5.0f
#define PLACE_RED_Y_CM                 12.0f
#define PLACE_GREEN_X_CM                5.0f
#define PLACE_GREEN_Y_CM              -12.0f
#define PLACE_BLUE_X_CM                20.0f
#define PLACE_BLUE_Y_CM                 0.0f
#define PLACE_YELLOW_X_CM              12.0f
#define PLACE_YELLOW_Y_CM              12.0f
#define PLACE_PURPLE_X_CM              12.0f
#define PLACE_PURPLE_Y_CM             -12.0f
#define PLACE_Z_CM                     -5.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t dis;
uint8_t running_state;
uint8_t finish_count;
float target_x;
float target_y;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch,FILE *f)
{
	/* 采用轮询方式发鿿1字节数据，超时时间设置为无限等待 */
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}

static float clamp_float(float value, float min_value, float max_value)
{
	if (value < min_value)
	{
		return min_value;
	}
	if (value > max_value)
	{
		return max_value;
	}
	return value;
}

static void vision_to_arm_target(const RecognitionHanleTypeDef* vision, float* arm_x, float* arm_y)
{
	float pixel_dx = (float)vision->position.x - VISION_CENTER_X;
	float pixel_dy = (float)vision->position.y - VISION_CENTER_Y;

	*arm_x = VISION_CAPTURE_X_CM + pixel_dy * VISION_X_CM_PER_PIXEL + CAMERA_TO_CLAW_X_OFFSET_CM;
	*arm_y = VISION_CAPTURE_Y_CM + pixel_dx * VISION_Y_CM_PER_PIXEL + CAMERA_TO_CLAW_Y_OFFSET_CM;

	*arm_x = clamp_float(*arm_x, VISION_TARGET_MIN_X_CM, VISION_TARGET_MAX_X_CM);
	*arm_y = clamp_float(*arm_y, VISION_TARGET_MIN_Y_CM, VISION_TARGET_MAX_Y_CM);
}

static void move_to_color_place(uint8_t color_id)
{
	switch(color_id)
	{
		case 1:
			robot_arm_coordinate_set(PLACE_RED_X_CM, PLACE_RED_Y_CM, PLACE_Z_CM, 0, -90, 90, 1000);
			break;

		case 2:
			robot_arm_coordinate_set(PLACE_GREEN_X_CM, PLACE_GREEN_Y_CM, PLACE_Z_CM, 0, -90, 90, 1000);
			break;

		case 3:
			robot_arm_coordinate_set(PLACE_BLUE_X_CM, PLACE_BLUE_Y_CM, PLACE_Z_CM, 0, -90, 90, 1000);
			break;

		case 4:
			robot_arm_coordinate_set(PLACE_YELLOW_X_CM, PLACE_YELLOW_Y_CM, PLACE_Z_CM, 0, -90, 90, 1000);
			break;

		case 5:
			robot_arm_coordinate_set(PLACE_PURPLE_X_CM, PLACE_PURPLE_Y_CM, PLACE_Z_CM, 0, -90, 90, 1000);
			break;

		default:
			robot_arm_coordinate_set(VISION_CAPTURE_X_CM, VISION_CAPTURE_Y_CM, VISION_APPROACH_Z_CM, 0, -90, 90, 1000);
			break;
	}
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
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE(); 
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  led_init();
  robot_arm_init();
  buzzer_init();
  ultrasound_init();
  wonder_mv_init();
  robot_arm_coordinate_set(VISION_CAPTURE_X_CM, VISION_CAPTURE_Y_CM, VISION_CAPTURE_Z_CM, 0, -90, 90, 500);
  robot_arm_claw_set(CLAW_OPEN_ANGLE, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	switch(running_state)
	{
		case 0:
			wonder_mv_color_recognition(&color);
			if(color.id != 0)
			{
				HAL_Delay(100);
				wonder_mv_color_recognition(&color);
				if(color.id != 0)
				{
					vision_to_arm_target(&color, &target_x, &target_y);
					robot_arm_coordinate_set(target_x, target_y, VISION_APPROACH_Z_CM, 0, -90, 90, 1000);
					HAL_Delay(1100);
					running_state = 1;
					break;
				}
			}
			HAL_Delay(10);
			break;
			
		case 1:
			robot_arm_coordinate_set(target_x, target_y, VISION_GRAB_Z_CM, 0, -90, 90, 1000);
			HAL_Delay(1100);
			running_state = 2;
			break;
		
		case 2:
			robot_arm_claw_set(CLAW_CLOSE_ANGLE, 200);
			HAL_Delay(200);
			running_state = 3;
			break;
		
		case 3:
			robot_arm_coordinate_set(target_x, target_y, VISION_CAPTURE_Z_CM, 0, -90, 90, 1000);
			HAL_Delay(1100);
			running_state = 4;
			break;
		
		case 4:
			move_to_color_place(color.id);
			memset(&color,0,sizeof(RecognitionHanleTypeDef));
			HAL_Delay(1100);
			running_state = 5;
			robot_arm_claw_set(CLAW_OPEN_ANGLE, 0);
			HAL_Delay(500);
			break;		
		
		case 5:
			robot_arm_coordinate_set(VISION_CAPTURE_X_CM, VISION_CAPTURE_Y_CM, VISION_CAPTURE_Z_CM, 0, -90, 90, 500);
			HAL_Delay(600);
			running_state = 0;
			break;
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
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
