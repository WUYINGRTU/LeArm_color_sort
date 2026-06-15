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
#include "robot_arm.h"
#include "wonder_mv.h"
#include "ps2_porting.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	TRACK_STATE_DETECTION = 0,
	TRACK_STATE_TRACK_Y,
	TRACK_STATE_TRACK_X,
	TRACK_STATE_APPROACH,
	TRACK_STATE_GRAB_DOWN,
	TRACK_STATE_CLAW_CLOSE,
	TRACK_STATE_LIFT,
	TRACK_STATE_RETURN
} TrackStateTypeDef;

typedef enum
{
	TARGET_COLOR = 0,
	TARGET_NUMBER
} TargetKindTypeDef;

RecognitionHanleTypeDef color_result;
RecognitionHanleTypeDef number_result;
RecognitionHanleTypeDef vision_result;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COLOR_ID_RED                    1U
#define COLOR_ID_GREEN                  2U
#define COLOR_ID_BLUE                   3U

#define KEY1_DEBOUNCE_MS               50U
#define LED1_FLASH_ON_MS              300U
#define LED1_FLASH_OFF_MS             300U

#define VISION_CAPTURE_X_CM            15.0f
#define VISION_CAPTURE_Y_CM             0.0f
#define VISION_CAPTURE_Z_CM            -3.0f

#define VISION_X_CM_PER_PIXEL          -0.100f
#define VISION_Y_CM_PER_PIXEL          -0.050f
#define CAMERA_TO_CLAW_X_OFFSET_CM       7.0f //夹爪偏移量已修正，勿更改
#define CAMERA_TO_CLAW_Y_OFFSET_CM       0.0f

#define VISION_APPROACH_Z_CM            2.0f
#define VISION_GRAB_Z_CM              -14.0f
#define VISION_TARGET_MIN_X_CM         10.0f
#define VISION_TARGET_MAX_X_CM         20.0f
#define VISION_TARGET_MIN_Y_CM        -10.0f
#define VISION_TARGET_MAX_Y_CM         10.0f

#define TRACK_CENTER_X                160.0f
#define TRACK_CENTER_Y                120.0f
#define TRACK_DEADBAND_X_PX             8.0f
#define TRACK_DEADBAND_Y_PX             8.0f
#define TRACK_STABLE_FRAMES             3U
#define TRACK_LOST_MAX_FRAMES          10U
#define TRACK_CORRECTION_GAIN           0.30f
#define TRACK_MAX_STEP_X_CM             0.5f
#define TRACK_MAX_STEP_Y_CM             0.5f

#define CLAW_OPEN_ANGLE                90.0f
#define CLAW_CLOSE_ANGLE                0.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TrackStateTypeDef fsm_state = TRACK_STATE_DETECTION;
TargetKindTypeDef current_target = TARGET_COLOR;
uint8_t selected_color_id = COLOR_ID_RED;
uint8_t stable_count;
uint8_t lost_count;
uint8_t last_chassis_mode;
uint8_t key1_last_raw_state = GPIO_PIN_SET;
uint8_t key1_stable_state = GPIO_PIN_SET;
uint32_t key1_last_change_tick;
float target_x;
float target_y;
float grab_x;
float grab_y;
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
	if(value < min_value)
	{
		return min_value;
	}
	if(value > max_value)
	{
		return max_value;
	}
	return value;
}

static void clear_recognition_result(RecognitionHanleTypeDef* result)
{
	result->id = 0;
	result->position.x = 0;
	result->position.y = 0;
	result->position.w = 0;
	result->position.h = 0;
}

static void reset_tracking_state(void)
{
	clear_recognition_result(&color_result);
	clear_recognition_result(&number_result);
	clear_recognition_result(&vision_result);
	stable_count = 0;
	lost_count = 0;
	target_x = VISION_CAPTURE_X_CM;
	target_y = VISION_CAPTURE_Y_CM;
	grab_x = VISION_CAPTURE_X_CM;
	grab_y = VISION_CAPTURE_Y_CM;
}

static void reset_tracking_sequence(void)
{
	current_target = TARGET_COLOR;
	reset_tracking_state();
}

static void move_to_capture_pose(uint32_t time_ms)
{
	robot_arm_coordinate_set(VISION_CAPTURE_X_CM, VISION_CAPTURE_Y_CM, VISION_CAPTURE_Z_CM, 0, -90, 90, time_ms);
}

static void update_grab_target(void)
{
	grab_x = clamp_float(target_x + CAMERA_TO_CLAW_X_OFFSET_CM, VISION_TARGET_MIN_X_CM, VISION_TARGET_MAX_X_CM);
	grab_y = clamp_float(target_y + CAMERA_TO_CLAW_Y_OFFSET_CM, VISION_TARGET_MIN_Y_CM, VISION_TARGET_MAX_Y_CM);
}

static void key1_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStruct.Pin = KEY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

	key1_last_raw_state = (uint8_t)HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
	key1_stable_state = key1_last_raw_state;
	key1_last_change_tick = HAL_GetTick();
}

static void update_selected_color_led(void)
{
	switch(selected_color_id)
	{
		case COLOR_ID_RED:
			led_on(2);
			break;

		case COLOR_ID_GREEN:
			led_flash(2, LED1_FLASH_ON_MS, LED1_FLASH_OFF_MS, 0);
			break;

		case COLOR_ID_BLUE:
			led_off(2);
			break;

		default:
			selected_color_id = COLOR_ID_RED;
			led_on(2);
			break;
	}
}

static void select_next_color(void)
{
	switch(selected_color_id)
	{
		case COLOR_ID_RED:
			selected_color_id = COLOR_ID_GREEN;
			break;

		case COLOR_ID_GREEN:
			selected_color_id = COLOR_ID_BLUE;
			break;

		default:
			selected_color_id = COLOR_ID_RED;
			break;
	}

	update_selected_color_led();
}

static void scan_key1_color_select(void)
{
	uint8_t raw_state = (uint8_t)HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
	uint32_t now = HAL_GetTick();

	if(raw_state != key1_last_raw_state)
	{
		key1_last_raw_state = raw_state;
		key1_last_change_tick = now;
	}

	if((now - key1_last_change_tick) >= KEY1_DEBOUNCE_MS && raw_state != key1_stable_state)
	{
		key1_stable_state = raw_state;
		if(key1_stable_state == GPIO_PIN_RESET)
		{
			select_next_color();
		}
	}
}

static uint8_t is_valid_number_id(uint8_t id)
{
	return id >= 1U && id <= 5U;
}

static uint8_t read_current_target(void)
{
	if(ps2_is_chassis_mode())
	{
		return 0;
	}

	if(!wonder_mv_color_number_recognition(&color_result, &number_result))
	{
		return 0;
	}

	if(current_target == TARGET_COLOR)
	{
		if(color_result.id != selected_color_id)
		{
			return 0;
		}

		vision_result = color_result;
		return 1;
	}

	if(is_valid_number_id(number_result.id))
	{
		vision_result = number_result;
		return 1;
	}

	return 0;
}

static void advance_to_next_target(void)
{
	if(current_target == TARGET_COLOR)
	{
		current_target = TARGET_NUMBER;
	}
	else
	{
		current_target = TARGET_COLOR;
	}

	reset_tracking_state();
}

static void handle_tracking_loss(void)
{
	stable_count = 0;
	lost_count++;

	if(lost_count >= TRACK_LOST_MAX_FRAMES)
	{
		reset_tracking_state();
		move_to_capture_pose(500);
		HAL_Delay(600);
		fsm_state = TRACK_STATE_DETECTION;
	}
	else
	{
		HAL_Delay(20);
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
  key1_gpio_init();
  update_selected_color_led();
  robot_arm_init();
  buzzer_init();
  wonder_mv_init();
  ps2_init();
  reset_tracking_sequence();
  move_to_capture_pose(500);
  robot_arm_claw_set(CLAW_OPEN_ANGLE, 0);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint8_t chassis_mode;

	ps2_handler();
	chassis_mode = ps2_is_chassis_mode();
	if(chassis_mode)
	{
		if(!last_chassis_mode)
		{
			i2c1_recover();
			reset_tracking_sequence();
			fsm_state = TRACK_STATE_DETECTION;
			last_chassis_mode = 1;
		}
		HAL_Delay(10);
		continue;
	}

	if(last_chassis_mode)
	{
		i2c1_recover();
		HAL_Delay(100);
		reset_tracking_sequence();
		fsm_state = TRACK_STATE_DETECTION;
		last_chassis_mode = 0;
	}

	switch(fsm_state)
	{
	  case TRACK_STATE_DETECTION:
		  if(current_target == TARGET_COLOR)
		  {
			  scan_key1_color_select();
		  }
		  if(read_current_target())
		  {
			  HAL_Delay(100);
			  if(read_current_target())
			  {
				  target_x = VISION_CAPTURE_X_CM;
				  target_y = VISION_CAPTURE_Y_CM;
				  stable_count = 0;
				  lost_count = 0;
				  fsm_state = TRACK_STATE_TRACK_Y;
			  }
		  }
		  HAL_Delay(10);
		  break;

	  case TRACK_STATE_TRACK_Y:
	  {
		  float pixel_dx;
		  float step_y;

		  if(!read_current_target())
		  {
			  handle_tracking_loss();
			  break;
		  }

		  lost_count = 0;
		  pixel_dx = (float)vision_result.position.x - TRACK_CENTER_X;

		  if(pixel_dx >= -TRACK_DEADBAND_X_PX && pixel_dx <= TRACK_DEADBAND_X_PX)
		  {
			  stable_count++;
			  if(stable_count >= TRACK_STABLE_FRAMES)
			  {
				  stable_count = 0;
				  fsm_state = TRACK_STATE_TRACK_X;
			  }
			  HAL_Delay(20);
			  break;
		  }

		  stable_count = 0;
		  step_y = pixel_dx * VISION_Y_CM_PER_PIXEL * TRACK_CORRECTION_GAIN;
		  step_y = clamp_float(step_y, -TRACK_MAX_STEP_Y_CM, TRACK_MAX_STEP_Y_CM);
		  target_y = clamp_float(target_y + step_y, VISION_TARGET_MIN_Y_CM, VISION_TARGET_MAX_Y_CM);
		  robot_arm_coordinate_set(target_x, target_y, VISION_CAPTURE_Z_CM, 0, -90, 90, 80);
		  HAL_Delay(90);
		  break;
	  }

	  case TRACK_STATE_TRACK_X:
	  {
		  float pixel_dy;
		  float step_x;

		  if(!read_current_target())
		  {
			  handle_tracking_loss();
			  break;
		  }

		  lost_count = 0;
		  pixel_dy = (float)vision_result.position.y - TRACK_CENTER_Y;

		  if(pixel_dy >= -TRACK_DEADBAND_Y_PX && pixel_dy <= TRACK_DEADBAND_Y_PX)
		  {
			  stable_count++;
			  if(stable_count >= TRACK_STABLE_FRAMES)
			  {
				  update_grab_target();
				  fsm_state = TRACK_STATE_APPROACH;
			  }
			  HAL_Delay(20);
			  break;
		  }

		  stable_count = 0;
		  step_x = pixel_dy * VISION_X_CM_PER_PIXEL * TRACK_CORRECTION_GAIN;
		  step_x = clamp_float(step_x, -TRACK_MAX_STEP_X_CM, TRACK_MAX_STEP_X_CM);
		  target_x = clamp_float(target_x + step_x, VISION_TARGET_MIN_X_CM, VISION_TARGET_MAX_X_CM);
		  robot_arm_coordinate_set(target_x, target_y, VISION_CAPTURE_Z_CM, 0, -90, 90, 80);
		  HAL_Delay(90);
		  break;
	  }

	  case TRACK_STATE_APPROACH:
		  robot_arm_coordinate_set(grab_x, grab_y, VISION_APPROACH_Z_CM, 0, -90, 90, 1000);
		  HAL_Delay(1100);
		  fsm_state = TRACK_STATE_GRAB_DOWN;
		  break;

	  case TRACK_STATE_GRAB_DOWN:
		  robot_arm_coordinate_set(grab_x, grab_y, VISION_GRAB_Z_CM, 0, -90, 90, 1000);
		  HAL_Delay(1100);
		  fsm_state = TRACK_STATE_CLAW_CLOSE;
		  break;

	  case TRACK_STATE_CLAW_CLOSE:
		  robot_arm_claw_set(CLAW_CLOSE_ANGLE, 200);
		  HAL_Delay(300);
		  fsm_state = TRACK_STATE_LIFT;
		  break;

	  case TRACK_STATE_LIFT:
		  robot_arm_coordinate_set(grab_x, grab_y, VISION_CAPTURE_Z_CM, 0, -90, 90, 1000);
		  HAL_Delay(1100);
		  fsm_state = TRACK_STATE_RETURN;
		  break;

	  case TRACK_STATE_RETURN:
		  robot_arm_claw_set(CLAW_OPEN_ANGLE, 0);
		  HAL_Delay(500);
		  move_to_capture_pose(500);
		  HAL_Delay(600);
		  advance_to_next_target();
		  fsm_state = TRACK_STATE_DETECTION;
		  break;

	  default:
		  reset_tracking_sequence();
		  move_to_capture_pose(500);
		  HAL_Delay(600);
		  fsm_state = TRACK_STATE_DETECTION;
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
