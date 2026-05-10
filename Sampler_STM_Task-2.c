/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t adc_value = 0;
volatile uint8_t adc_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_value = (uint8_t)HAL_ADC_GetValue(&hadc1);
        adc_ready = 1;
    }
}

uint32_t get_dist(void)
{
    HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim16, 0);
    while (__HAL_TIM_GET_COUNTER(&htim16) < 10);
    HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_COUNTER(&htim16, 0);
    while (HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin) == GPIO_PIN_RESET)
        if (__HAL_TIM_GET_COUNTER(&htim16) > 30000) return 999;

    __HAL_TIM_SET_COUNTER(&htim16, 0);
    while (HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin) == GPIO_PIN_SET)
        if (__HAL_TIM_GET_COUNTER(&htim16) > 30000) return 999;

    return __HAL_TIM_GET_COUNTER(&htim16) / 58;
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim16);
  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t buf[32];
  uint8_t byte;
  uint8_t idx;

  while (1)
  {
    /* USER CODE END WHILE */

    idx = 0;
    memset(buf, 0, sizeof(buf));

    while (1)
    {
        HAL_UART_Receive(&huart1, &byte, 1, HAL_MAX_DELAY);
        if (idx < 32) buf[idx++] = byte;
        if (idx >= 3 &&
            buf[idx-3] == 255 &&
            buf[idx-2] == 254 &&
            buf[idx-1] == 253) break;
    }

    uint8_t  method = buf[0];
    uint8_t  param  = buf[1];

    if (method == 1)
    {
        uint32_t num_samples = (uint32_t)param * 5000;
        adc_ready = 0;

        for (uint32_t i = 0; i < num_samples; i++)
        {
            while (!adc_ready);
            adc_ready = 0;
            HAL_UART_Transmit(&huart1, &adc_value, 1, HAL_MAX_DELAY);
        }

        uint8_t end[3] = {255, 254, 253};
        HAL_UART_Transmit(&huart1, end, 3, HAL_MAX_DELAY);
    }
    else
    {
        uint32_t start_dist   = param - 2;
        uint32_t stop_dist    = param + 2;
        uint32_t sample_count = 0;

        while (get_dist() > start_dist)
            HAL_Delay(60);

        uint8_t start[3] = {123, 124, 125};
        HAL_UART_Transmit(&huart1, start, 3, HAL_MAX_DELAY);

        adc_ready = 0;
        while (1)
        {
            if (adc_ready)
            {
                adc_ready = 0;
                HAL_UART_Transmit(&huart1, &adc_value, 1, HAL_MAX_DELAY);
                sample_count++;
                if (sample_count >= 300)
                {
                    sample_count = 0;
                    if (get_dist() > stop_dist) break;
                }
            }
        }

        uint8_t end[3] = {255, 254, 253};
        HAL_UART_Transmit(&huart1, end, 3, HAL_MAX_DELAY);
        HAL_Delay(500);
    }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
