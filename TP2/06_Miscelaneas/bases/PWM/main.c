/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>   // <<-- necesario para bool, true, false

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/* Estado global */
/* Estado global */
/* Estado global */
static bool pwm_enabled = true;

/* Prototipos */
int   uart_readline(char *buf, int maxlen);
int   menu_get_choice(void);
void  menu_print_pwm(void);
void  menu_handle_pwm(int choice);
void  pwm_set_frequency(uint32_t freq_hz, bool use_12bit);
void  pwm_set_duty(uint32_t channel, uint8_t duty_pct);
void  pwm_enable(bool en);

/* ------------------------------------------------------------------------
// Lee hasta '\r' o maxlen-1, ecoa cada carácter y devuelve longitud.
------------------------------------------------------------------------ */
int uart_readline(char *buf, int maxlen) {
    int  idx = 0;
    char c;
    while (1) {
        HAL_UART_Receive(&huart1, (uint8_t*)&c, 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t*)&c, 1, HAL_MAX_DELAY);
        if (c == '\r' || idx >= maxlen - 1) break;
        buf[idx++] = c;
    }
    buf[idx] = '\0';
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    return idx;
}

/* ------------------------------------------------------------------------
// Menú principal: lee la línea completa y convierte a entero.
// Si la cadena es "q", devuelve -1 para salir.
------------------------------------------------------------------------ */
int menu_get_choice(void) {
    char buf[4] = {0};
    uart_readline(buf, sizeof(buf));
    if (buf[0]=='q' && buf[1]=='\0') return -1;
    return atoi(buf);
}

/* ------------------------------------------------------------------------
   PWM Helpers
------------------------------------------------------------------------ */
void pwm_set_frequency(uint32_t freq_hz, bool use_12bit) {
    const uint32_t timer_clk = 72000000;
    if (use_12bit) {
        uint32_t presc = timer_clk / (freq_hz * 4096UL) - 1;
        htim1.Instance->PSC = presc;
        htim1.Instance->ARR = 4095;
    } else {
        htim1.Instance->PSC = 1;
        htim1.Instance->ARR = 36000000UL / freq_hz - 1;
    }
    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
}

void pwm_set_duty(uint32_t channel, uint8_t duty_pct) {
    uint32_t period = htim1.Instance->ARR + 1;
    uint32_t ccr    = (period * duty_pct) / 100;
    __HAL_TIM_SET_COMPARE(&htim1, channel, ccr);
}

void pwm_enable(bool en) {
    if (en) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    } else {
        HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop (&htim1, TIM_CHANNEL_4);
    }
    pwm_enabled = en;
}

/* ------------------------------------------------------------------------
   Menú UART
------------------------------------------------------------------------ */
void menu_print_pwm(void) {
    const char *m =
        "\r\n--- PWM Control Menu ---\r\n"
        "8) PWM1 Frequency (Hz)\r\n"
        "9) PWM1 Duty (%)\r\n"
        "10) PWM2 Frequency (Hz)\r\n"
        "11) PWM2 Duty (%)\r\n"
        "12) Toggle PWM On/Off\r\n"
        "13) One-Shot Pulse (ms)\r\n"
        "14) Show Current Settings\r\n"
        "q) Exit Menu\r\n"
        "Choice: ";
    HAL_UART_Transmit(&huart1, (uint8_t*)m, strlen(m), HAL_MAX_DELAY);
}

void menu_handle_pwm(int choice) {
    char s[16];
    switch (choice) {
        case 8:
            memset(s,0,sizeof(s));
            HAL_UART_Transmit(&huart1,(uint8_t*)"PWM1 Freq (Hz): ",16,HAL_MAX_DELAY);
            uart_readline(s,sizeof(s));
            pwm_set_frequency((uint32_t)atoi(s), true);
            HAL_UART_Transmit(&huart1,(uint8_t*)"OK\r\n",4,HAL_MAX_DELAY);
            break;
        case 9:
            memset(s,0,sizeof(s));
            HAL_UART_Transmit(&huart1,(uint8_t*)"PWM1 Duty %: ",14,HAL_MAX_DELAY);
            uart_readline(s,sizeof(s));
            pwm_set_duty(TIM_CHANNEL_1,(uint8_t)atoi(s));
            HAL_UART_Transmit(&huart1,(uint8_t*)"OK\r\n",4,HAL_MAX_DELAY);
            break;
        case 10:
            memset(s,0,sizeof(s));
            HAL_UART_Transmit(&huart1,(uint8_t*)"PWM2 Freq (Hz): ",16,HAL_MAX_DELAY);
            uart_readline(s,sizeof(s));
            pwm_set_frequency((uint32_t)atoi(s), true);
            HAL_UART_Transmit(&huart1,(uint8_t*)"OK\r\n",4,HAL_MAX_DELAY);
            break;
        case 11:
            memset(s,0,sizeof(s));
            HAL_UART_Transmit(&huart1,(uint8_t*)"PWM2 Duty %: ",14,HAL_MAX_DELAY);
            uart_readline(s,sizeof(s));
            pwm_set_duty(TIM_CHANNEL_4,(uint8_t)atoi(s));
            HAL_UART_Transmit(&huart1,(uint8_t*)"OK\r\n",4,HAL_MAX_DELAY);
            break;
        case 12:
            pwm_enable(!pwm_enabled);
            HAL_UART_Transmit(&huart1,
                (uint8_t*)(pwm_enabled?"Enabled\r\n":"Disabled\r\n"),
                pwm_enabled?8:9,HAL_MAX_DELAY);
            break;
        case 13:
            memset(s,0,sizeof(s));
            HAL_UART_Transmit(&huart1,(uint8_t*)"Pulse width (ms): ",18,HAL_MAX_DELAY);
            uart_readline(s,sizeof(s));
            {
                uint32_t ms = (uint32_t)atoi(s);
                pwm_set_duty(TIM_CHANNEL_1,100);
                pwm_set_duty(TIM_CHANNEL_4,100);
                HAL_Delay(ms);
                pwm_set_duty(TIM_CHANNEL_1,0);
                pwm_set_duty(TIM_CHANNEL_4,0);
            }
            HAL_UART_Transmit(&huart1,(uint8_t*)"Pulse done\r\n",12,HAL_MAX_DELAY);
            break;
        case 14:
            {
                char out[128];
                uint32_t psc   = htim1.Instance->PSC + 1;
                uint32_t arr   = htim1.Instance->ARR + 1;
                uint32_t freq  = 72000000 / (psc * arr);
                uint32_t c1    = __HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_1);
                uint32_t c4    = __HAL_TIM_GET_COMPARE(&htim1,TIM_CHANNEL_4);
                uint8_t d1     = (c1*100)/arr;
                uint8_t d4     = (c4*100)/arr;
                int len = snprintf(out,sizeof(out),
                    "\r\n--- Current PWM Settings ---\r\n"
                    "Enabled    = %s\r\n"
                    "Frequency  = %lu Hz\r\n"
                    "PWM1 Duty  = %u %%\r\n"
                    "PWM2 Duty  = %u %%\r\n"
                    "Resolution = %lu steps\r\n"
                    "-----------------------------\r\n",
                    pwm_enabled?"ON":"OFF",
                    freq,d1,d4,arr
                );
                HAL_UART_Transmit(&huart1,(uint8_t*)out,len,HAL_MAX_DELAY);
            }
            break;
        default:
            HAL_UART_Transmit(&huart1,(uint8_t*)"\r\nInvalid choice\r\n",18,HAL_MAX_DELAY);
    }
}


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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Arranca PWM en TIM1 CH1 (PA8) y CH4 (PA11)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  while (1) {
      menu_print_pwm();
      int choice = menu_get_choice();
      if (choice < 0) break;
      menu_handle_pwm(choice);
  }

  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
  while (1) { __WFI(); }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4095;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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