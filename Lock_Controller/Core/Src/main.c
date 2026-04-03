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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include "app_hardware.h"
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

/* USER CODE BEGIN PV */

HV_DAC_Handle_t hhvdac = {
    .spi      = SPI2,
    .cs_port  = DAC_CS_GPIO_Port,
    .cs_pin   = DAC_CS_Pin,
    .max_voltage = 100.0f,
};        

HVAMP_Handle_t hhvamp = {
    .en_port    = HV_AMP_EN_GPIO_Port,
    .en_pin     = HV_AMP_EN_Pin,
    .fault_port = HV_AMP_FAULT_GPIO_Port,
    .fault_pin  = HV_AMP_FAULT_Pin,
};

const Button_Config_t btn_map[] = {
    {GPIOB, GPIO_KEY1_Pin, BTN_ID_DOWN},
    {GPIOB, GPIO_KEY2_Pin, BTN_ID_UP},
    {GPIOB, GPIO_KEY3_Pin, BTN_ID_ENTER}
};

Button_Handle_t hbtn = {
    .configs = btn_map,
    .count   = 3,
    .htim    = &htim15,
};

ST7735_Handle_t hlcd = {
    .spi         = &hspi3,
    .cs_port     = LCD_CS_GPIO_Port,   .cs_pin  = LCD_CS_Pin,
    .dc_port     = LCD_DC_GPIO_Port,   .dc_pin  = LCD_DC_Pin,
    .res_port    = LCD_RST_GPIO_Port,  .res_pin = LCD_RST_Pin,
    .blk_timer   = &htim16,
    .blk_channel = TIM_CHANNEL_1,
};

PDADC_Handle_t hpdadc = {
    .hadc_master = &hadc1,
    .hadc_slave  = &hadc2,
    .htim_trig   = &htim1,
    .busy        = 0U,
};

static AppHardware_t hw_inst = {
    .hbtn  = &hbtn,
    .hvdac = &hhvdac,
    .hvamp = &hhvamp,
    .hlcd  = &hlcd,
    .hpdadc = &hpdadc,
};
AppHardware_t * const g_hw = &hw_inst;

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
  MX_DMA_Init();
  MX_USB_PCD_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  PDADC_Init(g_hw->hpdadc);
  ST7735_Init(g_hw->hlcd);
  // DBG_DAC_Init();
  HVDAC_Init(g_hw->hvdac); 
  HVAMP_Init(g_hw->hvamp); 
  BTN_Init(g_hw->hbtn);
  APP_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (HVAMP_ReadFault(g_hw->hvamp)) {
        HVAMP_Disable(g_hw->hvamp);
        APP_SetFault(APP_FAULT_HVAMP);
    }

    Button_Event_t evt;
    while (BTN_GetEvent(&evt)) {
      APP_OnButton(evt);
    }

    APP_Process();
    APP_RenderIfNeeded();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM15) {
        BTN_Process(&hbtn);
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
