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
#include "spi.h"
#include "tim.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_context.h"
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

static App_Context_t app_inst = {
    .hbtn  = &hbtn,
    .hvdac = &hhvdac,
    .hvamp = &hhvamp,
    .hlcd  = &hlcd,
};
App_Context_t * const g_app = &app_inst;

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
  ST7735_Init(g_app->hlcd);
  // DBG_DAC_Init();
  HVDAC_Init(g_app->hvdac); 
  HVAMP_Init(g_app->hvamp); 
  BTN_Init(g_app->hbtn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (HVAMP_ReadFault(g_app->hvamp)) {
        HVAMP_Disable(g_app->hvamp);
      }

    Button_Event_t evt;
    if (BTN_GetEvent(&evt)) {
      // if (current_page != NULL && current_page->OnEvent != NULL) {
      //   current_page->OnEvent(&evt);
      // }
    }
	
	ST7735_Handle_t *lcd = g_app->hlcd;

    /* ---------------------------------------------------------- */
    /* 1. Border check — 逐像素画边框，验证四角和边缘像素是否正确 */
    /* ---------------------------------------------------------- */
    ST7735_FillScreen(lcd, ST7735_BLACK);
    for (int x = 0; x < ST7735_WIDTH; x++) {
        ST7735_DrawPixel(lcd, x, 0,              ST7735_RED);
        ST7735_DrawPixel(lcd, x, ST7735_HEIGHT-1, ST7735_RED);
    }
    for (int y = 0; y < ST7735_HEIGHT; y++) {
        ST7735_DrawPixel(lcd, 0,             y, ST7735_RED);
        ST7735_DrawPixel(lcd, ST7735_WIDTH-1, y, ST7735_RED);
    }
    HAL_Delay(2000);

    /* ---------------------------------------------------------- */
    /* 2. FillRectangle — 几个不同尺寸的填充矩形，含边缘越界情形 */
    /* ---------------------------------------------------------- */
    ST7735_FillScreen(lcd, ST7735_BLACK);
    // 正常矩形
    ST7735_FillRectangle(lcd, 10,  5,  40, 25, ST7735_RED);
    ST7735_FillRectangle(lcd, 60,  5,  40, 25, ST7735_GREEN);
    ST7735_FillRectangle(lcd, 110, 5,  40, 25, ST7735_BLUE);
    // 跨右边界，应只画可见部分
    ST7735_FillRectangle(lcd, ST7735_WIDTH-10, 35, 30, 20, ST7735_YELLOW);
    // 跨下边界
    ST7735_FillRectangle(lcd, 10, ST7735_HEIGHT-10, 40, 30, ST7735_CYAN);
    // 负坐标起点，应从左/上边缘开始画
    ST7735_FillRectangle(lcd, -5, 60, 30, 15, ST7735_MAGENTA);
    HAL_Delay(2000);

    /* ---------------------------------------------------------- */
    /* 3. DrawRectangle — 空心矩形，验证 thick 参数             */
    /* ---------------------------------------------------------- */
    ST7735_FillScreen(lcd, ST7735_BLACK);
    ST7735_DrawRectangle(lcd, 5,   5,  60, 35, 1, ST7735_WHITE);
    ST7735_DrawRectangle(lcd, 80,  5,  60, 35, 3, ST7735_YELLOW);
    ST7735_DrawRectangle(lcd, 5,  50,  60, 25, 5, ST7735_CYAN);
    // thick 超过半宽，应退化为实心矩形
    ST7735_DrawRectangle(lcd, 80, 50,  20, 20, 20, ST7735_RED);
    HAL_Delay(2000);

    /* ---------------------------------------------------------- */
    /* 4. DrawLine — 水平、垂直、斜线，验证 thick               */
    /* ---------------------------------------------------------- */
    ST7735_FillScreen(lcd, ST7735_BLACK);
    // 水平线
    ST7735_DrawLine(lcd, 5,  10, ST7735_WIDTH-5,  10, 1, ST7735_WHITE);
    ST7735_DrawLine(lcd, 5,  20, ST7735_WIDTH-5,  20, 3, ST7735_RED);
    // 垂直线
    ST7735_DrawLine(lcd, 20,  5, 20, ST7735_HEIGHT-5, 1, ST7735_GREEN);
    ST7735_DrawLine(lcd, 35,  5, 35, ST7735_HEIGHT-5, 3, ST7735_BLUE);
    // 对角线
    ST7735_DrawLine(lcd, 50,  5, ST7735_WIDTH-5, ST7735_HEIGHT-5, 1, ST7735_YELLOW);
    ST7735_DrawLine(lcd, 50, 15, ST7735_WIDTH-5, ST7735_HEIGHT-5, 3, ST7735_CYAN);
    // 跨边界线，应被裁剪
    ST7735_DrawLine(lcd, -10, 40, ST7735_WIDTH+10, 40, 1, ST7735_MAGENTA);
    HAL_Delay(2000);

    /* ---------------------------------------------------------- */
    /* 5. FillCircle / DrawCircle                                 */
    /* ---------------------------------------------------------- */
    ST7735_FillScreen(lcd, ST7735_BLACK);
    // 实心圆
    ST7735_FillCircle(lcd, 25, 25, 20, ST7735_RED);
    ST7735_FillCircle(lcd, 80, 25, 15, ST7735_GREEN);
    // 空心圆，不同 thick
    ST7735_DrawCircle(lcd, 25, 60, 18, 1, ST7735_WHITE);
    ST7735_DrawCircle(lcd, 75, 60, 18, 3, ST7735_YELLOW);
    // 圆心在边缘外，测试裁剪
    ST7735_FillCircle(lcd, 0,  0,  15, ST7735_CYAN);
    ST7735_FillCircle(lcd, ST7735_WIDTH, ST7735_HEIGHT, 15, ST7735_MAGENTA);
    HAL_Delay(2000);

    /* ---------------------------------------------------------- */
    /* 6. Font — 三种字体，验证换行和截断                        */
    /* ---------------------------------------------------------- */
    ST7735_FillScreen(lcd, ST7735_BLACK);
    ST7735_WriteString(lcd, 0, 0,
        "Font_7x10\nABCDEFGHIJKLMNOPQRSTUVWXYZ\n0123456789!@#$%",
        Font_7x10, ST7735_WHITE, ST7735_BLACK);
    HAL_Delay(2000);

    ST7735_FillScreen(lcd, ST7735_BLACK);
    ST7735_WriteString(lcd, 0, 0,
        "Font_11x18\nHello World\n0123456789",
        Font_11x18, ST7735_GREEN, ST7735_BLACK);
    HAL_Delay(2000);

    ST7735_FillScreen(lcd, ST7735_BLACK);
    ST7735_WriteString(lcd, 0, 0,
        "Font16x26\nABC\n123",
        Font_16x26, ST7735_BLUE, ST7735_BLACK);
    HAL_Delay(2000);

    /* ---------------------------------------------------------- */
    /* 7. Color fill — 依次填充所有预定义颜色                    */
    /* ---------------------------------------------------------- */
    const struct { uint16_t color; const char *name; uint16_t fg; } colors[] = {
        { ST7735_BLACK,   "BLACK",   ST7735_WHITE   },
        { ST7735_BLUE,    "BLUE",    ST7735_WHITE   },
        { ST7735_RED,     "RED",     ST7735_WHITE   },
        { ST7735_GREEN,   "GREEN",   ST7735_BLACK   },
        { ST7735_CYAN,    "CYAN",    ST7735_BLACK   },
        { ST7735_MAGENTA, "MAGENTA", ST7735_WHITE   },
        { ST7735_YELLOW,  "YELLOW",  ST7735_BLACK   },
        { ST7735_WHITE,   "WHITE",   ST7735_BLACK   },
    };
    for (int i = 0; i < 8; i++) {
        ST7735_FillScreen(lcd, colors[i].color);
        ST7735_WriteString(lcd, 2, 2, colors[i].name, Font_11x18,
                           colors[i].fg, colors[i].color);
        HAL_Delay(500);
    }

    /* ---------------------------------------------------------- */
    /* 8. Brightness — 从暗到亮再到暗，验证 PWM 背光             */
    /* ---------------------------------------------------------- */
    ST7735_FillScreen(lcd, ST7735_WHITE);
    ST7735_WriteString(lcd, 2, 2, "Brightness", Font_11x18, ST7735_BLACK, ST7735_WHITE);
    for (int b = 0; b <= 100; b += 5)  { ST7735_SetBrightness(lcd, b); HAL_Delay(40); }
    for (int b = 100; b >= 0; b -= 5)  { ST7735_SetBrightness(lcd, b); HAL_Delay(40); }
    ST7735_SetBrightness(lcd, 100);
    HAL_Delay(500);

    /* ---------------------------------------------------------- */
    /* 9. InvertColors — 正反色切换                              */
    /* ---------------------------------------------------------- */
    ST7735_FillScreen(lcd, ST7735_BLACK);
    ST7735_FillRectangle(lcd, 10, 10, 40, 40, ST7735_RED);
    ST7735_FillRectangle(lcd, 60, 10, 40, 40, ST7735_GREEN);
    ST7735_WriteString(lcd, 2, 60, "Normal", Font_11x18, ST7735_WHITE, ST7735_BLACK);
    HAL_Delay(1500);
    ST7735_InvertColors(lcd, false);
    ST7735_WriteString(lcd, 2, 60, "Invert", Font_11x18, ST7735_WHITE, ST7735_BLACK);
    HAL_Delay(1500);
    ST7735_InvertColors(lcd, true);
    HAL_Delay(500);

    /* ---------------------------------------------------------- */
    /* 10. Gamma — 四档 gamma 对比                               */
    /* ---------------------------------------------------------- */
    const struct { GammaDef g; const char *name; } gammas[] = {
        { GAMMA_10, "Gamma 1.0" },
        { GAMMA_18, "Gamma 1.8" },
        { GAMMA_22, "Gamma 2.2" },
        { GAMMA_25, "Gamma 2.5" },
    };
    for (int i = 0; i < 4; i++) {
        ST7735_SetGamma(lcd, gammas[i].g);
        ST7735_FillScreen(lcd, ST7735_BLACK);
        // 灰阶梯度条，gamma 差异最明显
        for (int s = 0; s < 8; s++) {
            uint8_t v = s * 32;
            uint16_t gray = ST7735_COLOR565(v, v, v);
            ST7735_FillRectangle(lcd, s * (ST7735_WIDTH/8), 5,
                                 ST7735_WIDTH/8, 30, gray);
        }
        ST7735_WriteString(lcd, 2, 42, gammas[i].name, Font_7x10,
                           ST7735_WHITE, ST7735_BLACK);
        HAL_Delay(1500);
    }
    // 恢复默认 gamma
    ST7735_SetGamma(lcd, GAMMA_22);

    HAL_Delay(1000);
    
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
