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
#include <stdio.h>
#include "can_protocol.h"
#include "can_unpack.h"
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

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ICACHE_Init(void);
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_LPUART1_UART_Init();
  MX_ICACHE_Init();

  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("SolarSense CAN receiver — QSTM U585\r\n");

  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType       = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex  = 0;
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1    = 0x000;  /* accept all standard IDs */
  sFilterConfig.FilterID2    = 0x000;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
      FDCAN_REJECT, FDCAN_REJECT,
      FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_Start(&hfdcan1);
  printf("CAN init OK — listening at 500kbps\r\n");
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    /* PH10 (red): 1 Hz heartbeat blink, independent of CAN traffic */
    static uint32_t last_blink = 0;
    static uint32_t last_rx    = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_blink >= 500) {
      HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
      last_blink = now;
    }
    /* PH14 (green): solid ON while frames are arriving, OFF when the bus goes quiet */
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14,
                      (last_rx != 0 && now - last_rx < 200) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
      FDCAN_RxHeaderTypeDef rxHeader;
      uint8_t rxData[8];
      HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData);
      last_rx = now;  /* PH14 (green): mark RX activity (drives the LED above) */

      uint32_t id   = rxHeader.Identifier;
      uint32_t tick = HAL_GetTick();

      switch (id) {
      case SS_ID_PANEL1: {
        SS_Panel_t p; can_unpack_panel(rxData, &p);
        printf("[%06lu] RX 0x101 P1_REF  V=%umV  I=%ldµA  P=%ldµW\r\n",
               (unsigned long)tick, (unsigned)p.v_mv, (long)p.i_ua, (long)p.p_uw);
        break;
      }
      case SS_ID_PANEL2: {
        SS_Panel_t p; can_unpack_panel(rxData, &p);
        printf("[%06lu] RX 0x102 P2_SOIL V=%umV  I=%ldµA  P=%ldµW\r\n",
               (unsigned long)tick, (unsigned)p.v_mv, (long)p.i_ua, (long)p.p_uw);
        break;
      }
      case SS_ID_PANEL3: {
        SS_Panel_t p; can_unpack_panel(rxData, &p);
        printf("[%06lu] RX 0x103 P3_SOIL V=%umV  I=%ldµA  P=%ldµW\r\n",
               (unsigned long)tick, (unsigned)p.v_mv, (long)p.i_ua, (long)p.p_uw);
        break;
      }
      case SS_ID_UV: {
        SS_UV_t u; can_unpack_uv(rxData, &u);
        printf("[%06lu] RX 0x104 UV      counts=%u\r\n",
               (unsigned long)tick, (unsigned)u.uv_counts);
        break;
      }
      case SS_ID_THERMO: {
        SS_Thermo_t t; can_unpack_thermo(rxData, &t);
        printf("[%06lu] RX 0x201 TC      surface=%.1fC  cold=%.1fC\r\n",
               (unsigned long)tick,
               t.surface_c10 / 10.0f, t.cold_c10 / 10.0f);
        break;
      }
      case SS_ID_BATTERY: {
        SS_Battery_t b; can_unpack_battery(rxData, &b);
        printf("[%06lu] RX 0x202 BAT     %umV  %u%%\r\n",
               (unsigned long)tick, (unsigned)b.bat_mv, (unsigned)b.soc);
        break;
      }
      case SS_ID_SEN66_PM: {
        SS_SEN66_PM_t pm; can_unpack_sen66_pm(rxData, &pm);
        printf("[%06lu] RX 0x301 PM      1.0=%.1f  2.5=%.1f  4=%.1f  10=%.1f ug/m3\r\n",
               (unsigned long)tick,
               pm.pm1_x10 / 10.0f, pm.pm25_x10 / 10.0f,
               pm.pm4_x10 / 10.0f, pm.pm10_x10 / 10.0f);
        break;
      }
      case SS_ID_SEN66_GAS: {
        SS_SEN66_Gas_t g; can_unpack_sen66_gas(rxData, &g);
        printf("[%06lu] RX 0x302 GAS     CO2=%uppm  VOC=%u  NOx=%u\r\n",
               (unsigned long)tick,
               (unsigned)g.co2_ppm, (unsigned)g.voc, (unsigned)g.nox);
        break;
      }
      case SS_ID_BME280: {
        SS_BME280_t b; can_unpack_bme280(rxData, &b);
        printf("[%06lu] RX 0x303 BME     P=%luPa  T=%.2fC\r\n",
               (unsigned long)tick,
               (unsigned long)b.press_pa, b.temp_c100 / 100.0f);
        break;
      }
      case SS_ID_HX94C_RAIN: {
        SS_HX94C_Rain_t h; can_unpack_hx94c_rain(rxData, &h);
        printf("[%06lu] RX 0x304 ENV     RH=%.2f%%  T=%.2fC  rain=%s\r\n",
               (unsigned long)tick,
               h.rh_pct100 / 100.0f, h.temp_c100 / 100.0f,
               h.rain ? "WET" : "DRY");
        break;
      }
      case SS_ID_HEARTBEAT: {
        SS_Heartbeat_t hb; can_unpack_heartbeat(rxData, &hb);
        printf("[%06lu] HB  seq=%lu  flags=0x%02X\r\n",
               (unsigned long)tick, (unsigned long)hb.seq, (unsigned)hb.flags);
        break;
      }
      default:
        printf("[%06lu] RX 0x%03lX  ? [%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
               (unsigned long)tick, (unsigned long)id,
               rxData[0], rxData[1], rxData[2], rxData[3],
               rxData[4], rxData[5], rxData[6], rxData[7]);
        break;
      }
    }
    /* USER CODE END WHILE */
  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;   /* 80MHz / (8×20TQ) = 500kbps */
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* PH10 (red) = 1 Hz heartbeat blink, PH14 (green) = RX-activity LED (both active-HIGH) */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10 | GPIO_PIN_14, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
#ifdef USE_FULL_ASSERT
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
