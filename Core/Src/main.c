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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ----- WF100D sensor (single device, no mux) ----- */
#define WF100D_ADDR8      (0x6D << 1) /* 8-bit I2C address */
#define WF_REG_SPI_CTRL   0x00        /* soft reset */
#define WF_REG_STATUS     0x02        /* bit0 = DRDY */
#define WF_REG_DATA       0x06        /* 0x06..0x08 : 24-bit pressure */
#define WF_REG_TEMP       0x09        /* 0x09..0x0A : 16-bit temperature */
#define WF_REG_CMD        0x30        /* bit3 = Sco, bits2:0 = measurement mode */
#define WF_CMD_START      0x0A        /* combined conversion (010) + Sco (1<<3) */
#define WF_CMD_SOFT_RESET 0x24        /* Soft_reset: sets bits 5 and 2, auto-clears */
#define WF_STATUS_DRDY    0x01

/* Pressure transfer function (from Adafruit_WF100DPZ, tunable for the 200 kPa
 * gauge part):  pressure_kPa = (signed_raw / 2^23) * SCALE + OFFSET
 * Constants held in centi-kPa (x100) so the conversion stays integer-only. */
#define WF_P_DIV       8388608LL      /* 2^23 */
#define WF_P_SCALE_C   25000LL        /* 250.00 kPa */
#define WF_P_OFFSET_C  2500LL         /*  25.00 kPa */

/* Send a null-terminated string over the ST-Link VCP (USART2). */
static void uart_print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

/* Read one measurement (pressure + temperature) from the sensor.
 * Returns HAL_OK on success; *p_raw is sign-extended 24-bit pressure,
 * *t_raw is signed 16-bit temperature (LSB = 1/256 C). */
static HAL_StatusTypeDef wf100d_read(int32_t *p_raw, int16_t *t_raw)
{
  uint8_t cmd = WF_CMD_START;
  uint8_t status;
  uint8_t buf[5];

  /* Start a conversion. */
  if (HAL_I2C_Mem_Write(&hi2c1, WF100D_ADDR8, WF_REG_CMD,
                        I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Poll DRDY (bit0 of Status) with a timeout. */
  uint32_t start = HAL_GetTick();
  do
  {
    if (HAL_GetTick() - start > 100) return HAL_TIMEOUT;
    if (HAL_I2C_Mem_Read(&hi2c1, WF100D_ADDR8, WF_REG_STATUS,
                         I2C_MEMADD_SIZE_8BIT, &status, 1, 100) != HAL_OK)
    {
      return HAL_ERROR;
    }
  } while ((status & WF_STATUS_DRDY) == 0);

  /* Read 24-bit pressure (0x06..0x08) and 16-bit temperature (0x09..0x0A). */
  if (HAL_I2C_Mem_Read(&hi2c1, WF100D_ADDR8, WF_REG_DATA,
                       I2C_MEMADD_SIZE_8BIT, buf, 3, 100) != HAL_OK ||
      HAL_I2C_Mem_Read(&hi2c1, WF100D_ADDR8, WF_REG_TEMP,
                       I2C_MEMADD_SIZE_8BIT, &buf[3], 2, 100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  int32_t p = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
  if (p & 0x800000) p |= 0xFF000000;          /* sign-extend 24 -> 32 */
  *p_raw = p;
  *t_raw = (int16_t)((buf[3] << 8) | buf[4]);  /* signed 16-bit */

  return HAL_OK;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uart_print("\r\n=== WF100D sensor read ===\r\n");

  /* Soft reset so the sensor starts from a known state. */
  uint8_t rst = WF_CMD_SOFT_RESET;
  HAL_I2C_Mem_Write(&hi2c1, WF100D_ADDR8, WF_REG_SPI_CTRL,
                    I2C_MEMADD_SIZE_8BIT, &rst, 1, 100);
  HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    int32_t p_raw;
    int16_t t_raw;
    char line[80];

    if (wf100d_read(&p_raw, &t_raw) == HAL_OK)
    {
      /* Fixed-point print (no float). Pressure via the affine transfer
       * function in centi-kPa; temperature = raw/256 C. */
      long p_ckpa = (long)((int64_t)p_raw * WF_P_SCALE_C / WF_P_DIV + WF_P_OFFSET_C);
      long t_centi = (long)t_raw * 100 / 256;
      snprintf(line, sizeof(line),
               "P=%s%ld.%02ld kPa (raw %ld)  T=%s%ld.%02ld C\r\n",
               p_ckpa < 0 ? "-" : "", labs(p_ckpa) / 100, labs(p_ckpa) % 100,
               (long)p_raw,
               t_centi < 0 ? "-" : "", labs(t_centi) / 100, labs(t_centi) % 100);
      uart_print(line);
    }
    else
    {
      uart_print("read error\r\n");
    }

    HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
