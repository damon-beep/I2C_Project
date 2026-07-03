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
#include "spi.h"
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

/* ----- WF100D sensor (single device, 3-wire half-duplex SPI) -----
 * CSB is tied to GND, so the STM32 does not drive a chip-select (NSS is
 * software). Register access frames a command byte then data on one line.
 *
 * ASSUMPTION (not in the datasheet pages I have): the command byte carries
 * the register address in bits[6:0], with bit7 = 1 for READ, 0 for WRITE.
 * If reads return garbage, flip WF_SPI_READ (try 0x00) or the SPI mode in
 * spi.c (SPI_PHASE_2EDGE / SPI_POLARITY_HIGH = mode 3). */
#define WF_SPI_READ       0x80        /* OR into reg addr to request a read */

/* Chip-select. 3-wire SPI needs CS to toggle per transaction, so CSB must be
 * wired to this GPIO (NOT tied to GND). Pick any free pin; PA4 is unused. */
#define WF_IO_PORT        GPIOA
#define WF_CS_PIN         GPIO_PIN_4    /* PA4  chip select      */
#define WF_SCK_PIN        GPIO_PIN_5    /* PA5  clock            */
#define WF_SDIO_PIN       GPIO_PIN_7    /* PA7  bidir data       */
#define WF_CS_LOW()       HAL_GPIO_WritePin(WF_IO_PORT, WF_CS_PIN, GPIO_PIN_RESET)
#define WF_CS_HIGH()      HAL_GPIO_WritePin(WF_IO_PORT, WF_CS_PIN, GPIO_PIN_SET)
#define WF_SCK_HIGH()     HAL_GPIO_WritePin(WF_IO_PORT, WF_SCK_PIN, GPIO_PIN_SET)
#define WF_SCK_LOW()      HAL_GPIO_WritePin(WF_IO_PORT, WF_SCK_PIN, GPIO_PIN_RESET)

#define WF_REG_SPI_CTRL   0x00        /* soft reset */
#define WF_REG_PART_ID    0x01        /* OTP part id (non-zero) -> link check */
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

/* Write one register: command byte (reg, write) followed by the value.
 * Half-duplex, so a single transmit of both bytes. */
/* --- Bit-banged 3-wire SPI, mode 0 (CPOL=0/CPHA=0), MSB first, on the same
 * pins as SPI1 (PA5=SCK, PA7=SDIO). Gives exact control of the clock count and
 * line turnaround, avoiding the HAL 1-line half-duplex over-run. --- */

/* Set the bidirectional data pin to output or input. */
static void sdio_dir(uint32_t mode)   /* GPIO_MODE_OUTPUT_PP | GPIO_MODE_INPUT */
{
  GPIO_InitTypeDef g = {0};
  g.Pin = WF_SDIO_PIN;
  g.Mode = mode;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WF_IO_PORT, &g);
}

/* Clock one byte out (data set while clock low, sampled by slave on rising). */
static void spi_out_byte(uint8_t b)
{
  for (int i = 7; i >= 0; i--)
  {
    HAL_GPIO_WritePin(WF_IO_PORT, WF_SDIO_PIN,
                      (b & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    WF_SCK_HIGH();
    WF_SCK_LOW();
  }
}

/* Clock one byte in (sampled on the rising edge, MSB first). */
static uint8_t spi_in_byte(void)
{
  uint8_t b = 0;
  for (int i = 7; i >= 0; i--)
  {
    WF_SCK_HIGH();
    if (HAL_GPIO_ReadPin(WF_IO_PORT, WF_SDIO_PIN) == GPIO_PIN_SET) b |= (1 << i);
    WF_SCK_LOW();
  }
  return b;
}

static HAL_StatusTypeDef wf_write_reg(uint8_t reg, uint8_t val)
{
  WF_CS_LOW();
  sdio_dir(GPIO_MODE_OUTPUT_PP);
  spi_out_byte((uint8_t)(reg & 0x7F));
  spi_out_byte(val);
  WF_CS_HIGH();
  return HAL_OK;
}

/* Send the read command byte, turn the line around, then clock in n bytes,
 * all inside one CS-low frame. */
static HAL_StatusTypeDef wf_read_regs(uint8_t reg, uint8_t *buf, uint16_t n)
{
  WF_CS_LOW();
  sdio_dir(GPIO_MODE_OUTPUT_PP);
  spi_out_byte((uint8_t)(reg | WF_SPI_READ));
  sdio_dir(GPIO_MODE_INPUT);
  for (uint16_t k = 0; k < n; k++) buf[k] = spi_in_byte();
  WF_CS_HIGH();
  return HAL_OK;
}

/* Read one measurement (pressure + temperature) from the sensor.
 * Returns HAL_OK on success; *p_raw is sign-extended 24-bit pressure,
 * *t_raw is signed 16-bit temperature (LSB = 1/256 C). */
static HAL_StatusTypeDef wf100d_read(int32_t *p_raw, int16_t *t_raw)
{
  uint8_t status;
  uint8_t buf[5];

  /* Start a conversion. */
  if (wf_write_reg(WF_REG_CMD, WF_CMD_START) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Poll DRDY (bit0 of Status) with a timeout. */
  uint32_t start = HAL_GetTick();
  do
  {
    if (HAL_GetTick() - start > 100) return HAL_TIMEOUT;
    if (wf_read_regs(WF_REG_STATUS, &status, 1) != HAL_OK)
    {
      return HAL_ERROR;
    }
  } while ((status & WF_STATUS_DRDY) == 0);

  /* Read 24-bit pressure (0x06..0x08) and 16-bit temperature (0x09..0x0A). */
  if (wf_read_regs(WF_REG_DATA, buf, 3) != HAL_OK ||
      wf_read_regs(WF_REG_TEMP, &buf[3], 2) != HAL_OK)
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
  /* SPI1 peripheral not used: the sensor is driven by bit-banged 3-wire SPI
   * on PA5/PA7/PA4 (see USER CODE below), which avoids the HAL 1-line
   * half-duplex over-run. */
  /* USER CODE BEGIN 2 */
  uart_print("\r\n=== WF100D sensor read (SPI) ===\r\n");

  /* Configure the bit-bang SPI GPIOs: PA4=CS, PA5=SCK, PA7=SDIO (all outputs
   * to start; SDIO is flipped to input during reads). Idle CS high, SCK low. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef io = {0};
  io.Pin = WF_CS_PIN | WF_SCK_PIN | WF_SDIO_PIN;
  io.Mode = GPIO_MODE_OUTPUT_PP;
  io.Pull = GPIO_NOPULL;
  io.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WF_IO_PORT, &io);
  WF_CS_HIGH();
  WF_SCK_LOW();

  /* DIAGNOSTIC: no writes at all -- isolate the read path. If PART_ID reads
   * 0x87 here (and in the loop), bit-bang reads work and the earlier soft-reset
   * write was corrupting SPI_Ctrl. */
  uint8_t pid = 0;
  char idline[32];
  wf_read_regs(WF_REG_PART_ID, &pid, 1);
  snprintf(idline, sizeof(idline), "PART_ID = 0x%02X (read-only test)\r\n", pid);
  uart_print(idline);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* DIAGNOSTIC: read-only. No writes anywhere. Read PART_ID twice and STATUS
     * to see if bit-bang reads are stable on their own. */
    uint8_t pid1 = 0, pid2 = 0, st = 0;
    char line[64];

    wf_read_regs(WF_REG_PART_ID, &pid1, 1);
    wf_read_regs(WF_REG_PART_ID, &pid2, 1);
    wf_read_regs(WF_REG_STATUS, &st, 1);

    snprintf(line, sizeof(line), "PID=%02X PID=%02X ST=%02X\r\n", pid1, pid2, st);
    uart_print(line);

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
