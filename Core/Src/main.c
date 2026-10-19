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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "spif.h"
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
static spif_t w25q64;
static uint8_t flash_tx[SPIF_PAGE_SIZE];
static uint8_t flash_rx[SPIF_PAGE_SIZE];
static char jedec_id_line[] = "JEDEC: 00 00 00";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Flash_Log(char *line1, char *line2, char *line3)
{
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString(line1, Font_6x8, White);
  ssd1306_SetCursor(0, 16);
  ssd1306_WriteString(line2, Font_6x8, White);
  ssd1306_SetCursor(0, 32);
  ssd1306_WriteString(line3, Font_6x8, White);
  ssd1306_UpdateScreen();
}

static void Flash_LogError(char *stage, spif_err_t error)
{
  char error_line[] = "SPIF error: 0";
  error_line[12] = (char)('0' + error);
  Flash_Log(stage, error_line, jedec_id_line);
}

static void Flash_FormatJedecId(const uint8_t *id)
{
  static const char hex_digits[] = "0123456789ABCDEF";
  const uint8_t positions[] = {7U, 10U, 13U};
  uint32_t i;

  for (i = 0U; i < 3U; i++)
  {
    jedec_id_line[positions[i]] = hex_digits[id[i] >> 4];
    jedec_id_line[positions[i] + 1U] = hex_digits[id[i] & 0x0FU];
  }
}

static HAL_StatusTypeDef W25Q_ReadJedecId(uint8_t *id)
{
  uint8_t command = 0x9FU;
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(&hspi1, &command, 1U, 100U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi1, id, 3U, 100U);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return status;
}

static void W25Q_Init(void)
{
  HAL_Delay(120U);
}

static void W25Q64_Test(void)
{
  spif_err_t result;
  HAL_StatusTypeDef spi_status;
  uint8_t jedec_id[3] = {0U, 0U, 0U};
  uint32_t test_address;
  uint32_t i;

  Flash_Log("W25Q64 TEST", "Waiting 120 ms", "");
  W25Q_Init();

  spi_status = W25Q_ReadJedecId(jedec_id);
  Flash_FormatJedecId(jedec_id);
  if (spi_status != HAL_OK)
  {
    Flash_Log("JEDEC SPI read fail", "Check SPI/CS wiring", jedec_id_line);
    return;
  }
  Flash_Log("Raw JEDEC ID", jedec_id_line, "Starting SPIF init");

  result = spif_init(&w25q64, &hspi1, GPIOA, GPIO_PIN_4);
  if ((result != SPIF_ERR_NONE) || (w25q64.size == 0U))
  {
    Flash_LogError("Flash init failed", result);
    return;
  }

  if ((w25q64.manufacturer != SPIF_MANUFACTURER_WINBOND) ||
      (w25q64.memory_type != 0x40U) ||
      (w25q64.size != 0x00800000UL) ||
      (w25q64.sector_size == 0U))
  {
    Flash_Log("Unexpected flash", "Expected W25Q64", "Check JEDEC ID");
    return;
  }

  test_address = w25q64.size - w25q64.sector_size;
  Flash_Log("W25Q64 TEST", "Erase last sector", "Reserve this sector");
  result = spif_erase_sector(&w25q64,
                             SPIF_ADDRESS_TO_SECTOR(&w25q64, test_address));
  if (result != SPIF_ERR_NONE)
  {
    Flash_LogError("Sector erase failed", result);
    return;
  }

  result = spif_read(&w25q64, test_address, flash_rx, sizeof(flash_rx));
  if (result != SPIF_ERR_NONE)
  {
    Flash_LogError("Erase read failed", result);
    return;
  }
  for (i = 0U; i < sizeof(flash_rx); i++)
  {
    if (flash_rx[i] != 0xFFU)
    {
      Flash_Log("Erase verify FAIL", "Read data != FF", "");
      return;
    }
    flash_tx[i] = (uint8_t)(i ^ 0xA5U);
  }

  Flash_Log("W25Q64 TEST", "Writing 256 bytes", "");
  result = spif_write(&w25q64, test_address, flash_tx, sizeof(flash_tx));
  if (result != SPIF_ERR_NONE)
  {
    Flash_LogError("Page program failed", result);
    return;
  }

  result = spif_read(&w25q64, test_address, flash_rx, sizeof(flash_rx));
  if (result != SPIF_ERR_NONE)
  {
    Flash_LogError("Data read failed", result);
    return;
  }
  for (i = 0U; i < sizeof(flash_tx); i++)
  {
    if (flash_rx[i] != flash_tx[i])
    {
      Flash_Log("Readback verify FAIL", "Data mismatch", "");
      return;
    }
  }

  Flash_Log("W25Q64 TEST", "PASS", "Erase/write/read OK");
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  W25Q64_Test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1000U);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
