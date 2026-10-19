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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "spif.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VIDEO_FRAME_COUNT       744U
#define VIDEO_FRAME_SIZE        SSD1306_BUFFER_SIZE
#define VIDEO_DATA_ADDRESS      4096U
#define VIDEO_DATA_SIZE         (VIDEO_FRAME_COUNT * VIDEO_FRAME_SIZE)
#define VIDEO_HEADER_SIZE       16U
#define VIDEO_UPLOAD_TIMEOUT    10000U
#define VIDEO_FRAME_RATE        24U
#define INTRO_PAGE_DURATION_MS  2000U
_Static_assert((SSD1306_WIDTH == 128U) && (SSD1306_HEIGHT == 64U),
               "IKUN playback requires a 128x64 OLED");
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static spif_t w25q64;
static uint8_t frame_buffer[SSD1306_BUFFER_SIZE];
static uint8_t flash_buffer[SPIF_PAGE_SIZE];
static char jedec_id_line[] = "JEDEC: 00 00 00";
static const uint8_t intro_glyphs[2][6][32] = {
  {
    {0x20U, 0x02U, 0x70U, 0x0AU, 0x1EU, 0x12U, 0x10U, 0x12U, 0x10U, 0x02U, 0xFFU, 0x7FU, 0x10U, 0x02U, 0x10U, 0x22U, 0x50U, 0x22U, 0x30U, 0x12U, 0x18U, 0x0CU, 0x16U, 0x44U, 0x10U, 0x4AU, 0x10U, 0x51U, 0xD4U, 0x60U, 0x08U, 0x40U}, /* 我 */
    {0x00U, 0x00U, 0xFEU, 0x3FU, 0x00U, 0x01U, 0x00U, 0x01U, 0x80U, 0x00U, 0x80U, 0x00U, 0xC0U, 0x02U, 0xA0U, 0x04U, 0x90U, 0x08U, 0x88U, 0x10U, 0x84U, 0x20U, 0x82U, 0x20U, 0x81U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U}, /* 不 */
    {0x00U, 0x00U, 0xFEU, 0x3FU, 0x20U, 0x02U, 0x20U, 0x02U, 0xFCU, 0x1FU, 0x24U, 0x12U, 0x24U, 0x12U, 0xFCU, 0x1FU, 0x40U, 0x00U, 0xFEU, 0x3FU, 0x20U, 0x04U, 0x10U, 0x02U, 0x38U, 0x01U, 0xC0U, 0x01U, 0x60U, 0x0EU, 0x1CU, 0x10U}, /* 要 */
    {0x44U, 0x00U, 0x44U, 0x3EU, 0xFEU, 0x22U, 0x44U, 0x22U, 0x44U, 0x22U, 0x7CU, 0x3EU, 0x44U, 0x22U, 0x44U, 0x22U, 0x7CU, 0x22U, 0x44U, 0x3EU, 0x44U, 0x22U, 0xFFU, 0x22U, 0x20U, 0x21U, 0x44U, 0x21U, 0x82U, 0x28U, 0x41U, 0x10U}, /* 期 */
    {0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0xFCU, 0x1FU, 0x84U, 0x10U, 0x84U, 0x10U, 0x84U, 0x10U, 0x84U, 0x10U, 0x84U, 0x10U, 0xFCU, 0x1FU, 0x84U, 0x10U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U}, /* 中 */
    {0x40U, 0x00U, 0x40U, 0x10U, 0xFCU, 0x0BU, 0x40U, 0x04U, 0x40U, 0x02U, 0xFFU, 0x7FU, 0x80U, 0x00U, 0x40U, 0x00U, 0xF0U, 0x1FU, 0x48U, 0x00U, 0x24U, 0x00U, 0xE2U, 0x0FU, 0x01U, 0x08U, 0x00U, 0x08U, 0x00U, 0x05U, 0x00U, 0x02U}, /* 考 */
  },
  {
    {0x80U, 0x08U, 0x80U, 0x08U, 0xDEU, 0x3FU, 0x92U, 0x08U, 0x92U, 0x08U, 0x12U, 0x00U, 0x52U, 0x10U, 0x52U, 0x10U, 0x92U, 0x08U, 0x92U, 0x08U, 0x1EU, 0x05U, 0x12U, 0x02U, 0x00U, 0x05U, 0x80U, 0x08U, 0x40U, 0x10U, 0x30U, 0x60U}, /* 哎 */
    {0x00U, 0x04U, 0x80U, 0x04U, 0x8EU, 0x04U, 0x4AU, 0x04U, 0x4AU, 0x3FU, 0x2AU, 0x25U, 0xEAU, 0x24U, 0x8AU, 0x24U, 0x4AU, 0x24U, 0x4AU, 0x24U, 0x2EU, 0x25U, 0xE2U, 0x25U, 0x00U, 0x23U, 0x00U, 0x22U, 0x00U, 0x29U, 0x80U, 0x10U}, /* 呦 */
    {0x10U, 0x01U, 0x10U, 0x01U, 0x10U, 0x01U, 0x88U, 0x7FU, 0x88U, 0x40U, 0x4CU, 0x20U, 0x2CU, 0x04U, 0x0AU, 0x04U, 0x89U, 0x14U, 0x88U, 0x24U, 0x48U, 0x24U, 0x48U, 0x44U, 0x28U, 0x44U, 0x08U, 0x04U, 0x08U, 0x05U, 0x08U, 0x02U}, /* 你 */
    {0x00U, 0x00U, 0xFCU, 0x1FU, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0xFFU, 0x7FU, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 0x00U}, /* 干 */
    {0x00U, 0x02U, 0x00U, 0x04U, 0xEFU, 0x7FU, 0x29U, 0x09U, 0x29U, 0x09U, 0x29U, 0x09U, 0xE9U, 0x3FU, 0x29U, 0x09U, 0x29U, 0x1BU, 0xA9U, 0x1DU, 0xAFU, 0x2DU, 0x69U, 0x2BU, 0x10U, 0x49U, 0x10U, 0x09U, 0x08U, 0x09U, 0x00U, 0x09U}, /* 嘛 */
    {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x9CU, 0x00U, 0x62U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U}, /* ~ */
  }
};

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

static uint32_t Video_ReadLe32(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static void Video_WriteLe32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static uint32_t Video_Crc32Update(uint32_t crc, const uint8_t *data, uint32_t len)
{
  uint32_t i;
  uint32_t bit;

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
  }

  return crc;
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

static bool W25Q_Init(void)
{
  spif_err_t result;
  HAL_StatusTypeDef spi_status;
  uint8_t jedec_id[3] = {0U, 0U, 0U};

  HAL_Delay(120U);

  spi_status = W25Q_ReadJedecId(jedec_id);
  Flash_FormatJedecId(jedec_id);
  if (spi_status != HAL_OK)
  {
    Flash_Log("JEDEC SPI read fail", "Check SPI/CS wiring", jedec_id_line);
    return false;
  }

  result = spif_init(&w25q64, &hspi1, GPIOA, GPIO_PIN_4);
  if ((result != SPIF_ERR_NONE) || (w25q64.size == 0U))
  {
    Flash_LogError("Flash init failed", result);
    return false;
  }

  if ((w25q64.manufacturer != SPIF_MANUFACTURER_WINBOND) ||
      (w25q64.memory_type != 0x40U) ||
      (w25q64.size != 0x00800000UL) ||
      (w25q64.sector_size == 0U) ||
      ((VIDEO_DATA_ADDRESS + VIDEO_DATA_SIZE) > w25q64.size))
  {
    Flash_Log("Unexpected flash", "Expected W25Q64", jedec_id_line);
    return false;
  }

  return true;
}

static bool Video_ReadHeader(void)
{
  uint8_t header[VIDEO_HEADER_SIZE];
  spif_err_t result;

  result = spif_read(&w25q64, 0U, header, sizeof(header));
  if (result != SPIF_ERR_NONE)
  {
    Flash_LogError("Video header read", result);
    return false;
  }

  return (memcmp(header, "IKUN", 4U) == 0) &&
         (Video_ReadLe32(&header[4]) == VIDEO_DATA_SIZE) &&
         (Video_ReadLe32(&header[12]) == VIDEO_FRAME_COUNT);
}

static bool Video_SendStatus(const uint8_t *status, uint16_t len)
{
  return HAL_UART_Transmit(&huart1, (uint8_t *)status, len, 1000U) == HAL_OK;
}

static bool Video_Upload(void)
{
  static const uint8_t ready_status[] = {'R', 'D', 'Y', '!'};
  static const uint8_t size_error[] = {'S', 'I', 'Z', 'E'};
  static const uint8_t data_status[] = {'D', 'A', 'T', 'A'};
  static const uint8_t pass_status[] = {'P', 'A', 'S', 'S'};
  static const uint8_t fail_status[] = {'F', 'A', 'I', 'L'};
  static const uint8_t ack = 0x06U;
  static const uint8_t nak = 0x15U;
  uint8_t metadata[8];
  uint8_t header[VIDEO_HEADER_SIZE] = {0U};
  uint8_t header_verify[VIDEO_HEADER_SIZE];
  uint32_t expected_size;
  uint32_t expected_crc;
  uint32_t received_crc = 0xFFFFFFFFUL;
  uint32_t verify_crc = 0xFFFFFFFFUL;
  uint32_t offset;
  uint32_t sector;
  uint32_t sector_count;
  uint32_t data_end = VIDEO_DATA_ADDRESS + VIDEO_DATA_SIZE;
  spif_err_t result;

  if (!Video_SendStatus(ready_status, sizeof(ready_status)) ||
      (HAL_UART_Receive(&huart1, metadata, sizeof(metadata), 3000U) != HAL_OK))
  {
    return false;
  }

  expected_size = Video_ReadLe32(metadata);
  expected_crc = Video_ReadLe32(&metadata[4]);
  if ((expected_size != VIDEO_DATA_SIZE) ||
      (w25q64.sector_size == 0U) ||
      (data_end > w25q64.size))
  {
    (void)Video_SendStatus(size_error, sizeof(size_error));
    return false;
  }

  sector_count = (data_end + w25q64.sector_size - 1U) / w25q64.sector_size;
  Flash_Log("Uploading IKUN", "Erasing video sectors", "Please wait...");
  for (sector = 0U; sector < sector_count; sector++)
  {
    result = spif_erase_sector(&w25q64, sector);
    if (result != SPIF_ERR_NONE)
    {
      Flash_LogError("Video erase failed", result);
      (void)Video_SendStatus(fail_status, sizeof(fail_status));
      return false;
    }
  }

  if (!Video_SendStatus(data_status, sizeof(data_status)))
  {
    return false;
  }

  for (offset = 0U; offset < VIDEO_DATA_SIZE; offset += SPIF_PAGE_SIZE)
  {
    if (HAL_UART_Receive(&huart1, flash_buffer, SPIF_PAGE_SIZE,
                         VIDEO_UPLOAD_TIMEOUT) != HAL_OK)
    {
      (void)Video_SendStatus(fail_status, sizeof(fail_status));
      return false;
    }

    received_crc = Video_Crc32Update(received_crc, flash_buffer, SPIF_PAGE_SIZE);
    result = spif_write(&w25q64, VIDEO_DATA_ADDRESS + offset,
                        flash_buffer, SPIF_PAGE_SIZE);
    if (result != SPIF_ERR_NONE)
    {
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)&nak, 1U, 1000U);
      Flash_LogError("Video write failed", result);
      return false;
    }
    if (HAL_UART_Transmit(&huart1, (uint8_t *)&ack, 1U, 1000U) != HAL_OK)
    {
      return false;
    }
  }

  received_crc ^= 0xFFFFFFFFUL;
  if (received_crc != expected_crc)
  {
    (void)Video_SendStatus(fail_status, sizeof(fail_status));
    Flash_Log("Video CRC failed", "Upload data mismatch", "");
    return false;
  }

  for (offset = 0U; offset < VIDEO_DATA_SIZE; offset += SPIF_PAGE_SIZE)
  {
    result = spif_read(&w25q64, VIDEO_DATA_ADDRESS + offset,
                       flash_buffer, SPIF_PAGE_SIZE);
    if (result != SPIF_ERR_NONE)
    {
      Flash_LogError("Video verify read", result);
      (void)Video_SendStatus(fail_status, sizeof(fail_status));
      return false;
    }
    verify_crc = Video_Crc32Update(verify_crc, flash_buffer, SPIF_PAGE_SIZE);
  }

  verify_crc ^= 0xFFFFFFFFUL;
  if (verify_crc != expected_crc)
  {
    (void)Video_SendStatus(fail_status, sizeof(fail_status));
    Flash_Log("Video verify failed", "Flash CRC mismatch", "");
    return false;
  }

  memcpy(header, "IKUN", 4U);
  Video_WriteLe32(&header[4], VIDEO_DATA_SIZE);
  Video_WriteLe32(&header[8], expected_crc);
  Video_WriteLe32(&header[12], VIDEO_FRAME_COUNT);
  result = spif_write(&w25q64, 0U, header, sizeof(header));
  if (result != SPIF_ERR_NONE)
  {
    Flash_LogError("Video header write", result);
    (void)Video_SendStatus(fail_status, sizeof(fail_status));
    return false;
  }

  result = spif_read(&w25q64, 0U, header_verify, sizeof(header_verify));
  if ((result != SPIF_ERR_NONE) ||
      (memcmp(header, header_verify, sizeof(header)) != 0))
  {
    Flash_Log("Video header verify", "Flash write failed", "");
    (void)Video_SendStatus(fail_status, sizeof(fail_status));
    return false;
  }

  Flash_Log("Video upload done", "Starting playback", "");
  return Video_SendStatus(pass_status, sizeof(pass_status));
}

static bool Video_PlayFrame(uint32_t frame)
{
  uint32_t address = VIDEO_DATA_ADDRESS + (frame * VIDEO_FRAME_SIZE);
  spif_err_t result = spif_read(&w25q64, address, frame_buffer, sizeof(frame_buffer));

  if (result != SPIF_ERR_NONE)
  {
    Flash_LogError("Frame read failed", result);
    return false;
  }

  if (ssd1306_FillBuffer(frame_buffer, sizeof(frame_buffer)) != SSD1306_OK)
  {
    Flash_Log("Frame buffer error", "Check OLED config", "");
    return false;
  }

  ssd1306_UpdateScreen();
  return true;
}

static void Video_ShowIntroPage(uint8_t page)
{
  uint8_t character;
  uint8_t y;

  ssd1306_Fill(Black);
  for (character = 0U; character < 6U; character++)
  {
    for (y = 0U; y < 16U; y++)
    {
      uint8_t column;

      for (column = 0U; column < 16U; column++)
      {
        uint8_t bitmap_byte =
          intro_glyphs[page][character][(y * 2U) + (column / 8U)];

        if ((bitmap_byte & (uint8_t)(1U << (column % 8U))) != 0U)
        {
          ssd1306_DrawPixel((uint8_t)(16U + (character * 16U) + column),
                            (uint8_t)(24U + y), White);
        }
      }
    }
  }

  ssd1306_UpdateScreen();
  HAL_Delay(INTRO_PAGE_DURATION_MS);
}

static void Video_ShowIntro(void)
{
  Video_ShowIntroPage(0U);
  Video_ShowIntroPage(1U);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  if (!W25Q_Init())
  {
    while (1)
    {
      HAL_Delay(1000U);
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  uint32_t current_frame = 0U;
  bool video_available = Video_ReadHeader();
  static const uint8_t upload_command[] = {'I', 'K', 'U', 'P'};
  static uint32_t frame_period_remainder = 0U;
  uint32_t frame_start;
  uint32_t frame_duration;

  if (video_available)
  {
    Video_ShowIntro();
  }
  else
  {
    Flash_Log("No video in W25Q", "Send IKUP via UART", "115200 8N1");
  }

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t command[sizeof(upload_command)];

    frame_start = HAL_GetTick();
    if ((HAL_UART_Receive(&huart1, command, sizeof(command), 1U) == HAL_OK) &&
        (memcmp(command, upload_command, sizeof(upload_command)) == 0))
    {
      if (Video_Upload())
      {
        video_available = Video_ReadHeader();
        current_frame = 0U;
        if (video_available)
        {
          Video_ShowIntro();
          frame_period_remainder = 0U;
          frame_start = HAL_GetTick();
        }
      }
      else
      {
        video_available = Video_ReadHeader();
      }
    }

    if (video_available)
    {
      frame_duration = 1000U / VIDEO_FRAME_RATE;
      frame_period_remainder += 1000U % VIDEO_FRAME_RATE;
      if (frame_period_remainder >= VIDEO_FRAME_RATE)
      {
        frame_period_remainder -= VIDEO_FRAME_RATE;
        frame_duration++;
      }

      if (Video_PlayFrame(current_frame))
      {
        current_frame = (current_frame + 1U) % VIDEO_FRAME_COUNT;
        if ((HAL_GetTick() - frame_start) < frame_duration)
        {
          HAL_Delay(frame_duration - (HAL_GetTick() - frame_start));
        }
      }
      else
      {
        video_available = false;
      }
    }
    else
    {
      HAL_Delay(100U);
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
