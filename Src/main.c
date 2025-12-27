/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "lptim.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt_delay.h"
#include "st7789.h"
#include "init.h"

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
static VL53L4CX_Object_t VL53L4CXDev;
static VL53L4CX_Result_t VL53L4CXResult;
static VL53L4CX_ProfileConfig_t VL53L4CXProfile;
static char DistanceString[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
static void VL53L4CX_Ranging_Init(void);
static void VL53L4CX_Ranging_Loop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/* Configure the MPU attributes for the QSPI 256MB without instruction access */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = QSPI_BASE;
	MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
	MPU_InitStruct.SubRegionDisable = 0x00;
	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/* Configure the MPU attributes for the QSPI 8MB (QSPI Flash Size) to Cacheable WT */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER1;
	MPU_InitStruct.BaseAddress = QSPI_BASE;
	MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
	MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
	MPU_InitStruct.SubRegionDisable = 0x00;
	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void CPU_CACHE_Enable(void) {
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

static void LED_Blink(uint32_t Hdelay, uint32_t Ldelay) {
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_SET);
	HAL_Delay(Hdelay - 1);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_RESET);
	HAL_Delay(Ldelay - 1);
}

void I2C_Scan_Debug(void) {
    printf("Scanning I2C bus...\r\n");
    HAL_StatusTypeDef res;

    // 스캔 시작 전 I2C 상태 확인
    uint32_t state = HAL_I2C_GetState(&hi2c1);
    uint32_t error = HAL_I2C_GetError(&hi2c1);
    char status_msg[64];
    snprintf(status_msg, sizeof(status_msg), "St:%lu, Err:%lu", state, error);
    ST7789_DrawUser8x16(0, 32, status_msg, ST7789_YELLOW, ST7789_BLACK);

    for (uint16_t i = 0; i < 128; i++) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
        if (res == HAL_OK) {
            char msg[32];
            snprintf(msg, sizeof(msg), "Found: 0x%02X", (i << 1));
            ST7789_DrawUser8x16(0, 48, msg, ST7789_GREEN, ST7789_BLACK);
            return;
        }
    }
    ST7789_DrawUser8x16(0, 48, "No Device Found", ST7789_RED, ST7789_BLACK);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

#ifdef W25Qxx
    SCB->VTOR = QSPI_BASE;
  #endif
	MPU_Config();
	CPU_CACHE_Enable();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_LPTIM1_Init();
  MX_LPTIM2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_LPTIM3_Init();
  MX_LPTIM4_Init();
  MX_TIM7_Init();
  MX_LPTIM5_Init();
  MX_ADC3_Init();
  MX_TIM15_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
	BSP_I2C1_Init();
	DWT_Delay_Init();
	ST7789_Init();

	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,300);

	ST7789_DrawUser8x16(0, 0, "Hello World", ST7789_WHITE, ST7789_BLACK);
	HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_RESET); // 1. 센서 끄기 (Reset)
	HAL_Delay(100);                                                // 2. 잠시 대기 (방전)
	HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);   // 3. 센서 켜기 (Boot)
	HAL_Delay(100);                                                // 4. 부팅 대기 (필수)

	I2C_Scan_Debug();
	HAL_Delay(2000); // 결과 확인할 시간 벌기

	VL53L4CX_Ranging_Init();
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, GPIO_PIN_RESET);
	ST7789_DrawUser8x16(0, 32, "TOF Ready", ST7789_GREEN, ST7789_BLACK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		VL53L4CX_Ranging_Loop();

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 1;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int32_t BSP_VL53L4CX_Init(void) {
	return 0; // 성공(0) 반환
}

int32_t BSP_VL53L4CX_DeInit(void) {
	return 0; // 성공(0) 반환
}

// I2C 쓰기
int32_t BSP_VL53L4CX_WriteReg(uint16_t DevAddr, uint8_t *pData, uint16_t Len) {
	// 타임아웃 100ms
	if (HAL_I2C_Master_Transmit(&hi2c1, DevAddr, pData, Len, 100) != HAL_OK) {
		return -1;
	}
	return 0;
}

// I2C 읽기
int32_t BSP_VL53L4CX_ReadReg(uint16_t DevAddr, uint8_t *pData, uint16_t Len) {
	if (HAL_I2C_Master_Receive(&hi2c1, DevAddr, pData, Len, 100) != HAL_OK) {
		return -1;
	}
	return 0;
}

// 시간(ms)
int32_t BSP_VL53L4CX_GetTick(void) {
	return (int32_t) HAL_GetTick();
}

static void VL53L4CX_Ranging_Init(void) {
	int32_t status;
	uint32_t id = 0;
	VL53L4CX_IO_t IO_Struct = { 0 }; // 0으로 초기화하여 쓰레기값 방지

	/* IO 구조체 채우기 (빠짐없이 모두 연결) */
	IO_Struct.Init = BSP_VL53L4CX_Init;    // 추가됨
	IO_Struct.DeInit = BSP_VL53L4CX_DeInit;  // 추가됨
	IO_Struct.Address = 0x52;                 // I2C 주소
	IO_Struct.WriteReg = BSP_VL53L4CX_WriteReg;
	IO_Struct.ReadReg = BSP_VL53L4CX_ReadReg;
	IO_Struct.GetTick = BSP_VL53L4CX_GetTick;

	/* 드라이버에 IO 등록 */
	status = VL53L4CX_RegisterBusIO(&VL53L4CXDev, &IO_Struct);
	if (status != VL53L4CX_OK) {
		// 에러 코드 확인용 (status 값을 화면에 표시)
		char err_str[20];
		snprintf(err_str, sizeof(err_str), "IO ERR: %ld", status);
		ST7789_DrawUser8x16(0, 16, err_str, ST7789_RED, ST7789_BLACK);
		Error_Handler();
	}

	/* ID 읽기 */
	status = VL53L4CX_ReadID(&VL53L4CXDev, &id);
	if (status != VL53L4CX_OK) {
		ST7789_DrawUser8x16(0, 16, "ID READ ERR", ST7789_RED, ST7789_BLACK);
		Error_Handler();
	}

	// ID가 0xEBAA인지 확인
	if (id != VL53L4CX_ID) {
		char id_str[20];
		snprintf(id_str, sizeof(id_str), "ID: 0x%lX", id);
		ST7789_DrawUser8x16(0, 16, id_str, ST7789_RED, ST7789_BLACK);
		// ID가 틀려도 일단 진행해보려면 Error_Handler 주석 처리
		// Error_Handler();
	}

	/* 센서 부팅/초기화 */
	status = VL53L4CX_Init(&VL53L4CXDev);
	if (status != VL53L4CX_OK) {
		// 초기화 실패 시 에러 코드 표시
		char err_str[20];
		snprintf(err_str, sizeof(err_str), "INIT ERR: %ld", status);
		ST7789_DrawUser8x16(0, 64, err_str, ST7789_RED, ST7789_BLACK);
		Error_Handler();
	}

	/* 프로파일 설정 */
	VL53L4CXProfile.RangingProfile = VL53L4CX_PROFILE_LONG;
	VL53L4CXProfile.TimingBudget = 50;
	VL53L4CXProfile.Frequency = 20; // 타이밍 버젯보다 주기가 길어야 안정적 (1000/20 = 50ms)
	VL53L4CXProfile.EnableAmbient = 0;
	VL53L4CXProfile.EnableSignal = 0;

	status = VL53L4CX_ConfigProfile(&VL53L4CXDev, &VL53L4CXProfile);
	if (status != VL53L4CX_OK) {
		ST7789_DrawUser8x16(0, 16, "PROF ERR", ST7789_RED, ST7789_BLACK);
		Error_Handler();
	}

	/* 측정 시작 */
	status = VL53L4CX_Start(&VL53L4CXDev, VL53L4CX_MODE_BLOCKING_CONTINUOUS);
	if (status != VL53L4CX_OK) {
		ST7789_DrawUser8x16(0, 16, "START ERR", ST7789_RED, ST7789_BLACK);
		Error_Handler();
	}
}

static void VL53L4CX_Ranging_Loop(void) {
	int32_t status;
	uint32_t distance_mm = 0;

	status = VL53L4CX_GetDistance(&VL53L4CXDev, &VL53L4CXResult);

	if (status == VL53L4CX_OK) {
		// 타겟이 1개 이상일 때만 표시
		if (VL53L4CXResult.ZoneResult[0].NumberOfTargets > 0) {
			// 상태 코드가 0(Valid)인 경우 신뢰도 높음
			// if (VL53L4CXResult.ZoneResult[0].Status[0] == 0) ...

			distance_mm = VL53L4CXResult.ZoneResult[0].Distance[0];
			snprintf(DistanceString, sizeof(DistanceString), "Dist: %4lu mm",
					distance_mm);
			ST7789_DrawUser8x16(0, 16, DistanceString, ST7789_WHITE,
					ST7789_BLACK);
		} else {
			ST7789_DrawUser8x16(0, 16, "No Target   ", ST7789_WHITE,
					ST7789_BLACK);
		}
	} else {
		// 측정 에러는 자주 발생할 수 있으니 화면 깜빡임 방지를 위해 필요한 경우만 출력
		// ST7789_DrawUser8x16(0, 16, "GetDist ERR ", ST7789_RED, ST7789_BLACK);
	}

	HAL_Delay(50); // Loop 지연
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
	while (1) {
		LED_Blink(500, 500);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
