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
#include "h3lis.h"
#define H3LIS_ADDR_LOW   (0x18 << 1)
#define H3LIS_ADDR_HIGH  (0x19 << 1)

#include "dps368.h"

#include "IMU.h"

#include "w25n.h"

#include "fsm.h"

#include "mtr_control.h"

#include "config.h"

#include "sensor_fusion.h"
#include "sensor_mount.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t timestamp_ms;
    int8_t   h3lis_accel[3];
    float    imu_accel[3];
    float    imu_gyro[3];
    float    temperature_C;
    float    pressure_hPa;
    float    altitude_m;
    uint8_t  fsm_state;
    uint32_t loop_us;      /* how long the PREVIOUS loop body took */
    uint32_t loop_us_max;  /* worst loop body since boot, must stay < 20000 */
    float    fusion_altitude_m;   /* above the pad */
    float    fusion_velocity_mps; /* vertical, +up */
    float    fusion_accel_vert;   /* m/s^2, gravity removed, +up */
    float    fusion_tilt_deg;     /* nose from vertical */
    uint8_t  fusion_flags;        /* FUSION_FLAG_* bits */
} TelemetryRecord_t;

/* fusion_flags bits in the telemetry record */
#define FUSION_FLAG_READY     (1u << 0)
#define FUSION_FLAG_IN_FLIGHT (1u << 1)
#define FUSION_FLAG_DESCENT   (1u << 2)
#define FUSION_FLAG_BARO_OK   (1u << 3)
#define FUSION_FLAG_HIGH_G    (1u << 4)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
Accel_t accel;
uint8_t accelResult; // expected 0x32
AccelData_t accelData;

Baro_t baro;
int32_t tmp_raw, prs_raw;
float temperature_C, pressure_hPa, altitude_m;
uint8_t baroResult; // expected 0x10

IMU imu;
uint8_t IMUResult; // expected 0x6C

W25N_Handle_t nand;
W25N_Status_t nandStatus; // expected W25N_OK after Verify_ID
TelemetryRecord_t telemetry;

uint32_t loop_us, loop_us_max; /* loop body timing, see LoopTimer_* */

FusionOutput fusion;
int32_t last_prs_raw; /* a changed pressure is the only sign of a new baro sample */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Loop timing from the DWT cycle counter, so the worst-case loop time
 * (sensors + fusion + airbrake + logging) can be checked on the bench. */
static void LoopTimer_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t LoopTimer_Elapsed_us(uint32_t start_cycles)
{
    return (DWT->CYCCNT - start_cycles) / (SystemCoreClock / 1000000u);
}

static void PrintTelemetryRecord(uint32_t page_addr, const uint8_t *data, uint32_t length)
{
    (void)length;
    const TelemetryRecord_t *rec = (const TelemetryRecord_t *)data;

    printf("page %lu | t=%lums | h3lis(%d,%d,%d) | imu_a(%.2f,%.2f,%.2f) mg | imu_g(%.2f,%.2f,%.2f) mdps | %.2fC %.2fhPa %.2fm\r\n",
           (unsigned long)page_addr, (unsigned long)rec->timestamp_ms,
           rec->h3lis_accel[0], rec->h3lis_accel[1], rec->h3lis_accel[2],
           rec->imu_accel[0], rec->imu_accel[1], rec->imu_accel[2],
           rec->imu_gyro[0], rec->imu_gyro[1], rec->imu_gyro[2],
           rec->temperature_C, rec->pressure_hPa, rec->altitude_m);
    printf("  state: %s | loop %luus (max %luus)\r\n", FSM_StateName((FSM_State_t)rec->fsm_state),
           (unsigned long)rec->loop_us, (unsigned long)rec->loop_us_max);
    printf("  fusion: alt %.1fm vel %.1fm/s acc %.2fm/s2 tilt %.1fdeg flags 0x%02x\r\n",
           rec->fusion_altitude_m, rec->fusion_velocity_mps, rec->fusion_accel_vert,
           rec->fusion_tilt_deg, rec->fusion_flags);
}

static uint8_t FusionFlags(const FusionOutput *f)
{
    return (uint8_t)((f->ready     ? FUSION_FLAG_READY     : 0u) |
                     (f->in_flight ? FUSION_FLAG_IN_FLIGHT : 0u) |
                     (f->descent   ? FUSION_FLAG_DESCENT   : 0u) |
                     (f->baro_ok   ? FUSION_FLAG_BARO_OK   : 0u) |
                     (f->high_g    ? FUSION_FLAG_HIGH_G    : 0u));
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
  MX_I2C3_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  Motor_t airbrake_motor;

  for (int i = 0; i < STARTUP_BEEP_COUNT; i++) {
  		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  		  HAL_Delay(STARTUP_BEEP_ON_MS);
  		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  		  HAL_Delay(STARTUP_BEEP_OFF_MS);
  	}


  HAL_Delay(STARTUP_BEEP_OFF_MS);

  // starts at MOTOR_STARTUP_ANGLE_DEG (closed), nothing else moves it at boot
  mtr_init(&airbrake_motor, &htim5, TIM_CHANNEL_1);

  accel.i2c_handle = &hi2c1;
  accel.device_addr = H3LIS_ADDR_HIGH;
  accelResult = h3lis_init(&accel);

  baro.i2c_handle = &hi2c3;
  baro.device_addr = DPS368_I2C_ADDR;
  baroResult = dps368_init(&baro);
  if (baroResult != DPS368_WHOAMI_VAL) {
	  // nothing
  }
  dps368_config_tmp(&baro, BARO_TMP_MEAS_RATE, BARO_TMP_OVERSAMPLING);
  dps368_config_prs(&baro, BARO_PRS_MEAS_RATE, BARO_PRS_OVERSAMPLING);
  dps368_set_opmode(&baro, DPS368_OPMODE_BACKGROUND_PRS_TMP);

  IMUResult = IMU_Initialise(&imu, &hspi3, GPIOA, GPIO_PIN_15);

  // whoami seems to fail, but the values are read normally??? or atleast print normally
//    if (IMUResult != IMU_WHOAMI_VAL) {
//        while (1) {
//            HAL_Delay(100); /* trap here so a wiring/SPI-mode fault is obvious on a debugger */
//        }
//    }

  nand.hspi = &hspi1;
  nand.cs_port = GPIOA;
  nand.cs_pin = GPIO_PIN_4;
  nand.timeout = NAND_TIMEOUT_MS;

  W25N_Reset(&nand);
  nandStatus = W25N_Verify_ID(&nand);
  if (nandStatus != W25N_OK) {
	  printf("NAND: JEDEC ID mismatch - check wiring/CS\r\n");
  }

  nandStatus = W25N_Unlock_All_Blocks(&nand);
  if (nandStatus != W25N_OK) {
	printf("NAND: failed to clear block protection\r\n");
  }

  FSM_t fsm;
  FSM_Init(&fsm);

  if (FSM_GetState(&fsm) == FSM_STATE_READY) {
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	  HAL_Delay(READY_BEEP_MS);
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

  }

  // the first ~2 s of updates calibrate the gyro and pad level, so the board
  // must be still from here
  fusion_init();

  LoopTimer_Init();
  uint32_t prevTick = HAL_GetTick();

  // reading all the data
//  W25N_Log_Dump(&nand, LOG_DUMP_PAGE_COUNT, &telemetry, sizeof(telemetry), PrintTelemetryRecord);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint32_t timeTick = HAL_GetTick();

	  if (timeTick - prevTick >= MIN_CONTROL_LOOP_PERIOD_MS) {
		  float dt_s = (float)(timeTick - prevTick) * 0.001f;
		  prevTick = timeTick;
		  uint32_t loopStart = DWT->CYCCNT;

		  h3lis_read(&accel, &accelData);

		  dps368_get_result(&baro, &tmp_raw, &prs_raw);
		  temperature_C = tmp_raw / 100.0f;      /* tmp_raw is °C x100 */
		  pressure_hPa  = prs_raw / 100.0f;      /* prs_raw is Pa, /100 -> hPa */
		  altitude_m = dps368_get_altitude(prs_raw);

		  ReadAllIMU(&imu);

		  // raw chip axes -> rocket body frame happens in sensor_mount.h
		  int8_t hg[3] = { accelData.accel_x, accelData.accel_y, accelData.accel_z };
		  bool baro_new = (prs_raw != last_prs_raw);
		  last_prs_raw = prs_raw;
		  FusionInput fusion_in = sensor_mount_input(imu.acceleration, imu.gyro, hg,
		                                             altitude_m, baro_new);
		  fusion = fusion_update(&fusion_in, dt_s);

		  bool changed = FSM_NewReading(
			  &fsm, 0, 0, 0, /* h3lis int8 accel: unused by fsm.c's logic */
			  // the FSM expects the vertical axis as its y input - see IMU_AXIS_* in config.h
			  imu.acceleration[IMU_AXIS_LATERAL_A], imu.acceleration[IMU_AXIS_VERTICAL], imu.acceleration[IMU_AXIS_LATERAL_B],
			  imu.gyro[0], imu.gyro[1], imu.gyro[2], altitude_m, timeTick);

//		  printf("imu accel (mg): %.2f %.2f %.2f\r\n", imu.acceleration[0], imu.acceleration[1], imu.acceleration[2]);
//		  printf("imu gyro (mdps): %.2f %.2f %.2f\r\n", imu.gyro[0], imu.gyro[1], imu.gyro[2]);
//
//		  printf("temp: %.2f C\r\n", temperature_C);
//		  printf("pressure: %.2f hPa\r\n", pressure_hPa);
//		  printf("altitude: %.2f m\r\n", altitude_m);
//
//		  printf("state: %s\r\n", FSM_StateName(FSM_GetState(&fsm)));

		  telemetry.timestamp_ms = timeTick;
		  telemetry.h3lis_accel[0] = accelData.accel_x;
		  telemetry.h3lis_accel[1] = accelData.accel_y;
		  telemetry.h3lis_accel[2] = accelData.accel_z;
		  telemetry.imu_accel[0] = imu.acceleration[0];
		  telemetry.imu_accel[1] = imu.acceleration[1];
		  telemetry.imu_accel[2] = imu.acceleration[2];
		  telemetry.imu_gyro[0] = imu.gyro[0];
		  telemetry.imu_gyro[1] = imu.gyro[1];
		  telemetry.imu_gyro[2] = imu.gyro[2];
		  telemetry.temperature_C = temperature_C;
		  telemetry.pressure_hPa = pressure_hPa;
		  telemetry.altitude_m = altitude_m;
		  telemetry.fsm_state = (uint8_t)FSM_GetState(&fsm);
		  telemetry.loop_us = loop_us;
		  telemetry.loop_us_max = loop_us_max;
		  telemetry.fusion_altitude_m = fusion.altitude;
		  telemetry.fusion_velocity_mps = fusion.velocity;
		  telemetry.fusion_accel_vert = fusion.accel_vert;
		  telemetry.fusion_tilt_deg = fusion.tilt_deg;
		  telemetry.fusion_flags = FusionFlags(&fusion);

		  if (FSM_GetState(&fsm) != FSM_STATE_LANDED) {
			  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			  nandStatus = W25N_Log_Write(&nand, &telemetry, sizeof(telemetry));
		  }

		  if (nandStatus != W25N_OK) {
			  printf("NAND write failed at page %lu, status %d\r\n", (unsigned long)nand.log_next_page, nandStatus);
		  }

		  loop_us = LoopTimer_Elapsed_us(loopStart);
		  if (loop_us > loop_us_max) {
			  loop_us_max = loop_us;
		  }
	  }

//	  HAL_Delay(500);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2928;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1465;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 7;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
    ITM_SendChar(ch);
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
