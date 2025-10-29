/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define r 0.035f
#define L 0.2711f
#define h 0.1f
#define pi 3.1415926f
#define N 18
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

osThreadId vControlTaskHandle;
osThreadId vTaskLedHandle;
/* USER CODE BEGIN PV */



// --- FreeRTOS Timing ---
// Period for the main control loop task (2 milliseconds)
const TickType_t periodo = pdMS_TO_TICKS(2); // Convert ms to FreeRTOS ticks

// --- Filter Buffers (Potentially unused if using IIR filter now) ---
float buffer[N+1] = {0};  // Circular buffer for FIR filter input (motor 1?)
float buffer2[N+1] = {0}; // Circular buffer for FIR filter input (motor 2?)

// --- Robot State Variables (Odometry) ---
// These are updated by vControlTask and read by the SPI ISR
float x = 0;   // Robot X position (meters) in global frame
float y = 0;   // Robot Y position (meters) in global frame
float th = 0; // Robot orientation (theta) (radians) in global frame
float xp = 0;  // Robot linear velocity along its own X-axis (m/s)
float yp = 0;  // Robot linear velocity along its own Y-axis (m/s) - Note: often v_x, v_y used

// --- SPI Communication Variables ---
float maxspeed = 0.5f; // Maximum speed/duty cycle limit received from host
float ux = 0; // Desired X velocity command from host (global frame)
float uy = 0; // Desired Y velocity command from host (global frame)
uint8_t tx_data[25]; // SPI transmit buffer (STM32 -> Host: odometry)
uint8_t rx_data[15]; // SPI receive buffer (Host -> STM32: commands)

// --- Motor Control Variables ---
float w1d = 0; // Desired angular velocity for wheel 1 (left?) (rad/s)
float w2d = 0; // Desired angular velocity for wheel 2 (right?) (rad/s)
float u1 = 0;  // Control output (duty cycle / voltage) for motor 1
float u2 = 0;  // Control output (duty cycle / voltage) for motor 2
float iew1 = 0; // Integral error accumulator for PI controller (motor 1)
float iew2 = 0; // Integral error accumulator for PI controller (motor 2)

// --- Intermediate Kinematics/State Variables ---
float th1 = 0; // Current angle of wheel 1 (radians)
float th2 = 0; // Current angle of wheel 2 (radians)
float tha1 = 0; // Previous angle of wheel 1 (radians)
float tha2 = 0; // Previous angle of wheel 2 (radians)
float w1 = 0;  // Calculated raw angular velocity of wheel 1 (rad/s)
float w2 = 0;  // Calculated raw angular velocity of wheel 2 (rad/s)
float w1_fc = 0; // Filtered angular velocity of wheel 1 (rad/s) used for control
float w2_fc = 0; // Filtered angular velocity of wheel 2 (rad/s) used for control
float w1_fca = 0; // Previous filtered velocity for wheel 1 (IIR filter state)
float w2_fca = 0; // Previous filtered velocity for wheel 2 (IIR filter state)
float alpha = 0.1f; // Smoothing factor for the IIR velocity filter (0 < alpha <= 1)
float v = 0;   // Robot linear velocity (m/s)
float w = 0;   // Robot angular velocity (rad/s)
float vd = 0;  // Desired robot linear velocity (m/s) calculated from ux, uy
float wd = 0;  // Desired robot angular velocity (rad/s) calculated from ux, uy

// --- Constants & Control Parameters ---
float Ts = 0.002; // Control loop sample time (seconds) - MUST MATCH 'periodo'!

// --- LED Control Variables ---
float ze = 0.2;  // Threshold for LED state (zero-ish speed)
float ze2 = 0.6; // Threshold for LED state (turning indication?)
uint8_t dato = 0; // Data byte encoding the LED state to send to ESP32

// --- Encoder Raw Data ---
float ae1 = 0; // Wheel 1 angle intermediate (float)
float ae2 = 0; // Wheel 2 angle intermediate (float)
uint32_t e1 = 0; // Raw encoder count for wheel 1 (from TIM2)
uint32_t e2 = 0; // Raw encoder count for wheel 2 (from TIM5)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
void vControlTask_Handler(void const * argument);
void vTaskLed_Handler(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Calculates a simple 8-bit checksum (sum of first 4 bytes).
 * @param data Pointer to the start of the 4 data bytes.
 * @retval uint8_t The calculated checksum.
 */
uint8_t add_checksum(uint8_t *data) {
    uint8_t sum = 0;
    // Sum the first 4 bytes
    for (int i = 0; i < 4; i++) {
        sum += data[i];
    }
    // Return the lower 8 bits of the sum
    return sum; // Implicit modulo 256
}

/**
 * @brief SPI1 Transmit/Receive Complete Callback (ISR context).
 * Handles communication with the host computer (e.g., Raspberry Pi).
 * Receives velocity commands (ux, uy, maxspeed) and prepares odometry
 * data (x, y, th, xp, yp) for the next transmission.
 * @param hspi SPI handle.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    // This function runs in an Interrupt Service Routine (ISR) context!

    // --- Receive and Process Data from Host ---
    // Verify checksums for the three received float values
    if ((rx_data[4]  == add_checksum(&rx_data[0]))  && // Checksum for ux
        (rx_data[9]  == add_checksum(&rx_data[5]))  && // Checksum for uy
        (rx_data[14] == add_checksum(&rx_data[10]))) { // Checksum for maxspeed

        // Checksums match, copy data into global volatile variables
        // Use memcpy for potentially non-aligned access or type punning safety
        memcpy((void*)&ux,       &rx_data[0],  4); // Copy received ux command
        memcpy((void*)&uy,       &rx_data[5],  4); // Copy received uy command
        memcpy((void*)&maxspeed, &rx_data[10], 4); // Copy received maxspeed limit
    } else {
        // Checksum error detected
        // Set commands to zero as a safety measure
        ux = 0.0f;
        uy = 0.0f;
        // Optionally, set a flag or log the error
    }

    // --- Prepare Data for Next Transmission to Host ---
    // Need to read shared odometry variables (x, y, th, xp, yp) safely.
    UBaseType_t uxSavedInterruptStatus;
	// Enter critical section (ISR-safe version) to prevent race conditions with vControlTask
	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // Prepare odometry data packet (5 floats + 5 checksums = 25 bytes)
    memcpy(&tx_data[0],  (void*)&x, 4); // Copy current x position
    tx_data[4]  = add_checksum(&tx_data[0]); // Calculate checksum for x

    memcpy(&tx_data[5],  (void*)&y, 4); // Copy current y position
    tx_data[9]  = add_checksum(&tx_data[5]); // Calculate checksum for y

    memcpy(&tx_data[10], (void*)&xp, 4); // Copy current x velocity (robot frame)
    tx_data[14] = add_checksum(&tx_data[10]); // Calculate checksum for xp

    memcpy(&tx_data[15], (void*)&yp, 4); // Copy current y velocity (robot frame)
    tx_data[19] = add_checksum(&tx_data[15]); // Calculate checksum for yp

    memcpy(&tx_data[20], (void*)&th, 4); // Copy current orientation (theta)
    tx_data[24] = add_checksum(&tx_data[20]); // Calculate checksum for theta

    // Exit critical section, restoring previous interrupt state
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

    // --- Re-arm SPI Reception ---
    // Start the next non-blocking SPI transaction (Slave waits for Master)
    // Assumes the Host will initiate the next transfer.
    HAL_SPI_TransmitReceive_IT(&hspi1, tx_data, rx_data, 25);
}


/**
 * @brief Sets the PWM duty cycle for Motor 1 (Left?).
 * Handles direction based on the sign of fduty.
 * Clamps the duty cycle to [-maxspeed, maxspeed].
 * @param fduty Desired duty cycle (-1.0 to 1.0, scaled relative to maxspeed).
 */
void Mi1(float fduty) {
    // Clamp duty cycle based on the dynamic maxspeed received via SPI
	if(fduty >= maxspeed) { fduty = maxspeed; }
	if(fduty <= -maxspeed) { fduty = -maxspeed; }

    // Scale float duty cycle (-maxspeed to +maxspeed) to PWM timer period (0 to 32767?)
    // Assuming Period = 33999, so max duty is roughly half? Check timer config.
    // Let's assume 32767 corresponds to maxspeed.
    // This scaling might need adjustment based on timer Period value.
	fduty = fduty * (32767.0f / maxspeed); // Scale based on maxspeed? Recheck this logic. Usually scales to Period.
    // --> Alternative assuming 33999 is Period for 100% duty:
    // fduty = fduty * 33999.0f;

	int16_t duty = (int16_t)fduty; // Cast scaled duty to integer

    // Set PWM channels based on direction (sign of duty)
    if (duty >= 0) { // Forward direction
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty); // PWM output on CH2
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);    // Direction pin (CH3) low/off
    } else { // Reverse direction
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);    // PWM output (CH2) off
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -duty); // Direction pin (CH3) uses PWM magnitude
    }
}

/**
 * @brief Sets the PWM duty cycle for Motor 2 (Right?).
 * Similar logic to Mi1.
 * @param fduty Desired duty cycle (-1.0 to 1.0, scaled relative to maxspeed).
 */
void Md2(float fduty) {
    // Clamp duty cycle
	if(fduty >= maxspeed) { fduty = maxspeed; }
	if(fduty <= -maxspeed) { fduty = -maxspeed; }

    // Scale float duty cycle to PWM timer range (assuming Period = 33999)
    // Check scaling logic as in Mi1.
	fduty = fduty * (32767.0f / maxspeed); // Recheck scaling
    // --> Alternative: fduty = fduty * 33999.0f;

	int16_t duty = (int16_t)fduty;

    // Set PWM channels based on direction
    if (duty >= 0) { // Forward
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty); // PWM on CH1
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);    // Direction CH2 low
    } else { // Reverse
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);    // PWM CH1 off
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, -duty); // Direction CH2 PWM
    }
}

/**
 * @brief Sends a 4-bit data value ('dato') to specific GPIO pins.
 * Used to signal robot state to the ESP32.
 */
void enviarDato() {
    // Extract individual bits from the global 'dato' byte
    uint8_t bit1 = (dato >> 0) & 1; // Bit 0 -> PB9
    uint8_t bit2 = (dato >> 1) & 1; // Bit 1 -> PB7
    uint8_t bit4 = (dato >> 2) & 1; // Bit 2 -> PB5 (Note: name implies bit 4?)
    uint8_t bit8 = (dato >> 3) & 1; // Bit 3 -> PC13 (Note: name implies bit 8?)

    // Write bit values to the corresponding GPIO pins
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, bit1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, bit2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, bit4 ? GPIO_PIN_SET : GPIO_PIN_RESET); // Check pin mapping
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, bit8 ? GPIO_PIN_SET : GPIO_PIN_RESET); // Check pin mapping
}

/**
 * @brief Determines the LED state ('dato') based on filtered wheel speeds.
 * Sets the global 'dato' variable which is then sent by enviarDato().
 * @param wi Filtered angular velocity of wheel 1 (left?).
 * @param wd Filtered angular velocity of wheel 2 (right?).
 */
void enviarleds(float wi, float wd){
  // Logic to determine the 4-bit state based on wheel speeds
  if (wi >= -ze && wi <= ze && wd >= -ze && wd <= ze) { // Both wheels stopped or very slow
    dato = 0; // State 0: Stopped
  }
  else if (!(wi >= -ze2 && wi <= ze2 && wd >= -ze2 && wd <= ze2) && (wd >= wi * 1.4f)) { // Turning significantly (Right wheel faster?)
    dato = 6; // State 6: Turning Right? (Check mapping)
  }
  else if (!(wi >= -ze2 && wi <= ze2 && wd >= -ze2 && wd <= ze2) && (wi >= wd * 1.4f)) { // Turning significantly (Left wheel faster?)
    dato = 7; // State 7: Turning Left? (Check mapping)
  }
  else if (wi >= ze && wd >= ze) { // Both moving forward
    dato = 3; // State 3: Moving Forward?
  }
  else if (wi <= -ze && wd <= -ze) { // Both moving backward
    dato = 5; // State 5: Moving Backward?
  }
  // Note: Add cases for pivot turns (wi > 0, wd < 0) etc. if needed

  // Send the determined state to the GPIO pins
  enviarDato();
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_SPI_TransmitReceive_IT(&hspi1, tx_data, rx_data, 25);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  //uint32_t t0_ticks = __HAL_TIM_GET_COUNTER(&htim4);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of vControlTask */
  osThreadDef(vControlTask, vControlTask_Handler, osPriorityHigh, 0, 1025);
  vControlTaskHandle = osThreadCreate(osThread(vControlTask), NULL);

  /* definition and creation of vTaskLed */
  osThreadDef(vTaskLed, vTaskLed_Handler, osPriorityNormal, 0, 512);
  vTaskLedHandle = osThreadCreate(osThread(vTaskLed), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 33999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1699;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 33999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_vControlTask_Handler */
/**
  * @brief Function implementing the vControlTask thread (High Priority).
  * This is the main real-time control loop running every 2ms.
  * It reads encoders, calculates velocities, filters noise,
  * updates odometry, calculates PI control output, and drives motors.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_vControlTask_Handler */
void vControlTask_Handler(void const * argument)
{
  /* USER CODE BEGIN 5 */
  // Variable to store the last wake time for periodic execution
  TickType_t tick_time;
  // Initialize tick_time with the current time ONCE before the loop
  tick_time = xTaskGetTickCount();

  /* Infinite loop */
  for(;;) // This loop runs forever, managed by FreeRTOS
  {
        // --- 1. Read Encoders ---
        // Get raw 32-bit counter values from encoder timers
		e1 = __HAL_TIM_GET_COUNTER(&htim2); // Left wheel encoder?
		e2 = __HAL_TIM_GET_COUNTER(&htim5); // Right wheel encoder?
        // Cast to signed float for calculations
		ae1 = (float)(int32_t)e1;
		ae2 = (float)(int32_t)e2;
        // Convert raw counts to angle in radians
        // Formula: (counts * 2*pi) / (counts_per_revolution_motor * gear_ratio * pulses_per_revolution_encoder_mode)
        // Check your encoder specs and gear ratio. 4.0*332.0*27.0 seems large.
        // Assuming: 4x encoding, 332 PPR encoder?, 27:1 gearbox? -> 35856 counts/rev
		th1 = (ae1 * 2.0f * pi) / (4.0f * 332.0f * 27.0f); // Wheel 1 angle (rad)
		th2 = (ae2 * 2.0f * pi) / (4.0f * 332.0f * 27.0f); // Wheel 2 angle (rad)

        // --- 2. Calculate Raw Wheel Velocities ---
        // Velocity = (current_angle - previous_angle) / sample_time
		w1 = (th1 - tha1) / Ts; // Wheel 1 raw angular velocity (rad/s)
		w2 = (th2 - tha2) / Ts; // Wheel 2 raw angular velocity (rad/s)

        // --- 3. Spike Rejection ---
        // Limit impossibly high velocities caused by noise or encoder glitches
		const float MAX_REALISTIC_W = 15.0f; // Define a max plausible velocity (rad/s)

		if (w1 > MAX_REALISTIC_W) {
			w1 = w1_fca; // If spike detected, use previous filtered value instead
		} else if (w1 < -MAX_REALISTIC_W) {
			w1 = w1_fca;
		}
		if (w2 > MAX_REALISTIC_W) {
			w2 = w2_fca; // Use previous filtered value
		} else if (w2 < -MAX_REALISTIC_W) {
			w2 = w2_fca;
		}

        // --- 4. Filter Velocities (IIR Low Pass Filter) ---
        // Smooth out noise from differentiation using exponential moving average
        // w_filtered_new = alpha * w_raw_new + (1 - alpha) * w_filtered_old
		w1_fc = (alpha * w1) + (1.0f - alpha) * w1_fca; // Filtered velocity wheel 1
		w2_fc = (alpha * w2) + (1.0f - alpha) * w2_fca; // Filtered velocity wheel 2

        // --- 5. Update Previous State Variables (IMPORTANT!) ---
        // Store current values to be used as 'previous' in the next iteration
		tha1 = th1;     // Update previous angle for wheel 1
		tha2 = th2;     // Update previous angle for wheel 2
		w1_fca = w1_fc; // Update previous filtered velocity for wheel 1
		w2_fca = w2_fc; // Update previous filtered velocity for wheel 2

        // --- 6. Calculate Odometry (Robot Pose Update) ---
        // Forward Kinematics: Calculate robot's linear and angular velocity from wheel speeds
		v = (w1_fc + w2_fc) * r / 2.0f; // Use filtered velocities for odometry? (Or raw w1, w2?) - Using filtered here.
		w = (w2_fc - w1_fc) * r / L;   // Robot angular velocity (rad/s)

        // Calculate change in angle over the sample time
		float dth = w * Ts;

        // Integrate angle (theta) - Protect shared variable 'th'
		if (isfinite(dth) && (w < 13.0f) && (w > -13.0f)) { // Check for valid values and limits
		  taskENTER_CRITICAL(); // --- Start Critical Section ---
		  th += dth;            // Update global theta
		  taskEXIT_CRITICAL();  // --- End Critical Section ---
          // Normalize theta to [-pi, pi] if needed
          // if (th > pi) th -= 2.0f*pi;
          // else if (th < -pi) th += 2.0f*pi;
		}

        // Calculate robot velocity components in the ROBOT's frame (xp, yp)
        // Accounts for offset 'h' if it represents distance to control point
		xp = v * cosf(th) - h * sinf(th) * w; // Velocity along robot's x-axis
		yp = v * sinf(th) + h * cosf(th) * w; // Velocity along robot's y-axis

        // Calculate change in position (dx, dy) in global frame
		float dx = xp * Ts;
		float dy = yp * Ts;

        // Integrate position (x, y) - Protect shared variables 'x' and 'y'
        // Enter critical section ONCE for both x and y update
		taskENTER_CRITICAL(); // --- Start Critical Section ---
		if ((isfinite(dx)) && (xp < 1.5f) && (xp > -1.5f)) { // Check validity and limits
		    x += dx; // Update global x
		}
		if ((isfinite(dy)) && (yp < 1.5f) && (yp > -1.5f)) { // Check validity and limits
		    y += dy; // Update global y
		}
		taskEXIT_CRITICAL();  // --- End Critical Section ---


        // --- 7. Calculate Desired Wheel Velocities (Inverse Kinematics) ---
        // Convert desired global velocities (ux, uy) received from host
        // into desired robot frame velocities (vd, wd).
		vd = ux * cosf(th) + uy * sinf(th);           // Desired linear velocity (robot frame)
		wd = (-ux * sinf(th) + uy * cosf(th)) / h;   // Desired angular velocity (using offset 'h') - Check formula if h=0

        // Convert desired robot velocities (vd, wd) into desired wheel velocities (w1d, w2d)
		w2d = vd / r + wd * L / (2.0f * r); // Desired velocity for wheel 2 (right?)
		w1d = vd / r - wd * L / (2.0f * r); // Desired velocity for wheel 1 (left?)

        // --- 8. PI Control Calculation ---
        // Calculate error between filtered measured speed and desired speed
        // Update integral term (with anti-windup) and calculate PI output
        const float MAX_INTEGRATOR = 1.0f; // Anti-windup limit (tune this value)

        // PI Controller for Wheel 2 (Right?)
		iew2 = iew2 + (w2_fc - w2d) * Ts; // Accumulate integral error
        // Anti-Windup: Clamp integrator term
		if (iew2 > MAX_INTEGRATOR) iew2 = MAX_INTEGRATOR;
		else if (iew2 < -MAX_INTEGRATOR) iew2 = -MAX_INTEGRATOR;
        // Calculate PI output: u = Kp * error + Ki * integral_error
		u2 = -0.07f * (w2_fc - w2d) - 0.2f * iew2; // Kp = -0.07, Ki = -0.2 (Check signs)

        // PI Controller for Wheel 1 (Left?)
		iew1 = iew1 + (w1_fc - w1d) * Ts; // Accumulate integral error
        // Anti-Windup: Clamp integrator term
		if (iew1 > MAX_INTEGRATOR) iew1 = MAX_INTEGRATOR;
		else if (iew1 < -MAX_INTEGRATOR) iew1 = -MAX_INTEGRATOR;
        // Calculate PI output
		u1 = -0.07f * (w1_fc - w1d) - 0.2f * iew1; // Kp = -0.07, Ki = -0.2 (Check signs)


        // --- 9. Apply Control Output to Motors ---
		Md2(u2); // Set PWM for motor 2 (Right?)
		Mi1(u1); // Set PWM for motor 1 (Left?)

        // --- 10. Wait for Next Cycle ---
        // Delay execution until the next 2ms interval using vTaskDelayUntil
        // This ensures the loop runs at a consistent frequency (500Hz).
		vTaskDelayUntil(&tick_time, periodo);
	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vTaskLed_Handler */
/**
* @brief Function implementing the vTaskLed thread (Normal Priority).
* Runs periodically (every 100ms) to update the LED status
* based on the robot's current filtered wheel speeds.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskLed_Handler */
void vTaskLed_Handler(void const * argument)
{
  /* USER CODE BEGIN vTaskLed_Handler */
    /* Infinite loop */
	for (;;)
	{
		// Determine the LED state based on current filtered wheel speeds (w1_fc, w2_fc)
        // Note: Reads w1_fc, w2_fc which are written by vControlTask.
        // If high precision is not needed, direct read is okay due to lower priority.
        // For strict safety, a mutex or queue could be used.
		enviarleds(w1_fc, w2_fc);

		// Delay this task for 100ms
        // Uses vTaskDelay, which sleeps for *at least* 100ms from NOW.
		vTaskDelay(pdMS_TO_TICKS(100));
	}
  /* USER CODE END vTaskLed_Handler */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
