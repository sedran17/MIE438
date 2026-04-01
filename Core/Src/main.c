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
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- Stirrer Configurations --- */
// Note: These pulse limits assume your TIM12 is set to a 1 MHz clock counting to 20000 (for 50Hz / 20ms period).
// This is typical: e.g., Prescaler = (SystemCoreClock / 1000000) - 1, Counter Period (ARR) = 20000 - 1
#define SERVO_DOWN_PULSE 1000 // (~1.0ms pulse) Inside cup 
#define SERVO_UP_PULSE   1500 // (~1.5ms pulse) 45 deg outside cup

// If we need to toggle the motor
#define STIR_MOTOR_ON  GPIO_PIN_SET
#define STIR_MOTOR_OFF GPIO_PIN_RESET

/* --- Tea servo Configurations --- */
// Note: These pulse limits assume your TIM3 is set to a 1 MHz clock counting to 20000 (for 50Hz / 20ms period).
#define TEA_SERVO_POSITION_0   1000 // (~1.0ms pulse) 0 degrees
#define TEA_SERVO_POSITION_180 2000 // (~2.0ms pulse) 180 degrees

/* --- Stepper Motor Configurations --- */
#define STEPS_PER_REV 200
#define MICROSTEP_MULTIPLIER 1
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEP_MULTIPLIER)


/* ---- Kettle bounds --- */
#define LOWER_BOUND 75.0
#define UPPER_BOUND 85.0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint8_t step_sequence[4][4] = {
  {1, 0, 1, 0}, // Step 0: Coil A Forward, Coil B Forward
  {0, 1, 1, 0}, // Step 1: Coil A Reverse, Coil B Forward
  {0, 1, 0, 1}, // Step 2: Coil A Reverse, Coil B Reverse
  {1, 0, 0, 1}  // Step 3: Coil A Forward, Coil B Reverse
};

int8_t current_step_index = 0;

uint32_t temp = 0;
bool kettle_status = false;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

// --- Stepper Motor Functions ---
void Stepper_Set_Pins(uint8_t step);
void Stepper_Take_Step(uint8_t clockwise);
void Stepper_Rotate_Angle(float angle, uint8_t clockwise);
void Stepper_Rotate_Continuous(uint8_t clockwise);

// --- Stirrer Functions ---
void Stirrer_Servo_Move(uint32_t pulse_width);
void Stirrer_Motor_Set(GPIO_PinState state);
void Stirrer_Cycle(void);

// --- Powder Dispenser Functions ---
void Dispenser_Motor_Set(uint8_t state);
void Dispenser_Run(uint32_t duration_ms);

// --- Pump Functions ---
void Pump_Motor_Set(uint8_t state);
void Pump_Run(uint32_t duration_ms);

// --- Kettle Functions ---
void Kettle_Move(bool status);
float Temp_Sensor_Read(void);

// --- Tea bag ---
void Tea_Servo_Move(uint32_t pulse_width);
void Tea_Dispense(void);

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
  MX_USART2_UART_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  
  // Start the TIM12 PWM output for the Stirrer Servo (PB15 -> TIM12_CH2)
  extern TIM_HandleTypeDef htim12; 
  if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2) != HAL_OK)
  {
      // PWM starting error
  }
  
  // Start the TIM3 PWM output for the Tea Servo (PC7 -> TIM3_CH2)
  extern TIM_HandleTypeDef htim3;
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
      // PWM starting error
  }

  // Initialize servo out of the cup
  Stirrer_Servo_Move(SERVO_UP_PULSE);
  HAL_Delay(500);

  // Initialize tea servo to 0 position
  Tea_Servo_Move(TEA_SERVO_POSITION_0);
  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //ROTATE TO A SPECIFIC ANGLE ---
	        // Rotate 90 degrees clockwise
	        //Stepper_Rotate_Angle(90.0, 1);
	        HAL_Delay(2000); // Stop 2 seconds

	        // Rotate 45 degrees counter-clockwise
	        //Stepper_Rotate_Angle(45.0, 0);
	        HAL_Delay(2000);

            // DISPENSE POWDER ---
            // Run the dispenser motor for 2 seconds
            // Uncomment the next line when you want to dispense powder
            // Dispenser_Run(2000);
            // HAL_Delay(5000); 

            // STIR THE CUP ---
            // Uncomment the next line when you want to initiate a stirring cycle
            // Stirrer_Cycle();
            // HAL_Delay(5000);

	        //CONTINUOUS CIRCULAR MOTION
	        // Stepper_Rotate_Continuous(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	        Stepper_Take_Step(1);
	        HAL_Delay(5);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000 - 1 ;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 84 - 1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 20000 - 1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|COIL_IN3_Pin|COIL_IN4_Pin|COIL_IN2_Pin
                          |COIL_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KETTLE_Pin|DSP_DCM_Pin|STR_DCM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PMP_DCM_GPIO_Port, PMP_DCM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin COIL_IN3_Pin COIL_IN4_Pin COIL_IN2_Pin
                           COIL_IN1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|COIL_IN3_Pin|COIL_IN4_Pin|COIL_IN2_Pin
                          |COIL_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KETTLE_Pin DSP_DCM_Pin STR_DCM_Pin */
  GPIO_InitStruct.Pin = KETTLE_Pin|DSP_DCM_Pin|STR_DCM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PMP_DCM_Pin */
  GPIO_InitStruct.Pin = PMP_DCM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PMP_DCM_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Stirrer_Servo_Move(uint32_t pulse_width) {
    // Uses TIM12 Channel 2 for the servo on PB15
    extern TIM_HandleTypeDef htim12;
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pulse_width);
}

void Stirrer_Motor_Set(GPIO_PinState state) {
    // Toggles the DC Motor (PB14). Ensure STR_DCM is the label for PB14 in CubeIDE
    // Assuming CubeIDE generated STR_DCM_GPIO_Port and STR_DCM_Pin
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, state); 
}

void Stirrer_Cycle(void) {
    // 1. Move servo into the cup
    Stirrer_Servo_Move(SERVO_DOWN_PULSE);
    HAL_Delay(1000); // 1 second mechanical delay to reach the fluid

    // 2. Start stirring
    Stirrer_Motor_Set(STIR_MOTOR_ON);
    HAL_Delay(3000); // Stir for 3 seconds

    // 3. Stop stirring
    Stirrer_Motor_Set(STIR_MOTOR_OFF);

    // 4. Move servo back out of the cup
    Stirrer_Servo_Move(SERVO_UP_PULSE);
    HAL_Delay(1000); // 1 second mechanical delay to reach home
}

// --- Powder Dispenser Functions ---
void Dispenser_Motor_Set(uint8_t state) {
    if (state) {
        HAL_GPIO_WritePin(DSP_DCM_GPIO_Port, DSP_DCM_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DSP_DCM_GPIO_Port, DSP_DCM_Pin, GPIO_PIN_RESET);
    }
}

void Dispenser_Run(uint32_t duration_ms) {
    Dispenser_Motor_Set(1);       // Turn motor ON
    HAL_Delay(duration_ms);       // Wait for the requested time
    Dispenser_Motor_Set(0);       // Turn motor OFF
}

// --- Pump Functions ---
void Pump_Motor_Set(uint8_t state) {
    if (state) {
        HAL_GPIO_WritePin(PMP_DCM_GPIO_Port, PMP_DCM_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(PMP_DCM_GPIO_Port, PMP_DCM_Pin, GPIO_PIN_RESET);
    }
}

void Pump_Run(uint32_t duration_ms) {
    Pump_Motor_Set(1);       // Turn pump ON
    HAL_Delay(duration_ms);  // Wait for the requested time
    Pump_Motor_Set(0);       // Turn pump OFF
}

//Stepper Motor

void Stepper_Set_Pins(uint8_t step) {
    HAL_GPIO_WritePin(COIL_IN1_GPIO_Port, COIL_IN1_Pin, step_sequence[step][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN2_GPIO_Port, COIL_IN2_Pin, step_sequence[step][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN3_GPIO_Port, COIL_IN3_Pin, step_sequence[step][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN4_GPIO_Port, COIL_IN4_Pin, step_sequence[step][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stepper_Take_Step(uint8_t clockwise) {
    if (clockwise) {
        current_step_index++;
        if (current_step_index > 3) current_step_index = 0; // Wrap around to the start
    } else {
        current_step_index--;
        if (current_step_index < 0) current_step_index = 3; // Wrap around to the end
    }

    Stepper_Set_Pins(current_step_index);
}

void Stepper_Rotate_Angle(float angle, uint8_t clockwise) {
    uint32_t stepsRequired = (uint32_t)((angle / 360.0) * TOTAL_STEPS);

    for (uint32_t i = 0; i < stepsRequired; i++) {
        Stepper_Take_Step(clockwise);

        HAL_Delay(5);
    }

    HAL_GPIO_WritePin(COIL_IN1_GPIO_Port, COIL_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN2_GPIO_Port, COIL_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN3_GPIO_Port, COIL_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN4_GPIO_Port, COIL_IN4_Pin, GPIO_PIN_RESET);
}

void Stepper_Rotate_Continuous(uint8_t clockwise) {
    while (1) {
        Stepper_Take_Step(clockwise);
        HAL_Delay(5);
    }
}

// --- Kettle --
// Note: We need to define Tea_Servo_Move and Tea_Dispense
void Tea_Servo_Move(uint32_t pulse_width) {
    // Uses TIM3 Channel 2 for the servo on PC7
    extern TIM_HandleTypeDef htim3;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_width);
}

void Tea_Dispense(void) {
    // 1. Move servo 180 degrees to dump tea bag
    Tea_Servo_Move(TEA_SERVO_POSITION_180);
    HAL_Delay(1000); // 1 second mechanical delay

    // 2. Return servo to 0 degrees starting position
    Tea_Servo_Move(TEA_SERVO_POSITION_0);
    HAL_Delay(1000); // 1 second return delay
}

void Kettle_Move(bool status){
    if(status){
    	Stepper_Rotate_Angle(90, true); // Rotate 90 degrees forward to open/turn on
    }else{
    	Stepper_Rotate_Angle(90, false); // Rotate 90 degrees backward to close/turn off
    }
}

float Temp_Sensor_Read(void){
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 20);
    temp = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    // --- Convert ADC to voltage ---
    float voltage = (3.3f * temp) / 4095.0f;

    // --- Known resistor value ---
    float R_fixed = 10000.0f; // 10kΩ

    // --- Calculate thermistor resistance ---

    float R_thermistor = R_fixed * ((3.3f / voltage) - 1.0f);

    // --- Steinhart-Hart (Beta equation) ---
    float Beta = 3950.0f;
    float T0 = 298.15f;      // 25°C in Kelvin
    float R0 = 10000.0f;     // 10k at 25°C

    float tempK = 1.0f / ( (1.0f/T0) + (1.0f/Beta) * log(R_thermistor / R0) );
    float tempC = tempK - 273.15f;

    return tempC;
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
