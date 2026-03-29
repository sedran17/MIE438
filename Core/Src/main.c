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

/* --- Stepper Motor Configurations --- */
#define STEPS_PER_REV 200
#define MICROSTEP_MULTIPLIER 1
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEP_MULTIPLIER)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// --- Stepper Motor Functions ---
void rotateToAngle(float angle, uint8_t clockwise);
void rotateContinuous(uint8_t clockwise);

// --- Stirrer Functions ---
void Stirrer_Servo_Move(uint32_t pulse_width);
void Stirrer_Motor_Set(GPIO_PinState state);
void Stirrer_Cycle(void);

// --- Powder Dispenser Functions ---
void Dispenser_Motor_Set(uint8_t state);
void Dispenser_Run(uint32_t duration_ms);
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
  /* USER CODE BEGIN 2 */
  
  // Start the TIM12 PWM output for the Stirrer Servo (PB15 -> TIM12_CH2)
  extern TIM_HandleTypeDef htim12; 
  if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2) != HAL_OK)
  {
      // PWM starting error
  }
  
  // Initialize servo out of the cup
  Stirrer_Servo_Move(SERVO_UP_PULSE);
  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //ROTATE TO A SPECIFIC ANGLE ---
	        // Rotate 90 degrees clockwise
	        rotateToAngle(90.0, 1);
	        HAL_Delay(2000); // Stop 2 seconds

	        // Rotate 45 degrees counter-clockwise
	        rotateToAngle(45.0, 0);
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
	        // rotateContinuous(1);
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIR_PIN_Pin|STEP_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DIR_PIN_Pin STEP_PIN_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DIR_PIN_Pin|STEP_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

void rotateToAngle(float angle, uint8_t clockwise) {
    uint32_t stepsRequired = (uint32_t)((angle / 360.0) * TOTAL_STEPS);

    // Set direction pin
    if (clockwise) {
        HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_RESET);
    }

    // Generate step pulses
    for (uint32_t i = 0; i < stepsRequired; i++) {
        HAL_GPIO_WritePin(STEP_PIN_GPIO_Port, STEP_PIN_Pin, GPIO_PIN_SET);
        HAL_Delay(1); // 1ms delay (adjust later for speed)
        HAL_GPIO_WritePin(STEP_PIN_GPIO_Port, STEP_PIN_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
}

void rotateContinuous(uint8_t clockwise) {
    // Set direction pin
    if (clockwise) {
        HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_RESET);
    }

    // Infinite stepping loop
    while (1) {
        HAL_GPIO_WritePin(STEP_PIN_GPIO_Port, STEP_PIN_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(STEP_PIN_GPIO_Port, STEP_PIN_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
}

// --- Powder Dispenser Functions ---
void Dispenser_Motor_Set(uint8_t state) {
    if (state) {
        HAL_GPIO_WritePin(DCP_DSM_GPIO_Port, DCP_DSM_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DCP_DSM_GPIO_Port, DCP_DSM_Pin, GPIO_PIN_RESET);
    }
}

void Dispenser_Run(uint32_t duration_ms) {
    Dispenser_Motor_Set(1);       // Turn motor ON
    HAL_Delay(duration_ms);       // Wait for the requested time
    Dispenser_Motor_Set(0);       // Turn motor OFF
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
