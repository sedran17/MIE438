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
#include "ssd1306.h"
#include "characters.h"
#include "screen.h"
#include <stdint.h>
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
#define LOWER_BOUND 50.0
#define UPPER_BOUND 60.0

// --- Kettle servo ---
#define KETTLE_UP_PULSE 1000
#define KETTLE_DOWN_PULSE 1000

// --- central stepper location angles ---
#define POWDER_ANGLE 180
#define STUR_ANGLE 200
#define TEA_BAG_ANGLE 150
#define WATER_ANGLE 300
#define RETURN_ANGLE_BONUS 5 // used to ensure the arm gets back to the home point

// --- delay times for physical motion ---
#define STEPPER_DELAY 150
#define BAG_DELAY 2000
#define STURRER_SERVO 1000
#define STURRER_MOTOR 3000
#define TEA_DELAY 1000
#define GENERAL_DELAY 1000

// --- time to run each of the individual components ---
#define POWDER_TIME 1000
#define PUMP_DURATION 15000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim12;

/* USER CODE BEGIN PV */
const uint8_t step_sequence[4][4] = {
  {1, 0, 1, 0}, // Step 0: Coil A Forward, Coil B Forward
  {0, 1, 1, 0}, // Step 1: Coil A Reverse, Coil B Forward
  {0, 1, 0, 1}, // Step 2: Coil A Reverse, Coil B Reverse
  {1, 0, 0, 1}  // Step 3: Coil A Forward, Coil B Reverse
};

int8_t current_step_index = 0;

// For screen communication
I2C_HandleTypeDef hi2c1;

// Used to track the state of the screen
volatile uint8_t command = 0b00;
uint8_t order = 0b0000;
uint8_t state = 0;

// Debounce for buttons
volatile uint32_t last_press_time[3] = {0,0,0};

// kettle temperature variables
uint32_t temp = 0;
uint8_t kettle_status = 1;
volatile float tempC = 0;
float curr_temp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

// --- Stepper Motor Functions ---
void setMotorPins(uint8_t step);
void takeSingleStep(uint8_t clockwise);
void rotateToAngle(float angle, uint8_t clockwise);

// --- Stirrer Functions ---
void Stirrer_Servo_Move(uint32_t pulse_width);
void Stirrer_Motor_Set(GPIO_PinState state);
void Stirrer_Cycle(void);

// --- Powder Dispenser Functions ---
void Dispenser_Motor_Set(uint8_t state);
void Dispenser_Run(uint32_t duration_ms);

// --- Pump Functions ---
void Pump_Init(void);
void Pump_Motor_Set(uint8_t state);
void Pump_Run(uint32_t duration_ms);

// --- Kettle Functions ---
void Kettle_Servo_Move(uint32_t pulse_width);
void servo_kettle(uint8_t status);
float temp_sensor(void);

// --- Tea bag ---
void dispense_tea(void);

// --- Timer ---
void Set_Dispenser_Speed(uint8_t speed);

// --- Screen functions ---
//Screen initializations
struct Screen* init_home(void);
struct Screen* init_drink(void);
struct Screen* init_confirm(void);
struct Screen* init_prepare(void);

//Screen printing
void Print_Screen(struct Screen* screen);

//Screen navagation
struct Screen* Advance_Screen(struct Screen* screen, int direction);
void Advance_Screen_State(struct Screen* screen, int direction);

// Actually Making coffee
void make_coffee(uint8_t order);
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
  MX_I2C1_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	// Initialize pump
	Pump_Init();

  //initiallize screen pointers
	struct Screen *home = NULL;
	struct Screen *drink = NULL;
	struct Screen *confirm = NULL;
	struct Screen *prepare = NULL;

	// run initilization functions
	home = init_home();
	drink = init_drink();
	confirm = init_confirm();
	prepare = init_prepare();


	// make links between the screens
	home->next = drink;
	drink->next = confirm;
	drink->previous = home;
	confirm->next = prepare;
	confirm->previous = drink;
	prepare->next = home;

	//Turn on/clear/initialize screen
	SSD1306_Init();
	SSD1306_Fill(1);
	SSD1306_Update();
	SSD1306_Fill(0);
	SSD1306_Update();
	struct Screen* cur_screen = home;
	Print_Screen(cur_screen);
	SSD1306_Update();

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

	// Start the TIM1 PWM output for the kettle Servo (PA11 -> TIM1_CH4)
	extern TIM_HandleTypeDef htim1;
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
	{
	    // PWM starting error
	}

	// Initialize servo out of the cup
	Stirrer_Servo_Move(SERVO_UP_PULSE);
	HAL_Delay(500);

	// Initialize tea servo to 0 position
	Tea_Servo_Move(TEA_SERVO_POSITION_0);
	HAL_Delay(500);

	HAL_GPIO_WritePin(STR_DCM_GPIO_Port, STR_DCM_Pin, GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  curr_temp = temp_sensor();
	  //Control Loop
	  if(curr_temp < LOWER_BOUND && kettle_status == 0){
		//Kettle On
		kettle_status = 1;
		servo_kettle(kettle_status);
	  }
	  else if(curr_temp > UPPER_BOUND && kettle_status == 1){
		//Kettle Off
		kettle_status = 0;
		servo_kettle(kettle_status);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(command != 0){ // check flag variable to see if ANY button has been pressed
		  // what each command corresponds to can be found in the HAL_GPIO_EXTI_Callback function
		  if(command == 0b10){ //check specific code (checks for down)
			  Advance_Screen_State(cur_screen, 1);
			  SSD1306_Update();
		  } else if(command == 0b01){ //check specific code (checks for up)
			  Advance_Screen_State(cur_screen, -1);
			  SSD1306_Update();
		  } else { //check specific code (checks for select)
			  // select ALWAYS updates the overall screen, not just the screen specific state
			  // therefore, clear the screen then run the logic to determine what should be done next
			  // see screen.h for detailed comments on how the screen works
			  SSD1306_Fill(0);
			  // this layer deals with checking the state to handle the specific logic per screen
			  /*
			   * note that if any screens were to have identical button functionality, they could be combined
			   * this is because none of the functionality is screen-specific, just executed differently
			   */
			  if(state == 0){// home screen
				  //always advances
				  cur_screen = Advance_Screen(cur_screen, 1);
				  Print_Screen(cur_screen);
				  SSD1306_Update();
				  state = 1;
			  } else if(state == 1){ // drink screen
				  /*
				   * order is used to pass what the user selects to make_coffee which handles all the logic of the electro-mechinical system
				   * each bit is a boolean for a specific physical station
				   * this allows adding new functionality to be as easy as extending order and adding th masking logic to make_coffee
				   * order bit table:
				   * LSB: sturring station
				   * 	  tea bag station
				   * 	  coffee powder station
				   * MSB: hot water station
				   */
				  if(cur_screen->state != 3){// NOT back button
					  if(cur_screen->state == 0){// coffee
						  order = 0b1101;
					  } else if(cur_screen->state == 1){// tea
						  order = 0b1010;
					  } else if(cur_screen->state == 2){// water
						  order = 0b1000;
					  }
					  cur_screen = Advance_Screen(cur_screen, 1);
					  Print_Screen(cur_screen);
					  SSD1306_Update();
					  state = 2;
				  } else { // back button
					  order = 0b0000; // reset order variable
					  cur_screen = Advance_Screen(cur_screen, 0);
					  Print_Screen(cur_screen);
					  SSD1306_Update();
					  state = 0;
				  }
			  } else if(state == 2){ // confirm
				  if(cur_screen->state == 0){ // run the making coffee process
					  cur_screen = Advance_Screen(cur_screen, 1);
					  Print_Screen(cur_screen);
					  SSD1306_Update();

					  //__disable_irq(); // disable interrupts while drink is made
					  make_coffee(order); // pass order to drink making function

					  SSD1306_Fill(0);
					  cur_screen = Advance_Screen(cur_screen, 1);
					  Print_Screen(cur_screen);
					  SSD1306_Update();
					  state = 0;

					  //__enable_irq(); // reenable interrupts
				  } else { // back button
					  cur_screen = Advance_Screen(cur_screen, 0);
					  Print_Screen(cur_screen);
					  SSD1306_Update();
					  state = 1;
				  }
				  order = 0b0000; // reset order variable
			  }
		  }
		  command = 0b00; // reset command variable
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  htim3.Init.Period = 20000 - 1;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, COIL_IN3_Pin|COIL_IN4_Pin|COIL_IN1_Pin|COIL_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STR_DCM_GPIO_Port, STR_DCM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PMP_DCM_GPIO_Port, PMP_DCM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COIL_IN3_Pin COIL_IN4_Pin COIL_IN1_Pin COIL_IN2_Pin */
  GPIO_InitStruct.Pin = COIL_IN3_Pin|COIL_IN4_Pin|COIL_IN1_Pin|COIL_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STR_DCM_Pin */
  GPIO_InitStruct.Pin = STR_DCM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STR_DCM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PMP_DCM_Pin */
  GPIO_InitStruct.Pin = PMP_DCM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PMP_DCM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Stirrer_Servo_Move(uint32_t pulse_width) {
    // Uses TIM12 Channel 2 for the servo on PB15
    extern TIM_HandleTypeDef htim12;
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pulse_width);
}

void Stirrer_Motor_Set(uint8_t state) {
    // Toggles the DC Motor (PB13). Ensure STR_DCM is the label for PB14 in CubeIDE
    // Assuming CubeIDE generated STR_DCM_GPIO_Port and STR_DCM_Pin
	// The motoro is small enough that 3.3 volts is well enough (and we don't trigger OCP)
    if(state == 1){
    	HAL_GPIO_WritePin(STR_DCM_GPIO_Port, STR_DCM_Pin, GPIO_PIN_SET);
    } else {
    	HAL_GPIO_WritePin(STR_DCM_GPIO_Port, STR_DCM_Pin, GPIO_PIN_RESET);
    }
}

void Stirrer_Cycle(void) {
    // 1. Move servo into the cup
    Stirrer_Servo_Move(SERVO_DOWN_PULSE);
    HAL_Delay(GENERAL_DELAY); // 1 second mechanical delay to reach the fluid

    // 2. Start stirring
    Stirrer_Motor_Set(1);
    HAL_Delay(STURRER_MOTOR); // Stir for 3 seconds

    // 3. Stop stirring
    Stirrer_Motor_Set(0);

    // 4. Move servo back out of the cup
    Stirrer_Servo_Move(SERVO_UP_PULSE);
    HAL_Delay(GENERAL_DELAY); // 1 second mechanical delay to reach home
}

// --- Tea Bag Functions ---
/*
 * The tea bag dispenser is a scotch yoke
 * therefore the servo just rotates 180 degrees each way
 */
void Tea_Servo_Move(uint32_t pulse_width) {
    // Uses TIM3 Channel 2 for the servo on PC7
    extern TIM_HandleTypeDef htim3;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_width);
}

void dispense_tea(void) {
    // 1. Move servo 180 degrees to dump tea bag
    Tea_Servo_Move(TEA_SERVO_POSITION_180);
    HAL_Delay(TEA_DELAY); // 1 second mechanical delay

    // 2. Return servo to 0 degrees starting position
    Tea_Servo_Move(TEA_SERVO_POSITION_0);
    HAL_Delay(TEA_DELAY); // 1 second return delay
}

// --- Powder Dispenser Functions ---
void Dispenser_Motor_Set(uint8_t state) {
    if (state) {
        HAL_GPIO_WritePin(DSP_DCM_GPIO_Port, DSP_DCM_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(DSP_DCM_GPIO_Port, DSP_DCM_Pin, GPIO_PIN_RESET);
    }
}

/*
 * The powder dispenser is an archimedies screw that is fed by a hopper
 * turning the dc motor dispense powder
 * The motor is considerably more powerful than we need, thus PWM
 */
void Dispenser_Run(uint32_t duration_ms) {
    // 1. Fade in the dispenser motor
    for (int i = 0; i <= 255; i++) {
        Set_Dispenser_Speed(i);
        HAL_Delay(5); // ~1.2 seconds to reach full speed
    }

    // 2. Run at full speed for the requested duration
    HAL_Delay(duration_ms);

    // 3. Fade out the dispenser motor
    for (int i = 255; i >= 0; i--) {
        Set_Dispenser_Speed(i);
        HAL_Delay(5); // ~1.2 seconds to come to a stop
    }
}

// Function to set the speed of the motor
void Set_Dispenser_Speed(uint8_t speed)
{
	// Bounds checking
    if (speed > 255) {
        speed = 255;
    }

    uint32_t mapped_ccr_value = (uint32_t)speed * 20000 / 255;

    // Set the PWM duty cycle for the motor (TIM12 Channel 1 on PB14)
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, mapped_ccr_value);
}


//Stepper Motor
void setMotorPins(uint8_t step) {
    HAL_GPIO_WritePin(COIL_IN1_GPIO_Port, COIL_IN1_Pin, step_sequence[step][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN2_GPIO_Port, COIL_IN2_Pin, step_sequence[step][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN3_GPIO_Port, COIL_IN3_Pin, step_sequence[step][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN4_GPIO_Port, COIL_IN4_Pin, step_sequence[step][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// full step servo config
void takeSingleStep(uint8_t clockwise) {
    if (clockwise) {
        current_step_index++;
        if (current_step_index > 3) current_step_index = 0; // Wrap around to the start
    } else {
        current_step_index--;
        if (current_step_index < 0) current_step_index = 3; // Wrap around to the end
    }

    setMotorPins(current_step_index);
}

void rotateToAngle(float angle, uint8_t clockwise) {
    uint32_t stepsRequired = (uint32_t)((angle / 360.0) * TOTAL_STEPS); // degree scaling

    for (uint32_t i = 0; i < stepsRequired; i++) {
        takeSingleStep(clockwise);

        HAL_Delay(STEPPER_DELAY);
    }

    HAL_GPIO_WritePin(COIL_IN1_GPIO_Port, COIL_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN2_GPIO_Port, COIL_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN3_GPIO_Port, COIL_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COIL_IN4_GPIO_Port, COIL_IN4_Pin, GPIO_PIN_RESET);
}

/*
void rotateContinuous(uint8_t clockwise) {
    while (1) {
        takeSingleStep(clockwise);
        HAL_Delay(STEPPER_DELAY);
    }
}
*/


// --- Pump Functions ---
void Pump_Init(void){
	HAL_GPIO_WritePin(PMP_DCM_GPIO_Port, PMP_DCM_Pin, GPIO_PIN_SET);
}

void Pump_Motor_Set(uint8_t state) {
    if (state) {
        HAL_GPIO_WritePin(PMP_DCM_GPIO_Port, PMP_DCM_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(PMP_DCM_GPIO_Port, PMP_DCM_Pin, GPIO_PIN_SET);
    }
}

void Pump_Run(uint32_t duration_ms) {
    Pump_Motor_Set(1);       // Turn pump ON
    HAL_Delay(duration_ms);  // Wait for the requested time
    Pump_Motor_Set(0);       // Turn pump OFF
}

// --- Kettle functions ---
void Kettle_Servo_Move(uint32_t pulse_width) {
    // Uses TIM1 Channel 4 for the servo on PA11
    extern TIM_HandleTypeDef htim1;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_width);
}

void servo_kettle(uint8_t status){
    if(status){
    	Kettle_Servo_Move(KETTLE_UP_PULSE); //values need to be adjusted depending on servo
    }else{
    	Kettle_Servo_Move(KETTLE_DOWN_PULSE);
    }
}

float temp_sensor(void){
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK) {
            temp = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    if (temp ==0){
    	return -999.0f;
    }

    // --- Convert ADC to voltage ---
    float voltage = temp / 4095.0f;

    // --- Known resistor value ---
    float R_fixed = 10000; // 10k

    // --- Calculate thermistor resistance ---
    float R_thermistor = R_fixed * (voltage / (1.0f - voltage));

    // --- Steinhart-Hart (Beta equation) ---
    int Beta = 3950.0f;
    float T0 = 298.15f;      // 25°C in Kelvin
    float R0 = 10000.0f;     // 10k at 25°C

    float tempK = 1.0f / ( (1.0f/T0) + (1.0f/Beta) * log(R_thermistor / R0) );
    tempC = tempK - 273.15f+ 26.0f;

    return tempC;
}



//---Screen Functions---
//initializations
// see screen.h for detailed comments of how the screens work
// see ssd1306.c for details on how the interface level with the screen works
struct Screen* init_home(void){
	// Home screen setup code
	struct Screen *home = malloc(sizeof(struct Screen));

	// Line 1 (empty)
	home->line1 = NULL;
	home->len1 = 0;

	// Line 2
	int tmp12[] = {1,21,20,15,2,1,18};
	home->len2 = 7;
	home->line2 = malloc(sizeof(int) * home->len2);
	for (int i = 0; i < home->len2; i++) home->line2[i] = tmp12[i];

	// Line 3
	int tmp13[] = {19,20,1,18,20};
	home->len3 = 5;
	home->line3 = malloc(sizeof(int) * home->len3);
	for (int i = 0; i < home->len3; i++) home->line3[i] = tmp13[i];

	// Line 4 (empty)
	home->line4 = NULL;
	home->len4 = 0;

	home->state = 0;
	home->size = 1;
	home->invert_offset = 2;
	home->next = home;
	home->previous = home;

	return home;
}
struct Screen* init_drink(void){
	// Drink screen setup code
	struct Screen *drink = malloc(sizeof(struct Screen));

	// Line 1 (coffee)
	int tmp21[] = {28,0,3,15,6,6,5,5};
	drink->len1 = 8;
	drink->line1 = malloc(sizeof(int) * drink->len1);
	for (int i = 0; i < drink->len1; i++) drink->line1[i] = tmp21[i];

	// Line 2 (tea)
	int tmp22[] = {29,0,20,5,1};
	drink->len2 = 5;
	drink->line2 = malloc(sizeof(int) * drink->len2);
	for (int i = 0; i < drink->len2; i++) drink->line2[i] = tmp22[i];

	// Line 3 (water)
	int tmp23[] = {30,0,23,1,20,5,18};
	drink->len3 = 7;
	drink->line3 = malloc(sizeof(int) * drink->len3);
	for (int i = 0; i < drink->len3; i++) drink->line3[i] = tmp23[i];

	// Line 4 (back)
	int tmp24[] = {2,1,3,11,37,37,37};
	drink->len4 = 7;
	drink->line4 = malloc(sizeof(int) * drink->len4);
	for (int i = 0; i < drink->len4; i++) drink->line4[i] = tmp24[i];

	// State and circular links
	drink->state = 0;
	drink->size = 4;
	drink->invert_offset = 0;
	drink->next = drink;
	drink->previous = drink;

	return drink;
}
struct Screen* init_confirm(void){
	// Confirm screen
	struct Screen *confirm = malloc(sizeof(struct Screen));

	// Line 1 (empty)
	confirm->line1 = NULL;
	confirm->len1 = 0;

	// Line 2
	int tmp32[] = {3,15,14,6,9,18,13,38};
	confirm->len2 = 8;
	confirm->line2 = malloc(sizeof(int) * confirm->len2);
	for (int i = 0; i < confirm->len2; i++) confirm->line2[i] = tmp32[i];

	// Line 3
	int tmp33[] = {2,1,3,11,37,37,37};
	confirm->len3 = 7;
	confirm->line3 = malloc(sizeof(int) * confirm->len3);
	for (int i = 0; i < confirm->len3; i++) confirm->line3[i] = tmp33[i];

	// Line 4 (empty)
	confirm->line4 = NULL;
	confirm->len4 = 0;

	confirm->state = 0;
	confirm->size = 2;
	confirm->invert_offset = 1;
	confirm->next = confirm;
	confirm->previous = confirm;

	return confirm;
}
struct Screen* init_prepare(void){
	// Waiting screen
	struct Screen *prepare = malloc(sizeof(struct Screen));

	// Line 1 (empty)
	prepare->line1 = NULL;
	prepare->len1 = 0;

	// Line 2
	int tmp42[] = {16,18,5,16,1,18,9,14,7,37,37,37};
	prepare->len2 = 12;
	prepare->line2 = malloc(sizeof(int) * prepare->len2);
	for (int i = 0; i < prepare->len2; i++) prepare->line2[i] = tmp42[i];

	// Line 3
	prepare->line3 = NULL;
	prepare->len3 = 0;

	// Line 4 (empty)
	prepare->line4 = NULL;
	prepare->len4 = 0;

	prepare->state = 0;
	prepare->size = 1;
	prepare->invert_offset = 1;
	prepare->next = prepare;
	prepare->previous = prepare;

	return prepare;
}

//Print full screen
void Print_Screen(struct Screen* screen){
	  SSD1306_Word(screen->line1, screen->len1, 0);
	  SSD1306_Word(screen->line2, screen->len2, 1);
	  SSD1306_Word(screen->line3, screen->len3, 2);
	  SSD1306_Word(screen->line4, screen->len4, 3);
	  SSD1306_Invert(screen->state+ screen->invert_offset); // highlight selected line
}

//Move to next/previous screen
struct Screen* Advance_Screen(struct Screen* screen, int direction){
	if(direction == 1){
		return screen->next;
	}
	else{
		return screen->previous;
	}
}

//Move next screen specific state (corresponds to selected line)
void Advance_Screen_State(struct Screen* screen, int direction){
	SSD1306_Invert(screen->state+screen->invert_offset); // "erases" highlighting from previous line
	screen->state = (screen->state+screen->size+direction) % screen->size; //advances state variable
	SSD1306_Invert(screen->state+screen->invert_offset); // "highlights" new line
}

//actually makes coffee
void make_coffee(uint8_t order){
	int angle = 0;

	if((order & 0b0100) == 0b0100){ // mask for powder dispenser bit
		// move to powder
		rotateToAngle(POWDER_ANGLE-angle, 1);
		// track angle
		angle = POWDER_ANGLE;
		// dispense powder
		Dispenser_Run(POWDER_TIME);
	}
	if((order & 0b0010) == 0b0010){ // mask for tea bag
		// move to tea bag
		if(TEA_BAG_ANGLE-angle > 0){
			rotateToAngle(TEA_BAG_ANGLE-angle,1);
		} else{
			rotateToAngle(angle-TEA_BAG_ANGLE,0);
		}
		// track angle
		angle = TEA_BAG_ANGLE;
		// dispense tea bag
		dispense_tea();
	}
	if((order & 0b1000) == 0b1000){ // mask for water dispensing
		// move to water
		rotateToAngle(WATER_ANGLE-angle,1);
		// add amount moved to angle
		angle = WATER_ANGLE;
		// dispense water
		Pump_Run(PUMP_DURATION);
	}
	if((order & 0b0001) == 0b0001){ // mask for sturring
		// move to sturrer
		if(STUR_ANGLE-angle > 0){
			rotateToAngle(STUR_ANGLE-angle,1);
		} else{
			rotateToAngle(angle-STUR_ANGLE,0);
		}
		// add amount moved to angle
		angle = STUR_ANGLE;
		// do sturring
		Stirrer_Cycle();
	}
	// move back to starting point
	rotateToAngle(angle+RETURN_ANGLE_BONUS, 0);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t now = HAL_GetTick(); // take current time

	/*
	 * command table
	 * 0b00: no button
	 * 0b01: up button
	 * 0b10: down button
	 * 0b11: select button
	 */

	// all will check the current time against the last time they were pressed to act as deboucing
	//down
	if (GPIO_Pin == GPIO_PIN_0 && (now - last_press_time[0]) > 250) {
		last_press_time[0] = now;
		command = 0b01;
	}
	//up
	else if (GPIO_Pin == GPIO_PIN_1 && (now - last_press_time[1]) > 250) {
		last_press_time[1] = now;
		command = 0b10;
	}
	// select
	else if (GPIO_Pin == GPIO_PIN_2 && (now - last_press_time[2]) > 250) {
		last_press_time[2] = now;
		command = 0b11;
	}
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
