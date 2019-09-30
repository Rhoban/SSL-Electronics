/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  * <h2><center>&copy; Copyright (c) 2019 Adrien Boussicault (USER CODE).
  * </center></h2>
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <terminal.h>
#include "debug.h"
#include "usbd_cdc_if.h"
#include <time.h>
#include <encoder.h>
#include <system.h>
#include <motor.h>
#include <foc.h>
#include <errors.h>
#include <observer.h>
#include <priority.h>
#include <frequence_definitions.h>
#include <current.h>
#include <pwm.h>

// TIM1 is a center-aligned timer
#define TIM1_PERIOD CA_RELOAD(2*PWM_PERIOD) // The timer to generate all the pwm

// TIM2 is a counter-up timer
#define TIM2_PERIOD CU_RELOAD(PWM_PERIOD) // The timer to generate all the pwm
//#define TIM2_CC2_PULSE (PWM_PERIOD-CURRENT_MEDURE_HALF_PERIOD)
#define TIM2_CC2_PULSE delay_period
_Static_assert(TIM2_CC2_PULSE>0, "");
_Static_assert(TIM2_CC2_PULSE<PWM_PERIOD, "");
#define TIM2_CC1_PULSE TIM2_CC2_PULSE

// TIM3 is a counter-up timer
#define TIM3_PERIOD CU_RELOAD(SYCLK_TO_ENCODER_PERIOD) // Timer for the encoder.
#define TIM4_PERIOD 4 // Clock user to reset and synchronize all the other clock
#define TIM5_PERIOD 4294967295 // Clock used to launch background computation
#define TIM1_PRESCALAR 0
#define TIM3_PRESCALAR 0
#define TIM4_PRESCALAR 0
#define TIM5_PRESCALAR 1 //5000
#define COMP_DELAY 1
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
__IO uint16_t adc_data[ADC_DATA_SIZE] = {0};
extern volatile bool start_adc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TERMINAL_COMMAND(frequences, "Print all the frequences"){

  terminal_println( "Frequences :");
  terminal_println("");
  terminal_print("CLK_SYSCLK :");
  terminal_println_int(CLK_SYSCLK);
  terminal_print("PWM_FREQ :");
  terminal_println_int(PWM_FREQ);
  terminal_print("CENTER_ALIGNED_PWM_FREQ :");
  terminal_println_int(CENTER_ALIGNED_PWM_FREQ);
  terminal_print("ENCODER_FREQ :");
  terminal_println_int(ENCODER_FREQ);
  //terminal_print("ANGLE_OBSERVER_FREQ :");
  //terminal_println_int(ANGLE_OBSERVER_FREQ);
  terminal_print("PWM_DUTY_CYCLE_FREQ :");
  terminal_println_int(PWM_DUTY_CYCLE_FREQ);
  //terminal_print("ARG_COMMAND_FREQ :");
  //terminal_println_int(ARG_COMMAND_FREQ);
  //terminal_print("ANGLE_FREQ :");
  //terminal_println_int(ANGLE_FREQ);
  terminal_print("NORM_COMMAND_FREQ :");
  terminal_println_int(NORM_COMMAND_FREQ);
  terminal_print("AS5047D_UPDATE_FREQ : ");
  terminal_println_int(AS5047D_UPDATE_FREQ);
  terminal_print("AS5047D_UPDATE_FAILURE_FREQ : ");
  terminal_println_int(AS5047D_UPDATE_FAILURE_FREQ);

  terminal_println("");
  terminal_println("Periods :");
  terminal_println("");
  terminal_print("PWM_PERIOD :");
  terminal_println_int(PWM_PERIOD );
  terminal_print("CENTER_ALIGNED_PERIOD :");
  terminal_println_int(CENTER_ALIGNED_PERIOD);
  terminal_print("ENCODER_PERIOD :");
  terminal_println_int(ENCODER_PERIOD);
  //terminal_print("OBS_PERIOD :");
  //terminal_println_int(OBS_PERIOD);
  terminal_print("PWM_DUTY_CYCLE_PERIOD :");
  terminal_println_int(PWM_DUTY_CYCLE_PERIOD);
  terminal_print("NORM_PERIOD :");
  terminal_println_int(NORM_PERIOD);
  terminal_print("TIM1_PERIOD :");
  terminal_println_int(TIM1_PERIOD);
  terminal_print("TIM3_PERIOD :");
  terminal_println_int(TIM3_PERIOD);
  terminal_print("TIM4_PERIOD :");
  terminal_println_int(TIM4_PERIOD);
  terminal_print("TIM5_PERIOD :");
  terminal_println_int(TIM5_PERIOD);
  terminal_print("AS5047D_TIME_ns : ");
  terminal_println_int(AS5047D_TIME_ns);
  terminal_print("AS5047D_TIME_FAILURE_ns : ");
  terminal_println_int(AS5047D_TIME_FAILURE_ns);

  terminal_println("");
  terminal_println("Max values :");
  terminal_println("");
  terminal_print("MAXIMAL_PHASE_MOTOR_FREQ :");
  terminal_println_int(MAXIMAL_PHASE_MOTOR_FREQ);
  terminal_print("MAXIMAL_MOTOR_VELOCITY :");
  terminal_println_int(MAXIMAL_MOTOR_VELOCITY);

  terminal_println("");
  terminal_println("sampling factors :");
  terminal_println("");
  terminal_print("OVERSAMPLING_NUMBER :");
  terminal_println_int(OVERSAMPLING_NUMBER);
  terminal_print("NYQUIST_FACTOR :");
  terminal_println_int(NYQUIST_FACTOR);

  terminal_println("");
  terminal_println("Pulse :");
  terminal_print("ENC_SPI_DELAY :");
  terminal_println_int(ENC_SPI_DELAY);
}

TERMINAL_COMMAND(version, "firmware version")
{
  terminal_println(FIRMWARE_VERSION);
}

TERMINAL_COMMAND(led, "Set led")
{
  if(argc == 0){
    LED;
  }else if(argc == 1){
    if( atoi(argv[0])==1 ){
      LED_ON;
    }else{
      LED_OFF;
    }
  }else{
    terminal_println("Usage: led [1|0]");
  }
}

TERMINAL_COMMAND(test_us, "test micro oscillation on LED.")
{
  while(1){
    delay_us(1);
    LED;
  }
}

#ifdef DEBUG
TERMINAL_COMMAND(test_ms, "test milli oscillation on LED.")
{
  while(1){
    DELAY_MS(1);
    LED;
  }
}
#endif

TERMINAL_COMMAND(test_ns, "test milli oscillation on LED.")
{
  while(1){
    DELAY_AT_LEAST_NS(350);
    LED;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){
  if(hadc == &hadc1){
    RAISE_ERROR;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi){
  if(hspi == &hspi2){
    encoder_error_spi_call_back();
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){
  if(hspi == &hspi2){
    encoder_spi_call_back();
  }
}

TERMINAL_PARAMETER_BOOL(st, "decl", false);

void start_and_synchronize_timers(){
  // We start timer 1, 2 and 4
  if( HAL_TIM_Base_Start(&htim1) != HAL_OK ){
    raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  }
  if( HAL_TIM_Base_Start(&htim2) != HAL_OK ){
    raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  }
  if( HAL_TIM_Base_Start(&htim3) != HAL_OK ){
    raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  }
  
  //
  // Now, we will synchronize all timer by using the timer 4
  // Timer 4 is configured to send a trigger output (whose name is ITR3)
  // at each update.
  // Timer 1, 2, 3 are in slave mode, configured in reset mode and are waiting
  // for an input signal from TIM 4 (by using ITR3) to be reseted.
  //  
  if( HAL_TIM_Base_Start(&htim4) != HAL_OK ){
     raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  }
  delay_us(1); // We wait a little, to give time to TIM4 to be updated() (and 
  //DELAY_MS(10000); // We wait a little, to give time to TIM4 to be updated() (and 
                // ant trigger an output signal.
  if( HAL_TIM_Base_Stop(&htim4) != HAL_OK ){
    raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  }

  // We can now start the PWM of timer 2
  #ifdef PWM_ON_FOR_ADC_TICK
    if( HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ){
      RAISE_ERROR;
    }
  #endif
  if( HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK ){
    raise_error(ERROR_CURRENT, ADC_PWM);
  }
  #ifdef PWM_ON_FOR_ENCODER_TICK
    if( HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK ){
      RAISE_ERROR;
    }
  #endif
  // We can now start Output compare for 2 
  //if( HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2) != HAL_OK){
  //  raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  //};

  // We can now start Output compare and interuption for timer 1, 2 and 3.
  if( HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1) != HAL_OK){
    raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  };
  if( HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK){
    raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  };

  if( HAL_TIM_Base_Start_IT(&htim3) != HAL_OK ){
    raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  }

  if( HAL_TIM_Base_Start(&htim5) != HAL_OK ){
    raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  }
  if( HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_1) != HAL_OK){
    raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  };
  if( HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1) != HAL_OK){
    raise_error(ERROR_TIMER_INIT_AT_LINE, __LINE__);
  };
}

volatile bool complete_current_adc = false;
// Call back when ADC is finishedDC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
  if( hadc == &hadc1 ){
    //COOLDOWN(1000000){
    //  uint32_t v = hadc1.Instance->SR;
    //  BINT(v);
    //}
    COOLDOWN(2000000){
      terminal_println("-");
      for( uint32_t i=0; i< ADC_DATA_SIZE; i++ ){
        terminal_print("ADC : ");
        terminal_println_float(
          ((adc_data[i]-2048.0)*STM32_ADC_VOLTAGE_MAX*CURRENT_AO_GAIN)/4096
        );
      }
    }
    complete_current_adc = true;
  }
}
/* USER CODE END 0 */


static volatile uint32_t sr, cr1, cr2; 
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
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  serial_init();
  terminal_init(get_serial());
  
  system_init();
  encoder_init(&hspi2, ENC_INT_CS_GPIO_Port,ENC_INT_CS_Pin);
  //encoder_init(&hspi2, ENC_EXT_CS_GPIO_Port,ENC_EXT_CS_Pin);
  observer_init();
  motor_init();
  foc_init();
  #ifdef ACTIVATE_LOG
  log_init();
  #endif

  start_and_synchronize_timers();

  start_adc = true;
 
  sr = hadc1.Instance->SR;
  cr1 = hadc1.Instance->CR1;
  cr2 = hadc1.Instance->CR2;
  encoder_start();
  motor_start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    system_tick();
    encoder_tick();
    serial_tick();
    terminal_tick();
    #ifdef ACTIVATE_LOG
    log_tick();
    #endif
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = CLK_PLLM;
  RCC_OscInitStruct.PLL.PLLN = CLK_PLLN;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  _Static_assert( CLK_PLLP == RCC_PLLP_DIV4, "");
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
}

TERMINAL_COMMAND(test, ""){
  PRINTT("--- sr");
  BINT(sr);
  PRINTT("--- cr1");
  BINT(cr1);
  PRINTT("--- cr2");
  BINT(cr2);
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
  
  hadc1.Init.ScanConvMode = ENABLE;
  //hadc1.Init.ScanConvMode = DISABLE;

  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  //hadc1.Init.ContinuousConvMode = ENABLE;
  //hadc1.Init.DiscontinuousConvMode = DISABLE;
  //hadc1.Init.NbrOfDiscConversion = 0;

  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  //hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  //hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
  //hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;

  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  //hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.NbrOfConversion = ADC_DATA_SIZE;
  //hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  //hadc1.Init.EOCSelection = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_I_U;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }

#if ADC_DATA_SIZE > 1
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_I_V;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
#if ADC_DATA_SIZE > 2
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_I_W;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
#if ADC_DATA_SIZE > 3
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_I_U;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
#if ADC_DATA_SIZE > 4
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_I_V;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
#if ADC_DATA_SIZE > 5
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_I_W;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
#endif
#endif
#endif
#endif
#endif
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = ENCODER_SPI_BAUDRATE;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIM1_PRESCALAR;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = TIM1_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    main_error_handler(__LINE__);
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
    main_error_handler(__LINE__);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = TIM1_PRESCALAR;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  //if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  //sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  //sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = TIM2_CC1_PULSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  //sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  //if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sConfigOC.Pulse = TIM2_CC2_PULSE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIM3_PRESCALAR;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIM3_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = ENC_SPI_DELAY;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ENC_SPI_DELAY;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = TIM4_PRESCALAR;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = TIM4_PERIOD;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = TIM5_PRESCALAR;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = TIM5_PERIOD;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = COMP_DELAY;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    main_error_handler(__LINE__);
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, ENCODER_DMA_RX, ENCODER_DMA_RX_SUBPRIORITY);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, ENCODER_DMA_TX, ENCODER_DMA_TX_SUBPRIORITY);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, ADC_PRIORITY, ADC_SUBPRIORITY);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENC_INT_CS_GPIO_Port, ENC_INT_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, UEN_Pin|VEN_Pin|WEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENC_EXT_CS_GPIO_Port, ENC_EXT_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_INT_CS_Pin */
  GPIO_InitStruct.Pin = ENC_INT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ENC_INT_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UEN_Pin VEN_Pin WEN_Pin */
  GPIO_InitStruct.Pin = UEN_Pin|VEN_Pin|WEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_EXT_CS_Pin */
  GPIO_InitStruct.Pin = ENC_EXT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ENC_EXT_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void main_error_handler(uint32_t value)
{
  /* USER CODE BEGIN main_error_handler_Debug */
  raise_error(ERROR_STM32_HAL_LIBRARY, value);
  /* USER CODE END main_error_handler_Debug */
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
  raise_error(ERROR_ASSERTION, line);
  raise_error(ERROR_STRING, (uint32_t) file);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
