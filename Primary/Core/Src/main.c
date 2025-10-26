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
#include "adc.h"
#include "dma.h"
#include "dts.h"
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "Pyro.h"

#include "InterBoardCom.h"
#include <stdint.h>
#include "cli_app.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile uint8_t dma_waiting_ws2812;
uint16_t adc3_buf[3];
uint32_t uid[3];

extern float ADC_Temperature, ADC_V_Sense, ADC_V_Ref;
extern uint8_t SPI1_State; // 0: Ready, 1: TX busy, 2: RX busy
extern volatile uint8_t SPI1_ReceivePending; // Flag to indicate a pending receive request

extern StateMachine_t flight_sm;
extern bool is_groundstation;

volatile uint32_t tim7_ms = 0;
uint32_t tim7_target_ms;
volatile uint32_t tim13_ms = 0;
uint32_t tim13_target_ms;
volatile uint32_t tim14_ms = 0;
uint32_t tim14_target_ms;
volatile uint32_t tim16_ms = 0;
uint32_t tim16_target_ms;
volatile uint32_t tim17_ms = 0;
uint32_t tim17_target_ms;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
//#define TRANSMITTER

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

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  //SCB_EnableDCache();

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
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI6_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_DTS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_DTS_Start(&hdts);

  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  // find out board no.
  uid[0] = HAL_GetUIDw0();
  uid[1] = HAL_GetUIDw1();
  uid[2] = HAL_GetUIDw2();
  gpa_mega = GPA_MegaFromUID(uid);

  // if board is a ground station, set flag
  if (gpa_mega == GPA_MEGA_3 || gpa_mega == GPA_MEGA_1) {
    is_groundstation = true;
    cli_target_mode = CLI_TARGET_MODE_EXTERNAL; // Groundstation always uses internal target mode
  }

  // define output signal names
  signalPlotter_init();

  // state machine init
  StateMachine_Init(&flight_sm, STATE_FLIGHT_STARTUP);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    continue;
    //HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
    //HAL_Delay(1000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

  /*Handle GPS_INT by sending Queue to FreeRTOS funktion*/
  if (GPIO_Pin == GNSS_TX_RDY_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t sendData = 0x10;
    if(xQueueIsQueueEmptyFromISR(InterruptQueue)) {
      xQueueSendFromISR(InterruptQueue, &sendData, &xHigherPriorityTaskWoken);
    } //Send the GPIO_Pin to the Interrupt queue to be handled by the task
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Perform a context switch if needed
  }
  
  /*Handle NRF_INT by sending Queue to FreeRTOS funktion*/
  if (GPIO_Pin == NRF_INT_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t sendData = 0x11;
    xQueueSendFromISR(InterruptQueue, &sendData, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
  if (hspi->Instance == SPI1) {
    SPI1_State = 0;
    // DMA transfer complete callback for SPI1
    // Process the received data in receiveBuffer
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // Deactivate CS
    InterBoardPacket_t receivedPacket = InterBoardCom_ReceivePacket();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(InterBoardCom_Queue, &receivedPacket, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        // Reactivate DMA receive on error
        return;
    }
}

/*UNUSED FOR NOW, NEEDED IF DMA TEMP IS REQ*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // Calculate The Temperature
  ADC_V_Ref = (float)((V_REF_INT * 4095.0)/adc3_buf[0]);
  ADC_V_Sense = (float)(adc3_buf[1] * ADC_V_Ref) / 4095.0;
  ADC_Temperature = (((V_AT_25C - ADC_V_Sense) * 1000.0) /AVG_SLOPE) + 25.0;
}

uint32_t HAL_GetTickUS(){
  return TIM5->CNT; // Get the current value of TIM5 counter
}

uint32_t HAL_GetTickDiffUS(uint32_t start){
  uint32_t now = HAL_GetTickUS();
  if (now >= start) {
    return now - start;
  } else {
    // Handle overflow
    return (0xFFFFFFFF - start + now + 1);
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 31;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 10;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 2048;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_SPI4;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 6;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_1);
        dma_waiting_ws2812 = 0;
    }
}
/* USER CODE END 4 */

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
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  // Timer for handling maximum event delay times of StateMachine
  else if (htim->Instance == TIM7) {
    tim7_ms++;
    if (tim7_ms >= tim7_target_ms) {
      tim7_ms = 0;
      HAL_TIM_Base_Stop_IT(&htim7);

      // handle max state timer elapsed
      switch (flight_sm.currentFlightState) {
        case STATE_FLIGHT_BURN:             StateMachine_Dispatch(&flight_sm, EVENT_FLIGHT_BURNOUT_DETECTED); break;
        case STATE_FLIGHT_COAST:            StateMachine_Dispatch(&flight_sm, EVENT_FLIGHT_DROGUE_COMMANDED); break;
        case STATE_FLIGHT_AWAIT_DROGUE:     StateMachine_Dispatch(&flight_sm, EVENT_FLIGHT_TOUCHDOWN); break;
        case STATE_FLIGHT_DESCEND_DROGUE:   StateMachine_Dispatch(&flight_sm, EVENT_FLIGHT_MAIN_COMMANDED); break;
        case STATE_FLIGHT_AWAIT_MAIN:       StateMachine_Dispatch(&flight_sm, EVENT_FLIGHT_TOUCHDOWN); break;
        case STATE_FLIGHT_DESCEND_MAIN:     StateMachine_Dispatch(&flight_sm, EVENT_FLIGHT_TOUCHDOWN); break;
        default: break; // error?
      }
    }
  }
  // Timer for handling drogue deploy servo signal
  else if (htim->Instance == TIM13) {
    tim13_ms++;
    if (tim13_ms >= tim13_target_ms) {
      tim13_ms = 0;
      HAL_TIM_Base_Stop_IT(&htim13);

      RetractDrogue();
    }
  }
  // Timer for handling general delay
  else if (htim->Instance == TIM14) {
    tim14_ms++;
    if (tim14_ms >= tim14_target_ms) {
      tim14_ms = 0;
      HAL_TIM_Base_Stop_IT(&htim14);
    }
  }
  // Timer for handling main deploy servo retraction
  else if (htim->Instance == TIM16) {
    tim16_ms++;

    RetractMainHandler(tim16_ms);

    if (tim16_ms >= tim16_target_ms) {
      tim16_ms = 0;
      HAL_TIM_Base_Stop_IT(&htim16);
    }
  }
  // Timer for handling general delay
  else if (htim->Instance == TIM17) {
    tim17_ms++;
    if (tim17_ms >= tim17_target_ms) {
      tim17_ms = 0;
      HAL_TIM_Base_Stop_IT(&htim17);
    }
  }
  
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
