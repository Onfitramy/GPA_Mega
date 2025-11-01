/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_tim3_ch3;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}
/* USER CODE BEGIN PV */
// Fault diagnostic variables
volatile uint32_t fault_pc = 0;
volatile uint32_t fault_lr = 0;
volatile uint32_t fault_psr = 0;
volatile uint32_t fault_cfsr = 0;
volatile uint32_t fault_hfsr = 0;
volatile uint32_t fault_bfar = 0;
volatile uint32_t fault_mmfar = 0;
volatile uint32_t fault_stack[8] = {0};
volatile const char* fault_task_name = "Unknown";
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void HardFault_Handler_C(uint32_t *stack_ptr);
/* USER CODE END PFP */

// Enhanced fault handler
void HardFault_Handler_C(uint32_t *stack_ptr) {
    // Capture stacked registers
    fault_stack[0] = stack_ptr[0]; // R0
    fault_stack[1] = stack_ptr[1]; // R1
    fault_stack[2] = stack_ptr[2]; // R2
    fault_stack[3] = stack_ptr[3]; // R3
    fault_stack[4] = stack_ptr[4]; // R12
    fault_stack[5] = stack_ptr[5]; // LR (return address)
    fault_stack[6] = stack_ptr[6]; // PC (program counter where fault occurred)
    fault_stack[7] = stack_ptr[7]; // PSR (status register)
    
    fault_pc = stack_ptr[6];  // PC
    fault_lr = stack_ptr[5];  // LR
    fault_psr = stack_ptr[7]; // PSR
    
    // Read fault status registers
    fault_cfsr = SCB->CFSR;   // Configurable Fault Status Register
    fault_hfsr = SCB->HFSR;   // HardFault Status Register
    fault_bfar = SCB->BFAR;   // BusFault Address Register
    fault_mmfar = SCB->MMFAR; // MemManage Fault Address Register
    
    // Get current task name (if FreeRTOS is running)
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    if (current_task != NULL) {
        fault_task_name = pcTaskGetName(current_task);
    }
    
    // Set LED to red
    SetLED_color(COLOR_RED);
    
    // Trigger breakpoint if debugger attached
    __BKPT(0);
    
    // Infinite loop
    while(1) {
    }
}
/* USER CODE END 0 */

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  // Determine which stack pointer was in use and call C handler
  __asm volatile (
    "TST LR, #4           \n"  // Test bit 2 of LR (EXC_RETURN)
    "ITE EQ               \n"  // If-Then-Else
    "MRSEQ R0, MSP        \n"  // Main stack pointer
    "MRSNE R0, PSP        \n"  // Process stack pointer
    "B HardFault_Handler_C\n"  // Jump to C handler with stack pointer in R0
  );
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    SetLED_color(COLOR_RED);
    #ifdef DEBUG
     __BKPT(0);
    #endif
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  printf("\n=== BusFault ===\n");
  printf("CFSR: 0x%08lX\n", SCB->CFSR);
  printf("BFAR: 0x%08lX\n", SCB->BFAR);
  if (SCB->CFSR & (1 << 15)) {
    printf("BusFault Address: 0x%08lX\n", SCB->BFAR);
  }
  SetLED_color(COLOR_RED);
  __BKPT(0);
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(MAG_INT_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(IMU2_DEN_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(IMU2_INT_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim8_ch1);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim3_ch3);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  if (__HAL_GPIO_EXTI_GET_IT(NRF_INT_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(NRF_INT_Pin);
    }

  else if (__HAL_GPIO_EXTI_GET_IT(BMP_INT_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(BMP_INT_Pin);
    }

  else if (__HAL_GPIO_EXTI_GET_IT(MAG_DRDY_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(MAG_DRDY_Pin);
    }
  /* USER CODE END EXTI9_5_IRQn 0 */
  /*HAL_GPIO_EXTI_IRQHandler(NRF_INT_Pin);
  HAL_GPIO_EXTI_IRQHandler(BMP_INT_Pin);
  HAL_GPIO_EXTI_IRQHandler(MAG_DRDY_Pin);*/
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  if (__HAL_GPIO_EXTI_GET_IT(GNSS_TX_RDY_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(GNSS_TX_RDY_Pin);
    }
  
  else if (__HAL_GPIO_EXTI_GET_IT(GNSS_TP_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(GNSS_TP_Pin);
    }
  
  else if (__HAL_GPIO_EXTI_GET_IT(IMU1_INT1_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(IMU1_INT1_Pin);
    }

  else if (__HAL_GPIO_EXTI_GET_IT(IMU1_INT2_Pin) != RESET)
    {
        HAL_GPIO_EXTI_IRQHandler(IMU1_INT2_Pin);
    }
  /* USER CODE END EXTI15_10_IRQn 0 */
  //HAL_GPIO_EXTI_IRQHandler(GNSS_TP_Pin);
  //HAL_GPIO_EXTI_IRQHandler(IMU1_INT1_Pin);
  //HAL_GPIO_EXTI_IRQHandler(IMU1_INT2_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */

  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */

  /* USER CODE END TIM17_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS global interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/**
  * @brief This function handles BDMA channel0 global interrupt.
  */
void BDMA_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN BDMA_Channel0_IRQn 0 */

  /* USER CODE END BDMA_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN BDMA_Channel0_IRQn 1 */

  /* USER CODE END BDMA_Channel0_IRQn 1 */
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
