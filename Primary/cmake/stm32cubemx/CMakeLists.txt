cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_PWR_LDO_SUPPLY 
	USE_HAL_DRIVER 
	STM32H723xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../USB_DEVICE/App
    ../../USB_DEVICE/Target
    ../../Drivers/STM32H7xx_HAL_Driver/Inc
    ../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ../../Middlewares/Third_Party/FreeRTOS/Source/include
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ../../Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ../../Drivers/CMSIS/Include

    ../../Drivers/Components/LED
    ../../Drivers/Components/IMU1
    ../../Drivers/Components/IMU2
    ../../Drivers/Components/MAG
    ../../Drivers/Components/BARO
    ../../Drivers/Components/GPS
    ../../Drivers/Components/Servo
    ../../Drivers/Components/PYRO
    ../../Drivers/Components/Stepper
    ../../Drivers/Components/NRF
    ../../Drivers/CMSIS/DSP_DRIVERS/Include
    ../../Drivers/CMSIS/DSP_DRIVERS/Bin
    ../../Drivers/Libraries
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/gpio.c
    ../../Core/Src/freertos.c
    ../../Core/Src/FreeRTOS_CLI.c
    ../../Core/Src/cli_app.c
    ../../Core/Src/adc.c
    ../../Core/Src/dma.c
    ../../Core/Src/i2c.c
    ../../Core/Src/memorymap.c
    ../../Core/Src/spi.c
    ../../Core/Src/tim.c
    ../../Core/Src/stm32h7xx_it.c
    ../../Core/Src/stm32h7xx_hal_msp.c
    ../../Core/Src/stm32h7xx_hal_timebase_tim.c
    ../../USB_DEVICE/App/usb_device.c
    ../../USB_DEVICE/App/usbd_desc.c
    ../../USB_DEVICE/App/usbd_cdc_if.c
    ../../USB_DEVICE/Target/usbd_conf.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
    ../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c
    ../../Core/Src/system_stm32h7xx.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32h723xx.s

    ../../Drivers/Components/LED/ws2812.c
    ../../Drivers/Components/IMU1/LSM6DSR.c
    ../../Drivers/Components/IMU2/ISM330DHCX.c
    ../../Drivers/Components/MAG/LIS3MDL.c
    ../../Drivers/Components/BARO/bmp390.c
    ../../Drivers/Components/GPS/SAM-M8Q.c
    ../../Drivers/Components/GPS/u_ubx_protocol.c
    ../../Drivers/Components/Servo/SERVO.c
    ../../Drivers/Components/Servo/SERVO_cfg.c
    ../../Drivers/Components/PYRO/Pyro.c
    ../../Drivers/Components/Stepper/Stepper.c
    ../../Drivers/Components/NRF/NRF24L01P.c

    ../../Drivers/Libraries/signalPlotter.c
    ../../Drivers/Libraries/calibration_data.c
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


