# Describes the hierarchy of includes
stm32h7xx_hal.h can be included at every level if needed

# Overview
calibration_data.h -> Components -> _components.h -> Libraries -> _libraries.h -> Application code (main_app.c, statemachine.h)

# Notes

Libraries might be interconnected, but instances where the include each other should be tighly controlled and only happen inside the .c file