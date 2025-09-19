#include "PowerUnit.h"

void PU_enableRecovery() {
    HAL_GPIO_WritePin(Recovery_GPIO_Port, Recovery_Pin, 1);
}

void PU_enableCamera() {
    HAL_GPIO_WritePin(CAMS_GPIO_Port, CAMS_Pin, 1);
}

void PU_enableACS() {
    HAL_GPIO_WritePin(ACS_GPIO_Port, ACS_Pin, 1);
}

void PU_disableRecovery() {
    HAL_GPIO_WritePin(Recovery_GPIO_Port, Recovery_Pin, 0);
}

void PU_disableCamera() {
    HAL_GPIO_WritePin(CAMS_GPIO_Port, CAMS_Pin, 0);
}

void PU_disableACS() {
    HAL_GPIO_WritePin(ACS_GPIO_Port, ACS_Pin, 0);
}

void Camera_SwitchOn() {
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 0);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 1);
}

void Camera_WifiOn() {
    HAL_GPIO_WritePin(GPIO24_GPIO_Port, GPIO24_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO24_GPIO_Port, GPIO24_Pin, 1);
}

void Camera_StartRecording() {
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
}

void Camera_SwitchOff() {
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 0);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 1);
}

void Camera_WifiOff() {
    HAL_GPIO_WritePin(GPIO24_GPIO_Port, GPIO24_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO24_GPIO_Port, GPIO24_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO21_GPIO_Port, GPIO21_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
}

void Camera_StopRecording() {
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
}

void Camera_SkipDate() {
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 0);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIO23_GPIO_Port, GPIO23_Pin, 1);
}