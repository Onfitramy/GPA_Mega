#ifndef PowerUnit_H_
#define PowerUnit_H_

#include "main.h"

void PU_enableRecovery();
void PU_enableCamera();
void PU_enableACS();

void PU_disableRecovery();
void PU_disableCamera();
void PU_disableACS();

void Camera_SwitchOn();
void Camera_WifiOn();
void Camera_StartRecording();

void Camera_SwitchOff();
void Camera_WifiOff();
void Camera_StopRecording();

void Camera_SkipDate();

#endif /* PowerUnit_H_ */