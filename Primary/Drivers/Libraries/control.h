#ifndef CONTROL_H
#define CONTROL_H

#include <math.h>

#define LENGTH_AE 45.0f
#define LENGTH_BE 50.0f
#define LENGTH_BD 140.0f

#define POSITION_BX -2.0f
#define POSITION_CX 18.0f
#define POSITION_CY 155.6f

#define F_DIST 6.0f

#define ROD_SLOPE 8.0f

void CircleIntersectionPoints(float x1, float y1, float r1, float x2, float y2, float r2,
                              float *xi1, float *yi1, float *xi2, float *yi2, int *num_intersections);

void ACSAngleFromStepperPosition(float stepper_pos, float *acs_angle_deg);
void StepperPositionFromACSAngle(float acs_angle_deg, float *stepper_pos);


#endif // CONTROL_H