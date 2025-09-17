#include "control.h"

float fconstrain(float variable, float min, float max) {
    if(variable > max) {
        return max;
    } else if(variable < min) {
        return min;
    } else {
        return variable;
    }
}