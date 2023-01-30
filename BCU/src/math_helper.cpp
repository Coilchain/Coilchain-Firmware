#include <stdint.h>

#include <math_helper.h>

/* Maps x from range [in_min, in_max] to range [out_min, out_max] */
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrainf(float x, float min, float max)
{
    if (x > max){
        return max;
    }
    else if (x < min){
        return min;
    }
    else {
        return x;
    }
}