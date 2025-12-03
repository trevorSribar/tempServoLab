#include "throttle_input.h"
volatile float32_t lastReadVoltage = 0.0;

float32_t throttle_get_ADC_Val(){
    return (float32_t)ADC_readResult(THROTTLE_ADC, THROTTLE_ADC_SOC);
}

float32_t throttle_get_newest_speedFreq(){
    float32_t currentThrottleVal = throttle_get_ADC_Val();
    lastReadVoltage = currentThrottleVal;
    if(currentThrottleVal<throttle_thresh_max_n3){
        return throttle_speed_n3;
    }
    else if(currentThrottleVal<throttle_thresh_max_n2){
        return throttle_speed_n2;
    }
    else if(currentThrottleVal<throttle_thresh_max_n1){
        return throttle_speed_n1;
    }
    else if(currentThrottleVal<throttle_thresh_max_0){
        return throttle_speed_0;
    }
    else if(currentThrottleVal<throttle_thresh_max_1){
        return throttle_speed_1;
    }
    else if(currentThrottleVal<throttle_thresh_max_2){
        return throttle_speed_2;
    }
    else{
        return throttle_speed_3;
    }
}





