#include "throttle_input.h"
volatile float32_t lastReadVoltage = 0.0;
uint16_t currentIndex = 0;
float32_t averagingArr[maxIndex];
float32_t currentTotal = 0;

uint16_t AdcBuf[ADC_BUF_LEN];

float32_t throttle_get_ADC_Val(){
    //return 
    float32_t tempThrottle = (float32_t)ADC_readResult(THROTTLE_ADC, THROTTLE_ADC_SOC);
    tempThrottle = tempThrottle * 3.3/4095.0;
    return tempThrottle;
    // static uint16_t *ThrottleAdcPtr = AdcBuf;
    // *ThrottleAdcPtr++ = ADC_readResult(THROTTLE_ADC, THROTTLE_ADC_SOC);
    // ThrottleAdcPtr = (ThrottleAdcPtr - AdcBuf) % ADC_BUF_LEN + AdcBuf;
    //return (float32_t)*ThrottleAdcPtr;
}

float32_t throttle_get_newest_speedFreq(){
    //float32_t currentThrottleVal = throttle_get_ADC_Val();
    //lastReadVoltage = currentThrottleVal;
    lastReadVoltage = throttle_get_ADC_Val();
    float32_t currentThrottleVal = lastReadVoltage;

    //averaging
    averagingArr[currentIndex] = lastReadVoltage;
    currentTotal = currentTotal+lastReadVoltage-averagingArr[(currentIndex+1)%maxIndex];
    currentIndex=(currentIndex+1)%maxIndex;
    currentThrottleVal = currentTotal/(maxIndex-1);

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





