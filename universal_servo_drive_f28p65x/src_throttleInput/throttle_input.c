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

    //averaging
    averagingArr[currentIndex] = lastReadVoltage; //save current value into averaging array
    currentTotal = currentTotal+lastReadVoltage-averagingArr[(currentIndex+1)%maxIndex]; //have the running total add the neweset and remove the oldest
    currentIndex=(currentIndex+1)%maxIndex; //iterate averaging index

    float32_t currentThrottleVal = currentTotal/(maxIndex-1);//average the number of voltages we have read (minus one from removing oldest)

    //setting of speed
    if(currentThrottleVal > throttle_thresh_low && currentThrottleVal < throttle_thresh_high){
        return 0;
    }
    // https://www.desmos.com/calculator/ynxfuuraxl 
    else if(currentThrottleVal < throttle_thresh_low){
        return (-1*throttle_speed_max+(throttle_speed_max/throttle_thresh_low * currentThrottleVal)); //-max+ max/deadLow * input
    }
    else{
        return ((throttle_speed_max/throttle_thresh_low * currentThrottleVal) - (throttle_thresh_high* throttle_speed_max/throttle_thresh_low)); // max/deadLow * input - deadHigh*max/deadLow
    }
}





