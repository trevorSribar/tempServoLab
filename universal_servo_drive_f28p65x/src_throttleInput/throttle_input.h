#ifndef THROTTLE_INPUT_H
#define THROTTLE_INPUT_H

//! \file   src_throttleInput/throttle_input.h

#include "hal.h"
#include "board.h"

#define ENABLED 1
#define DISABLED 0

#define ADC_BUF_LEN 50
#define maxIndex 100

#define throttle_input_enable ENABLED

#define THROTTLE_ADC_SOC    ADCC_CONFIG_SOC5
#define THROTTLE_ADC        ADCC_CONFIG_RESULT_BASE

#define throttle_thresh_low  (float32_t)1.8
#define throttle_thresh_high (float32_t)1.5

#define throttle_speed_max   (float32_t)60

float32_t throttle_get_ADC_Val();

float32_t throttle_get_newest_speedFreq();

#endif //this should work




