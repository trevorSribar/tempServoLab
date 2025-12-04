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

#define throttle_thresh_max_n3  (float32_t)0.35
#define throttle_thresh_max_n2  (float32_t)0.9
#define throttle_thresh_max_n1  (float32_t)1.4
#define throttle_thresh_max_0   (float32_t)2.1
#define throttle_thresh_max_1   (float32_t)2.7
#define throttle_thresh_max_2   (float32_t)3.15

#define throttle_speed_n3   (float32_t)-120.0
#define throttle_speed_n2   (float32_t)-80.0
#define throttle_speed_n1   (float32_t)-30.0
#define throttle_speed_0    (float32_t)0.0
#define throttle_speed_1    (float32_t)30.0
#define throttle_speed_2    (float32_t)80.0
#define throttle_speed_3    (float32_t)120.0

float32_t throttle_get_ADC_Val();

float32_t throttle_get_newest_speedFreq();

#endif //this should work




