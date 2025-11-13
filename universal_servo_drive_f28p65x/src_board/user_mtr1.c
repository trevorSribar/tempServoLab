//#############################################################################
// $Copyright:
// Copyright (C) 2017-2025 Texas Instruments Incorporated - http://www.ti.com/
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
//! \file   /solutions/x/common/source/user_mtr1.c
//! \brief  Function for setting initialization data for setting user parameters
//!
//

#include "user.h"

#include "hal.h"

// the globals
USER_Params userParams_M1;
#pragma DATA_SECTION(userParams_M1,"user_data");

//*****************************************************************************
//
// USER_setParams, set control parameters for motor 1
//
//*****************************************************************************
void USER_setMotor1Params(userParams_Handle handle)
{
    USER_Params *objUser = (USER_Params *)handle;

    objUser->numIsrTicksPerCtrlTick = 1;
    objUser->numIsrTicksPerTrajTick = 1;

    objUser->numCtrlTicksPerCurrentTick = 1;
    objUser->numCtrlTicksPerSpeedTick = USER_M1_NUM_ISR_TICKS_PER_SPEED_TICK;

    objUser->numCurrentSensors = USER_M1_NUM_CURRENT_SENSORS;
    objUser->numVoltageSensors = USER_M1_NUM_VOLTAGE_SENSORS;

    objUser->motor_numPolePairs = USER_MOTOR1_NUM_POLE_PAIRS;

#if defined(MOTOR1_ENC)
    objUser->motor_numEncSlots = USER_MOTOR1_NUM_ENC_SLOTS;
#endif // MOTOR1_ENC

    objUser->dcBus_nominal_V = USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;

    objUser->systemFreq_MHz = USER_SYSTEM_FREQ_MHz;

    objUser->voltage_sf = USER_M1_VOLTAGE_SF;

    objUser->current_sf = USER_M1_CURRENT_SF;

    objUser->maxVsMag_pu = USER_M1_MAX_VS_MAG_PU;

    objUser->motor_ratedFlux_Wb = USER_MOTOR1_RATED_FLUX_VpHz / MATH_TWO_PI;

    objUser->motor_Rr_Ohm = USER_MOTOR1_Rr_Ohm;
    objUser->motor_Rs_Ohm = USER_MOTOR1_Rs_Ohm;

    objUser->motor_Ls_d_H = USER_MOTOR1_Ls_d_H;
    objUser->motor_Ls_q_H = USER_MOTOR1_Ls_q_H;

    objUser->maxCurrent_A = USER_MOTOR1_MAX_CURRENT_A;

    objUser->Vd_sf = USER_M1_VD_SF;
    objUser->maxVsMag_V = USER_MOTOR1_RATED_VOLTAGE_V * objUser->maxVsMag_pu;

    objUser->IdRated_A = USER_MOTOR1_MAGNETIZING_CURRENT_A;

//    objUser->BWc_rps = MATH_TWO_PI * (float32_t)200.0f;
    objUser->BWc_rps = MATH_TWO_PI * USER_M1_PWM_FREQ_kHz * USER_M1_CURRENT_BW_SF;
//    objUser->BWdelta = (float32_t)20.0f;
    objUser->BWdelta = USER_M1_PWM_FREQ_kHz * USER_M1_BW_DELTA_SF;

    objUser->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                               objUser->motor_numPolePairs *
                               objUser->motor_ratedFlux_Wb /
                               (float32_t) (2.0f * USER_MOTOR1_INERTIA_Kgm2);

    objUser->ctrlFreq_Hz = USER_M1_ISR_FREQ_Hz;

    objUser->trajFreq_Hz = USER_M1_ISR_FREQ_Hz;
    objUser->ctrlPeriod_sec = USER_M1_CTRL_PERIOD_sec;

    objUser->maxAccel_Hzps = USER_M1_MAX_ACCEL_Hzps;

//    objUser->RoverL_Kp_sf = USER_M1_R_OVER_L_KP_SF;
//    objUser->RoverL_min_rps = MATH_TWO_PI * (float32_t)5.0f;
//    objUser->RoverL_max_rps = MATH_TWO_PI * (float32_t)5000.0f;

    objUser->Rs_Ohm = (float32_t)0.0;

    objUser->Ls_d_H = (float32_t)1.0e-6;
    objUser->Ls_q_H = (float32_t)1.0e-6;

    objUser->maxFrequency_Hz = USER_MOTOR1_FREQ_MAX_Hz;

    return;
} // end of USER_setParams() function
