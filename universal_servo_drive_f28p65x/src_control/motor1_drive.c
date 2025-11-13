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


//! \file   /solutions/universal_servo_drive/common/source/motor1_drive.c
//!
//! \brief  This project is used to implement sensored-FOC motor control
//!         with Encoders.
//!

//
// include the related header files
//
#include "sys_settings.h"
#include "sys_main.h"
#include "motor1_drive.h"
#include "board.h" // syscfg

#pragma CODE_SECTION(motor1CtrlISR, ".TI.ramfunc");
#pragma INTERRUPT(motor1CtrlISR, {HP});

// the globals

//!< the hardware abstraction layer object to motor control
volatile MOTOR_Handle motorHandle_M1;
#pragma DATA_SECTION(motorHandle_M1,"foc_data");

volatile MOTOR_Vars_t motorVars_M1;
#pragma DATA_SECTION(motorVars_M1, "foc_data");

MOTOR_SetVars_t motorSetVars_M1;
#pragma DATA_SECTION(motorSetVars_M1, "foc_data");

HAL_MTR_Obj    halMtr_M1;
#pragma DATA_SECTION(halMtr_M1, "foc_data");

//!< the current Clarke transform object
CLARKE_Obj    clarke_I_M1;
#pragma DATA_SECTION(clarke_I_M1, "foc_data");

//!< the inverse Park transform object
IPARK_Obj     ipark_V_M1;
#pragma DATA_SECTION(ipark_V_M1, "foc_data");

//!< the Park transform object
PARK_Obj      park_I_M1;
#pragma DATA_SECTION(park_I_M1, "foc_data");

//!< the Park transform object
PARK_Obj      park_V_M1;
#pragma DATA_SECTION(park_V_M1, "foc_data");

//!< the Id PI controller object
PI_Obj        pi_Id_M1;
#pragma DATA_SECTION(pi_Id_M1, "foc_data");

//!< the Iq PI controller object
PI_Obj        pi_Iq_M1;
#pragma DATA_SECTION(pi_Iq_M1, "foc_data");

//!< the speed PI controller object
PI_Obj        pi_spd_M1;
#pragma DATA_SECTION(pi_spd_M1, "foc_data");

//!< the space vector generator object
SVGEN_Obj     svgen_M1;
#pragma DATA_SECTION(svgen_M1, "foc_data");

#if defined(MOTOR1_OVM)
//!< the handle for the space vector generator current
SVGENCURRENT_Obj svgencurrent_M1;
#pragma DATA_SECTION(svgencurrent_M1, "foc_data");
#endif  // MOTOR1_OVM

//!< the speed reference trajectory object
TRAJ_Obj     traj_spd_M1;
#pragma DATA_SECTION(traj_spd_M1, "foc_data");

#if defined(MOTOR1_FWC)
//!< the fwc PI controller object
PI_Obj       pi_fwc_M1;
#pragma DATA_SECTION(pi_fwc_M1, "foc_data");
#endif  // MOTOR1_FWC

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_ENC)
//!< the Angle Generate onject for open loop control
ANGLE_GEN_Obj    angleGen_M1;
#pragma DATA_SECTION(angleGen_M1, "foc_data");
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ENC

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
//!< the Vs per Freq object for open loop control
VS_FREQ_Obj    VsFreq_M1;
#pragma DATA_SECTION(VsFreq_M1, "foc_data");
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(MOTOR1_ENC)
//!< the handle for the enc object
ENC_Obj enc_M1;
#pragma DATA_SECTION(enc_M1, "foc_data");

//!< the handle for the speedcalc object
SPDCALC_Obj speedcalc_M1;
#pragma DATA_SECTION(speedcalc_M1, "foc_data");
#endif  // MOTOR1_ENC

#if defined(MOTOR1_MTPA)
//!< the Maximum torque per ampere (MTPA) object
MTPA_Obj     mtpa_M1;
#pragma DATA_SECTION(mtpa_M1, "foc_data");
#endif  // MOTOR1_MTPA

#if defined(MOTOR1_FILTERIS)
//!< first order current filter object
FILTER_FO_Obj    filterIs_M1[3];

#pragma DATA_SECTION(filterIs_M1, "foc_data");
#endif  // MOTOR1_FILTERIS

#if defined(BENCHMARK_TEST)
BMTEST_Vars_t bmarkTestVars;

#pragma DATA_SECTION(bmarkTestVars, "foc_data");
#endif  // BENCHMARK_TEST


// the control handles for motor 1
void initMotor1Handles(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    obj->motorNum = MTR_1;

    // initialize the driver
    obj->halMtrHandle = HAL_MTR1_init(&halMtr_M1, sizeof(halMtr_M1));

    obj->motorSetsHandle = &motorSetVars_M1;
    obj->userParamsHandle = &userParams_M1;

    return;
}

// initialize control parameters for motor 1
void initMotor1CtrlParameters(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    // initialize the user parameters
    USER_setMotor1Params(obj->userParamsHandle);

    // set the driver parameters
    HAL_MTR_setParams(obj->halMtrHandle, obj->userParamsHandle);

    objSets->Kp_spd = 0.05f;
    objSets->Ki_spd = 0.005f;

    objSets->Kp_fwc = USER_M1_FWC_KP;
    objSets->Ki_fwc = USER_M1_FWC_KI;

    objSets->angleFWCMax_rad = USER_M1_FWC_MAX_ANGLE_RAD;
    objSets->overModulation = USER_M1_MAX_VS_MAG_PU;

//    objSets->RsOnLineCurrent_A = 0.1f * USER_MOTOR1_MAX_CURRENT_A;

    objSets->lostPhaseSet_A = USER_M1_LOST_PHASE_CURRENT_A;
    objSets->unbalanceRatioSet = USER_M1_UNBALANCE_RATIO;
    objSets->overLoadSet_W = USER_M1_OVER_LOAD_POWER_W;

    objSets->toqueFailMinSet_Nm = USER_M1_TORQUE_FAILED_SET;
    objSets->speedFailMaxSet_Hz = USER_M1_FAIL_SPEED_MAX_HZ;
    objSets->speedFailMinSet_Hz = USER_M1_FAIL_SPEED_MIN_HZ;

    objSets->stallCurrentSet_A = USER_M1_STALL_CURRENT_A;
    objSets->IsFailedChekSet_A = USER_M1_FAULT_CHECK_CURRENT_A;

    objSets->maxPeakCurrent_A = USER_M1_FULL_SCALE_CURRENT_A * 0.475f;
    objSets->overCurrent_A = USER_MOTOR1_OVER_CURRENT_A;
    objSets->currentInv_sf = USER_M1_CURRENT_INV_SF;

    objSets->overVoltageFault_V = USER_M1_OVER_VOLTAGE_FAULT_V;
    objSets->overVoltageNorm_V = USER_M1_OVER_VOLTAGE_NORM_V;
    objSets->underVoltageFault_V = USER_M1_UNDER_VOLTAGE_FAULT_V;
    objSets->underVoltageNorm_V = USER_M1_UNDER_VOLTAGE_NORM_V;

    objSets->overCurrentTimesSet = USER_M1_OVER_CURRENT_TIMES_SET;
    objSets->voltageFaultTimeSet = USER_M1_VOLTAGE_FAULT_TIME_SET;
    objSets->motorStallTimeSet = USER_M1_STALL_TIME_SET;
    objSets->startupFailTimeSet = USER_M1_STARTUP_FAIL_TIME_SET;

    objSets->overSpeedTimeSet = USER_M1_OVER_SPEED_TIME_SET;
    objSets->overLoadTimeSet = USER_M1_OVER_LOAD_TIME_SET;
    objSets->unbalanceTimeSet = USER_M1_UNBALANCE_TIME_SET;
    objSets->lostPhaseTimeSet = USER_M1_LOST_PHASE_TIME_SET;

    objSets->stopWaitTimeSet = USER_M1_STOP_WAIT_TIME_SET;
    objSets->restartWaitTimeSet = USER_M1_RESTART_WAIT_TIME_SET;
    objSets->restartTimesSet = USER_M1_START_TIMES_SET;


    objSets->dacCMPValH = 2048U + 1024U;    // set default positive peak value
    objSets->dacCMPValL = 2048U - 1024U;    // set default negative peak value

    obj->senseData.current_sf = objUser->current_sf * USER_M1_SIGN_CURRENT_SF;

    obj->senseData.voltage_sf = objUser->voltage_sf;
    obj->senseData.dcBusvoltage_sf = objUser->voltage_sf;

    obj->speedStart_Hz = USER_MOTOR1_SPEED_START_Hz;
    obj->speedForce_Hz = USER_MOTOR1_SPEED_FORCE_Hz;

    obj->accelerationMax_Hzps = USER_MOTOR1_ACCEL_MAX_Hzps;
    obj->accelerationStart_Hzps = USER_MOTOR1_ACCEL_START_Hzps;

    obj->VsRef_pu = 0.98f * USER_M1_MAX_VS_MAG_PU;
    obj->VsRef_V =
            0.98f * USER_M1_MAX_VS_MAG_PU * USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;

    obj->IsSet_A = USER_MOTOR1_TORQUE_CURRENT_A;

    obj->fluxCurrent_A = USER_MOTOR1_FLUX_CURRENT_A;
    obj->alignCurrent_A = USER_MOTOR1_ALIGN_CURRENT_A;
    obj->startCurrent_A = USER_MOTOR1_STARTUP_CURRENT_A;
    obj->maxCurrent_A = USER_MOTOR1_MAX_CURRENT_A;

    obj->anglePhaseAdj_rad = MATH_PI * 0.001f;

    obj->power_sf = MATH_TWO_PI / USER_MOTOR1_NUM_POLE_PAIRS;
    obj->VIrmsIsrScale = objUser->ctrlFreq_Hz;

    obj->stopWaitTimeCnt = 0;
    obj->flagEnableRestart = false;

    obj->faultMtrMask.all = MTR1_FAULT_MASK_SET;
    obj->operateMode = OPERATE_MODE_SPEED;

    obj->svmMode = SVM_MIN_C;
    obj->flagEnableFWC = true;

    // true - enables SSIPD start, false - disables SSIPD
    obj->flagEnableSSIPD = false;

    obj->flagEnableSpeedCtrl = true;
    obj->flagEnableCurrentCtrl = true;

    obj->IsSet_A = 0.0f;

    obj->startupTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);    // 2.0s

#if defined(MOTOR1_ENC)
    obj->estimatorMode = ESTIMATOR_MODE_ENC;

    obj->flagEnableAlignment = true;

    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * USER_M1_ALIGN_TIME_S);
#else   // Not select algorithm
#error Not select a right estimator for this project
#endif  // !MOTOR1_ENC

    obj->speed_int_Hz = 0.0f;
    obj->speed_Hz = 0.0f;

    obj->speedAbs_Hz = 0.0f;
    obj->speedFilter_Hz = 0.0f;

#if defined(MOTOR1_FWC)
    obj->piHandle_fwc = PI_init(&pi_fwc_M1, sizeof(pi_fwc_M1));

    // set the FWC controller
    PI_setGains(obj->piHandle_fwc, USER_M1_FWC_KP, USER_M1_FWC_KI);
    PI_setUi(obj->piHandle_fwc, 0.0);
    PI_setMinMax(obj->piHandle_fwc, USER_M1_FWC_MAX_ANGLE_RAD,
                 USER_M1_FWC_MIN_ANGLE_RAD);
#endif  // MOTOR1_FWC

#ifdef MOTOR1_MTPA
    // initialize the Maximum torque per ampere (MTPA)
    obj->mtpaHandle = MTPA_init(&mtpa_M1, sizeof(mtpa_M1));

    // compute the motor constant for MTPA
    MTPA_computeParameters(obj->mtpaHandle,
                           objUser->motor_Ls_d_H,
                           objUser->motor_Ls_q_H,
                           objUser->motor_ratedFlux_Wb);
#endif  // MOTOR1_MTPA

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_ENC)
    // initialize the angle generate module
    obj->angleGenHandle = ANGLE_GEN_init(&angleGen_M1, sizeof(angleGen_M1));

    ANGLE_GEN_setParams(obj->angleGenHandle, objUser->ctrlPeriod_sec);
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ENC

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->Idq_set_A.value[0] = 0.0f;
    obj->Idq_set_A.value[1] = obj->startCurrent_A;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    // initialize the Vs per Freq module
    obj->VsFreqHandle = VS_FREQ_init(&VsFreq_M1, sizeof(VsFreq_M1));

    VS_FREQ_setVsMagPu(obj->VsFreqHandle, objUser->maxVsMag_pu);

    VS_FREQ_setMaxFreq(obj->VsFreqHandle, USER_MOTOR1_FREQ_MAX_Hz);

    VS_FREQ_setProfile(obj->VsFreqHandle,
                       USER_MOTOR1_FREQ_LOW_Hz, USER_MOTOR1_FREQ_HIGH_Hz,
                       USER_MOTOR1_VOLT_MIN_V, USER_MOTOR1_VOLT_MAX_V);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(MOTOR1_ENC)
    // initialize the enc handle
    obj->encHandle = ENC_init(&enc_M1, sizeof(enc_M1));

    // set the ENC controller parameters
    ENC_setQEPHandle(obj->encHandle, MTR1_QEP_BASE);
    ENC_setParams(obj->encHandle, obj->userParamsHandle);

    // initialize the apll handle
    obj->spdcalcHandle = SPDCALC_init(&speedcalc_M1, sizeof(speedcalc_M1));

    // set the SPEEDCALC controller parameters
    SPDCALC_setParams(obj->spdcalcHandle, obj->userParamsHandle);
#endif  // MOTOR1_ENC

    // initialize the Clarke modules
    obj->clarkeHandle_I = CLARKE_init(&clarke_I_M1, sizeof(clarke_I_M1));

    // set the Clarke parameters
    setupClarke_I(obj->clarkeHandle_I, objUser->numCurrentSensors);

    // initialize the inverse Park module
    obj->iparkHandle_V = IPARK_init(&ipark_V_M1, sizeof(ipark_V_M1));

    // initialize the Park module
    obj->parkHandle_I = PARK_init(&park_I_M1, sizeof(park_I_M1));

    // initialize the Park module
    obj->parkHandle_V = PARK_init(&park_V_M1, sizeof(park_V_M1));

    // initialize the PI controllers
    obj->piHandle_Id  = PI_init(&pi_Id_M1, sizeof(pi_Id_M1));
    obj->piHandle_Iq  = PI_init(&pi_Iq_M1, sizeof(pi_Iq_M1));
    obj->piHandle_spd = PI_init(&pi_spd_M1, sizeof(pi_spd_M1));

    // initialize the speed reference trajectory
    obj->trajHandle_spd = TRAJ_init(&traj_spd_M1, sizeof(traj_spd_M1));

    // configure the speed reference trajectory (Hz)
    TRAJ_setTargetValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setMinValue(obj->trajHandle_spd, -objUser->maxFrequency_Hz);
    TRAJ_setMaxValue(obj->trajHandle_spd, objUser->maxFrequency_Hz);
    TRAJ_setMaxDelta(obj->trajHandle_spd, (objUser->maxAccel_Hzps * objUser->ctrlPeriod_sec));

    // initialize the space vector generator module
    obj->svgenHandle = SVGEN_init(&svgen_M1, sizeof(svgen_M1));

    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);

    HAL_setTriggerPrams(&obj->pwmData, USER_SYSTEM_FREQ_MHz,
                        0.01f, 0.01f, 0.14f);

#if defined(MOTOR1_FILTERIS)
    obj->flagEnableFilterIs = true;

    // assign the current filter handle (low pass filter)
    obj->filterHandle_Is[0] = FILTER_FO_init((void *)(&filterIs_M1[0]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Is[1] = FILTER_FO_init((void *)(&filterIs_M1[1]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Is[2] = FILTER_FO_init((void *)(&filterIs_M1[2]), sizeof(FILTER_FO_Obj));

    obj->filterIsPole_rps = USER_M1_IS_FILTER_POLE_rps;     //

    float32_t beta_lp_Is = obj->filterIsPole_rps * objUser->ctrlPeriod_sec;

    float32_t a1_Is = (beta_lp_Is - (float32_t)2.0f) / (beta_lp_Is + (float32_t)2.0f);
    float32_t b0_Is = beta_lp_Is / (beta_lp_Is + (float32_t)2.0f);
    float32_t b1_Is = b0_Is;

    // set filter coefficients for current filters (low pass filter)
    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[0], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[0], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[0], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[1], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[1], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[1], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[2], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[2], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[2], 0.0f, 0.0f);
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_OVM)
    // Initialize and setup the 100% SVM generator
    obj->svgencurrentHandle =
            SVGENCURRENT_init(&svgencurrent_M1, sizeof(svgencurrent_M1));

    SVGENCURRENT_setup(obj->svgencurrentHandle, 1.0f,
                       USER_M1_PWM_FREQ_kHz, USER_SYSTEM_FREQ_MHz);
#endif  // MOTOR1_OVM

#ifdef BRAKE_ENABLE

    obj->brakingCurrent_A = USER_MOTOR1_BRAKE_CURRENT_A;

    obj->brakingTimeDelay = USER_MOTOR1_BRAKE_TIME_DELAY;

    obj->flagEnableBraking = false;
    obj->brakingMode = HARDSWITCH_BRAKE_MODE;
#endif  // BRAKE_ENABLE

    // setup the controllers, speed, d/q-axis current pid regulator
    setupControllers(handle);

#if defined(MOTOR1_PI_TUNE)
    // set the coefficient of the controllers gains
    setupControllerSF(handle);
#endif      // MOTOR1_PI_TUNE

    // disable the PWM
    HAL_disablePWM(obj->halMtrHandle);

#if defined(BENCHMARK_TEST)
    bmarkTestVars.recordDataCount = 0;
    bmarkTestVars.recordTicksSet = 15;
#endif  // BENCHMARK_TEST

    return;
}   // end of initMotor1CtrlParameters() function

void runMotor1OffsetsCalculation(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

#if defined(SDFM_I_SENSE)
    // calculate motor protection value
    // TODO: Add SDFM Comparator filter config here...

    obj->senseData.offset_I.value[0]  = USER_M1_IA_OFFSET_SDFM;
    obj->senseData.offset_I.value[1]  = USER_M1_IB_OFFSET_SDFM;
    obj->senseData.offset_I.value[2]  = USER_M1_IC_OFFSET_SDFM;

    if(obj->flagEnableOffsetCalc == true)
    {
        float32_t offsetK1 = 0.998001f;  // Offset filter coefficient K1: 0.05/(T+0.05);
        float32_t offsetK2 = 0.001999f;  // Offset filter coefficient K2: T/(T+0.05);

        uint16_t offsetCnt;

        MATH_Vec3 offset_I_calc = {0,0,0};

        // Set offsets to zero for calculation routine
        obj->senseData.offset_I.value[0]  = 0.0f;
        obj->senseData.offset_I.value[1]  = 0.0f;
        obj->senseData.offset_I.value[2]  = 0.0f;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

        // enable the PWM
        HAL_enablePWM(obj->halMtrHandle);

        for(offsetCnt = 0; offsetCnt < 32000; offsetCnt++)
        {
            // clear the SDFM interrupt flag
            SDFM_clearInterruptFlag(MTR1_SDFM_INT_BASE, MTR1_SDFM_INT_FLAG);

            // Wait for new SDFM data to arrive
            while(SDFM_getFIFOISRStatus(MTR1_SDFM_INT_BASE, MTR1_SDFM_INT_FILTER_NUM)== false);

            HAL_readMtr1SDFMData(&obj->senseData);

            if(offsetCnt >= 2000)       // Ignore the first 2000 times
            {
                // Offsets in phase current sensing
                offset_I_calc.value[0] =
                        offsetK1 * offset_I_calc.value[0] +
                        obj->senseData.I_A.value[0] * offsetK2;

                offset_I_calc.value[1] =
                        offsetK1 * offset_I_calc.value[1] +
                        obj->senseData.I_A.value[1] * offsetK2;

                offset_I_calc.value[2] =
                        offsetK1 * offset_I_calc.value[2] +
                        obj->senseData.I_A.value[2] * offsetK2;
            }
            else if(offsetCnt <= 1000)
            {
                // enable the PWM
                HAL_enablePWM(obj->halMtrHandle);
            }
        } // for()

        // disable the PWM, SDFM interrupt will have propogated to CPU already
        HAL_disablePWM(obj->halMtrHandle);

        // Assign offsets to calculated values
        obj->senseData.offset_I.value[0]  = offset_I_calc.value[0];
        obj->senseData.offset_I.value[1]  = offset_I_calc.value[1];
        obj->senseData.offset_I.value[2]  = offset_I_calc.value[2];

    }   // flagEnableOffsetCalc = true

    // Check current and voltage offset
    if( (obj->senseData.offset_I.value[0] > USER_M1_IA_OFFSET_SDFM_MAX) ||
        (obj->senseData.offset_I.value[0] < USER_M1_IA_OFFSET_SDFM_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->senseData.offset_I.value[1] > USER_M1_IB_OFFSET_SDFM_MAX) ||
        (obj->senseData.offset_I.value[1] < USER_M1_IB_OFFSET_SDFM_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->senseData.offset_I.value[2] > USER_M1_IC_OFFSET_SDFM_MAX) ||
        (obj->senseData.offset_I.value[2] < USER_M1_IC_OFFSET_SDFM_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if((obj->faultMtrNow.bit.voltageOffset == 0) &&
            (obj->faultMtrNow.bit.currentOffset == 0))
    {
        obj->flagEnableOffsetCalc = false;
    }
#else // !SDFM_I_SENSE
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    // calculate motor protection value
    calcMotorOverCurrentThreshold(handle);

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

        // Offsets in phase current sensing
    ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_PPB,
                              USER_M1_IA_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_PPB,
                              USER_M1_IB_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_PPB,
                              USER_M1_IC_OFFSET_AD);

    obj->senseData.offset_I.value[0]  = USER_M1_IA_OFFSET_AD;
    obj->senseData.offset_I.value[1]  = USER_M1_IB_OFFSET_AD;
    obj->senseData.offset_I.value[2]  = USER_M1_IC_OFFSET_AD;

    if(obj->flagEnableOffsetCalc == true)
    {
        float32_t offsetK1 = 0.998001f;  // Offset filter coefficient K1: 0.05/(T+0.05);
        float32_t offsetK2 = 0.001999f;  // Offset filter coefficient K2: T/(T+0.05);
        float32_t invCurrentSf = 1.0f / obj->senseData.current_sf;

        uint16_t offsetCnt;

        DEVICE_DELAY_US(2.0f);      // delay 2us

        ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_PPB, 0);
        ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_PPB, 0);
        ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_PPB, 0);

        obj->senseData.offset_I.value[0] =
                 obj->senseData.offset_I.value[0] * obj->senseData.current_sf;
        obj->senseData.offset_I.value[1] =
                 obj->senseData.offset_I.value[1] * obj->senseData.current_sf;
        obj->senseData.offset_I.value[2] =
                 obj->senseData.offset_I.value[2] * obj->senseData.current_sf;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

        // enable the PWM
        HAL_enablePWM(obj->halMtrHandle);

        for(offsetCnt = 0; offsetCnt < 32000; offsetCnt++)
        {
            // clear the ADC interrupt flag
            ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

            while(ADC_getInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM) == false);

            HAL_readMtr1ADCData(&obj->senseData);

            if(offsetCnt >= 2000)       // Ignore the first 2000 times
            {
                // Offsets in phase current sensing
                obj->senseData.offset_I.value[0] =
                        offsetK1 * obj->senseData.offset_I.value[0] +
                        obj->senseData.I_A.value[0] * offsetK2;

                obj->senseData.offset_I.value[1] =
                        offsetK1 * obj->senseData.offset_I.value[1] +
                        obj->senseData.I_A.value[1] * offsetK2;

                obj->senseData.offset_I.value[2] =
                        offsetK1 * obj->senseData.offset_I.value[2] +
                        obj->senseData.I_A.value[2] * offsetK2;
            }
            else if(offsetCnt <= 1000)
            {
                // enable the PWM
                HAL_enablePWM(obj->halMtrHandle);
            }
        } // for()

        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);

        obj->senseData.offset_I.value[0] =
                 obj->senseData.offset_I.value[0] * invCurrentSf;
        obj->senseData.offset_I.value[1] =
                 obj->senseData.offset_I.value[1] * invCurrentSf;
        obj->senseData.offset_I.value[2] =
                 obj->senseData.offset_I.value[2] * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_PPB,
                                  (uint16_t)obj->senseData.offset_I.value[0]);

        ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_PPB,
                                  (uint16_t)obj->senseData.offset_I.value[1]);

        ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_PPB,
                                  (uint16_t)obj->senseData.offset_I.value[2]);
    }   // flagEnableOffsetCalc = true

    // Check current and voltage offset
    if( (obj->senseData.offset_I.value[0] > USER_M1_IA_OFFSET_AD_MAX) ||
        (obj->senseData.offset_I.value[0] < USER_M1_IA_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->senseData.offset_I.value[1] > USER_M1_IB_OFFSET_AD_MAX) ||
        (obj->senseData.offset_I.value[1] < USER_M1_IB_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->senseData.offset_I.value[2] > USER_M1_IC_OFFSET_AD_MAX) ||
        (obj->senseData.offset_I.value[2] < USER_M1_IC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if((obj->faultMtrNow.bit.voltageOffset == 0) &&
            (obj->faultMtrNow.bit.currentOffset == 0))
    {
        obj->flagEnableOffsetCalc = false;
    }
#endif // SDFM_I_SENSE

    return;
} // end of runMotor1OffsetsCalculation() function



void runMotor1Control(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(obj->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(obj->userParamsHandle);

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == true)
    {
        if(HAL_getMtrTripFaults(obj->halMtrHandle) != 0)
        {
            obj->faultMtrNow.bit.moduleOverCurrent = 1;
        }
    }

    obj->faultMtrPrev.all |= obj->faultMtrNow.all;
    obj->faultMtrUse.all = obj->faultMtrNow.all & obj->faultMtrMask.all;

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

    if(obj->flagClearFaults == true)
    {
        HAL_clearMtrFaultStatus(obj->halMtrHandle);

        obj->faultMtrNow.all &= MTR_FAULT_CLEAR;
        obj->flagClearFaults = false;
    }

    if(obj->flagEnableRunAndIdentify == true)
    {
        // Had some faults to stop the motor
        if(obj->faultMtrUse.all != 0)
        {
            if(obj->flagRunIdentAndOnLine == true)
            {
                obj->flagRunIdentAndOnLine = false;
                obj->motorState = MOTOR_FAULT_STOP;

                obj->stopWaitTimeCnt = objSets->restartWaitTimeSet;
                obj->restartTimesCnt++;

                if(obj->flagEnableRestart == false)
                {
                    obj->flagEnableRunAndIdentify = false;
                    obj->stopWaitTimeCnt = 0;
                }
            }
            else if(obj->stopWaitTimeCnt == 0)
            {
                if(obj->restartTimesCnt < objSets->restartTimesSet)
                {
                    obj->flagClearFaults = 1;
                }
                else
                {
                    obj->flagEnableRunAndIdentify = false;
                }
            }
        }
        // Restart
        else if((obj->flagRunIdentAndOnLine == false) &&
                (obj->stopWaitTimeCnt == 0))
        {
            restartMotorControl(handle);
        }
    }
    // if(obj->flagEnableRunAndIdentify == false)
    else if(obj->flagRunIdentAndOnLine == true)
    {
        stopMotorControl(handle);

        obj->stopWaitTimeCnt = objSets->stopWaitTimeSet;
    }

    if(obj->flagRunIdentAndOnLine == true)
    {
        if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
        {
            // enable the PWM
            HAL_enablePWM(obj->halMtrHandle);
        }

        {


            if(obj->speedRef_Hz > 0.0f)
            {
                obj->direction = 1.0f;
            }
            else
            {
                obj->direction = -1.0f;
            }

            // Sets the target speed for the speed trajectory
        #if defined(MOTOR1_ENC)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
        #else   // !MOTOR1_ENC
        #error No select a right estimator for motor_1 control
        #endif  // MOTOR1_ENC

            if((fabsf(obj->speed_Hz) > obj->speedStart_Hz) ||
                    (obj->motorState == MOTOR_CTRL_RUN))
            {
                //  Sets the acceleration / deceleration for the speed trajectory
                TRAJ_setMaxDelta(obj->trajHandle_spd,
                  (obj->accelerationMax_Hzps * objUser->ctrlPeriod_sec));

                PI_setMinMax(obj->piHandle_spd, -obj->maxCurrent_A, obj->maxCurrent_A);

                SVGEN_setMode(obj->svgenHandle, obj->svmMode);

                if(obj->motorState == MOTOR_CL_RUNNING)
                {
                    obj->stateRunTimeCnt++;

                    if(obj->stateRunTimeCnt == obj->startupTimeDelay)
                    {
                        obj->Idq_out_A.value[0] = 0.0f;
                        obj->motorState = MOTOR_CTRL_RUN;
                    }
                }
            }
            else
            {
                TRAJ_setMaxDelta(obj->trajHandle_spd,
                  (obj->accelerationStart_Hzps * objUser->ctrlPeriod_sec));

                if(obj->speed_int_Hz >= 0.0f)
                {
                    PI_setMinMax(obj->piHandle_spd, 0.0f, obj->startCurrent_A);
                }
                else
                {
                    PI_setMinMax(obj->piHandle_spd, -obj->startCurrent_A, 0.0f);
                }
            }
        }

        // Identification
#if(DMC_BUILDLEVEL == DMC_LEVEL_3)
        obj->Idq_out_A.value[0] = obj->Idq_set_A.value[0];
        obj->Idq_out_A.value[1] = obj->Idq_set_A.value[1] * obj->direction;

#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)
    }
    else
    {
        // reset motor control parameters
        resetMotorControl(handle);
    }

    if(obj->flagSetupController == true)
    {
        // update the controller
        updateControllers(handle);
    }
    else
    {
        obj->flagSetupController = true;

        setupControllers(handle);
    }

    // update the global variables
    updateGlobalVariables(handle);

    return;
}   // end of the runMotor1Control() function

__interrupt void motor1CtrlISR(void)
{
#if defined(ISR_MEASURE_EN)
    GPIO_writePin(ISR_MEASURE_IO, 1);
#endif

    motorVars_M1.ISRCount++;

    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)motorHandle_M1;
    USER_Params *objUser = (USER_Params *)(obj->userParamsHandle);

#if defined(SDFM_I_SENSE)
    // Read SDFM data, Phase currents
    HAL_readMtr1SDFMData(&obj->senseData);
#endif // SDFM_I_SENSE

    // read the ADC sensed data
    HAL_readMtr1ADCData(&obj->senseData);

#if defined(MOTOR1_FILTERIS)
    // run first order filters for current sensing
    obj->adcIs_A.value[0] = FILTER_FO_run(obj->filterHandle_Is[0], obj->senseData.I_A.value[0]);
    obj->adcIs_A.value[1] = FILTER_FO_run(obj->filterHandle_Is[1], obj->senseData.I_A.value[1]);
    obj->adcIs_A.value[2] = FILTER_FO_run(obj->filterHandle_Is[2], obj->senseData.I_A.value[2]);

    if(obj->flagEnableFilterIs == true)
    {
        obj->senseData.I_A.value[0] = obj->adcIs_A.value[0];
        obj->senseData.I_A.value[1] = obj->adcIs_A.value[1];
        obj->senseData.I_A.value[2] = obj->adcIs_A.value[2];
    }
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_OVM)
    // Over Modulation Supporting, run the current reconstruction algorithm
    SVGENCURRENT_RunRegenCurrent(obj->svgencurrentHandle,
                                 &obj->senseData.I_A, &obj->adcDataPrev);
#endif  // MOTOR1_OVM

#if defined(MOTOR1_ENC)
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // run Clarke transform on current
    CLARKE_run_threeInput(obj->clarkeHandle_I, &obj->senseData.I_A, &obj->Iab_A);

    if(obj->flagRunIdentAndOnLine == true)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = false;
        obj->enableCurrentCtrl = false;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->senseData.VdcBus_V;


    ENC_run(obj->encHandle);
    obj->angleENC_rad = ENC_getElecAngle(obj->encHandle);

    SPDCALC_run(obj->spdcalcHandle, obj->angleENC_rad);
    obj->speedENC_Hz = SPDCALC_getSpeedHz(obj->spdcalcHandle);

    obj->speed_Hz = obj->speedENC_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabsf(obj->speedFilter_Hz);

    obj->stateRunTimeCnt++;

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleENC_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = false;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE)
        {
            obj->motorState = MOTOR_CL_RUNNING;
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = false;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        ANGLE_GEN_setAngle(obj->angleGenHandle, 0.0f);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == false))
        {
            obj->stateRunTimeCnt = 0;
            obj->motorState = MOTOR_OL_START;

            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);
            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = __cos(obj->angleFOC_rad);
    phasor.value[1] = __sin(obj->angleFOC_rad);

    // set the phasor in the Park transform
    PARK_setPhasor(obj->parkHandle_I, &phasor);

    // run the Park transform
    PARK_run(obj->parkHandle_I, &(obj->Iab_A), (MATH_vec2 *)&(obj->Idq_in_A));

// End of MOTOR1_ENC
//------------------------------------------------------------------------------
#else   // No Any Estimator
#error Not select a right estimator for this project
#endif  // (ESTIMATOR)

//---------- Common Speed and Current Loop for all observers -------------------
#if(DMC_BUILDLEVEL >= DMC_LEVEL_4)

#if defined(SFRA_ENABLE)

    if(sfraCollectStart == true)
    {
        collectSFRA(motorHandle_M1);    // Collect noise feedback from loop
    }

    //  SFRA injection
    injectSFRA();                   // create SFRA Noise per 'sfraTestLoop'

    sfraCollectStart = true;       // enable SFRA data collection
#endif  // SFRA_ENABLE

    // run the speed controller
    obj->counterSpeed++;

    if(obj->counterSpeed >= objUser->numCtrlTicksPerSpeedTick)
    {
        obj->counterSpeed = 0;

        obj->speed_reg_Hz = obj->speed_Hz;

        if(obj->enableSpeedCtrl == true)
        {
            obj->Is_ffwd_A = 0.0f;


#if defined(SFRA_ENABLE)
            PI_run_series(obj->piHandle_spd,
                   (obj->speed_int_Hz + sfraNoiseSpd), obj->speed_reg_Hz,
                   obj->Is_ffwd_A, (float32_t *)&obj->IsRef_A);
#else     // !SFRA_ENABLE
            PI_run_series(obj->piHandle_spd,
                   obj->speed_int_Hz, obj->speed_reg_Hz,
                   obj->Is_ffwd_A, (float32_t *)&obj->IsRef_A);
#endif  // !SFRA_ENABLE
        }
        else if((obj->motorState >= MOTOR_CL_RUNNING))
        {
            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->IsSet_A;
            }
            else
            {
                obj->IsRef_A = -obj->IsSet_A;
            }

            // for switching back speed closed-loop control
            PI_setUi(obj->piHandle_spd, obj->IsRef_A);
        }
    }
#if defined(MOTOR1_FWC) && defined(MOTOR1_MTPA)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad =
                (obj->angleFWC_rad > obj->angleMTPA_rad) ?
                        obj->angleFWC_rad : obj->angleMTPA_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if((obj->flagEnableFWC == true) || (obj->flagEnableMTPA == true))
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                __sqrt((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->senseData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == true)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
    else if(obj->counterSpeed == 4)   // MTPA
    {
        if(obj->flagEnableMTPA == true)
        {
            obj->angleMTPA_rad =
                    MTPA_computeCurrentAngle(obj->mtpaHandle, obj->IsRef_A);
        }
        else
        {
            obj->angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_FWC)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad = obj->angleFWC_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if(obj->flagEnableFWC == true)
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                __sqrt((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->senseData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == true)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_MTPA)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad = obj->angleMTPA_rad;

        fwcPhasor.value[0] = __cos(obj->angleCurrent_rad);
        fwcPhasor.value[1] = __sin(obj->angleCurrent_rad);

        if(obj->flagEnableMTPA == true)
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 4)   // MTPA
    {
        if(obj->flagEnableMTPA == true)
        {
            obj->angleMTPA_rad = MTPA_computeCurrentAngle(obj->mtpaHandle, obj->IsRef_A);
        }
        else
        {
            obj->angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#else   // !MOTOR1_MTPA && !MOTOR1_FWC
    obj->Idq_out_A.value[1] = obj->IsRef_A;
#endif  // !MOTOR1_MTPA && !MOTOR1_FWC/

#if !defined(STEP_RP_EN)
    obj->IdqRef_A.value[0] = obj->Idq_out_A.value[0] + obj->IdRated_A;
#endif  // STEP_RP_EN

#if !defined(STEP_RP_EN)
    obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1];
#else   // STEP_RP_EN
    if(GRAPH_getBufferMode(&stepRPVars) != GRAPH_STEP_RP_TORQUE)
    {
        obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1];
    }
    else
    {
        PI_setUi(obj->piHandle_spd, obj->IdqRef_A.value[1]);
    }
#endif  // STEP_RP_EN


#elif(DMC_BUILDLEVEL == DMC_LEVEL_3)
    obj->IdqRef_A.value[0] = obj->Idq_set_A.value[0];
    obj->IdqRef_A.value[1] = obj->Idq_set_A.value[1];
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

    if(obj->enableCurrentCtrl == true)
    {
        obj->Vdq_ffwd_V.value[0] = 0.0f;
        obj->Vdq_ffwd_V.value[1] = 0.0f;

        // Maximum voltage output
        obj->VsMax_V = objUser->maxVsMag_pu * obj->senseData.VdcBus_V;
        PI_setMinMax(obj->piHandle_Id, -obj->VsMax_V, obj->VsMax_V);

#if defined(SFRA_ENABLE)
        // run the Id controller
        PI_run_series(obj->piHandle_Id,
                      (obj->IdqRef_A.value[0] + sfraNoiseId), obj->Idq_in_A.value[0],
                      obj->Vdq_ffwd_V.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = __sqrt((obj->VsMax_V * obj->VsMax_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
        PI_run(obj->piHandle_Iq, (obj->IdqRef_A.value[1] + sfraNoiseIq),
               obj->Idq_in_A.value[1], (float32_t*)&obj->Vdq_out_V.value[1]);

#else     // !SFRA_ENABLE
        // run the Id controller
        PI_run_series(obj->piHandle_Id,
                      obj->IdqRef_A.value[0], obj->Idq_in_A.value[0],
                      obj->Vdq_ffwd_V.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = __sqrt((obj->VsMax_V * obj->VsMax_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
        PI_run(obj->piHandle_Iq, obj->IdqRef_A.value[1],
               obj->Idq_in_A.value[1], (float32_t*)&obj->Vdq_out_V.value[1]);
#endif  // !SFRA_ENABLE

    }

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    VS_FREQ_run(obj->VsFreqHandle, obj->speed_int_Hz);
    obj->Vdq_out_V.value[0] = VS_FREQ_getVd_out(obj->VsFreqHandle);
    obj->Vdq_out_V.value[1] = VS_FREQ_getVq_out(obj->VsFreqHandle);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(PHASE_ADJ_EN)
    if(obj->flagPhaseAdjustEnable == true)
    {
        obj->angleFOCAdj_rad =
                MATH_incrAngle(obj->angleFOC_rad, obj->anglePhaseAdj_rad);

        // compute the sin/cos phasor
        phasor.value[0] = __cos(obj->angleFOCAdj_rad);
        phasor.value[1] = __sin(obj->angleFOCAdj_rad);
    }
    else
    {
        obj->angleFOCAdj_rad = obj->angleFOC_rad;
    }
#endif  // PHASE_ADJ_EN

    // set the phasor in the inverse Park transform
    IPARK_setPhasor(obj->iparkHandle_V, &phasor);

    // run the inverse Park module
    IPARK_run(obj->iparkHandle_V,
              &obj->Vdq_out_V, &obj->Vab_out_V);

    // setup the space vector generator (SVGEN) module
    SVGEN_setup(obj->svgenHandle,
                obj->oneOverDcBus_invV);

    // run the space vector generator (SVGEN) module
    SVGEN_run(obj->svgenHandle,
              &obj->Vab_out_V, &(obj->pwmData.Vabc_pu));

#if(DMC_BUILDLEVEL == DMC_LEVEL_1)
    // output 50%
    obj->pwmData.Vabc_pu.value[0] = 0.0f;
    obj->pwmData.Vabc_pu.value[1] = 0.0f;
    obj->pwmData.Vabc_pu.value[2] = 0.0f;
#endif

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == false)
    {
        // clear PWM data
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;
    }

#if defined(MOTOR1_OVM)
    else
    {
        // run the PWM compensation and current ignore algorithm
        SVGENCURRENT_compPWMData(obj->svgencurrentHandle,
                                 &obj->pwmData.Vabc_pu, &obj->pwmDataPrev);
    }

    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    obj->ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(obj->svgencurrentHandle);
    obj->midVolShunt = SVGENCURRENT_getVmid(obj->svgencurrentHandle);

    // Set trigger point in the middle of the low side pulse
    HAL_setTrigger(obj->halMtrHandle,
                   &obj->pwmData, obj->ignoreShuntNextCycle, obj->midVolShunt);
#else   // !MOTOR1_OVM
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);
#endif  // !MOTOR1_OVM

    // Collect current and voltage data to calculate the RMS value
    collectRMSData(motorHandle_M1);
//------------------------------------------------------------------------------

#if defined(BENCHMARK_TEST)
    recordSpeedData(motorHandle_M1);
#endif  // BENCHMARK_TEST

#if defined(STEP_RP_EN)
    // Collect predefined data into arrays
    GRAPH_updateBuffer(&stepRPVars);
#endif  // STEP_RP_EN

#if defined(DATALOGF2_EN)
    if(DATALOGIF_enable(datalogHandle) == true)
    {
        DATALOGIF_updateWithDMA(datalogHandle);

        // Force trig DMA channel to save the data
        HAL_trigDMAforDLOG(halHandle, 0);
        HAL_trigDMAforDLOG(halHandle, 1);
    }
#elif defined(DATALOGF4_EN) || defined(DATALOGI4_EN)
    if(DATALOGIF_enable(datalogHandle) == true)
    {
        DATALOGIF_updateWithDMA(datalogHandle);

        // Force trig DMA channel to save the data
        HAL_trigDMAforDLOG(halHandle, 0);
        HAL_trigDMAforDLOG(halHandle, 1);
        HAL_trigDMAforDLOG(halHandle, 2);
        HAL_trigDMAforDLOG(halHandle, 3);
    }
#endif  // DATALOGF4_EN || DATALOGF2_EN

#if defined(DAC128S_ENABLE)
    DAC128S_writeData(dac128sHandle);
#endif  // DAC128S_ENABLE

#if defined(DAC_ON_CHIP_ENABLE)
    float32_t dacGain = (2.0f * 4096.0f / USER_M1_FULL_SCALE_CURRENT_A);
    uint16_t dacOffset = (uint16_t)(0.5f * 4096.0f);
    // dacVal = ((data * gain) + offset) & (uint16_t)0xFFF;
    uint16_t dacVal = ((int16_t)(motorVars_M1.senseData.I_A.value[0] *
            dacGain) + dacOffset) & (uint16_t)0xFFF;

    DAC_setShadowValue(MTR1_DAC_BASE, dacVal);
#endif // DAC_ON_CHIP_ENABLE

#if defined(SDFM_I_SENSE)
    // Acknowledge SDFM interrupt
    HAL_ackMtr1SDFMInt();
#else
    // acknowledge the ADC interrupt
    HAL_ackMtr1ADCInt();
#endif // SDFM_I_SENSE

#if defined(ISR_MEASURE_EN)
    GPIO_writePin(ISR_MEASURE_IO, 0);
#endif

    return;
} // end of motor1CtrlISR() function

//
//-- end of this file ----------------------------------------------------------
//
