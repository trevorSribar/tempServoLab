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


//! \file   /solutions/universal_servo_drive/common/source/motor_comm.c
//!
//! \brief  This project is used to implement motor control with FAST, eSMO
//!         Encoder, and Hall sensors based sensored/sensorless-FOC.
//!         Supports multiple TI EVM boards
//!


//
// include the related header files
//
#include "sys_settings.h"
#include "sys_main.h"

#if !defined(SDFM_I_SENSE)
//! \brief calculate motor over current threshold
void calcMotorOverCurrentThreshold(MOTOR_Handle handle)
{
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    float32_t overCurrent_A;

    overCurrent_A = (objSets->overCurrent_A > objSets->maxPeakCurrent_A) ?
                     objSets->maxPeakCurrent_A : objSets->overCurrent_A;

    int16_t cmpValue = (int16_t)(overCurrent_A * objSets->currentInv_sf);

    objSets->dacCMPValH = USER_M1_IS_OFFSET_CMPSS + cmpValue;
    objSets->dacCMPValH = (objSets->dacCMPValH > 4095U) ?
                           4095U : objSets->dacCMPValH;

    objSets->dacCMPValL = (cmpValue < USER_M1_IS_OFFSET_CMPSS) ?
                           (USER_M1_IS_OFFSET_CMPSS - cmpValue) : 1U;

    return;
}
#endif // !SDFM_I_SENSE

// Sets up control parameters for stopping motor
void stopMotorControl(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    obj->speed_int_Hz = 0.0f;

    obj->flagRunIdentAndOnLine = false;

#ifdef BRAKE_ENABLE
    if(obj->motorState == MOTOR_BRAKE_STOP)
    {
        if(obj->brakingMode == HARDSWITCH_BRAKE_MODE)
        {
            // Exit braking PWM mode
            HAL_exitBrakeResetPWM(obj->halMtrHandle);
        }

        obj->motorState = MOTOR_STOP_IDLE;
        obj->flagEnableBraking = false;

        obj->IsRef_A = 0.0f;
        PI_setUi(obj->piHandle_spd, 0.0f);
        PI_setRefValue(obj->piHandle_spd, 0.0f);
    }
#endif  // BRAKE_ENABLE

#if defined(MOTOR1_ENC)
    if(obj->motorState == MOTOR_FAULT_STOP)
    {
        ENC_resetState(obj->encHandle);

        obj->motorState = MOTOR_STOP_IDLE;
    }
    else if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->motorState = MOTOR_NORM_STOP;
    }
    else
    {
        obj->motorState = MOTOR_STOP_IDLE;
    }
#else
    obj->motorState = MOTOR_STOP_IDLE;
#endif  // MOTOR1_ENC

    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);

    obj->restartTimesCnt = 0;

    return;
}

// Sets up control parameters for restarting motor
void restartMotorControl(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

#ifdef BRAKE_ENABLE
    if(obj->motorState == MOTOR_BRAKE_STOP)
    {
        if(obj->brakingMode == HARDSWITCH_BRAKE_MODE)
        {
            // Exit braking PWM mode
            HAL_exitBrakeResetPWM(obj->halMtrHandle);
        }

        obj->motorState = MOTOR_STOP_IDLE;
        obj->flagEnableBraking = false;

        obj->IsRef_A = 0.0f;
        PI_setUi(obj->piHandle_spd, 0.0f);
        PI_setRefValue(obj->piHandle_spd, 0.0f);
    }
#endif  // BRAKE_ENABLE

#if defined(MOTOR1_ENC)
    if(obj->estimatorMode == ESTIMATOR_MODE_ENC)
    {
        if(obj->motorState == MOTOR_NORM_STOP)
        {
            obj->motorState = MOTOR_CL_RUNNING;
        }
        else
        {
            obj->motorState = MOTOR_ALIGNMENT;
            obj->flagEnableAlignment = true;

            ENC_setState(obj->encHandle, ENC_ALIGNMENT);
        }
    }
    else
    {
        obj->motorState = MOTOR_ALIGNMENT;
        obj->flagEnableAlignment = true;

        ENC_setState(obj->encHandle, ENC_ALIGNMENT);
    }
#endif  // MOTOR1_ENC

    obj->speed_int_Hz = 0.0f;

    obj->speedAbs_Hz = 0.0f;
    obj->speedFilter_Hz = 0.0f;

    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);

    obj->flagRunIdentAndOnLine = true;
    obj->stateRunTimeCnt = 0;
    obj->startSumTimesCnt++;

#if defined(SFRA_ENABLE)
    sfraCollectStart = false;       // disable SFRA data collection
#endif  // SFRA_ENABLE


    return;
}

// Resets motor control parameters for restarting motor
void resetMotorControl(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;


    TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);

    TRAJ_setTargetValue(obj->trajHandle_spd, 0.0f);

    // disable the PWM
    HAL_disablePWM(obj->halMtrHandle);

    // clear integral outputs of the controllers
    PI_setRefValue(obj->piHandle_Id, 0.0f);
    PI_setRefValue(obj->piHandle_Iq, 0.0f);
    PI_setRefValue(obj->piHandle_spd, 0.0f);

    PI_setUi(obj->piHandle_Id, 0.0f);
    PI_setUi(obj->piHandle_Iq, 0.0f);
    PI_setUi(obj->piHandle_spd, 0.0f);

    // clear current references
    obj->Idq_out_A.value[0] = 0.0f;
    obj->Idq_out_A.value[1] = 0.0f;

    obj->IdRated_A = 0.0f;
    obj->IsRef_A = 0.0f;

    obj->angleCurrent_rad = 0.0f;

#if defined(MOTOR1_FWC)
    PI_setUi(obj->piHandle_fwc, 0.0f);
#endif  // MOTOR1_FWC

    obj->stateRunTimeCnt = 0;
    obj->motorStallTimeCnt = 0;
    obj->startupFailTimeCnt = 0;

    obj->overSpeedTimeCnt = 0;
    obj->overLoadTimeCnt = 0;
    obj->lostPhaseTimeCnt = 0;
    obj->unbalanceTimeCnt = 0;

    SVGEN_setMode(obj->svgenHandle, SVM_COM_C);

    return;
}

// timer base is 5ms
void runMotorMonitor(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    if(obj->stopWaitTimeCnt > 0)
    {
        obj->stopWaitTimeCnt--;
    }

    // Check if DC bus voltage is over threshold
    if(obj->senseData.VdcBus_V > objSets->overVoltageFault_V)
    {
        if(obj->overVoltageTimeCnt > objSets->voltageFaultTimeSet)
        {
            obj->faultMtrNow.bit.overVoltage = 1;
        }
        else
        {
            obj->overVoltageTimeCnt++;
        }
    }
    else if(obj->senseData.VdcBus_V < objSets->overVoltageNorm_V)
    {
        if(obj->overVoltageTimeCnt == 0)
        {
            obj->faultMtrNow.bit.overVoltage = 0;
        }
        else
        {
            obj->overVoltageTimeCnt--;
        }
    }

    // Check if DC bus voltage is under threshold
    if(obj->senseData.VdcBus_V < objSets->underVoltageFault_V)
    {
        if(obj->underVoltageTimeCnt > objSets->voltageFaultTimeSet)
        {
            obj->faultMtrNow.bit.underVoltage = 1;
        }
        else
        {
            obj->underVoltageTimeCnt++;
        }
    }
    else if(obj->senseData.VdcBus_V > objSets->underVoltageNorm_V)
    {
        if(obj->underVoltageTimeCnt == 0)
        {
            obj->faultMtrNow.bit.underVoltage = 0;
        }
        else
        {
            obj->underVoltageTimeCnt--;
        }
    }

    // check these faults when motor is running
    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        // Over Load Check
        if(obj->powerActive_W > objSets->overLoadSet_W)
        {
            if(obj->overLoadTimeCnt > objSets->overLoadTimeSet)
            {
                obj->faultMtrNow.bit.overLoad = 1;
                obj->overLoadTimeCnt = 0;
            }
            else
            {
                obj->overLoadTimeCnt++;
            }
        }
        else if(obj->overLoadTimeCnt > 0)
        {
            obj->overLoadTimeCnt--;
        }

        // Motor Stall
        if( (obj->Is_A > objSets->stallCurrentSet_A)
                && (obj->speedAbs_Hz < objSets->speedFailMinSet_Hz))
        {
            if(obj->motorStallTimeCnt > objSets->motorStallTimeSet)
            {
                obj->faultMtrNow.bit.motorStall = 1;
                obj->motorStallTimeCnt = 0;
            }
            else
            {
                obj->motorStallTimeCnt++;
            }
        }
        else if(obj->motorStallTimeCnt > 0)
        {
            obj->motorStallTimeCnt--;
        }

        // (obj->torque_Nm < objSets->toqueFailMinSet_Nm)
        // Motor Lost Phase Fault Check
        if( (obj->speedAbs_Hz > objSets->speedFailMinSet_Hz) &&
            ( (obj->Irms_A[0] < objSets->lostPhaseSet_A) ||
              (obj->Irms_A[1] < objSets->lostPhaseSet_A) ||
              (obj->Irms_A[2] < objSets->lostPhaseSet_A)) )
        {
            if(obj->lostPhaseTimeCnt > objSets->lostPhaseTimeSet)
            {
                obj->faultMtrNow.bit.motorLostPhase = 1;
                obj->lostPhaseTimeCnt = 0;
            }
            else
            {
                obj->lostPhaseTimeCnt++;
            }
        }
        else if(obj->lostPhaseTimeCnt > 0)
        {
            obj->lostPhaseTimeCnt--;
        }

        // Only when the torque is great than a setting value
        if(obj->Is_A > objSets->IsFailedChekSet_A)
        {
            // Motor Phase Current Unbalance
            if(obj->unbalanceRatio > objSets->unbalanceRatioSet)
            {
                if(obj->unbalanceTimeCnt > objSets->unbalanceTimeSet)
                {
                    obj->faultMtrNow.bit.currentUnbalance = 1;
                    obj->unbalanceTimeCnt = 0;
                }
                else
                {
                    obj->unbalanceTimeCnt++;
                }
            }
            else if(obj->unbalanceTimeCnt > 0)
            {
                obj->unbalanceTimeCnt--;
            }

            // Motor Over speed
            if(obj->speedAbs_Hz > objSets->speedFailMaxSet_Hz)
            {
                if(obj->overSpeedTimeCnt > objSets->overSpeedTimeSet)
                {
                    obj->faultMtrNow.bit.overSpeed = 1;
                    obj->overSpeedTimeCnt = 0;
                }
                else
                {
                    obj->overSpeedTimeCnt++;
                }
            }
            else if(obj->overSpeedTimeCnt > 0)
            {
                obj->overSpeedTimeCnt--;
            }

            // Motor Startup Failed
            if( (obj->Is_A < objSets->stallCurrentSet_A)
               && (obj->speedAbs_Hz < objSets->speedFailMinSet_Hz))
            {
                if(obj->startupFailTimeCnt > objSets->startupFailTimeSet)
                {
                    obj->faultMtrNow.bit.startupFailed = 1;
                    obj->startupFailTimeCnt = 0;
                }
                else
                {
                    obj->startupFailTimeCnt++;
                }
            }
            else if(obj->startupFailTimeCnt > 0)
            {
                obj->startupFailTimeCnt--;
            }

        } // obj->Is_A > objSets->IsFailedChekSet_A
    } // obj->operateState == OPERATE_State_Run

    return;
}

void collectRMSData(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    obj->IrmsCalSum[0] += obj->senseData.I_A.value[0] * obj->senseData.I_A.value[0];
    obj->IrmsCalSum[1] += obj->senseData.I_A.value[1] * obj->senseData.I_A.value[1];
    obj->IrmsCalSum[2] += obj->senseData.I_A.value[2] * obj->senseData.I_A.value[2];

    obj->VIrmsIsrCnt++;

    if(obj->VIrmsIsrCnt > obj->VIrmsIsrSet)
    {
        obj->IrmsPrdSum[0] = obj->IrmsCalSum[0];
        obj->IrmsPrdSum[1] = obj->IrmsCalSum[1];
        obj->IrmsPrdSum[2] = obj->IrmsCalSum[2];

        obj->IrmsCalSum[0] = 0.0f;
        obj->IrmsCalSum[1] = 0.0f;
        obj->IrmsCalSum[2] = 0.0f;

        obj->VIrmsIsrCnt = 0;
        obj->flagVIrmsCal = true;
    }
}

void calculateRMSData(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    float32_t IrmsMax_A, IrmsMin_A, VIrmsIsrSet;

    if(obj->flagVIrmsCal == true)
    {
        obj->flagVIrmsCal = false;

        obj->Irms_A[0] =
                sqrtf(obj->IrmsPrdSum[0] * obj->IrmsCalSF);

        obj->Irms_A[1] =
                sqrtf(obj->IrmsPrdSum[1] * obj->IrmsCalSF);

        obj->Irms_A[2] =
                sqrtf(obj->IrmsPrdSum[2] * obj->IrmsCalSF);

        if(obj->Irms_A[0] > obj->Irms_A[1])
        {
            IrmsMax_A = obj->Irms_A[0];
            IrmsMin_A = obj->Irms_A[1];
        }
        else
        {
            IrmsMax_A = obj->Irms_A[0];
            IrmsMin_A = obj->Irms_A[1];
        }

        IrmsMax_A = (obj->Irms_A[2] > IrmsMax_A) ? obj->Irms_A[2] : IrmsMax_A;
        IrmsMin_A = (obj->Irms_A[2] < IrmsMin_A) ? obj->Irms_A[2] : IrmsMin_A;

        if(obj->speedAbs_Hz != 0.0f)
        {
            VIrmsIsrSet = obj->VIrmsIsrScale / obj->speedAbs_Hz;

            VIrmsIsrSet = (VIrmsIsrSet > obj->VIrmsIsrScale) ?
                    obj->VIrmsIsrScale : VIrmsIsrSet;

            VIrmsIsrSet = (VIrmsIsrSet != 0) ?
                    VIrmsIsrSet: obj->VIrmsIsrScale;
        }
        else
        {
            VIrmsIsrSet = obj->VIrmsIsrScale;
        }

        obj->IrmsCalSF = 1.0f / VIrmsIsrSet;
        obj->VIrmsIsrSet = (uint16_t)(VIrmsIsrSet);

        obj->unbalanceRatio =
                (IrmsMax_A - IrmsMin_A) / (IrmsMax_A + IrmsMin_A);
    }
}

//// setupCurrentControllers()
//void setupCurrentControllers(MOTOR_Handle handle)
//{
//    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
//    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);
//
//    float32_t RoverL_Kp_sf = objUser->RoverL_Kp_sf;
//    float32_t dcBus_nominal_V = objUser->dcBus_nominal_V;
//    float32_t maxCurrent_A = objUser->maxCurrent_A;
//    float32_t RoverL_min_rps = objUser->RoverL_min_rps;
//    float32_t currentCtrlPeriod_sec =
//                (float32_t)objUser->numCtrlTicksPerCurrentTick /
//                    objUser->ctrlFreq_Hz;
//
//    float32_t outMax_V = objUser->Vd_sf * objUser->maxVsMag_V;
//    float32_t Kp = RoverL_Kp_sf * dcBus_nominal_V / maxCurrent_A;
//    float32_t Ki = RoverL_min_rps * currentCtrlPeriod_sec;
//
//    // set the Id controller
//    PI_setGains(obj->piHandle_Id, Kp, Ki);
//    PI_setUi(obj->piHandle_Id, 0.0f);
//    PI_setRefValue(obj->piHandle_Id, 0.0f);
//    PI_setFbackValue(obj->piHandle_Id, 0.0f);
//    PI_setFfwdValue(obj->piHandle_Id, 0.0f);
//    PI_setMinMax(obj->piHandle_Id, -outMax_V, outMax_V);
//
//    // set the Iq controller
//    PI_setGains(obj->piHandle_Iq, Kp, Ki);
//    PI_setUi(obj->piHandle_Iq, 0.0f);
//    PI_setRefValue(obj->piHandle_Iq, 0.0f);
//    PI_setFbackValue(obj->piHandle_Iq, 0.0f);
//    PI_setFfwdValue(obj->piHandle_Iq, 0.0f);
//    PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);
//
//    return;
//} // end of setupCurrentControllers() function

void setupControllers(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    float32_t Ls_d_H = objUser->motor_Ls_d_H;
    float32_t Ls_q_H = objUser->motor_Ls_q_H;

    float32_t Rs_Ohm = objUser->motor_Rs_Ohm;
    float32_t RdoverLd_rps = Rs_Ohm / Ls_d_H;
    float32_t RqoverLq_rps = Rs_Ohm / Ls_q_H;

    float32_t BWc_rps = objUser->BWc_rps;
    float32_t currentCtrlPeriod_sec =
                (float32_t)objUser->numCtrlTicksPerCurrentTick /
                objUser->ctrlFreq_Hz;

    float32_t outMax_V = objUser->Vd_sf *
            objUser->maxVsMag_V;

    float32_t Kp_Id = Ls_d_H * BWc_rps;
    float32_t Ki_Id = 0.25f * RdoverLd_rps * currentCtrlPeriod_sec;

    float32_t Kp_Iq = Ls_q_H * BWc_rps;
    float32_t Ki_Iq = 0.25f * RqoverLq_rps * currentCtrlPeriod_sec;

    // set the Id controller
    PI_setGains(obj->piHandle_Id, Kp_Id, Ki_Id);
    PI_setUi(obj->piHandle_Id, 0.0f);
    PI_setRefValue(obj->piHandle_Id, 0.0f);
    PI_setFbackValue(obj->piHandle_Id, 0.0f);
    PI_setFfwdValue(obj->piHandle_Id, 0.0f);

    PI_setMinMax(obj->piHandle_Id, -outMax_V, outMax_V);

    // set the Iq controller
    PI_setGains(obj->piHandle_Iq, Kp_Iq, Ki_Iq);

    PI_setUi(obj->piHandle_Iq, 0.0f);
    PI_setRefValue(obj->piHandle_Iq, 0.0f);
    PI_setFbackValue(obj->piHandle_Iq, 0.0f);
    PI_setFfwdValue(obj->piHandle_Iq, 0.0f);
    PI_setMinMax(obj->piHandle_Iq, 0.0f, 0.0f);

    // set the speed controller
    if(objUser->Kctrl_Wb_p_kgm2 <= 0.01f)
    {
        float32_t Kp_spd1 = 2.5f * objUser->maxCurrent_A / objUser->maxFrequency_Hz;
        float32_t Ki_spd1 = 5.0f * objUser->maxCurrent_A * objUser->ctrlPeriod_sec;

        PI_setGains(obj->piHandle_spd, Kp_spd1, Ki_spd1);
    }
    else
    {
        float32_t speedCtrlPeriod_sec =
            (float32_t)objUser->numCtrlTicksPerSpeedTick /
            objUser->ctrlFreq_Hz;

        float32_t BWdelta = objUser->BWdelta;

        float32_t Kctrl_Wb_p_kgm2 = objUser->Kctrl_Wb_p_kgm2;

        float32_t Kp_spd = BWc_rps / (BWdelta * Kctrl_Wb_p_kgm2);
        float32_t Ki_spd = BWc_rps * speedCtrlPeriod_sec / (BWdelta * BWdelta);

        PI_setGains(obj->piHandle_spd, Kp_spd, Ki_spd);
    }
    PI_setUi(obj->piHandle_spd, 0.0f);
    PI_setRefValue(obj->piHandle_spd, 0.0f);
    PI_setFbackValue(obj->piHandle_spd, 0.0f);
    PI_setFfwdValue(obj->piHandle_spd, 0.0f);

    PI_setMinMax(obj->piHandle_spd,
                 -objUser->maxCurrent_A,
                 objUser->maxCurrent_A);

    // copy the Id, Iq and speed controller parameters to motorVars
    getControllers(handle);

    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    objSets->Rs_Ohm = objUser->Rs_Ohm;
    objSets->Ls_d_H = objUser->Ls_d_H;
    objSets->Ls_q_H = objUser->Ls_q_H;
    objSets->flux_VpHz = objUser->motor_ratedFlux_Wb * MATH_TWO_PI;

    return;
} // end of setupControllers() function


//
#if defined(MOTOR1_FWC) || defined(MOTOR2_FWC)
void updateFWCParams(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    // Update FW control parameters
    PI_setGains(obj->piHandle_fwc, objSets->Kp_fwc, objSets->Ki_fwc);
    PI_setOutMin(obj->piHandle_fwc, objSets->angleFWCMax_rad);
}
#endif  // MOTOR1_FWC || MOTOR2_FWC

//
#if defined(MOTOR1_MTPA) || defined(MOTOR2_MTPA)
void updateMTPAParams(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    if(obj->flagUpdateMTPAParams == true)
    {
        //
        // update motor parameters according to current
        //
        obj->LsOnline_d_H = MTPA_updateLs_d_withLUT(obj->mtpaHandle, obj->Is_A);

        obj->LsOnline_q_H = MTPA_updateLs_q_withLUT(obj->mtpaHandle, obj->Is_A);

        obj->fluxOnline_Wb = objSets->flux_Wb;

        //
        // update the motor constant for MTPA based on
        // the update Ls_d and Ls_q which are the function of Is
        //
        MTPA_computeParameters(obj->mtpaHandle,
                               obj->LsOnline_d_H,
                               obj->LsOnline_q_H,
                               obj->fluxOnline_Wb);
    }

    return;
}
#endif  // MOTOR1_MTPA || MOTOR2_MTPA

// update motor control variables
void updateGlobalVariables(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    // Calculate the RMS stator current
    obj->Is_A = sqrtf(obj->Idq_in_A.value[0] * obj->Idq_in_A.value[0] +
                      obj->Idq_in_A.value[1] * obj->Idq_in_A.value[1]);

    // Calculate the RMS stator voltage
    obj->Vs_V = sqrtf(obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0] +
                      obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]);

    // Calculate the motor input power
    obj->powerInvertOut_W = obj->Vs_V * obj->Is_A * 1.50f;

    // Add a filter to calculate the motor input power
    obj->powerActive_W = obj->Vdq_out_V.value[0] * obj->Idq_in_A.value[0] +
                         obj->Vdq_out_V.value[1] * obj->Idq_in_A.value[1];

    return;
} // end of updateGlobalVariables() function


//! \brief     Sets the number of current sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
void setupClarke_I(CLARKE_Handle handle, const uint16_t numCurrentSensors)
{
    float32_t alpha_sf, beta_sf;

    // initialize the Clarke transform module for current
    if(3 == numCurrentSensors)
    {
        alpha_sf = MATH_ONE_OVER_THREE;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else if(2 == numCurrentSensors)
    {
        alpha_sf = 1.0f;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else
    {
        alpha_sf = 0.0f;
        beta_sf = 0.0f;
    }

    // set the parameters
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numCurrentSensors);

    return;
} // end of setupClarke_I() function

//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
void setupClarke_V(CLARKE_Handle handle,const uint16_t numVoltageSensors)
{
    float32_t alpha_sf,beta_sf;

    // initialize the Clarke transform module for voltage
    if(numVoltageSensors == 3)
    {
        alpha_sf = MATH_ONE_OVER_THREE;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else
    {
        alpha_sf = 0.0f;
        beta_sf = 0.0f;
    }

    // set the parameters
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numVoltageSensors);

    return;
} // end of setupClarke_V() function

#if defined(BENCHMARK_TEST)
void recordSpeedData(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;

    if( (bmarkTestVars.speedRef_Hz != objMtr->speed_int_Hz) ||
        (bmarkTestVars.flagResetRecord == true))
    {
        bmarkTestVars.speedRef_Hz = objMtr->speed_int_Hz;

        bmarkTestVars.speedMax_Hz = objMtr->speed_int_Hz;
        bmarkTestVars.speedMin_Hz = objMtr->speed_int_Hz;

        bmarkTestVars.recordDataCount = 0;
        bmarkTestVars.flagResetRecord = false;
    }

    if(bmarkTestVars.speedRef_Hz == objMtr->speedRef_Hz)
    {
        if(bmarkTestVars.speedMax_Hz < objMtr->speedAbs_Hz)
        {
            bmarkTestVars.speedMax_Hz = objMtr->speedAbs_Hz;
        }

        if(bmarkTestVars.speedMin_Hz > objMtr->speedAbs_Hz)
        {
            bmarkTestVars.speedMin_Hz = objMtr->speedAbs_Hz;
        }

        bmarkTestVars.speedDelta_Hz =
                bmarkTestVars.speedMax_Hz - bmarkTestVars.speedMin_Hz;

        bmarkTestVars.recordTicksCount++;

        if(bmarkTestVars.recordTicksCount >=  bmarkTestVars.recordTicksSet)
        {
            bmarkTestVars.recordTicksCount = 0;

            if(bmarkTestVars.recordDataCount >= SPEED_RECORD_INDEX_NUM)
            {
                bmarkTestVars.recordDataCount = 0;
            }

            bmarkTestVars.speedBuff_Hz[bmarkTestVars.recordDataCount] =  objMtr->speedAbs_Hz;
            bmarkTestVars.recordDataCount++;
        }
    }

    return;
}
#endif  // BENCHMARK_TEST

//
//-- end of this file ----------------------------------------------------------
//
