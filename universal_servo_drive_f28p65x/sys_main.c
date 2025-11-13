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


//! \file   /solutions/universal_servo_drive/common/source/sys_main.c
//!
//! \brief  This project is used to implement sensored-FOC motor control
//!         with Encoders.
//!

// include the related header files
//
#include "user.h"
#include "sys_settings.h"
#include "sys_main.h"
#include "board.h" // syscfg

volatile SYSTEM_Vars_t systemVars;
#pragma DATA_SECTION(systemVars,"sys_data");

#if defined(DAC128S_ENABLE)
DAC128S_Handle   dac128sHandle;        //!< the DAC128S interface handle
DAC128S_Obj      dac128s;              //!< the DAC128S interface object
#pragma DATA_SECTION(dac128sHandle,"sys_data");
#pragma DATA_SECTION(dac128s,"sys_data");

#define DAC_SCALE_SET       (4096.0f)     // 12bit
#endif  // DAC128S_ENABLE

#if defined(SFRA_ENABLE)
float32_t   sfraNoiseId;
float32_t   sfraNoiseIq;
float32_t   sfraNoiseSpd;
float32_t   sfraNoiseOut;
float32_t   sfraNoiseFdb;
SFRA_TEST_e sfraTestLoop;        //speedLoop;
bool        sfraCollectStart;

#pragma DATA_SECTION(sfraNoiseId, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseIq, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseSpd, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseOut, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraNoiseFdb, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraTestLoop, "SFRA_F32_Data");
#pragma DATA_SECTION(sfraCollectStart, "SFRA_F32_Data");
#endif  // SFRA_ENABLE

// **************************************************************************
// the functions
// !!! Please make sure that you had gone through the user guide, and follow the
// !!! guide to set up the kit and load the right code
void main(void)
{
    // Clear memory for system and controller
    // The variables must be assigned to these sector if need to be cleared to zero
    HAL_clearDataRAM((void *)loadStart_user_data, (uint16_t)loadSize_user_data);
    HAL_clearDataRAM((void *)loadStart_hal_data, (uint16_t)loadSize_hal_data);
    HAL_clearDataRAM((void *)loadStart_foc_data, (uint16_t)loadSize_foc_data);
    HAL_clearDataRAM((void *)loadStart_sys_data, (uint16_t)loadSize_sys_data);
    HAL_clearDataRAM((void *)loadStart_dmaBuf_data, (uint16_t)loadSize_dmaBuf_data);
    HAL_clearDataRAM((void *)loadStart_datalog_data, (uint16_t)loadSize_datalog_data);
    HAL_clearDataRAM((void *)loadStart_SFRA_F32_Data, (uint16_t)loadSize_SFRA_F32_Data);

#if defined(DATALOGF2_EN) && defined(STEP_RP_EN)
#error DATALOG and GRAPH_STEP_RESPONSE can't be used simultaneously on this device
#endif  // DATALOGF2_EN && STEP_RP_EN

    // Initialize device clock and peripherals
    Device_init();                  // call the function in device.c

    // Disable pin locks and enable internal pullups.
    Device_initGPIO();              // call the function in device.c

    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    Interrupt_initModule();         // call the function in driverlib.lib

    // Initializes the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    Interrupt_initVectorTable();    // call the function in driverlib.lib

    // sysconfig configuration function:
    Board_init();

#if defined(DAC_ON_CHIP_ENABLE)
#if defined(ADC_EXT_REF)
    DAC_tuneOffsetTrim(MTR1_DAC_BASE, 3.0f);
#else // !ADC_EXT_REF
    DAC_tuneOffsetTrim(MTR1_DAC_BASE, 3.3f);
#endif // ADC_EXT_REF check
#endif // DAC_ON_CHIP_ENABLE

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // set the driver parameters
    HAL_setParams(halHandle);

    // set the control parameters for motor 1

    motorHandle_M1 = (MOTOR_Handle)(&motorVars_M1);

    // set the reference speed, this can be replaced or removed
    motorVars_M1.flagEnableRunAndIdentify = false;

    motorVars_M1.speedRef_Hz = 60.0f;       // Hz

    initMotor1Handles(motorHandle_M1);
    initMotor1CtrlParameters(motorHandle_M1);

    // set up gate driver after completed GPIO configuration
    motorVars_M1.faultMtrNow.bit.gateDriver =
            HAL_MTR_setGateDriver(motorHandle_M1->halMtrHandle);

    // enable the ePWM module time base clock sync signal
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

#if defined(DATALOGF2_EN)
    // Initialize Datalog
    datalogHandle = DATALOGIF_init(&datalog, sizeof(datalog));
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

    HAL_setupDMAforDLOG(halHandle, 0, &datalogBuff1[0], &datalogBuff1[1]);
    HAL_setupDMAforDLOG(halHandle, 1, &datalogBuff2[0], &datalogBuff2[1]);

//    datalogObj->iptr[0] = &motorVars_M1.angleFOC_rad;
//    datalogObj->iptr[1] = &motorVars_M1.speed_Hz;
    datalogObj->iptr[0] = &motorVars_M1.senseData.I_A.value[0];
    datalogObj->iptr[1] = &motorVars_M1.senseData.I_A.value[1];
//    datalogObj->iptr[1] = &motorVars_M1.senseData.I_A.value[2];

#elif defined(DATALOGF4_EN) || defined(DATALOGI4_EN)
    // Initialize Datalog
    datalogHandle = DATALOGIF_init(&datalog, sizeof(datalog));
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

    HAL_setupDMAforDLOG(halHandle, 0, &datalogBuff1[0], &datalogBuff1[1]);
    HAL_setupDMAforDLOG(halHandle, 1, &datalogBuff2[0], &datalogBuff2[1]);
    HAL_setupDMAforDLOG(halHandle, 2, &datalogBuff3[0], &datalogBuff3[1]);
    HAL_setupDMAforDLOG(halHandle, 3, &datalogBuff4[0], &datalogBuff4[1]);

    // set datalog parameters
    datalogObj->iptr[0] = &motorVars_M1.senseData.I_A.value[0];
    datalogObj->iptr[1] = &motorVars_M1.senseData.I_A.value[1];
    datalogObj->iptr[2] = &motorVars_M1.senseData.I_A.value[2];
    datalogObj->iptr[3] = &motorVars_M1.angleFOC_rad;
//    datalogObj->iptr[1] = &motorVars_M1.speed_Hz;

#endif  // DATALOGF2_EN

#if defined(DAC128S_ENABLE)
    // initialize the DAC128S
    dac128sHandle = DAC128S_init(&dac128s);

    SPI_setTxFifoTransmitDelay(DAC_SPI_BASE, 0x04);
    SPI_clearInterruptStatus(DAC_SPI_BASE, SPI_INT_TXFF);

// The following settings are for output the values of different variables
// in each build level for debug. The User can select one of these groups in
// different build level as commented note
#define DAC_ENC_I_A   // define the DAC level

#if defined(DAC_ENC_I_A)
    dac128s.ptrData[0] = &motorVars_M1.angleENC_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.senseData.I_A.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.senseData.I_A.value[1];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.senseData.I_A.value[2];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = 2.0f * DAC_SCALE_SET / USER_M1_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_FULL_SCALE_CURRENT_A;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#elif defined(DAC_ENC_V_V)
    dac128s.ptrData[0] = &motorVars_M1.angleENC_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.senseData.V_V.value[0];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.senseData.V_V.value[1];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.senseData.V_V.value[2];        // CH_D

    dac128s.gain[0] = DAC_SCALE_SET / MATH_TWO_PI;
    dac128s.gain[1] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[2] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = 2.0f * DAC_SCALE_SET / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    dac128s.offset[0] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[1] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[2] = (uint16_t)(0.5f * DAC_SCALE_SET);
    dac128s.offset[3] = (uint16_t)(0.5f * DAC_SCALE_SET);
#endif  // (DMC_BUILDLEVEL <= DMC_LEVEL_3)
    DAC128S_writeCommand(dac128sHandle);
#endif  // DAC128S_ENABLE

#if defined(SFRA_ENABLE)
    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    configureSFRA(SFRA_GUI_PLOT_GH_H, USER_M1_ISR_FREQ_Hz);

    sfraNoiseId = 0.0f;
    sfraNoiseIq = 0.0f;
    sfraNoiseSpd = 0.0f;
    sfraNoiseOut = 0.0f;
    sfraNoiseFdb = 0.0f;
    sfraTestLoop = SFRA_TEST_D_AXIS;
    sfraCollectStart = false;
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
    GRAPH_init(&stepRPVars,
               &motorVars_M1.speedRef_Hz, &motorVars_M1.speed_Hz,
               &motorVars_M1.IdqRef_A.value[0], &motorVars_M1.Idq_in_A.value[0],
               &motorVars_M1.IdqRef_A.value[1], &motorVars_M1.Idq_in_A.value[1]);
#endif  // STEP_RP_EN

    systemVars.flagEnableSystem = true;

    motorVars_M1.flagEnableOffsetCalc = true;

    // run offset calibration for motor 1
    runMotor1OffsetsCalculation(motorHandle_M1);

    // enable global interrupts
    Interrupt_enableGlobal();

    // enable debug events
    ERTM;

    systemVars.powerRelayWaitTime_ms = POWER_RELAY_WAIT_TIME_ms;

    // Waiting for enable system flag to be set
    while(systemVars.flagEnableSystem == false)
    {
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

            systemVars.timerBase_1ms++;

            if(systemVars.timerBase_1ms > systemVars.powerRelayWaitTime_ms)
            {
                systemVars.flagEnableSystem = true;
                systemVars.timerBase_1ms = 0;
            }
        }
    }

    motorVars_M1.flagInitializeDone = true;

    while(systemVars.flagEnableSystem == true)
    {
        // loop while the enable system flag is true
        systemVars.mainLoopCnt++;

        // 1ms time base
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

            // toggle status LED on controller board
            systemVars.counterLED++;

            if(systemVars.counterLED > (uint16_t)(LED_BLINK_FREQ_Hz * 1000))
            {
                HAL_toggleGPIO(halHandle, LAUNCHPAD_LED1);     // Toggle on the LED

                systemVars.counterLED = 0;
            }

            systemVars.timerBase_1ms++;

            switch(systemVars.timerBase_1ms)
            {
                case 1:     // motor 1 protection check
                    runMotorMonitor(motorHandle_M1);
                    break;
                case 2:
                    calculateRMSData(motorHandle_M1);
                    break;
                case 3:
#if defined(MOTOR1_PI_TUNE)
                    // Tune the gains of the controllers
                    tuneControllerGains(motorHandle_M1);
#endif      // MOTOR1_PI_TUNE
                    break;
                case 4:     // calculate motor protection value
#if !defined(SDFM_I_SENSE)
                    calcMotorOverCurrentThreshold(motorHandle_M1);
#endif // !SDFM_I_SENSE
                    break;
                case 5:     // system control
                    systemVars.timerBase_1ms = 0;
                    systemVars.timerCnt_5ms++;
                    break;
            }

#if defined(SFRA_ENABLE)
            // SFRA test
            SFRA_F32_runBackgroundTask(&sfra1);
            SFRA_GUI_runSerialHostComms(&sfra1);
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
            // Generate Step response
            GRAPH_generateStepResponse(&stepRPVars);
#endif  // STEP_RP_EN

        }       // 1ms Timer

        // runs control for motor 1
        runMotor1Control(motorHandle_M1);

    } // end of while() loop

    // disable the PWM
    HAL_disablePWM(motorHandle_M1->halMtrHandle);

} // end of main() function

//
//-- end of this file ----------------------------------------------------------
//
