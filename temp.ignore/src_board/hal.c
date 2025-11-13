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


//! \file   solutions/universal_servo_drive/f28p65x/drivers/hal_all.c
//! \brief  Contains the various functions related to the HAL object
//!
//

//
// the includes
//
#include "user.h"
#include "board.h" // syscfg

//
// drivers
//

// modules

// platforms
#include "hal.h"
#include "hal_obj.h"

// libraries
#include "datalogIF.h"

#ifdef _FLASH
#pragma CODE_SECTION(Flash_initModule, ".TI.ramfunc");
#endif

// **************************************************************************
// the defines


// **************************************************************************
// the globals
HAL_Handle    halHandle;      //!< the handle for the hardware abstraction layer
HAL_Obj       hal;            //!< the hardware abstraction layer object
#pragma DATA_SECTION(halHandle, "hal_data");
#pragma DATA_SECTION(hal, "hal_data");

// **************************************************************************
// the functions
HAL_Handle HAL_init(void *pMemory,const size_t numBytes)
{
    HAL_Handle handle;
    HAL_Obj *obj;

    if(numBytes < sizeof(HAL_Obj))
    {
        return((HAL_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_Handle)pMemory;

    // assign the object
    obj = (HAL_Obj *)handle;

    // Two ADC modules in this device
    // initialize the ADC handles
    obj->adcHandle[0] = ADCA_BASE;
    obj->adcHandle[1] = ADCB_BASE;
    obj->adcHandle[2] = ADCC_BASE;

    // initialize the ADC results
    obj->adcResult[0] = ADCARESULT_BASE;
    obj->adcResult[1] = ADCBRESULT_BASE;
    obj->adcResult[2] = ADCCRESULT_BASE;

    // initialize DMA handle
    obj->dmaHandle = DMA_BASE;              //!< the DMA handle

#if defined(DATALOGF2_EN)
    // initialize DMA channel handle
    obj->dmaChHandle[0] = DMA_DATALOG1_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[1] = DMA_DATALOG2_BASE;     //!< the DMA Channel handle
#elif defined(DATALOGF4_EN) || defined(DATALOGI4_EN)
    obj->dmaChHandle[0] = DMA_DATALOG1_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[1] = DMA_DATALOG2_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[2] = DMA_DATALOG3_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[3] = DMA_DATALOG4_BASE;     //!< the DMA Channel handle
#endif  // DATALOGF2_EN | DATALOGF4_EN

    // initialize timer handles
    obj->timerHandle[0] = CPUTIMER0_BASE;
    obj->timerHandle[1] = CPUTIMER1_BASE;
    obj->timerHandle[2] = CPUTIMER2_BASE;

    return(handle);
} // end of HAL_init() function

HAL_MTR_Handle HAL_MTR1_init(void *pMemory, const size_t numBytes)
{
    HAL_MTR_Handle handle;
    HAL_MTR_Obj *obj;

    if(numBytes < sizeof(HAL_MTR_Obj))
    {
        return((HAL_MTR_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_MTR_Handle)pMemory;

    // assign the object
    obj = (HAL_MTR_Obj *)handle;


    // initialize PWM handles for Motor 1
    obj->pwmHandle[0] = MTR1_PWM_U_BASE;        //!< the PWM handle
    obj->pwmHandle[1] = MTR1_PWM_V_BASE;        //!< the PWM handle
    obj->pwmHandle[2] = MTR1_PWM_W_BASE;        //!< the PWM handle

#if !defined(SDFM_I_SENSE)
#if defined(DRV8300DRGE_EVM)
    // initialize CMPSS handle
    obj->cmpssHandle[0] = MTR1_CMPSS_U_BASE;  //!< the CMPSS handle
    obj->cmpssHandle[1] = MTR1_CMPSS_U_BASE;  //!< the CMPSS handle
    obj->cmpssHandle[2] = MTR1_CMPSS_V_BASE;  //!< the CMPSS handle
    obj->cmpssHandle[3] = MTR1_CMPSS_V_BASE;  //!< the CMPSS handle
    obj->cmpssHandle[4] = MTR1_CMPSS_W_H_BASE;  //!< the CMPSS handle
    obj->cmpssHandle[5] = MTR1_CMPSS_W_L_BASE;  //!< the CMPSS handle
#else
    // initialize CMPSS handle
    obj->cmpssHandle[0] = MTR1_CMPSS_U_H_BASE;  //!< the CMPSS handle
    obj->cmpssHandle[1] = MTR1_CMPSS_U_L_BASE;  //!< the CMPSS handle
    obj->cmpssHandle[2] = MTR1_CMPSS_V_BASE;  //!< the CMPSS handle
    obj->cmpssHandle[3] = MTR1_CMPSS_V_BASE;  //!< the CMPSS handle
    obj->cmpssHandle[4] = MTR1_CMPSS_W_BASE;  //!< the CMPSS handle
    obj->cmpssHandle[5] = MTR1_CMPSS_W_BASE;  //!< the CMPSS handle
#endif //DRV8300DRGE_EVM
#endif // !SDFM_I_SENSE

    // No PGA modules in this device

    // Assign gateEnableGPIO
#if defined(BXL_3PHGANINV)
    obj->gateEnableGPIO = MTR1_GATE_EN;
    // BXL_3PHGANINV
#endif  // Assign gateEnableGPIO

    // initialize QEP driver
    obj->qepHandle = MTR1_QEP_BASE;             // the QEP handle

    obj->motorNum = MTR_1;

    return(handle);
} // end of HAL_MTR1_init() function

void HAL_setParams(HAL_Handle handle)
{
    // disable global interrupts
    Interrupt_disableGlobal();

#ifdef _FLASH
    //
    // Copy time critical code and flash setup code to RAM. This includes the
    // following functions: InitFlash();
    //
    // The RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart symbols
    // are created by the linker. Refer to the device .cmd file.
    //
    memcpy(&runStart_ctrlfuncs, &loadStart_ctrlfuncs, (size_t)&loadSize_ctrlfuncs);
#endif  // _FLASH

#if defined(DATALOGF4_EN) || defined(DATALOGF2_EN)
    // setup the DMA
    HAL_setupDMA();
#endif  // DATALOGF4_EN || DATALOGF2_EN

    return;
} // end of HAL_setParams() function

void HAL_MTR_setParams(HAL_MTR_Handle handle, USER_Params *pUserParams)
{
    HAL_setNumCurrentSensors(handle, pUserParams->numCurrentSensors);

    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    EPWM_setTimeBasePeriod(obj->pwmHandle[0], USER_M1_PWM_TBPRD_NUM);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], USER_M1_PWM_TBPRD_NUM);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], USER_M1_PWM_TBPRD_NUM);

    // disable the PWM
    HAL_disablePWM(handle);

    return;
} // end of HAL_MTR_setParams() function

// HAL_setupGate & HAL_enableDRV
#if defined(BXL_3PHGANINV)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE (nEN_uC) to low for enabling the DRV
    GPIO_writePin(obj->gateEnableGPIO, 0);

    return;
} // HAL_setupGate() function
// BXL_3PHGANINV
#endif  // HAL_setupGate & HAL_enableDRV

#if defined(DATALOGF4_EN) || defined(DATALOGF2_EN)
void HAL_setupDMA(void)
{
    // Initializes the DMA controller to a known state
    DMA_initController();

    // Sets DMA emulation mode, Continue DMA operation
    DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);

    return;
}    //end of HAL_setupDMA() function

void HAL_setupDMAforDLOG(HAL_Handle handle, const uint16_t dmaChNum,
                          const void *destAddr, const void *srcAddr)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    DMA_configAddresses(obj->dmaChHandle[dmaChNum], destAddr, srcAddr);

    // configure DMA Channel
    DMA_configBurst(obj->dmaChHandle[dmaChNum], DLOG_BURST_SIZE, 2, 2);
    DMA_configTransfer(obj->dmaChHandle[dmaChNum], DLOG_TRANSFER_SIZE, 1, 1);
    DMA_configWrap(obj->dmaChHandle[dmaChNum], 0xFFFF, 0, 0xFFFF, 0);

    DMA_configMode(obj->dmaChHandle[dmaChNum], DMA_TRIGGER_SOFTWARE,
                   DMA_CFG_ONESHOT_ENABLE | DMA_CFG_CONTINUOUS_ENABLE |
                   DMA_CFG_SIZE_32BIT);

    DMA_setInterruptMode(obj->dmaChHandle[dmaChNum], DMA_INT_AT_END);
    DMA_enableTrigger(obj->dmaChHandle[dmaChNum]);
    DMA_disableInterrupt(obj->dmaChHandle[dmaChNum]);

    return;
}    //end of HAL_setupDMAforDLOG() function
#endif  // DATALOGF4_EN || DATALOGF2_EN

void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory)
{
    uint16_t *pMemoryStart;
    uint16_t loopCount, loopLength;

    pMemoryStart = pMemory;
    loopLength = lengthMemory;

    for(loopCount = 0; loopCount < loopLength; loopCount++)
    {
        *(pMemoryStart + loopCount) = 0x0000;
    }
}   //end of HAL_clearDataRAM() function

void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                             const uint16_t dacValH, const uint16_t dacValL)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[1], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[2], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[3], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[4], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[5], dacValL);

    return;
}   // end of HAL_setMtrCMPSSDACValue() function

void HAL_setTriggerPrams(HAL_PWMData_t *pPWMData, const float32_t systemFreq_MHz,
                   const float32_t deadband_us, const float32_t noiseWindow_us,
                   const float32_t adcSample_us)
{
    uint16_t deadband =  (uint16_t)(deadband_us * systemFreq_MHz);
    uint16_t noiseWindow =  (uint16_t)(noiseWindow_us * systemFreq_MHz);
    uint16_t adcSample =  (uint16_t)(adcSample_us * systemFreq_MHz);

    pPWMData->deadband = deadband;
    pPWMData->noiseWindow = noiseWindow;
    pPWMData->adcSample = adcSample;

    pPWMData->minCMPValue = deadband + noiseWindow + adcSample;

    return;
}   // end of HAL_setTriggerPrams() function

bool HAL_MTR_setGateDriver(HAL_MTR_Handle handle)
{
    bool driverStatus = false;

    SysCtl_delay(5000U);

    // Setup Gate Enable
#if defined(BXL_3PHGANINV)
    // turn on the 3PhGaN if present
    HAL_enableDRV(handle);
    SysCtl_delay(1000U);

    // BXL_3PHGANINV
#endif  // Setup Gate Enable

    return(driverStatus);
}


// end of file
