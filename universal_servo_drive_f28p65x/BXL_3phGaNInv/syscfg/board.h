/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BOARD_H
#define BOARD_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
//
// PinMux Configurations
//
//*****************************************************************************

//
// ANALOG -> myANALOGPinMux0 Pinmux
//

//
// EPWM5 -> MTR1_PWM_U Pinmux
//
//
// EPWM5_A - GPIO Settings
//
#define GPIO_PIN_EPWM5_A 8
#define MTR1_PWM_U_EPWMA_GPIO 8
#define MTR1_PWM_U_EPWMA_PIN_CONFIG GPIO_8_EPWM5_A
//
// EPWM5_B - GPIO Settings
//
#define GPIO_PIN_EPWM5_B 9
#define MTR1_PWM_U_EPWMB_GPIO 9
#define MTR1_PWM_U_EPWMB_PIN_CONFIG GPIO_9_EPWM5_B

//
// EPWM4 -> MTR1_PWM_V Pinmux
//
//
// EPWM4_A - GPIO Settings
//
#define GPIO_PIN_EPWM4_A 6
#define MTR1_PWM_V_EPWMA_GPIO 6
#define MTR1_PWM_V_EPWMA_PIN_CONFIG GPIO_6_EPWM4_A
//
// EPWM4_B - GPIO Settings
//
#define GPIO_PIN_EPWM4_B 7
#define MTR1_PWM_V_EPWMB_GPIO 7
#define MTR1_PWM_V_EPWMB_PIN_CONFIG GPIO_7_EPWM4_B

//
// EPWM6 -> MTR1_PWM_W Pinmux
//
//
// EPWM6_A - GPIO Settings
//
#define GPIO_PIN_EPWM6_A 10
#define MTR1_PWM_W_EPWMA_GPIO 10
#define MTR1_PWM_W_EPWMA_PIN_CONFIG GPIO_10_EPWM6_A
//
// EPWM6_B - GPIO Settings
//
#define GPIO_PIN_EPWM6_B 11
#define MTR1_PWM_W_EPWMB_GPIO 11
#define MTR1_PWM_W_EPWMB_PIN_CONFIG GPIO_11_EPWM6_B

//
// EQEP1 -> MTR1_QEP Pinmux
//
//
// EQEP1_A - GPIO Settings
//
#define GPIO_PIN_EQEP1_A 20
#define MTR1_QEP_EQEPA_GPIO 20
#define MTR1_QEP_EQEPA_PIN_CONFIG GPIO_20_EQEP1_A
//
// EQEP1_B - GPIO Settings
//
#define GPIO_PIN_EQEP1_B 21
#define MTR1_QEP_EQEPB_GPIO 21
#define MTR1_QEP_EQEPB_PIN_CONFIG GPIO_21_EQEP1_B
//
// EQEP1_INDEX - GPIO Settings
//
#define GPIO_PIN_EQEP1_INDEX 23
#define MTR1_QEP_EQEPINDEX_GPIO 23
#define MTR1_QEP_EQEPINDEX_PIN_CONFIG GPIO_23_EQEP1_INDEX
//
// GPIO12 - GPIO Settings
//
#define MTR1_GATE_EN_GPIO_PIN_CONFIG GPIO_12_GPIO12
//
// GPIO14 - GPIO Settings
//
#define MTR1_PM_nFAULT_GPIO_PIN_CONFIG GPIO_14_GPIO14
//
// GPIO13 - GPIO Settings
//
#define LAUNCHPAD_LED1_GPIO_PIN_CONFIG GPIO_13_GPIO13

//
// SPID -> DAC_SPI Pinmux
//
//
// SPID_PICO - GPIO Settings
//
#define GPIO_PIN_SPID_PICO 91
#define DAC_SPI_SPIPICO_GPIO 91
#define DAC_SPI_SPIPICO_PIN_CONFIG GPIO_91_SPID_PICO
//
// SPID_CLK - GPIO Settings
//
#define GPIO_PIN_SPID_CLK 93
#define DAC_SPI_SPICLK_GPIO 93
#define DAC_SPI_SPICLK_PIN_CONFIG GPIO_93_SPID_CLK
//
// SPID_PTE - GPIO Settings
//
#define GPIO_PIN_SPID_PTE 94
#define DAC_SPI_SPIPTE_GPIO 94
#define DAC_SPI_SPIPTE_PIN_CONFIG GPIO_94_SPID_PTE

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
#define ADCA_CONFIG_BASE ADCA_BASE
#define ADCA_CONFIG_RESULT_BASE ADCARESULT_BASE
#define MTR1_IU ADC_SOC_NUMBER0
#define MTR1_IU_FORCE ADC_FORCE_SOC0
#define MTR1_IU_ADC_BASE ADCA_BASE
#define MTR1_IU_RESULT_BASE ADCARESULT_BASE
#define MTR1_IU_SAMPLE_WINDOW 75
#define MTR1_IU_TRIGGER_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define MTR1_IU_CHANNEL ADC_CH_ADCIN7
#define MTR1_VU ADC_SOC_NUMBER1
#define MTR1_VU_FORCE ADC_FORCE_SOC1
#define MTR1_VU_ADC_BASE ADCA_BASE
#define MTR1_VU_RESULT_BASE ADCARESULT_BASE
#define MTR1_VU_SAMPLE_WINDOW 100
#define MTR1_VU_TRIGGER_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define MTR1_VU_CHANNEL ADC_CH_ADCIN4
#define MTR1_VDC ADC_SOC_NUMBER2
#define MTR1_VDC_FORCE ADC_FORCE_SOC2
#define MTR1_VDC_ADC_BASE ADCA_BASE
#define MTR1_VDC_RESULT_BASE ADCARESULT_BASE
#define MTR1_VDC_SAMPLE_WINDOW 100
#define MTR1_VDC_TRIGGER_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define MTR1_VDC_CHANNEL ADC_CH_ADCIN14
#define MTR1_IU_PPB ADC_PPB_NUMBER1
#define MTR1_IU_PPB_SOC ADC_SOC_NUMBER0
void ADCA_CONFIG_init();

#define ADCB_CONFIG_BASE ADCB_BASE
#define ADCB_CONFIG_RESULT_BASE ADCBRESULT_BASE
#define MTR1_IV ADC_SOC_NUMBER0
#define MTR1_IV_FORCE ADC_FORCE_SOC0
#define MTR1_IV_ADC_BASE ADCB_BASE
#define MTR1_IV_RESULT_BASE ADCBRESULT_BASE
#define MTR1_IV_SAMPLE_WINDOW 75
#define MTR1_IV_TRIGGER_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define MTR1_IV_CHANNEL ADC_CH_ADCIN7
#define MTR1_VV ADC_SOC_NUMBER1
#define MTR1_VV_FORCE ADC_FORCE_SOC1
#define MTR1_VV_ADC_BASE ADCB_BASE
#define MTR1_VV_RESULT_BASE ADCBRESULT_BASE
#define MTR1_VV_SAMPLE_WINDOW 100
#define MTR1_VV_TRIGGER_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define MTR1_VV_CHANNEL ADC_CH_ADCIN4
#define MTR1_IV_PPB ADC_PPB_NUMBER1
#define MTR1_IV_PPB_SOC ADC_SOC_NUMBER0
void ADCB_CONFIG_init();

#define ADCC_CONFIG_BASE ADCC_BASE
#define ADCC_CONFIG_RESULT_BASE ADCCRESULT_BASE
#define MTR1_IW ADC_SOC_NUMBER0
#define MTR1_IW_FORCE ADC_FORCE_SOC0
#define MTR1_IW_ADC_BASE ADCC_BASE
#define MTR1_IW_RESULT_BASE ADCCRESULT_BASE
#define MTR1_IW_SAMPLE_WINDOW 75
#define MTR1_IW_TRIGGER_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define MTR1_IW_CHANNEL ADC_CH_ADCIN7
#define MTR1_VW ADC_SOC_NUMBER1
#define MTR1_VW_FORCE ADC_FORCE_SOC1
#define MTR1_VW_ADC_BASE ADCC_BASE
#define MTR1_VW_RESULT_BASE ADCCRESULT_BASE
#define MTR1_VW_SAMPLE_WINDOW 100
#define MTR1_VW_TRIGGER_SOURCE ADC_TRIGGER_EPWM5_SOCA
#define MTR1_VW_CHANNEL ADC_CH_ADCIN4
#define MTR1_IW_PPB ADC_PPB_NUMBER1
#define MTR1_IW_PPB_SOC ADC_SOC_NUMBER0
void ADCC_CONFIG_init();


//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// CMPSS Configurations
//
//*****************************************************************************
#define MTR1_CMPSS_U_H_BASE CMPSS9_BASE
#define MTR1_CMPSS_U_H_HIGH_COMP_BASE CMPSS9_BASE    
#define MTR1_CMPSS_U_H_LOW_COMP_BASE CMPSS9_BASE    
void MTR1_CMPSS_U_H_init();
#define MTR1_CMPSS_U_L_BASE CMPSS4_BASE
#define MTR1_CMPSS_U_L_HIGH_COMP_BASE CMPSS4_BASE    
#define MTR1_CMPSS_U_L_LOW_COMP_BASE CMPSS4_BASE    
void MTR1_CMPSS_U_L_init();
#define MTR1_CMPSS_V_BASE CMPSS7_BASE
#define MTR1_CMPSS_V_HIGH_COMP_BASE CMPSS7_BASE    
#define MTR1_CMPSS_V_LOW_COMP_BASE CMPSS7_BASE    
void MTR1_CMPSS_V_init();
#define MTR1_CMPSS_W_BASE CMPSS11_BASE
#define MTR1_CMPSS_W_HIGH_COMP_BASE CMPSS11_BASE    
#define MTR1_CMPSS_W_LOW_COMP_BASE CMPSS11_BASE    
void MTR1_CMPSS_W_init();

//*****************************************************************************
//
// CPUTIMER Configurations
//
//*****************************************************************************
#define CPUTIMER_TimeBase_BASE CPUTIMER0_BASE
void CPUTIMER_TimeBase_init();
#define CPUTIMER_CpuUsage_BASE CPUTIMER2_BASE
void CPUTIMER_CpuUsage_init();

//*****************************************************************************
//
// DAC Configurations
//
//*****************************************************************************
#define MTR1_DAC_BASE DACC_BASE
void MTR1_DAC_init();

//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
#define MTR1_PWM_U_BASE EPWM5_BASE
#define MTR1_PWM_U_TBPRD 5000
#define MTR1_PWM_U_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define MTR1_PWM_U_TBPHS 0
#define MTR1_PWM_U_CMPA 2500
#define MTR1_PWM_U_CMPB 2500
#define MTR1_PWM_U_CMPC 5
#define MTR1_PWM_U_CMPD 5
#define MTR1_PWM_U_DBRED 10
#define MTR1_PWM_U_DBFED 10
#define MTR1_PWM_U_TZA_ACTION EPWM_TZ_ACTION_LOW
#define MTR1_PWM_U_TZB_ACTION EPWM_TZ_ACTION_LOW
#define MTR1_PWM_U_OSHT_SOURCES (EPWM_TZ_SIGNAL_DCAEVT1 | EPWM_TZ_SIGNAL_DCBEVT1 | EPWM_TZ_SIGNAL_OSHT1)
#define MTR1_PWM_U_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define MTR1_PWM_V_BASE EPWM4_BASE
#define MTR1_PWM_V_TBPRD 5000
#define MTR1_PWM_V_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define MTR1_PWM_V_TBPHS 0
#define MTR1_PWM_V_CMPA 2500
#define MTR1_PWM_V_CMPB 2500
#define MTR1_PWM_V_CMPC 5
#define MTR1_PWM_V_CMPD 5
#define MTR1_PWM_V_DBRED 10
#define MTR1_PWM_V_DBFED 10
#define MTR1_PWM_V_TZA_ACTION EPWM_TZ_ACTION_LOW
#define MTR1_PWM_V_TZB_ACTION EPWM_TZ_ACTION_LOW
#define MTR1_PWM_V_OSHT_SOURCES (EPWM_TZ_SIGNAL_DCAEVT1 | EPWM_TZ_SIGNAL_DCBEVT1 | EPWM_TZ_SIGNAL_OSHT1)
#define MTR1_PWM_V_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define MTR1_PWM_W_BASE EPWM6_BASE
#define MTR1_PWM_W_TBPRD 5000
#define MTR1_PWM_W_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define MTR1_PWM_W_TBPHS 0
#define MTR1_PWM_W_CMPA 2500
#define MTR1_PWM_W_CMPB 2500
#define MTR1_PWM_W_CMPC 5
#define MTR1_PWM_W_CMPD 5
#define MTR1_PWM_W_DBRED 10
#define MTR1_PWM_W_DBFED 10
#define MTR1_PWM_W_TZA_ACTION EPWM_TZ_ACTION_LOW
#define MTR1_PWM_W_TZB_ACTION EPWM_TZ_ACTION_LOW
#define MTR1_PWM_W_OSHT_SOURCES (EPWM_TZ_SIGNAL_DCAEVT1 | EPWM_TZ_SIGNAL_DCBEVT1 | EPWM_TZ_SIGNAL_OSHT1)
#define MTR1_PWM_W_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED

//*****************************************************************************
//
// EPWMXBAR Configurations
//
//*****************************************************************************
void MTR1_IS_TRIP_CMPSS_init();
#define MTR1_IS_TRIP_CMPSS XBAR_TRIP7
#define MTR1_IS_TRIP_CMPSS_ENABLED_MUXES (XBAR_MUX07 | XBAR_MUX12 | XBAR_MUX13 | XBAR_MUX58 | XBAR_MUX62 | XBAR_MUX63)

//*****************************************************************************
//
// EQEP Configurations
//
//*****************************************************************************
#define MTR1_QEP_BASE EQEP1_BASE
void MTR1_QEP_init();

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
#define MTR1_GATE_EN 12
void MTR1_GATE_EN_init();
#define MTR1_PM_nFAULT 14
void MTR1_PM_nFAULT_init();
#define LAUNCHPAD_LED1 13
void LAUNCHPAD_LED1_init();

//*****************************************************************************
//
// INPUTXBAR Configurations
//
//*****************************************************************************
#define MTR1_PM_nFAULT_IN_SOURCE 14
#define MTR1_PM_nFAULT_IN_INPUT XBAR_INPUT1
void MTR1_PM_nFAULT_IN_init();

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************

// Interrupt Settings for INT_ADCA_CONFIG_1
// ISR need to be defined for the registered interrupts
#define INT_ADCA_CONFIG_1 INT_ADCA1
#define INT_ADCA_CONFIG_1_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP1
extern __interrupt void motor1CtrlISR(void);

//*****************************************************************************
//
// SPI Configurations
//
//*****************************************************************************
#define DAC_SPI_BASE SPID_BASE
#define DAC_SPI_BITRATE 4000000
#define DAC_SPI_DATAWIDTH 16
void DAC_SPI_init();

//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// SYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// Board Configurations
//
//*****************************************************************************
void	Board_init();
void	ADC_init();
void	ASYSCTL_init();
void	CMPSS_init();
void	CPUTIMER_init();
void	DAC_init();
void	EPWM_init();
void	EPWMXBAR_init();
void	EQEP_init();
void	GPIO_init();
void	INPUTXBAR_init();
void	INTERRUPT_init();
void	SPI_init();
void	SYNC_init();
void	SYSCTL_init();
void	PinMux_init();

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif  // end of BOARD_H definition
