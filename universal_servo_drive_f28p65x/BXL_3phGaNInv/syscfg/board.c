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

#include "board.h"

//*****************************************************************************
//
// Board Configurations
// Initializes the rest of the modules. 
// Call this function in your application if you wish to do all module 
// initialization.
// If you wish to not use some of the initializations, instead of the 
// Board_init use the individual Module_inits
//
//*****************************************************************************
void Board_init()
{
	EALLOW;

	PinMux_init();
	SYSCTL_init();
	INPUTXBAR_init();
	SYNC_init();
	ASYSCTL_init();
	ADC_init();
	CMPSS_init();
	CPUTIMER_init();
	DAC_init();
	EPWM_init();
	EPWMXBAR_init();
	EQEP_init();
	GPIO_init();
	SPI_init();
	INTERRUPT_init();

	EDIS;
}

//*****************************************************************************
//
// PINMUX Configurations
//
//*****************************************************************************
void PinMux_init()
{
	//
	// PinMux for modules assigned to CPU1
	//
	
	//
	// ANALOG -> myANALOGPinMux0 Pinmux
	//
	// Analog PinMux for A14/B14/C14
	GPIO_setPinConfig(GPIO_225_GPIO225);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(225, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A15/B15/C15
	GPIO_setPinConfig(GPIO_226_GPIO226);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(226, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A4
	GPIO_setPinConfig(GPIO_231_GPIO231);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(231, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A7, GPIO210
	GPIO_setPinConfig(GPIO_210_GPIO210);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(210, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B1/DACC_OUT
	GPIO_setPinConfig(GPIO_234_GPIO234);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(234, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B4, GPIO215
	GPIO_setPinConfig(GPIO_215_GPIO215);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(215, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B7, GPIO208
	GPIO_setPinConfig(GPIO_208_GPIO208);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(208, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C4, GPIO205
	GPIO_setPinConfig(GPIO_205_GPIO205);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(205, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C7, GPIO198
	GPIO_setPinConfig(GPIO_198_GPIO198);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(198, GPIO_ANALOG_ENABLED);
	//
	// EPWM5 -> MTR1_PWM_U Pinmux
	//
	GPIO_setPinConfig(MTR1_PWM_U_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(MTR1_PWM_U_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MTR1_PWM_U_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(MTR1_PWM_U_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(MTR1_PWM_U_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MTR1_PWM_U_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM4 -> MTR1_PWM_V Pinmux
	//
	GPIO_setPinConfig(MTR1_PWM_V_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(MTR1_PWM_V_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MTR1_PWM_V_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(MTR1_PWM_V_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(MTR1_PWM_V_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MTR1_PWM_V_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EPWM6 -> MTR1_PWM_W Pinmux
	//
	GPIO_setPinConfig(MTR1_PWM_W_EPWMA_PIN_CONFIG);
	GPIO_setPadConfig(MTR1_PWM_W_EPWMA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MTR1_PWM_W_EPWMA_GPIO, GPIO_QUAL_SYNC);

	GPIO_setPinConfig(MTR1_PWM_W_EPWMB_PIN_CONFIG);
	GPIO_setPadConfig(MTR1_PWM_W_EPWMB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MTR1_PWM_W_EPWMB_GPIO, GPIO_QUAL_SYNC);

	//
	// EQEP1 -> MTR1_QEP Pinmux
	//
	GPIO_setPinConfig(MTR1_QEP_EQEPA_PIN_CONFIG);
	GPIO_setPadConfig(MTR1_QEP_EQEPA_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MTR1_QEP_EQEPA_GPIO, GPIO_QUAL_3SAMPLE);

	GPIO_setPinConfig(MTR1_QEP_EQEPB_PIN_CONFIG);
	GPIO_setPadConfig(MTR1_QEP_EQEPB_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MTR1_QEP_EQEPB_GPIO, GPIO_QUAL_3SAMPLE);

	GPIO_setPinConfig(MTR1_QEP_EQEPINDEX_PIN_CONFIG);
	GPIO_setPadConfig(MTR1_QEP_EQEPINDEX_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MTR1_QEP_EQEPINDEX_GPIO, GPIO_QUAL_3SAMPLE);

	// GPIO12 -> MTR1_GATE_EN Pinmux
	GPIO_setPinConfig(GPIO_12_GPIO12);
	// GPIO14 -> MTR1_PM_nFAULT Pinmux
	GPIO_setPinConfig(GPIO_14_GPIO14);
	// GPIO13 -> LAUNCHPAD_LED1 Pinmux
	GPIO_setPinConfig(GPIO_13_GPIO13);
	//
	// SPID -> DAC_SPI Pinmux
	//
	GPIO_setPinConfig(DAC_SPI_SPIPICO_PIN_CONFIG);
	GPIO_setPadConfig(DAC_SPI_SPIPICO_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(DAC_SPI_SPIPICO_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(DAC_SPI_SPICLK_PIN_CONFIG);
	GPIO_setPadConfig(DAC_SPI_SPICLK_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(DAC_SPI_SPICLK_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(DAC_SPI_SPIPTE_PIN_CONFIG);
	GPIO_setPadConfig(DAC_SPI_SPIPTE_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(DAC_SPI_SPIPTE_GPIO, GPIO_QUAL_ASYNC);


}

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
void ADC_init(){
	ADCA_CONFIG_init();
	ADCB_CONFIG_init();
	ADCC_CONFIG_init();
}

void ADCA_CONFIG_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(ADCA_CONFIG_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_2_5V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(ADCA_CONFIG_BASE, ADC_CLK_DIV_4_0);
	//
	// Configures the analog-to-digital converter resolution and signal mode.
	//
	ADC_setMode(ADCA_CONFIG_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(ADCA_CONFIG_BASE, ADC_PULSE_END_OF_CONV);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(ADCA_CONFIG_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// Enable alternate timings for DMA trigger
	//
	ADC_enableAltDMATiming(ADCA_CONFIG_BASE);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(ADCA_CONFIG_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(ADCA_CONFIG_BASE, ADC_PRI_ALL_HIPRI);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM5_SOCA
	//	  	Channel			: ADC_CH_ADCIN7
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADCA_CONFIG_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM5_SOCA, ADC_CH_ADCIN7, 15U);
	ADC_setInterruptSOCTrigger(ADCA_CONFIG_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM5_SOCA
	//	  	Channel			: ADC_CH_ADCIN4
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADCA_CONFIG_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM5_SOCA, ADC_CH_ADCIN4, 20U);
	ADC_setInterruptSOCTrigger(ADCA_CONFIG_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 2 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM5_SOCA
	//	  	Channel			: ADC_CH_ADCIN14
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADCA_CONFIG_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM5_SOCA, ADC_CH_ADCIN14, 20U);
	ADC_setInterruptSOCTrigger(ADCA_CONFIG_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	//
	// ADC Interrupt 1 Configuration
	// 		Source	: ADC_INT_TRIGGER_EOC2
	// 		Interrupt Source: enabled
	// 		Continuous Mode	: disabled
	//
	//
	ADC_setInterruptSource(ADCA_CONFIG_BASE, ADC_INT_NUMBER1, ADC_INT_TRIGGER_EOC2);
	ADC_clearInterruptStatus(ADCA_CONFIG_BASE, ADC_INT_NUMBER1);
	ADC_disableContinuousMode(ADCA_CONFIG_BASE, ADC_INT_NUMBER1);
	ADC_enableInterrupt(ADCA_CONFIG_BASE, ADC_INT_NUMBER1);
			
	//
	// PPB Configuration: Configure high and low limits detection for ADCPPB
	//
	// Post Processing Block 1 Configuration
	// 		Configures a post-processing block (PPB) in the ADC.
	// 		PPB Number				: 1
	// 		SOC/EOC number			: 0
	// 		Calibration Offset		: 0
	// 		Reference Offset		: 0
	// 		Two's Complement		: Disabled
	// 		Trip High Limit			: 0
	// 		Trip Low Limit			: 0
	// 		Clear PPB Event Flags	: Disabled
	// 		Accumulation Limit		: 0
	// 		SyncInput Source		: ADC_SYNCIN_DISABLE
	// 		Comparator Source		: ADC_PPB_COMPSOURCE_RESULT
	// 		Right Shift				: 0
	// 		Absolute value				: false
	//
	ADC_setupPPB(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER0);
	ADC_disablePPBEvent(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_disablePPBEventInterrupt(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_setPPBCalibrationOffset(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1, 0);
	ADC_setPPBReferenceOffset(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1, 0);
	ADC_disablePPBTwosComplement(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1);
	ADC_setPPBTripLimits(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1, 0, 0);
	ADC_disablePPBEventCBCClear(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1);
	ADC_setPPBCountLimit(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1,0);
	ADC_selectPPBSyncInput(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1,ADC_SYNCIN_DISABLE);
	ADC_selectPPBCompareSource(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1,ADC_PPB_COMPSOURCE_RESULT);
	ADC_setPPBShiftValue(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1,0);
	ADC_disablePPBAbsoluteValue(ADCA_CONFIG_BASE, ADC_PPB_NUMBER1);
}

void ADCB_CONFIG_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(ADCB_CONFIG_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_2_5V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(ADCB_CONFIG_BASE, ADC_CLK_DIV_4_0);
	//
	// Configures the analog-to-digital converter resolution and signal mode.
	//
	ADC_setMode(ADCB_CONFIG_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(ADCB_CONFIG_BASE, ADC_PULSE_END_OF_CONV);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(ADCB_CONFIG_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// Enable alternate timings for DMA trigger
	//
	ADC_enableAltDMATiming(ADCB_CONFIG_BASE);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(ADCB_CONFIG_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(ADCB_CONFIG_BASE, ADC_PRI_ALL_HIPRI);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM5_SOCA
	//	  	Channel			: ADC_CH_ADCIN7
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADCB_CONFIG_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM5_SOCA, ADC_CH_ADCIN7, 15U);
	ADC_setInterruptSOCTrigger(ADCB_CONFIG_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM5_SOCA
	//	  	Channel			: ADC_CH_ADCIN4
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADCB_CONFIG_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM5_SOCA, ADC_CH_ADCIN4, 20U);
	ADC_setInterruptSOCTrigger(ADCB_CONFIG_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
			
	//
	// PPB Configuration: Configure high and low limits detection for ADCPPB
	//
	// Post Processing Block 1 Configuration
	// 		Configures a post-processing block (PPB) in the ADC.
	// 		PPB Number				: 1
	// 		SOC/EOC number			: 0
	// 		Calibration Offset		: 0
	// 		Reference Offset		: 0
	// 		Two's Complement		: Disabled
	// 		Trip High Limit			: 0
	// 		Trip Low Limit			: 0
	// 		Clear PPB Event Flags	: Disabled
	// 		Accumulation Limit		: 0
	// 		SyncInput Source		: ADC_SYNCIN_DISABLE
	// 		Comparator Source		: ADC_PPB_COMPSOURCE_RESULT
	// 		Right Shift				: 0
	// 		Absolute value				: false
	//
	ADC_setupPPB(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER0);
	ADC_disablePPBEvent(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_disablePPBEventInterrupt(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_setPPBCalibrationOffset(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1, 0);
	ADC_setPPBReferenceOffset(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1, 0);
	ADC_disablePPBTwosComplement(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1);
	ADC_setPPBTripLimits(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1, 0, 0);
	ADC_disablePPBEventCBCClear(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1);
	ADC_setPPBCountLimit(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1,0);
	ADC_selectPPBSyncInput(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1,ADC_SYNCIN_DISABLE);
	ADC_selectPPBCompareSource(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1,ADC_PPB_COMPSOURCE_RESULT);
	ADC_setPPBShiftValue(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1,0);
	ADC_disablePPBAbsoluteValue(ADCB_CONFIG_BASE, ADC_PPB_NUMBER1);
}

void ADCC_CONFIG_init(){
	//
	// ADC Initialization: Write ADC configurations and power up the ADC
	//
	// Set the analog voltage reference selection and ADC module's offset trims.
	// This function sets the analog voltage reference to internal (with the reference voltage of 1.65V or 2.5V) or external for ADC
	// which is same as ASysCtl APIs.
	//
	ADC_setVREF(ADCC_CONFIG_BASE, ADC_REFERENCE_EXTERNAL, ADC_REFERENCE_2_5V);
	//
	// Configures the analog-to-digital converter module prescaler.
	//
	ADC_setPrescaler(ADCC_CONFIG_BASE, ADC_CLK_DIV_4_0);
	//
	// Configures the analog-to-digital converter resolution and signal mode.
	//
	ADC_setMode(ADCC_CONFIG_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	//
	// Sets the timing of the end-of-conversion pulse
	//
	ADC_setInterruptPulseMode(ADCC_CONFIG_BASE, ADC_PULSE_END_OF_CONV);
	//
	// Powers up the analog-to-digital converter core.
	//
	ADC_enableConverter(ADCC_CONFIG_BASE);
	//
	// Delay for 1ms to allow ADC time to power up
	//
	DEVICE_DELAY_US(500);
	//
	// Enable alternate timings for DMA trigger
	//
	ADC_enableAltDMATiming(ADCC_CONFIG_BASE);
	//
	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	//
	// Disables SOC burst mode.
	//
	ADC_disableBurstMode(ADCC_CONFIG_BASE);
	//
	// Sets the priority mode of the SOCs.
	//
	ADC_setSOCPriority(ADCC_CONFIG_BASE, ADC_PRI_ALL_HIPRI);
	//
	// Start of Conversion 0 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM5_SOCA
	//	  	Channel			: ADC_CH_ADCIN7
	//	 	Sample Window	: 15 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADCC_CONFIG_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM5_SOCA, ADC_CH_ADCIN7, 15U);
	ADC_setInterruptSOCTrigger(ADCC_CONFIG_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	//
	// Start of Conversion 1 Configuration
	//
	//
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM5_SOCA
	//	  	Channel			: ADC_CH_ADCIN4
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	//
	ADC_setupSOC(ADCC_CONFIG_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM5_SOCA, ADC_CH_ADCIN4, 20U);
	ADC_setInterruptSOCTrigger(ADCC_CONFIG_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
			
	//
	// PPB Configuration: Configure high and low limits detection for ADCPPB
	//
	// Post Processing Block 1 Configuration
	// 		Configures a post-processing block (PPB) in the ADC.
	// 		PPB Number				: 1
	// 		SOC/EOC number			: 0
	// 		Calibration Offset		: 0
	// 		Reference Offset		: 0
	// 		Two's Complement		: Disabled
	// 		Trip High Limit			: 0
	// 		Trip Low Limit			: 0
	// 		Clear PPB Event Flags	: Disabled
	// 		Accumulation Limit		: 0
	// 		SyncInput Source		: ADC_SYNCIN_DISABLE
	// 		Comparator Source		: ADC_PPB_COMPSOURCE_RESULT
	// 		Right Shift				: 0
	// 		Absolute value				: false
	//
	ADC_setupPPB(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1, ADC_SOC_NUMBER0);
	ADC_disablePPBEvent(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_disablePPBEventInterrupt(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	ADC_setPPBCalibrationOffset(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1, 0);
	ADC_setPPBReferenceOffset(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1, 0);
	ADC_disablePPBTwosComplement(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1);
	ADC_setPPBTripLimits(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1, 0, 0);
	ADC_disablePPBEventCBCClear(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1);
	ADC_setPPBCountLimit(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1,0);
	ADC_selectPPBSyncInput(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1,ADC_SYNCIN_DISABLE);
	ADC_selectPPBCompareSource(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1,ADC_PPB_COMPSOURCE_RESULT);
	ADC_setPPBShiftValue(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1,0);
	ADC_disablePPBAbsoluteValue(ADCC_CONFIG_BASE, ADC_PPB_NUMBER1);
}


//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************
void ASYSCTL_init(){
	//
	// asysctl initialization
	//
	// Disables the temperature sensor output to the ADC.
	//
	ASysCtl_disableTemperatureSensor();
	//
	// Set the analog voltage reference selection to external.
	//
	ASysCtl_setAnalogReferenceExternal( ASYSCTL_VREFHIA | ASYSCTL_VREFHIB | ASYSCTL_VREFHIC );
}

//*****************************************************************************
//
// CMPSS Configurations
//
//*****************************************************************************
void CMPSS_init(){
	MTR1_CMPSS_U_H_init();
	MTR1_CMPSS_U_L_init();
	MTR1_CMPSS_V_init();
	MTR1_CMPSS_W_init();
}

void MTR1_CMPSS_U_H_init(){
    //
    // Select the value for CMP9HPMXSEL.
    //
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_9,2U);
    //
    // Select the value for CMP9LPMXSEL.
    //
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_9,0U);
    //
    // Sets the configuration for the high comparator.
    //
    CMPSS_configHighComparator(MTR1_CMPSS_U_H_BASE,(CMPSS_INSRC_DAC));
    //
    // Sets the configuration for the low comparator.
    //
    CMPSS_configLowComparator(MTR1_CMPSS_U_H_BASE,(CMPSS_INSRC_DAC));
    //
    // Sets the configuration for the internal comparator DACs.
    //
    CMPSS_configDACHigh(MTR1_CMPSS_U_H_BASE,(CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
    CMPSS_configDACLow(MTR1_CMPSS_U_H_BASE, CMPSS_DACSRC_SHDW);
    //
    // Sets the value of the internal DAC of the high comparator.
    //
    CMPSS_setDACValueHigh(MTR1_CMPSS_U_H_BASE,3584U);
    //
    // Sets the value of the internal DAC of the low comparator.
    //
    CMPSS_setDACValueLow(MTR1_CMPSS_U_H_BASE,0U);
    //
    // Sets the value of the internal DAC of the high comparator for Diode Emulation Support.
    //
    CMPSS_configHighDACShadowValueDE(MTR1_CMPSS_U_H_BASE, 0U);
    //
    // Sets the value of the internal DAC of the low comparator for Diode Emulation Support.
    //
    CMPSS_configLowDACShadowValueDE(MTR1_CMPSS_U_H_BASE, 0U);
    //
    // Set the DEACTIVE signal source for CMPSS
    //
    CMPSS_selectDEACTIVESource(MTR1_CMPSS_U_H_BASE, CMPSS_DEACTIVE_EPWM1);
    //
    //  Configures the digital filter of the high comparator.
    //
    CMPSS_configFilterHigh(MTR1_CMPSS_U_H_BASE, 32U, 32U, 30U);
    //
    // Configures the digital filter of the low comparator.
    //
    CMPSS_configFilterLow(MTR1_CMPSS_U_H_BASE, 0U, 1U, 1U);
    //
    // Initializes the digital filter of the high comparator.
    //
    CMPSS_initFilterHigh(MTR1_CMPSS_U_H_BASE);
    //
    // Sets the output signal configuration for the high comparator.
    //
    CMPSS_configOutputsHigh(MTR1_CMPSS_U_H_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the output signal configuration for the low comparator.
    //
    CMPSS_configOutputsLow(MTR1_CMPSS_U_H_BASE,(CMPSS_TRIPOUT_ASYNC_COMP | CMPSS_TRIP_ASYNC_COMP));
    //
    // Sets the comparator hysteresis settings.
    //
    CMPSS_setHysteresis(MTR1_CMPSS_U_H_BASE,0U);
    //
    // Configures the comparator subsystem's high ramp generator.
    //
    CMPSS_configRampHigh(MTR1_CMPSS_U_H_BASE, CMPSS_RAMP_DIR_DOWN, 0U,0U,0U,1U,true);
    //
    // Configures the comparator subsystem's low ramp generator.
    //
    CMPSS_configRampLow(MTR1_CMPSS_U_H_BASE, CMPSS_RAMP_DIR_DOWN, 0U,0U,0U,1U,true);
    //
    // Configures the high comparator's ramp generator clock divider
    //
    CMPSS_setRampClockDividerHigh(MTR1_CMPSS_U_H_BASE, CMPSS_RAMP_CLOCK_DIV1);
    //
    // Configures the low comparator's ramp generator clock divider
    //
    CMPSS_setRampClockDividerLow(MTR1_CMPSS_U_H_BASE, CMPSS_RAMP_CLOCK_DIV1);
    //
    // Disables reset of HIGH comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCHigh(MTR1_CMPSS_U_H_BASE);
    //
    // Disables reset of LOW comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCLow(MTR1_CMPSS_U_H_BASE);
    //
    // Sets the ePWM module blanking signal that holds trip in reset.
    //
    CMPSS_configBlanking(MTR1_CMPSS_U_H_BASE,1U);
    //
    // Disables an ePWM blanking signal from holding trip in reset.
    //
    CMPSS_disableBlanking(MTR1_CMPSS_U_H_BASE);
    //
    // Configures whether or not the digital filter latches are reset by PWMSYNC
    //
    CMPSS_configLatchOnPWMSYNC(MTR1_CMPSS_U_H_BASE,false,false);
    //
    // Enables the CMPSS module.
    //
    CMPSS_enableModule(MTR1_CMPSS_U_H_BASE);
    //
    // Delay for CMPSS DAC to power up.
    //
    DEVICE_DELAY_US(500);
    //
    // Causes a software reset of the high comparator digital filter output latch.
    //
    CMPSS_clearFilterLatchHigh(MTR1_CMPSS_U_H_BASE);
}
void MTR1_CMPSS_U_L_init(){
    //
    // Select the value for CMP4HPMXSEL.
    //
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_4,1U);
    //
    // Select the value for CMP4LPMXSEL.
    //
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_4,3U);
    //
    // Sets the configuration for the high comparator.
    //
    CMPSS_configHighComparator(MTR1_CMPSS_U_L_BASE,(CMPSS_INSRC_DAC));
    //
    // Sets the configuration for the low comparator.
    //
    CMPSS_configLowComparator(MTR1_CMPSS_U_L_BASE,(CMPSS_INSRC_DAC | CMPSS_INV_INVERTED));
    //
    // Sets the configuration for the internal comparator DACs.
    //
    CMPSS_configDACHigh(MTR1_CMPSS_U_L_BASE,(CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
    CMPSS_configDACLow(MTR1_CMPSS_U_L_BASE, CMPSS_DACSRC_SHDW);
    //
    // Sets the value of the internal DAC of the high comparator.
    //
    CMPSS_setDACValueHigh(MTR1_CMPSS_U_L_BASE,0U);
    //
    // Sets the value of the internal DAC of the low comparator.
    //
    CMPSS_setDACValueLow(MTR1_CMPSS_U_L_BASE,512U);
    //
    // Sets the value of the internal DAC of the high comparator for Diode Emulation Support.
    //
    CMPSS_configHighDACShadowValueDE(MTR1_CMPSS_U_L_BASE, 0U);
    //
    // Sets the value of the internal DAC of the low comparator for Diode Emulation Support.
    //
    CMPSS_configLowDACShadowValueDE(MTR1_CMPSS_U_L_BASE, 0U);
    //
    // Set the DEACTIVE signal source for CMPSS
    //
    CMPSS_selectDEACTIVESource(MTR1_CMPSS_U_L_BASE, CMPSS_DEACTIVE_EPWM1);
    //
    //  Configures the digital filter of the high comparator.
    //
    CMPSS_configFilterHigh(MTR1_CMPSS_U_L_BASE, 32U, 32U, 30U);
    //
    // Configures the digital filter of the low comparator.
    //
    CMPSS_configFilterLow(MTR1_CMPSS_U_L_BASE, 32U, 32U, 30U);
    //
    // Initializes the digital filter of the low comparator.
    //
    CMPSS_initFilterLow(MTR1_CMPSS_U_L_BASE);
    //
    // Sets the output signal configuration for the high comparator.
    //
    CMPSS_configOutputsHigh(MTR1_CMPSS_U_L_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the output signal configuration for the low comparator.
    //
    CMPSS_configOutputsLow(MTR1_CMPSS_U_L_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the comparator hysteresis settings.
    //
    CMPSS_setHysteresis(MTR1_CMPSS_U_L_BASE,0U);
    //
    // Configures the comparator subsystem's high ramp generator.
    //
    CMPSS_configRampHigh(MTR1_CMPSS_U_L_BASE, CMPSS_RAMP_DIR_DOWN, 0U,0U,0U,1U,true);
    //
    // Configures the comparator subsystem's low ramp generator.
    //
    CMPSS_configRampLow(MTR1_CMPSS_U_L_BASE, CMPSS_RAMP_DIR_DOWN, 0U,0U,0U,1U,true);
    //
    // Configures the high comparator's ramp generator clock divider
    //
    CMPSS_setRampClockDividerHigh(MTR1_CMPSS_U_L_BASE, CMPSS_RAMP_CLOCK_DIV1);
    //
    // Configures the low comparator's ramp generator clock divider
    //
    CMPSS_setRampClockDividerLow(MTR1_CMPSS_U_L_BASE, CMPSS_RAMP_CLOCK_DIV1);
    //
    // Disables reset of HIGH comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCHigh(MTR1_CMPSS_U_L_BASE);
    //
    // Disables reset of LOW comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCLow(MTR1_CMPSS_U_L_BASE);
    //
    // Sets the ePWM module blanking signal that holds trip in reset.
    //
    CMPSS_configBlanking(MTR1_CMPSS_U_L_BASE,1U);
    //
    // Disables an ePWM blanking signal from holding trip in reset.
    //
    CMPSS_disableBlanking(MTR1_CMPSS_U_L_BASE);
    //
    // Configures whether or not the digital filter latches are reset by PWMSYNC
    //
    CMPSS_configLatchOnPWMSYNC(MTR1_CMPSS_U_L_BASE,false,false);
    //
    // Enables the CMPSS module.
    //
    CMPSS_enableModule(MTR1_CMPSS_U_L_BASE);
    //
    // Delay for CMPSS DAC to power up.
    //
    DEVICE_DELAY_US(500);
    //
    // Causes a software reset of the low comparator digital filter output latch.
    //
    CMPSS_clearFilterLatchLow(MTR1_CMPSS_U_L_BASE);
}
void MTR1_CMPSS_V_init(){
    //
    // Select the value for CMP7HPMXSEL.
    //
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_7,2U);
    //
    // Select the value for CMP7LPMXSEL.
    //
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_7,2U);
    //
    // Sets the configuration for the high comparator.
    //
    CMPSS_configHighComparator(MTR1_CMPSS_V_BASE,(CMPSS_INSRC_DAC));
    //
    // Sets the configuration for the low comparator.
    //
    CMPSS_configLowComparator(MTR1_CMPSS_V_BASE,(CMPSS_INSRC_DAC | CMPSS_INV_INVERTED));
    //
    // Sets the configuration for the internal comparator DACs.
    //
    CMPSS_configDACHigh(MTR1_CMPSS_V_BASE,(CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
    CMPSS_configDACLow(MTR1_CMPSS_V_BASE, CMPSS_DACSRC_SHDW);
    //
    // Sets the value of the internal DAC of the high comparator.
    //
    CMPSS_setDACValueHigh(MTR1_CMPSS_V_BASE,3584U);
    //
    // Sets the value of the internal DAC of the low comparator.
    //
    CMPSS_setDACValueLow(MTR1_CMPSS_V_BASE,512U);
    //
    // Sets the value of the internal DAC of the high comparator for Diode Emulation Support.
    //
    CMPSS_configHighDACShadowValueDE(MTR1_CMPSS_V_BASE, 0U);
    //
    // Sets the value of the internal DAC of the low comparator for Diode Emulation Support.
    //
    CMPSS_configLowDACShadowValueDE(MTR1_CMPSS_V_BASE, 0U);
    //
    // Set the DEACTIVE signal source for CMPSS
    //
    CMPSS_selectDEACTIVESource(MTR1_CMPSS_V_BASE, CMPSS_DEACTIVE_EPWM1);
    //
    //  Configures the digital filter of the high comparator.
    //
    CMPSS_configFilterHigh(MTR1_CMPSS_V_BASE, 32U, 32U, 30U);
    //
    // Configures the digital filter of the low comparator.
    //
    CMPSS_configFilterLow(MTR1_CMPSS_V_BASE, 32U, 32U, 30U);
    //
    // Initializes the digital filter of the high comparator.
    //
    CMPSS_initFilterHigh(MTR1_CMPSS_V_BASE);
    //
    // Initializes the digital filter of the low comparator.
    //
    CMPSS_initFilterLow(MTR1_CMPSS_V_BASE);
    //
    // Sets the output signal configuration for the high comparator.
    //
    CMPSS_configOutputsHigh(MTR1_CMPSS_V_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the output signal configuration for the low comparator.
    //
    CMPSS_configOutputsLow(MTR1_CMPSS_V_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the comparator hysteresis settings.
    //
    CMPSS_setHysteresis(MTR1_CMPSS_V_BASE,0U);
    //
    // Configures the comparator subsystem's high ramp generator.
    //
    CMPSS_configRampHigh(MTR1_CMPSS_V_BASE, CMPSS_RAMP_DIR_DOWN, 0U,0U,0U,1U,true);
    //
    // Configures the comparator subsystem's low ramp generator.
    //
    CMPSS_configRampLow(MTR1_CMPSS_V_BASE, CMPSS_RAMP_DIR_DOWN, 0U,0U,0U,1U,true);
    //
    // Configures the high comparator's ramp generator clock divider
    //
    CMPSS_setRampClockDividerHigh(MTR1_CMPSS_V_BASE, CMPSS_RAMP_CLOCK_DIV1);
    //
    // Configures the low comparator's ramp generator clock divider
    //
    CMPSS_setRampClockDividerLow(MTR1_CMPSS_V_BASE, CMPSS_RAMP_CLOCK_DIV1);
    //
    // Disables reset of HIGH comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCHigh(MTR1_CMPSS_V_BASE);
    //
    // Disables reset of LOW comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCLow(MTR1_CMPSS_V_BASE);
    //
    // Sets the ePWM module blanking signal that holds trip in reset.
    //
    CMPSS_configBlanking(MTR1_CMPSS_V_BASE,1U);
    //
    // Disables an ePWM blanking signal from holding trip in reset.
    //
    CMPSS_disableBlanking(MTR1_CMPSS_V_BASE);
    //
    // Configures whether or not the digital filter latches are reset by PWMSYNC
    //
    CMPSS_configLatchOnPWMSYNC(MTR1_CMPSS_V_BASE,false,false);
    //
    // Enables the CMPSS module.
    //
    CMPSS_enableModule(MTR1_CMPSS_V_BASE);
    //
    // Delay for CMPSS DAC to power up.
    //
    DEVICE_DELAY_US(500);
    //
    // Causes a software reset of the low comparator digital filter output latch.
    //
    CMPSS_clearFilterLatchLow(MTR1_CMPSS_V_BASE);
}
void MTR1_CMPSS_W_init(){
    //
    // Select the value for CMP11HPMXSEL.
    //
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_11,1U);
    //
    // Select the value for CMP11LPMXSEL.
    //
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_11,1U);
    //
    // Sets the configuration for the high comparator.
    //
    CMPSS_configHighComparator(MTR1_CMPSS_W_BASE,(CMPSS_INSRC_DAC));
    //
    // Sets the configuration for the low comparator.
    //
    CMPSS_configLowComparator(MTR1_CMPSS_W_BASE,(CMPSS_INSRC_DAC | CMPSS_INV_INVERTED));
    //
    // Sets the configuration for the internal comparator DACs.
    //
    CMPSS_configDACHigh(MTR1_CMPSS_W_BASE,(CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW));
    CMPSS_configDACLow(MTR1_CMPSS_W_BASE, CMPSS_DACSRC_SHDW);
    //
    // Sets the value of the internal DAC of the high comparator.
    //
    CMPSS_setDACValueHigh(MTR1_CMPSS_W_BASE,3584U);
    //
    // Sets the value of the internal DAC of the low comparator.
    //
    CMPSS_setDACValueLow(MTR1_CMPSS_W_BASE,512U);
    //
    // Sets the value of the internal DAC of the high comparator for Diode Emulation Support.
    //
    CMPSS_configHighDACShadowValueDE(MTR1_CMPSS_W_BASE, 0U);
    //
    // Sets the value of the internal DAC of the low comparator for Diode Emulation Support.
    //
    CMPSS_configLowDACShadowValueDE(MTR1_CMPSS_W_BASE, 0U);
    //
    // Set the DEACTIVE signal source for CMPSS
    //
    CMPSS_selectDEACTIVESource(MTR1_CMPSS_W_BASE, CMPSS_DEACTIVE_EPWM1);
    //
    //  Configures the digital filter of the high comparator.
    //
    CMPSS_configFilterHigh(MTR1_CMPSS_W_BASE, 32U, 32U, 30U);
    //
    // Configures the digital filter of the low comparator.
    //
    CMPSS_configFilterLow(MTR1_CMPSS_W_BASE, 32U, 32U, 30U);
    //
    // Initializes the digital filter of the high comparator.
    //
    CMPSS_initFilterHigh(MTR1_CMPSS_W_BASE);
    //
    // Initializes the digital filter of the low comparator.
    //
    CMPSS_initFilterLow(MTR1_CMPSS_W_BASE);
    //
    // Sets the output signal configuration for the high comparator.
    //
    CMPSS_configOutputsHigh(MTR1_CMPSS_W_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the output signal configuration for the low comparator.
    //
    CMPSS_configOutputsLow(MTR1_CMPSS_W_BASE,(CMPSS_TRIPOUT_FILTER | CMPSS_TRIP_FILTER));
    //
    // Sets the comparator hysteresis settings.
    //
    CMPSS_setHysteresis(MTR1_CMPSS_W_BASE,0U);
    //
    // Configures the comparator subsystem's high ramp generator.
    //
    CMPSS_configRampHigh(MTR1_CMPSS_W_BASE, CMPSS_RAMP_DIR_DOWN, 0U,0U,0U,1U,true);
    //
    // Configures the comparator subsystem's low ramp generator.
    //
    CMPSS_configRampLow(MTR1_CMPSS_W_BASE, CMPSS_RAMP_DIR_DOWN, 0U,0U,0U,1U,true);
    //
    // Configures the high comparator's ramp generator clock divider
    //
    CMPSS_setRampClockDividerHigh(MTR1_CMPSS_W_BASE, CMPSS_RAMP_CLOCK_DIV1);
    //
    // Configures the low comparator's ramp generator clock divider
    //
    CMPSS_setRampClockDividerLow(MTR1_CMPSS_W_BASE, CMPSS_RAMP_CLOCK_DIV1);
    //
    // Disables reset of HIGH comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCHigh(MTR1_CMPSS_W_BASE);
    //
    // Disables reset of LOW comparator digital filter output latch on PWMSYNC
    //
    CMPSS_disableLatchResetOnPWMSYNCLow(MTR1_CMPSS_W_BASE);
    //
    // Sets the ePWM module blanking signal that holds trip in reset.
    //
    CMPSS_configBlanking(MTR1_CMPSS_W_BASE,1U);
    //
    // Disables an ePWM blanking signal from holding trip in reset.
    //
    CMPSS_disableBlanking(MTR1_CMPSS_W_BASE);
    //
    // Configures whether or not the digital filter latches are reset by PWMSYNC
    //
    CMPSS_configLatchOnPWMSYNC(MTR1_CMPSS_W_BASE,false,false);
    //
    // Enables the CMPSS module.
    //
    CMPSS_enableModule(MTR1_CMPSS_W_BASE);
    //
    // Delay for CMPSS DAC to power up.
    //
    DEVICE_DELAY_US(500);
    //
    // Causes a software reset of the low comparator digital filter output latch.
    //
    CMPSS_clearFilterLatchLow(MTR1_CMPSS_W_BASE);
}

//*****************************************************************************
//
// CPUTIMER Configurations
//
//*****************************************************************************
void CPUTIMER_init(){
	CPUTIMER_TimeBase_init();
	CPUTIMER_CpuUsage_init();
}

void CPUTIMER_TimeBase_init(){
	CPUTimer_setEmulationMode(CPUTIMER_TimeBase_BASE, CPUTIMER_EMULATIONMODE_RUNFREE);
	CPUTimer_setPreScaler(CPUTIMER_TimeBase_BASE, 0U);
	CPUTimer_setPeriod(CPUTIMER_TimeBase_BASE, 200000U);
	CPUTimer_disableInterrupt(CPUTIMER_TimeBase_BASE);
	CPUTimer_stopTimer(CPUTIMER_TimeBase_BASE);

	CPUTimer_reloadTimerCounter(CPUTIMER_TimeBase_BASE);
	CPUTimer_startTimer(CPUTIMER_TimeBase_BASE);
}
void CPUTIMER_CpuUsage_init(){
	CPUTimer_setEmulationMode(CPUTIMER_CpuUsage_BASE, CPUTIMER_EMULATIONMODE_RUNFREE);
	CPUTimer_selectClockSource(CPUTIMER_CpuUsage_BASE, CPUTIMER_CLOCK_SOURCE_SYS, CPUTIMER_CLOCK_PRESCALER_1);
	CPUTimer_setPreScaler(CPUTIMER_CpuUsage_BASE, 0U);
	CPUTimer_setPeriod(CPUTIMER_CpuUsage_BASE, 4294967295U);
	CPUTimer_disableInterrupt(CPUTIMER_CpuUsage_BASE);
	CPUTimer_stopTimer(CPUTIMER_CpuUsage_BASE);

	CPUTimer_reloadTimerCounter(CPUTIMER_CpuUsage_BASE);
	CPUTimer_startTimer(CPUTIMER_CpuUsage_BASE);
}

//*****************************************************************************
//
// DAC Configurations
//
//*****************************************************************************
void DAC_init(){
	MTR1_DAC_init();
}

void MTR1_DAC_init(){
	//
	// Set DAC reference voltage.
	//
	DAC_setReferenceVoltage(MTR1_DAC_BASE, DAC_REF_ADC_VREFHI);
	//
	// Set DAC gain mode.
	//
	DAC_setGainMode(MTR1_DAC_BASE, DAC_GAIN_ONE);
	//
	// Set DAC load mode.
	//
	DAC_setLoadMode(MTR1_DAC_BASE, DAC_LOAD_SYSCLK);
	//
	// Enable the DAC output
	//
	DAC_enableOutput(MTR1_DAC_BASE);
	//
	// Set the DAC shadow output
	//
	DAC_setShadowValue(MTR1_DAC_BASE, 0U);

	//
	// Delay for buffered DAC to power up.
	//
	DEVICE_DELAY_US(500);
}

//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
void EPWM_init(){
    EPWM_setClockPrescaler(MTR1_PWM_U_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(MTR1_PWM_U_BASE, 5000);	
    EPWM_setTimeBaseCounter(MTR1_PWM_U_BASE, 0);	
    EPWM_setTimeBaseCounterMode(MTR1_PWM_U_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_disablePhaseShiftLoad(MTR1_PWM_U_BASE);	
    EPWM_setPhaseShift(MTR1_PWM_U_BASE, 0);	
    EPWM_setSyncInPulseSource(MTR1_PWM_U_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5);	
    EPWM_enableSyncOutPulseSource(MTR1_PWM_U_BASE, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(MTR1_PWM_U_BASE, EPWM_COUNTER_COMPARE_A, 2500);	
    EPWM_setCounterCompareShadowLoadMode(MTR1_PWM_U_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(MTR1_PWM_U_BASE, EPWM_COUNTER_COMPARE_B, 2500);	
    EPWM_setCounterCompareShadowLoadMode(MTR1_PWM_U_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(MTR1_PWM_U_BASE, EPWM_COUNTER_COMPARE_C, 5);	
    EPWM_setCounterCompareValue(MTR1_PWM_U_BASE, EPWM_COUNTER_COMPARE_D, 5);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(MTR1_PWM_U_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(MTR1_PWM_U_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(MTR1_PWM_U_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(MTR1_PWM_U_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(MTR1_PWM_U_BASE);	
    EPWM_setRisingEdgeDelayCount(MTR1_PWM_U_BASE, 10);	
    EPWM_setDeadBandDelayMode(MTR1_PWM_U_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(MTR1_PWM_U_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(MTR1_PWM_U_BASE);	
    EPWM_setFallingEdgeDelayCount(MTR1_PWM_U_BASE, 10);	
    EPWM_setTripZoneAction(MTR1_PWM_U_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_U_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_U_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_U_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_U_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_U_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_enableTripZoneSignals(MTR1_PWM_U_BASE, EPWM_TZ_SIGNAL_DCAEVT1 | EPWM_TZ_SIGNAL_DCBEVT1 | EPWM_TZ_SIGNAL_OSHT1);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_U_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCAH);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_U_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCAH);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_U_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCAL);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_U_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCAL);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_U_BASE, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_HIGH);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_U_BASE, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DCXL_HIGH);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_U_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_U_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_U_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCBH);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_U_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCBH);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_U_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCBL);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_U_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCBL);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_U_BASE, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DCXH_HIGH);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_U_BASE, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DCXL_HIGH);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_U_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_U_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_enableADCTrigger(MTR1_PWM_U_BASE, EPWM_SOC_A);	
    EPWM_setADCTriggerSource(MTR1_PWM_U_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPC);	
    EPWM_setADCTriggerEventPrescale(MTR1_PWM_U_BASE, EPWM_SOC_A, 1);	
    EPWM_setClockPrescaler(MTR1_PWM_V_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(MTR1_PWM_V_BASE, 5000);	
    EPWM_setTimeBaseCounter(MTR1_PWM_V_BASE, 0);	
    EPWM_setTimeBaseCounterMode(MTR1_PWM_V_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_disablePhaseShiftLoad(MTR1_PWM_V_BASE);	
    EPWM_setPhaseShift(MTR1_PWM_V_BASE, 0);	
    EPWM_setSyncInPulseSource(MTR1_PWM_V_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5);	
    EPWM_enableSyncOutPulseSource(MTR1_PWM_V_BASE, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(MTR1_PWM_V_BASE, EPWM_COUNTER_COMPARE_A, 2500);	
    EPWM_setCounterCompareShadowLoadMode(MTR1_PWM_V_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(MTR1_PWM_V_BASE, EPWM_COUNTER_COMPARE_B, 2500);	
    EPWM_setCounterCompareShadowLoadMode(MTR1_PWM_V_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(MTR1_PWM_V_BASE, EPWM_COUNTER_COMPARE_C, 5);	
    EPWM_setCounterCompareValue(MTR1_PWM_V_BASE, EPWM_COUNTER_COMPARE_D, 5);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(MTR1_PWM_V_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(MTR1_PWM_V_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(MTR1_PWM_V_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(MTR1_PWM_V_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(MTR1_PWM_V_BASE);	
    EPWM_setRisingEdgeDelayCount(MTR1_PWM_V_BASE, 10);	
    EPWM_setDeadBandDelayMode(MTR1_PWM_V_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(MTR1_PWM_V_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(MTR1_PWM_V_BASE);	
    EPWM_setFallingEdgeDelayCount(MTR1_PWM_V_BASE, 10);	
    EPWM_setTripZoneAction(MTR1_PWM_V_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_V_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_V_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_V_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_V_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_V_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_enableTripZoneSignals(MTR1_PWM_V_BASE, EPWM_TZ_SIGNAL_DCAEVT1 | EPWM_TZ_SIGNAL_DCBEVT1 | EPWM_TZ_SIGNAL_OSHT1);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_V_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCAH);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_V_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCAH);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_V_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCAL);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_V_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCAL);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_V_BASE, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_HIGH);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_V_BASE, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DCXL_HIGH);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_V_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_V_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_V_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCBH);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_V_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCBH);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_V_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCBL);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_V_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCBL);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_V_BASE, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DCXH_HIGH);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_V_BASE, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DCXL_HIGH);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_V_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_V_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_setClockPrescaler(MTR1_PWM_W_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);	
    EPWM_setTimeBasePeriod(MTR1_PWM_W_BASE, 5000);	
    EPWM_setTimeBaseCounter(MTR1_PWM_W_BASE, 0);	
    EPWM_setTimeBaseCounterMode(MTR1_PWM_W_BASE, EPWM_COUNTER_MODE_UP_DOWN);	
    EPWM_disablePhaseShiftLoad(MTR1_PWM_W_BASE);	
    EPWM_setPhaseShift(MTR1_PWM_W_BASE, 0);	
    EPWM_setSyncInPulseSource(MTR1_PWM_W_BASE, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5);	
    EPWM_enableSyncOutPulseSource(MTR1_PWM_W_BASE, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(MTR1_PWM_W_BASE, EPWM_COUNTER_COMPARE_A, 2500);	
    EPWM_setCounterCompareShadowLoadMode(MTR1_PWM_W_BASE, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(MTR1_PWM_W_BASE, EPWM_COUNTER_COMPARE_B, 2500);	
    EPWM_setCounterCompareShadowLoadMode(MTR1_PWM_W_BASE, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);	
    EPWM_setCounterCompareValue(MTR1_PWM_W_BASE, EPWM_COUNTER_COMPARE_C, 5);	
    EPWM_setCounterCompareValue(MTR1_PWM_W_BASE, EPWM_COUNTER_COMPARE_D, 5);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);	
    EPWM_setActionQualifierAction(MTR1_PWM_W_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);	
    EPWM_setDeadBandDelayPolarity(MTR1_PWM_W_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);	
    EPWM_setDeadBandDelayMode(MTR1_PWM_W_BASE, EPWM_DB_RED, true);	
    EPWM_setRisingEdgeDelayCountShadowLoadMode(MTR1_PWM_W_BASE, EPWM_RED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableRisingEdgeDelayCountShadowLoadMode(MTR1_PWM_W_BASE);	
    EPWM_setRisingEdgeDelayCount(MTR1_PWM_W_BASE, 10);	
    EPWM_setDeadBandDelayMode(MTR1_PWM_W_BASE, EPWM_DB_FED, true);	
    EPWM_setFallingEdgeDelayCountShadowLoadMode(MTR1_PWM_W_BASE, EPWM_FED_LOAD_ON_CNTR_ZERO);	
    EPWM_disableFallingEdgeDelayCountShadowLoadMode(MTR1_PWM_W_BASE);	
    EPWM_setFallingEdgeDelayCount(MTR1_PWM_W_BASE, 10);	
    EPWM_setTripZoneAction(MTR1_PWM_W_BASE, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_W_BASE, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_W_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_W_BASE, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_W_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_LOW);	
    EPWM_setTripZoneAction(MTR1_PWM_W_BASE, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_LOW);	
    EPWM_enableTripZoneSignals(MTR1_PWM_W_BASE, EPWM_TZ_SIGNAL_DCAEVT1 | EPWM_TZ_SIGNAL_DCBEVT1 | EPWM_TZ_SIGNAL_OSHT1);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_W_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCAH);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_W_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCAH);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_W_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCAL);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_W_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCAL);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_W_BASE, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_HIGH);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_W_BASE, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DCXL_HIGH);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_W_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_W_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_W_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCBH);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_W_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCBH);	
    EPWM_selectDigitalCompareTripInput(MTR1_PWM_W_BASE, EPWM_DC_TRIP_COMBINATION, EPWM_DC_TYPE_DCBL);	
    EPWM_enableDigitalCompareTripCombinationInput(MTR1_PWM_W_BASE, EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCBL);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_W_BASE, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DCXH_HIGH);	
    EPWM_setTripZoneDigitalCompareEventCondition(MTR1_PWM_W_BASE, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DCXL_HIGH);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_W_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
    EPWM_setDigitalCompareEventSource(MTR1_PWM_W_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);	
}

//*****************************************************************************
//
// EPWMXBAR Configurations
//
//*****************************************************************************
void EPWMXBAR_init(){
	MTR1_IS_TRIP_CMPSS_init();
}

void MTR1_IS_TRIP_CMPSS_init(){
		
	XBAR_setEPWMMuxConfig(MTR1_IS_TRIP_CMPSS, XBAR_EPWM_MUX07_CMPSS4_CTRIPL);
	XBAR_setEPWMMuxConfig(MTR1_IS_TRIP_CMPSS, XBAR_EPWM_MUX12_CMPSS7_CTRIPH);
	XBAR_setEPWMMuxConfig(MTR1_IS_TRIP_CMPSS, XBAR_EPWM_MUX13_CMPSS7_CTRIPL);
	XBAR_setEPWMMuxConfig(MTR1_IS_TRIP_CMPSS, XBAR_EPWM_MUX58_CMPSS9_CTRIPH);
	XBAR_setEPWMMuxConfig(MTR1_IS_TRIP_CMPSS, XBAR_EPWM_MUX62_CMPSS11_CTRIPH);
	XBAR_setEPWMMuxConfig(MTR1_IS_TRIP_CMPSS, XBAR_EPWM_MUX63_CMPSS11_CTRIPL);
	XBAR_enableEPWMMux(MTR1_IS_TRIP_CMPSS, XBAR_MUX07 | XBAR_MUX12 | XBAR_MUX13 | XBAR_MUX58 | XBAR_MUX62 | XBAR_MUX63);
}

//*****************************************************************************
//
// EQEP Configurations
//
//*****************************************************************************
void EQEP_init(){
	MTR1_QEP_init();
}

void MTR1_QEP_init(){
	//
	// Disable, clear all flags and interrupts
	//
	EQEP_disableInterrupt(MTR1_QEP_BASE,
		(EQEP_INT_GLOBAL     		|   
		EQEP_INT_POS_CNT_ERROR		|      
		EQEP_INT_PHASE_ERROR    	| 
		EQEP_INT_DIR_CHANGE    		| 
		EQEP_INT_WATCHDOG          	|   
		EQEP_INT_UNDERFLOW         	|
		EQEP_INT_OVERFLOW        	|
		EQEP_INT_POS_COMP_READY    	|	
		EQEP_INT_POS_COMP_MATCH   	|
		EQEP_INT_STROBE_EVNT_LATCH	| 
		EQEP_INT_INDEX_EVNT_LATCH 	|
		EQEP_INT_UNIT_TIME_OUT   	|
		EQEP_INT_QMA_ERROR));
	EQEP_clearInterruptStatus(MTR1_QEP_BASE,
		(EQEP_INT_GLOBAL     		|   
		EQEP_INT_POS_CNT_ERROR		|      
		EQEP_INT_PHASE_ERROR    	| 
		EQEP_INT_DIR_CHANGE    		| 
		EQEP_INT_WATCHDOG          	|   
		EQEP_INT_UNDERFLOW         	|
		EQEP_INT_OVERFLOW        	|
		EQEP_INT_POS_COMP_READY    	|	
		EQEP_INT_POS_COMP_MATCH   	|
		EQEP_INT_STROBE_EVNT_LATCH	| 
		EQEP_INT_INDEX_EVNT_LATCH 	|
		EQEP_INT_UNIT_TIME_OUT   	|
		EQEP_INT_QMA_ERROR));
	EQEP_SourceSelect source_MTR1_QEP =
	{
		EQEP_SOURCE_DEVICE_PIN, 		// eQEPA source
		EQEP_SOURCE_DEVICE_PIN,		// eQEPB source
		EQEP_SOURCE_DEVICE_PIN,  	// eQEP Index source 
	};
	//
	// Selects the source for eQEPA/B/I signals
	//
	EQEP_selectSource(MTR1_QEP_BASE, source_MTR1_QEP);
	//
	// Set the strobe input source of the eQEP module.
	//
	EQEP_setStrobeSource(MTR1_QEP_BASE,EQEP_STROBE_FROM_GPIO);
	//
	// Sets the polarity of the eQEP module's input signals.
	//
	EQEP_setInputPolarity(MTR1_QEP_BASE,false,false,false,false);
	//
	// Configures eQEP module's quadrature decoder unit.
	//
	EQEP_setDecoderConfig(MTR1_QEP_BASE, (EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_2X_RESOLUTION | EQEP_CONFIG_NO_SWAP | EQEP_CONFIG_IGATE_DISABLE));
	//
	// Set the emulation mode of the eQEP module.
	//
	EQEP_setEmulationMode(MTR1_QEP_BASE,EQEP_EMULATIONMODE_RUNFREE);
	//
	// Configures eQEP module position counter unit.
	//
	EQEP_setPositionCounterConfig(MTR1_QEP_BASE,EQEP_POSITION_RESET_MAX_POS,3999U);
	//
	// Sets the current encoder position.
	//
	EQEP_setPosition(MTR1_QEP_BASE,0U);
	//
	// Enables the eQEP module unit timer.
	//
	EQEP_enableUnitTimer(MTR1_QEP_BASE,20000U);
	//
	// Disables the eQEP module watchdog timer.
	//
	EQEP_disableWatchdog(MTR1_QEP_BASE);
	//
	// Configures the quadrature modes in which the position count can be latched.
	//
	EQEP_setLatchMode(MTR1_QEP_BASE,(EQEP_LATCH_UNIT_TIME_OUT|EQEP_LATCH_RISING_STROBE|EQEP_LATCH_RISING_INDEX));
	//
	// Set the quadrature mode adapter (QMA) module mode.
	//
	EQEP_setQMAModuleMode(MTR1_QEP_BASE,EQEP_QMA_MODE_BYPASS);
	//
	// Disable Direction Change During Index
	//
	EQEP_disableDirectionChangeDuringIndex(MTR1_QEP_BASE);
	//
	// Enables individual eQEP module interrupt sources.
	//
	EQEP_enableInterrupt(MTR1_QEP_BASE,(EQEP_INT_UNIT_TIME_OUT));
	//
	// Configures the mode in which the position counter is initialized.
	//
	EQEP_setPositionInitMode(MTR1_QEP_BASE,(EQEP_INIT_RISING_INDEX));
	//
	// Sets the software initialization of the encoder position counter.
	//
	EQEP_setSWPositionInit(MTR1_QEP_BASE,true);
	//
	// Sets the init value for the encoder position counter.
	//
	EQEP_setInitialPosition(MTR1_QEP_BASE,0U);
	//
	// Enables the eQEP module.
	//
	EQEP_enableModule(MTR1_QEP_BASE);
	//
	// Configures eQEP module edge-capture unit.
	//
	EQEP_setCaptureConfig(MTR1_QEP_BASE,EQEP_CAPTURE_CLK_DIV_128,EQEP_UNIT_POS_EVNT_DIV_32);
	//
	// Enables the eQEP module edge-capture unit.
	//
	EQEP_enableCapture(MTR1_QEP_BASE);
}

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
void GPIO_init(){
	MTR1_GATE_EN_init();
	MTR1_PM_nFAULT_init();
	LAUNCHPAD_LED1_init();
}

void MTR1_GATE_EN_init(){
	GPIO_writePin(MTR1_GATE_EN, 1);
	GPIO_setPadConfig(MTR1_GATE_EN, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(MTR1_GATE_EN, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(MTR1_GATE_EN, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(MTR1_GATE_EN, GPIO_CORE_CPU1);
}
void MTR1_PM_nFAULT_init(){
	GPIO_setPadConfig(MTR1_PM_nFAULT, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(MTR1_PM_nFAULT, GPIO_QUAL_3SAMPLE);
	GPIO_setDirectionMode(MTR1_PM_nFAULT, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(MTR1_PM_nFAULT, GPIO_CORE_CPU1);
}
void LAUNCHPAD_LED1_init(){
	GPIO_writePin(LAUNCHPAD_LED1, 0);
	GPIO_setPadConfig(LAUNCHPAD_LED1, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(LAUNCHPAD_LED1, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(LAUNCHPAD_LED1, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(LAUNCHPAD_LED1, GPIO_CORE_CPU1);
}

//*****************************************************************************
//
// INPUTXBAR Configurations
//
//*****************************************************************************
void INPUTXBAR_init(){
	MTR1_PM_nFAULT_IN_init();
}

void MTR1_PM_nFAULT_IN_init(){
	XBAR_setInputPin(INPUTXBAR_BASE, MTR1_PM_nFAULT_IN_INPUT, MTR1_PM_nFAULT_IN_SOURCE);
	XBAR_lockInput(INPUTXBAR_BASE, MTR1_PM_nFAULT_IN_INPUT);
}

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************
void INTERRUPT_init(){
	
	// Interrupt Settings for INT_ADCA_CONFIG_1
	// ISR need to be defined for the registered interrupts
	Interrupt_register(INT_ADCA_CONFIG_1, &motor1CtrlISR);
	Interrupt_enable(INT_ADCA_CONFIG_1);
}
//*****************************************************************************
//
// SPI Configurations
//
//*****************************************************************************
void SPI_init(){
	DAC_SPI_init();
}

void DAC_SPI_init(){
	SPI_disableModule(DAC_SPI_BASE);
	SPI_setConfig(DAC_SPI_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
				  SPI_MODE_CONTROLLER, DAC_SPI_BITRATE, DAC_SPI_DATAWIDTH);
	SPI_setPTESignalPolarity(DAC_SPI_BASE, SPI_PTE_ACTIVE_LOW);
	SPI_enableFIFO(DAC_SPI_BASE);
	SPI_setFIFOInterruptLevel(DAC_SPI_BASE, SPI_FIFO_TX4, SPI_FIFO_RX4);
	SPI_disableLoopback(DAC_SPI_BASE);
	SPI_setEmulationMode(DAC_SPI_BASE, SPI_EMULATION_FREE_RUN);
	SPI_enableModule(DAC_SPI_BASE);
}

//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************
void SYNC_init(){
	SysCtl_setSyncOutputConfig(SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT);
	//
	// SOCA
	//
	SysCtl_enableExtADCSOCSource(0);
	//
	// SOCB
	//
	SysCtl_enableExtADCSOCSource(0);
}
//*****************************************************************************
//
// SYSCTL Configurations
//
//*****************************************************************************
void SYSCTL_init(){
	//
    // sysctl initialization
	//
    SysCtl_setStandbyQualificationPeriod(2);
    SysCtl_configureType(SYSCTL_USBTYPE, 0, 0);
    SysCtl_configureType(SYSCTL_ECAPTYPE, 0, 0);
    SysCtl_configureType(SYSCTL_SDFMTYPE, 0, 0);
    SysCtl_configureType(SYSCTL_MEMMAPTYPE, 0, 0);
    SysCtl_selectErrPinPolarity(0);

    SysCtl_disableMCD();

    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM7, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM8, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM9, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM10, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM11, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM12, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM13, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM14, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM15, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM16, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM17, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM18, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP7, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SD1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SD2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SD3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SD4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SCIA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SCIB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SPIA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SPIB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SPIC, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SPID, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_I2CA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_I2CB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CANA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCC, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER7, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER8, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS7, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS8, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS9, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS10, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS11, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_DCC0, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_DCC1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_DCC2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_DACA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_DACC, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSITXA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSITXB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSIRXA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSIRXB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSIRXC, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSIRXD, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_LINA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_LINB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAT, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_HRCAL0, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_HRCAL1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_HRCAL2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_AESA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPG1, SYSCTL_CPUSEL_CPU1);

    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCC, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCC, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCC, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS5, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS6, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS7, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS7, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS7, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS8, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS8, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS8, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS9, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS9, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS9, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS10, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS10, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS10, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS11, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS11, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS11, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACC, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACC, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACC, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM5, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM6, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM7, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM7, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM7, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM8, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM8, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM8, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM9, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM9, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM9, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM10, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM10, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM10, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM11, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM11, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM11, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM12, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM12, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM12, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM13, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM13, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM13, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM14, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM14, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM14, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM15, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM15, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM15, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM16, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM16, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM16, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM17, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM17, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM17, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM18, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM18, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM18, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP5, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP6, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP5, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP6, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP7, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP7, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP7, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB5, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB6, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIC, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIC, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIC, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPID, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPID, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPID, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_PMBUSA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_PMBUSA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_PMBUSA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CANA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CANA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CANA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIATX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIATX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIATX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIARX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIARX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIARX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBTX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBTX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBTX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBRX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBRX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBRX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSICRX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSICRX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSICRX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIDRX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIDRX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIDRX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_USBA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_USBA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_USBA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM0, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM0, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM0, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAT, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAT, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAT, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_AESA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_AESA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_AESA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DMA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CPUBGCRC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1BGCRC);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ERAD);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EMIF1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM8);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM9);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM10);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM11);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM12);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM13);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM14);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM15);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM16);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM17);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM18);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_UARTA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_UARTB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPID);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PMBUSA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_MCANA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_MCANB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_USBA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS8);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS9);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS10);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS11);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSITXA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSITXB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSIRXA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSIRXB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSIRXC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSIRXD);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_LINA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_LINB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAT);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HRCAL0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HRCAL1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HRCAL2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_AESA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPG1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER8);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCSEAGGRCPU1);



}

