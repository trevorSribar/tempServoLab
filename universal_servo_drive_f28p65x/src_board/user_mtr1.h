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


//! \file   /solutions/universal_servo_drive/common/include/user_mtr1.h
//! \brief  Contains the user related definitions
//!         This file is used for each device includes F28002x, F28003x, F280013x,
//!         F280015x, and the other newer C2000 MCUs.
//!


#ifndef USER_MTR1_H
#define USER_MTR1_H


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

//*****************************************************************************
//
//! \defgroup USER USER_MTR1
//! @{
//
//*****************************************************************************

//
// the includes
// platforms
#include "hal.h"

// modules
#include "userParams.h"

#include "user_common.h"

// *****************************************************************************
// the defines

//------------------------------------------------------------------------------
#if defined(BXL_3PHGANINV)
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (48.0f)

#if defined(ADC_EXT_REF)
//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V    (74.09004739f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz      (1103.026917f)      // 9.76k/47nF

//! \brief Defines the maximum current at the AD converter
#define USER_M1_FULL_SCALE_CURRENT_A         (60.0f)     // gain=20 

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2246.0f)
#define USER_M1_IB_OFFSET_AD    (2246.0f)
#define USER_M1_IC_OFFSET_AD    (2246.0f)
#else // !ADC_EXT_REF
//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (81.49905213f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (1103.026917f)     // 33nF

//! \brief Defines the maximum current at the AD converter
#define USER_M1_FULL_SCALE_CURRENT_A         (33.0f)     // gain=20

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2048.0f)
#define USER_M1_IB_OFFSET_AD    (2048.0f)
#define USER_M1_IC_OFFSET_AD    (2048.0f)
#endif // ADC_EXT_REF check

//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the inline shunt resistor is connected to the
//!        output of the three-phase power inverter and is also connected to
//!        the inverting pin of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is positive that
//!        means the same pin of the inline shunt resistor is connected to the
//!        output of the three-phase power inverter and is also connected to
//!        the non-inverting pin of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF         (-1.0f)

//! \brief ADC current offset for CMPSS
#define USER_M1_IS_OFFSET_CMPSS     (uint16_t)((USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD) / 3.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.500514159f)
#define USER_M1_VB_OFFSET_SF    (0.506255884f)
#define USER_M1_VC_OFFSET_SF    (0.503381569f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (72.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (60.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (10.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (12.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (2.0f) //0.2f

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (50.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (20.0f) //was 10.0f

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (2.0f) //changed frm 0.2 A

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)

//------------------------------------------------------------------------------
//! \brief ADC current offsets checking value for A, B, and C phases
// the error threshold to check if the ADC offset of the phase current sensing circuit is correct.
#define USER_M1_IS_OFFSET_AD_DELTA      (150.0f)    // The value is 0.0f~1024.0f

// the high threshold of the ADC offsets checking value for A/B/C phase current
#define USER_M1_IA_OFFSET_AD_MAX        (USER_M1_IA_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IB_OFFSET_AD_MAX        (USER_M1_IB_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IC_OFFSET_AD_MAX        (USER_M1_IC_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)

// the low threshold of the ADC offsets checking value for A phase current
#define USER_M1_IA_OFFSET_AD_MIN        (USER_M1_IA_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IB_OFFSET_AD_MIN        (USER_M1_IB_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IC_OFFSET_AD_MIN        (USER_M1_IC_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)

#define USER_M1_IS_OFFSET_AD_MAX        (USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD + (USER_M1_IS_OFFSET_AD_DELTA * 3.0f))
#define USER_M1_IS_OFFSET_AD_MIN        (USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD - (USER_M1_IS_OFFSET_AD_DELTA * 3.0f))

// end of BXL_3PHGANINV

//------------------------------------------------------------------------------
#elif defined(BXL_LMG2100_MD)
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (48.0f)

#if defined(ADC_EXT_REF)
//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V    (74.09004739f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz      (1103.026917f)      // xk/ynF

//! \brief Defines the maximum current at the AD converter
#define USER_M1_FULL_SCALE_CURRENT_A         (60.0f)     // gain=x

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2246.0f)
#define USER_M1_IB_OFFSET_AD    (2246.0f)
#define USER_M1_IC_OFFSET_AD    (2246.0f)
#else // !ADC_EXT_REF
//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (81.49905213f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (1103.026917f)     // 33nF

//! \brief Defines the maximum current at the AD converter
#define USER_M1_FULL_SCALE_CURRENT_A         (33.0f)     // gain=20

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2048.0f)
#define USER_M1_IB_OFFSET_AD    (2048.0f)
#define USER_M1_IC_OFFSET_AD    (2048.0f)
#endif // ADC_EXT_REF check

//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the inline shunt resistor is connected to the
//!        output of the three-phase power inverter and is also connected to
//!        the inverting pin of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is positive that
//!        means the same pin of the inline shunt resistor is connected to the
//!        output of the three-phase power inverter and is also connected to
//!        the non-inverting pin of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF         (1.0f)

//! \brief ADC current offset for CMPSS
#define USER_M1_IS_OFFSET_CMPSS     (uint16_t)((USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD) / 3.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.500514159f)
#define USER_M1_VB_OFFSET_SF    (0.506255884f)
#define USER_M1_VC_OFFSET_SF    (0.503381569f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (58.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (55.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (10.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (12.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.2f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (50.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)

//------------------------------------------------------------------------------
//! \brief ADC current offsets checking value for A, B, and C phases
// the error threshold to check if the ADC offset of the phase current sensing circuit is correct.
#define USER_M1_IS_OFFSET_AD_DELTA      (150.0f)    // The value is 0.0f~1024.0f

// the high threshold of the ADC offsets checking value for A/B/C phase current
#define USER_M1_IA_OFFSET_AD_MAX        (USER_M1_IA_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IB_OFFSET_AD_MAX        (USER_M1_IB_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IC_OFFSET_AD_MAX        (USER_M1_IC_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)

// the low threshold of the ADC offsets checking value for A phase current
#define USER_M1_IA_OFFSET_AD_MIN        (USER_M1_IA_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IB_OFFSET_AD_MIN        (USER_M1_IB_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IC_OFFSET_AD_MIN        (USER_M1_IC_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)

#define USER_M1_IS_OFFSET_AD_MAX        (USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD + (USER_M1_IS_OFFSET_AD_DELTA * 3.0f))
#define USER_M1_IS_OFFSET_AD_MIN        (USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD - (USER_M1_IS_OFFSET_AD_DELTA * 3.0f))

// end of BXL_LMG2100_MD

//------------------------------------------------------------------------------
#elif defined(BXL_AMC0106_LMG_MD)
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (48.0f)

#if defined(ADC_EXT_REF)
//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V    (74.09004739f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz      (1103.026917f)      // 9.76k/47nF
#else // !ADC_EXT_REF
//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (81.49905213f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (1103.026917f)     // 33nF
#endif // ADC_EXT_REF check

//! \brief Defines the maximum current
#define USER_M1_FULL_SCALE_CURRENT_A            (64.0f) // 1mOhm, AMC0106M05 (+/-64mV_clipping)
//#define USER_M1_FULL_SCALE_CURRENT_A            (320.0f)// 1mOhm, AMC0106M25 (+/-320mV_clipping)

//! \brief Defines the MIN / MAX SDFM data output. See sysconfig SDFM tool for value
#define USER_M1_SDFM_MIN_MAX_DATA               (16384.0f) // Sinc 3, 64 OSR, 16-bit data

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_SDFM      (-0.0430486239f)
#define USER_M1_IB_OFFSET_SDFM      (-0.0315055549f)
#define USER_M1_IC_OFFSET_SDFM      (-0.0467696562f)

//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the inline shunt resistor is connected to the
//!        output of the three-phase power inverter and is also connected to
//!        the inverting pin of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is positive that
//!        means the same pin of the inline shunt resistor is connected to the
//!        output of the three-phase power inverter and is also connected to
//!        the non-inverting pin of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF         (1.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.500514159f)
#define USER_M1_VB_OFFSET_SF    (0.506255884f)
#define USER_M1_VC_OFFSET_SF    (0.503381569f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (72.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (60.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (10.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (12.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.2f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (50.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)

//------------------------------------------------------------------------------
//! \brief SDFM current offsets checking value for A, B, and C phases
// the error threshold to check if the SDFM offset of the phase current sensing
// circuit is correct.
#define USER_M1_IS_OFFSET_SDFM_DELTA      (1.0f)    // The value is X.0f~Y.0f

// the high threshold of the ADC offsets checking value for A/B/C phase current
#define USER_M1_IA_OFFSET_SDFM_MAX        (USER_M1_IA_OFFSET_SDFM + USER_M1_IS_OFFSET_SDFM_DELTA)
#define USER_M1_IB_OFFSET_SDFM_MAX        (USER_M1_IB_OFFSET_SDFM + USER_M1_IS_OFFSET_SDFM_DELTA)
#define USER_M1_IC_OFFSET_SDFM_MAX        (USER_M1_IC_OFFSET_SDFM + USER_M1_IS_OFFSET_SDFM_DELTA)

// the low threshold of the ADC offsets checking value for A phase current
#define USER_M1_IA_OFFSET_SDFM_MIN        (USER_M1_IA_OFFSET_SDFM - USER_M1_IS_OFFSET_SDFM_DELTA)
#define USER_M1_IB_OFFSET_SDFM_MIN        (USER_M1_IB_OFFSET_SDFM - USER_M1_IS_OFFSET_SDFM_DELTA)
#define USER_M1_IC_OFFSET_SDFM_MIN        (USER_M1_IC_OFFSET_SDFM - USER_M1_IS_OFFSET_SDFM_DELTA)

#define USER_M1_IS_OFFSET_SDFM_MAX        (USER_M1_IA_OFFSET_SDFM + USER_M1_IB_OFFSET_SDFM + USER_M1_IC_OFFSET_SDFM + (USER_M1_IS_OFFSET_SDFM_DELTA * 3.0f))
#define USER_M1_IS_OFFSET_SDFM_MIN        (USER_M1_IA_OFFSET_SDFM + USER_M1_IB_OFFSET_SDFM + USER_M1_IC_OFFSET_SDFM - (USER_M1_IS_OFFSET_SDFM_DELTA * 3.0f))

// end of BXL_AMC0106_LMG_MD

//------------------------------------------------------------------------------
#elif defined(DRV8300DRGE_EVM)
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (100.0f)

#if defined(ADC_EXT_REF)
//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V    (100.2757475f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz      (524.1723874f)

//! \brief Defines the maximum current at the AD converter
#define USER_M1_FULL_SCALE_CURRENT_A         (100.0f)     // gain=10

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2226.0f)
#define USER_M1_IB_OFFSET_AD    (2226.0f)
#define USER_M1_IC_OFFSET_AD    (2226.0f)
#else // !ADC_EXT_REF
//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (110.3033223f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (524.1723874f)

//! \brief Defines the maximum current at the AD converter
#define USER_M1_FULL_SCALE_CURRENT_A         (110.0f)     // gain=10

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2048.0f)
#define USER_M1_IB_OFFSET_AD    (2048.0f)
#define USER_M1_IC_OFFSET_AD    (2048.0f)
#endif // ADC_EXT_REF check

//! \brief Defines the sign of the current_sf based on
//!        the polarity of the current feedback circuit
//!
//!        the "sign" = -1.0f if the current feedback polarity is positive that
//!        means the same pin of the inline shunt resistor is connected to the
//!        output of the three-phase power inverter and is also connected to
//!        the inverting pin of the operational amplifier
//!
//!        the "sign" = 1.0f if the current feedback polarity is positive that
//!        means the same pin of the inline shunt resistor is connected to the
//!        output of the three-phase power inverter and is also connected to
//!        the non-inverting pin of the operational amplifier
#define USER_M1_SIGN_CURRENT_SF         (-1.0f)

//! \brief ADC current offset for CMPSS
#define USER_M1_IS_OFFSET_CMPSS     (uint16_t)((USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD) / 3.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.500514159f)
#define USER_M1_VB_OFFSET_SF    (0.506255884f)
#define USER_M1_VC_OFFSET_SF    (0.503381569f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (120.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (115.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (10.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (12.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.2f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.3f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (1500.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (35.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.3f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)

//------------------------------------------------------------------------------
//! \brief ADC current offsets checking value for A, B, and C phases
// the error threshold to check if the ADC offset of the phase current sensing circuit is correct.
#define USER_M1_IS_OFFSET_AD_DELTA      (150.0f)    // The value is 0.0f~1024.0f

// the high threshold of the ADC offsets checking value for A/B/C phase current
#define USER_M1_IA_OFFSET_AD_MAX        (USER_M1_IA_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IB_OFFSET_AD_MAX        (USER_M1_IB_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IC_OFFSET_AD_MAX        (USER_M1_IC_OFFSET_AD + USER_M1_IS_OFFSET_AD_DELTA)

// the low threshold of the ADC offsets checking value for A phase current
#define USER_M1_IA_OFFSET_AD_MIN        (USER_M1_IA_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IB_OFFSET_AD_MIN        (USER_M1_IB_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)
#define USER_M1_IC_OFFSET_AD_MIN        (USER_M1_IC_OFFSET_AD - USER_M1_IS_OFFSET_AD_DELTA)

#define USER_M1_IS_OFFSET_AD_MAX        (USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD + (USER_M1_IS_OFFSET_AD_DELTA * 3.0f))
#define USER_M1_IS_OFFSET_AD_MIN        (USER_M1_IA_OFFSET_AD + USER_M1_IB_OFFSET_AD + USER_M1_IC_OFFSET_AD - (USER_M1_IS_OFFSET_AD_DELTA * 3.0f))

// end of BXL_3PHGANINV

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#else   // No Board Selection
#error The board parameters are not defined in user_mtr1.h
#endif  // No Board Selection

//------------------------------------------------------------------------------
//! \brief ADC voltage offsets for A, B, and C phases
// the error threshold to check if the ADC offset of the phase voltage sensing circuit is correct
#define USER_M1_VA_OFFSET_SF_DELTA      (0.05f)     // The value is 0.0f ~ 0.5f

// the high threshold of the ADC offsets checking value for A/B/C phase voltage
#define USER_M1_VA_OFFSET_SF_MAX        (USER_M1_VA_OFFSET_SF + USER_M1_VA_OFFSET_SF_DELTA)
#define USER_M1_VB_OFFSET_SF_MAX        (USER_M1_VB_OFFSET_SF + USER_M1_VA_OFFSET_SF_DELTA)
#define USER_M1_VC_OFFSET_SF_MAX        (USER_M1_VC_OFFSET_SF + USER_M1_VA_OFFSET_SF_DELTA)

// the low threshold of the ADC offsets checking value for A/B/C phase voltage
#define USER_M1_VA_OFFSET_SF_MIN        (USER_M1_VA_OFFSET_SF - USER_M1_VA_OFFSET_SF_DELTA)
#define USER_M1_VB_OFFSET_SF_MIN        (USER_M1_VB_OFFSET_SF - USER_M1_VA_OFFSET_SF_DELTA)
#define USER_M1_VC_OFFSET_SF_MIN        (USER_M1_VC_OFFSET_SF - USER_M1_VA_OFFSET_SF_DELTA)

#define USER_M1_VS_OFFSET_SF_MAX        (USER_M1_VA_OFFSET_SF + USER_M1_VB_OFFSET_SF + USER_M1_VC_OFFSET_SF + (USER_M1_VA_OFFSET_SF_DELTA * 3.0f))
#define USER_M1_VS_OFFSET_SF_MIN        (USER_M1_VA_OFFSET_SF + USER_M1_VB_OFFSET_SF + USER_M1_VC_OFFSET_SF - (USER_M1_VA_OFFSET_SF_DELTA * 3.0f))

//******************************************************************************
//! \brief Defines the number of pwm clock ticks per isr clock tick
//!        Note: Valid values are 1, 2 or 3 only
#define USER_M1_NUM_PWM_TICKS_PER_ISR_TICK          (1)

//! \brief Defines the number of ISR clock ticks per current controller clock tick
//!
#define USER_M1_NUM_ISR_TICKS_PER_CURRENT_TICK      (1)

//! \brief Defines the number of ISR clock ticks per speed controller clock tick
//!
#define USER_M1_NUM_ISR_TICKS_PER_SPEED_TICK        (10)

//! \brief Defines the number of current sensors
//!
#define USER_M1_NUM_CURRENT_SENSORS                 (3)

//! \brief Defines the number of voltage sensors
//!
#define USER_M1_NUM_VOLTAGE_SENSORS                 (3)

//! \brief Defines the Pulse Width Modulation (PWM) frequency, kHz
//!
#define USER_M1_PWM_FREQ_kHz        (20.0f)
#define USER_M1_PWM_TBPRD_NUM       (uint16_t)(USER_SYSTEM_FREQ_MHz * 1000.0f / USER_M1_PWM_FREQ_kHz / 2.0f)

//! \brief Defines the Pulse Width Modulation (PWM) period, usec
//!
#define USER_M1_PWM_PERIOD_usec     (1000.0f / USER_M1_PWM_FREQ_kHz)


//! \brief Defines the Interrupt Service Routine (ISR) frequency, Hz
//!
#define USER_M1_ISR_FREQ_Hz         (USER_M1_PWM_FREQ_kHz * 1000.0f / (float32_t)USER_M1_NUM_PWM_TICKS_PER_ISR_TICK)

//! \brief Defines the Interrupt Service Routine (ISR) period, usec
//!
#define USER_M1_ISR_PERIOD_usec     (USER_M1_PWM_PERIOD_usec * (float32_t)USER_M1_NUM_PWM_TICKS_PER_ISR_TICK)


//! \brief Defines the direct voltage (Vd) scale factor
//!
#define USER_M1_VD_SF               (0.95f)

//! \brief Defines the voltage scale factor for the system
//!
#define USER_M1_VOLTAGE_SF          (USER_M1_ADC_FULL_SCALE_VOLTAGE_V / 4096.0f)

#if defined(SDFM_I_SENSE)
//! \brief Defines the current scale factor for the system
//!
#define USER_M1_CURRENT_SF          (USER_M1_FULL_SCALE_CURRENT_A / USER_M1_SDFM_MIN_MAX_DATA)


//! \brief Defines the current scale invert factor for the system
//!
#define USER_M1_CURRENT_INV_SF      (USER_M1_SDFM_MIN_MAX_DATA / USER_M1_FULL_SCALE_CURRENT_A)
#else
//! \brief Defines the current scale factor for the system
//!
#define USER_M1_CURRENT_SF          (USER_M1_FULL_SCALE_CURRENT_A / 4096.0f)


//! \brief Defines the current scale invert factor for the system
//!
#define USER_M1_CURRENT_INV_SF      (4096.0f / USER_M1_FULL_SCALE_CURRENT_A)
#endif

//! \brief Defines the analog voltage filter pole location, rad/s
//!
#define USER_M1_VOLTAGE_FILTER_POLE_rps  (MATH_TWO_PI * USER_M1_VOLTAGE_FILTER_POLE_Hz)

//! \brief Defines the maximum Vs magnitude in per units allowed
//! \brief This value sets the maximum magnitude for the output of the Id and
//! \brief Iq PI current controllers. The Id and Iq current controller outputs
//! \brief are Vd and Vq. The relationship between Vs, Vd, and Vq is:
//! \brief Vs = sqrt(Vd^2 + Vq^2).  In this FOC controller, the Vd value is set
//! \brief equal to USER_MAX_VS_MAG*USER_VD_MAG_FACTOR.
//! \brief so the Vq value is set equal to sqrt(USER_MAX_VS_MAG^2 - Vd^2).
//!
//! \brief Set USER_MAX_VS_MAG = 0.5 for a pure sinewave with a peak at
//! \brief SQRT(3)/2 = 86.6% duty cycle.  No current reconstruction
//! \brief is needed for this scenario.
//!
//! \brief Set USER_MAX_VS_MAG = 1/SQRT(3) = 0.5774 for a pure sinewave
//! \brief with a peak at 100% duty cycle.  Current reconstruction
//! \brief will be needed for this scenario (Lab08).
//!
//! \brief Set USER_MAX_VS_MAG = 2/3 = 0.6666 to create a trapezoidal
//! \brief voltage waveform.  Current reconstruction will be needed
//! \brief for this scenario (Lab08).
//!
//! \brief For space vector over-modulation, see lab08 for details on
//! \brief system requirements that will allow the SVM generator to
//! \brief go all the way to trapezoidal.
//!
#define USER_M1_MAX_VS_MAG_PU             (0.66f) // Use this one for BL#2 Open-loop control.
//#define USER_M1_MAX_VS_MAG_PU             (0.65f)
//#define USER_M1_MAX_VS_MAG_PU               (0.576f)
//#define USER_M1_MAX_VS_MAG_PU             (0.565f)
//#define USER_M1_MAX_VS_MAG_PU             (0.5f)


//! \brief Defines the reference Vs magnitude in per units allowed
//! \      Set the value equal from 0.5 to 0.95 of the maximum Vs magnitude
#define USER_M1_VS_REF_MAG_PU               (0.8f * USER_MAX_VS_MAG_PU)

//! \brief Defines the R/L Kp scale factor, pu
//! \brief Kp used during R/L is USER_M1_R_OVER_L_KP_SF * USER_M1_NOMINAL_DC_BUS_VOLTAGE_V / USER_MOTOR1_MAX_CURRENT_A;
//!
#define USER_M1_R_OVER_L_KP_SF              (0.02f)


//! \brief Defines maximum acceleration for the estimation speed profiles, Hz/sec
//!
#define USER_M1_MAX_ACCEL_Hzps              (2.0f)


//! \brief Defines the controller execution period, usec
//!
#define USER_M1_CTRL_PERIOD_usec            ((float32_t)USER_M1_ISR_PERIOD_usec)


//! \brief Defines the controller execution period, sec
//!
#define USER_M1_CTRL_PERIOD_sec             ((float32_t)USER_M1_CTRL_PERIOD_usec / 1000000.0f)

//! \brief Defines the near zero speed limit for electrical frequency estimation, Hz
//!        The flux integrator uses this limit to regulate flux integration
#define USER_M1_FREQ_NEARZEROSPEEDLIMIT_Hz      (0.5f)

//! \brief Defines the fraction of IdRated to use during inductance estimation
//!
#define USER_M1_IDRATED_FRACTION_FOR_L_IDENT    (0.5f)


//! \brief Defines the fraction of SpeedMax to use during inductance estimation
//!
#define USER_M1_SPEEDMAX_FRACTION_FOR_L_IDENT   (1.0f)


//! \brief Defines the Power Warp gain for computing Id reference
//!
#define USER_M1_PW_GAIN                         (1.0f)

//! \brief Defines the pole location for the voltage and current offset estimation, rad/s
//!
#define USER_M1_OFFSET_POLE_rps                 (20.0f)


//! \brief Defines the pole location for the speed control filter, rad/sec
//!
#define USER_M1_SPEED_POLE_rps                  (100.0f)


//! \brief Defines the pole location for the direction filter, rad/sec
//!
#define USER_M1_DIRECTION_POLE_rps              (MATH_TWO_PI * 10.0f)

//! \brief Defines the pole location for the flux estimation, rad/sec
//!
#define USER_M1_FLUX_POLE_rps                   (10.0f)


//! \brief Defines the pole location for the R/L estimation, rad/sec
//!
#define USER_M1_R_OVER_L_POLE_rps               (MATH_TWO_PI * 3.2f)


//! \brief Defines the convergence factor for the estimator
//!
#define USER_M1_EST_KAPPAQ                      (1.5f)

//! \brief Defines the scaling factor for the current controller bandwidth
//!
#define USER_M1_CURRENT_BW_SF                   (10.0f)

//! \brief Defines the scaling factor for the BWdelta (Phase Margin bandwidth)
//!
#define USER_M1_BW_DELTA_SF                     (1.0f)

//------------------------------------------------------------------------------
//! brief Define the Kp gain for Field Weakening Control
#define USER_M1_FWC_KP                          (0.0525f)

//! brief Define the Ki gain for Field Weakening Control
#define USER_M1_FWC_KI                          (0.00325f)

//! brief Define the maximum current vector angle for Field Weakening Control
#define USER_M1_FWC_MAX_ANGLE          -15.0f                        // degree
#define USER_M1_FWC_MAX_ANGLE_RAD      USER_M1_FWC_MAX_ANGLE /120.0f * MATH_PI  // rad //was 180.0f

//! brief Define the minimum current vector angle for Field Weakening Control
#define USER_M1_FWC_MIN_ANGLE          0.0f                          // degree
#define USER_M1_FWC_MIN_ANGLE_RAD      USER_M1_FWC_MIN_ANGLE /180.0f * MATH_PI  // rad

//! \brief Defines the number of DC bus over/under voltage setting time
//!  timer base = 5ms
#define USER_M1_VOLTAGE_FAULT_TIME_SET          (500U)      // in 5ms

//! \brief Defines the number of motor over load setting time
//!  timer base = 5ms, 1s
#define USER_M1_OVER_LOAD_TIME_SET              (200U)

//! \brief Defines the number of motor stall setting time
//!  timer base = 5ms, 1s
#define USER_M1_STALL_TIME_SET                  (200U)

//! \brief Defines the number of phase unbalanced setting time
//!  timer base = 5ms, 5s
#define USER_M1_UNBALANCE_TIME_SET              (1000U)

//! \brief Defines the number of lost phase setting time
//!  timer base = 5ms, 10s
#define USER_M1_LOST_PHASE_TIME_SET             (2000U)

//! \brief Defines the number of over speed setting time
//!  timer base = 5ms
#define USER_M1_OVER_SPEED_TIME_SET             (600U)

//! \brief Defines the number of startup failed setting time
//!  timer base = 5ms, 10s
#define USER_M1_STARTUP_FAIL_TIME_SET           (2000U)

//! \brief Defines the number of over load setting times
//!
#define USER_M1_OVER_CURRENT_TIMES_SET          (5U)

//! \brief Defines the number of stop wait time
//!  timer base = 5ms, 10s
#define USER_M1_STOP_WAIT_TIME_SET              (2000U)

//! \brief Defines the number of restart wait time
//!  timer base = 5ms, 10s
#define USER_M1_RESTART_WAIT_TIME_SET           (2000U)

//! \brief Defines the number of restart times when has a fault
//!
#define USER_M1_START_TIMES_SET                 (3U)

//! \brief Defines the alignment time (s)
//!
#define USER_M1_ALIGN_TIME_S                    (1.0f)

//! \brief Defines the QEP unit ticks
#define USER_M1_QEP_UNIT_TIMER_TICKS        (uint32_t)(USER_SYSTEM_FREQ_MHz/(2.0f * USER_M1_ISR_FREQ_Hz) * 1000000.0f)

//! \brief Defines the current filter pole location, Hz
#define USER_M1_IS_FILTER_POLE_Hz           (7500.0f)      // 7.5kHz

//! \brief Defines the current filter pole location, rad/s
//!
#define USER_M1_IS_FILTER_POLE_rps          (MATH_TWO_PI * USER_M1_IS_FILTER_POLE_Hz)


//! \brief Defines the voltage filter pole location, Hz
#define USER_M1_VS_FILTER_POLE_Hz           (30000.0f)     // 30.0kHz

//! \brief Defines the voltage filter pole location, rad/s
//!
#define USER_M1_VS_FILTER_POLE_rps          (MATH_TWO_PI * USER_M1_VS_FILTER_POLE_Hz)


//==============================================================================
// Only a few listed motor below are tested with the related algorithm as the comments
// TODO: Motor defines
// Motor defines
// High voltage PMSM Motors
//#define USER_MOTOR1 Estun_EMJ_04APB22            //* Tested, FAST/eSMO/ENC

// Low Voltage PMSM Motors
#define USER_MOTOR1 Teknic_M2310PLN04K            //* Tested, FAST/eSMO/ENC/HALL

//------------------------------------------------------------------------------
#if (USER_MOTOR1 == Teknic_M2310PLN04K)
// the motor type
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM

// the number of pole pairs of the motor
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)

// the rotor resistance value of the motor, in Ohm
#define USER_MOTOR1_Rr_Ohm                 (NULL)

// the stator resistance value of the motor, in Ohm
#define USER_MOTOR1_Rs_Ohm                 (0.44f) //was 0.393955578f

// the stator inductance value of the motor in the direct direction, in H
#define USER_MOTOR1_Ls_d_H                 (0.00054f) //was 0.000190442806f

// the stator inductance value of the motor in the quadrature direction, in H
#define USER_MOTOR1_Ls_q_H                 (0.00054f) //was same as above

// the rated flux value of the motor, in V/Hz
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0399353318f)

// the Id rated current value of the motor, in A. Induction motors only
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)

// the maximum current value for stator resistance (R_s) identification, in A
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.5f)

// the maximum current value to use for stator inductance identification, in A
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.0f)

// the maximum current value of the motor, in A
#define USER_MOTOR1_MAX_CURRENT_A          (4.5f) //was 6.6f

// the R/L excitation frequency for motor parameters identification, in Hz
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (60.0f)

// the inertia that describes the amount of mass, in Kg.m2
#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-06)

// the rated voltage of the motor, V
#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0f)          // V

// the minimum rotation frequency if the motor (Hz)
#define USER_MOTOR1_FREQ_MIN_Hz            (9.0f)           // Hz

// the maximum/base rotation frequency of the motor (Hz)
#define USER_MOTOR1_FREQ_MAX_Hz            (600.0f)         // Hz

// V/f Profile Parameters for open-loop in build level 2
// the low frequency f_low  of V/f profile, in Hz,
// set to 10% of rated motor frequency
#define USER_MOTOR1_FREQ_LOW_Hz            (5.0f)           // Hz

// the high frequency f_high of V/f profile, in Hz,
// set to 100% of rated motor frequency
#define USER_MOTOR1_FREQ_HIGH_Hz           (400.0f)         // Hz

// the minimum voltage V_min  of V/f profile,
// the value is suggested to set to 15% of rated motor voltage, in Volt.
#define USER_MOTOR1_VOLT_MIN_V             (1.0f)           // Volt

// the maximum voltage,  V_max of V/f profile,
// the value is suggested to set to 100% of rated motor voltage, in Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)          // Volt

// the current increasing delta value for running the motor with force open-loop , in A
#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A

// the current increasing delta value for motor rotor alignment, in A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A

// the current for running the motor with force open-loop or startup, in A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A

// the current for motor rotor alignment, in A
#define USER_MOTOR1_ALIGN_CURRENT_A        (0.3f)           // A //was 1.0f

// the current for start to run motor with closed-loop when the speed is
//  lower than the startup setting speed, in A
#define USER_MOTOR1_STARTUP_CURRENT_A      (1.5f)           // A

// the current for running the motor with torque control mode when start the motor, in A.
#define USER_MOTOR1_TORQUE_CURRENT_A       (3.0f)           // A

// the over-current threshold for the motor, in A.
// The value can be set to 50%~300% of the rated current of the motor
#define USER_MOTOR1_OVER_CURRENT_A         (20.0f)           // A 7.5f

// the speed threshold for start the motor, in Hz
#define USER_MOTOR1_SPEED_START_Hz         (35.0f)          // Hz

// the speed threshold for running the motor with force open-loop, in Hz
#define USER_MOTOR1_SPEED_FORCE_Hz         (30.0f)          // Hz

// the acceleration for start the motor, in Hz/s.
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)          // Hz/s

// the maximum acceleration for running the motor, in Hz/s
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)          // Hz/s

// the speed threshold for running the motor with flying start mode, in Hz
#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)           // Hz

// the current for motor brake, in A.
#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A

// the duration time for motor brake, in 5ms time base
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)         // 60s/5ms

#if defined(MOTOR1_ENC)
// Only for encoder
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)           // lines
#define USER_MOTOR1_ENC_POS_MAX            (USER_MOTOR1_NUM_ENC_SLOTS * 4 - 1)
#define USER_MOTOR1_ENC_POS_OFFSET         (668)            // lines
#endif  // MOTOR1_ENC

// Current and Speed PI Regulators Tuning Coefficient
// the low speed threshold for adjusting the Kp and Ki of the speed PI regulator
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)      // 10%~50% of the rated speed

// the high speed threshold for adjusting the Kp and Ki of the speed PI regulator
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)     // 50%~100% of the rated speed

// the gain coefficient to adjust the Kp of the speed PI regulator for startup
#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0

// the gain coefficient to adjust the Ki of the speed PI regulator for startup
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

// the low gain coefficient  to adjust the Kp of the speed PI regulator
#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0

// the low gain coefficient  to adjust the Ki of the speed PI regulator
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

// the high gain coefficient  to adjust the Kp of the speed PI regulator
#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0

// the high gain coefficient  to adjust the Ki of the speed PI regulator
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

// the low current threshold to adjust the Kp and Ki of the q-axis current PI regulator
#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)       // 10%~50% of the rated current

// the high current threshold to adjust the Kp and Ki of the q-axis current PI regulator
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)       // 50%~100% of the rated current

// the gain coefficient to adjust the Kp of the q-axis current PI regulator for startup
#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0

// the gain coefficient to adjust the Ki of the q-axis current PI regulator for startup
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

// the low gain coefficient to adjust the Kp of the q-axis current PI regulator
#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0

// the low gain coefficient to adjust the Ki of the q-axis current PI regulator
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

// the high gain coefficient to adjust the Kp of the d-axis current PI regulator
#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

// the high gain coefficient to adjust the Ki of the d-axis current PI regulator
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

// the gain coefficient to adjust the Kp of the q-axis current PI regulator
#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0

// the gain coefficient to adjust the Ki of the q-axis current PI regulator
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0


//-----------------------------------------------------------------------------
#elif (USER_MOTOR1 == Estun_EMJ_04APB22)
// Refer to the description of the following parameters for Teknic_M2310PLN04K
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (4)
#define USER_MOTOR1_Rr_Ohm                  (0.0f)
#define USER_MOTOR1_Rs_Ohm                  (2.3679111f)
#define USER_MOTOR1_Ls_d_H                  (0.00836551283f)
#define USER_MOTOR1_Ls_q_H                  (0.00836551283f)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.390533477f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (1.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.0f)
#define USER_MOTOR1_MAX_CURRENT_A           (6.5f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (40.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500)
#define USER_MOTOR1_INERTIA_Kgm2            (3.100017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

// Ls compensation coefficient
#define USER_MOTOR1_Ls_d_COMP_COEF         (0.15f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_q_COMP_COEF         (0.35f)          // 0.0f~0.5f
#define USER_MOTOR1_Ls_MIN_NUM_COEF        (0.55f)          // 0.5f~1.0f

#define USER_MOTOR1_RATED_VOLTAGE_V         (200.0f)
#define USER_MOTOR1_FREQ_MIN_Hz             (5.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_Hz             (400.0f)        // Hz

#define USER_MOTOR1_FREQ_LOW_Hz             (10.0f)         // Hz
#define USER_MOTOR1_FREQ_HIGH_Hz            (200.0f)        // Hz
#define USER_MOTOR1_VOLT_MIN_V              (20.0f)         // Volt
#define USER_MOTOR1_VOLT_MAX_V              (200.0f)        // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05f)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01f)         // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)          // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (1.0f)          // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (1.5f)          // A
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.0f)          // A
#define USER_MOTOR1_OVER_CURRENT_A          (6.5f)          // A

#define USER_MOTOR1_SPEED_START_Hz          (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)
#define USER_MOTOR1_ACCEL_START_Hzps        (10.0f)         // Hz/s
#define USER_MOTOR1_ACCEL_MAX_Hzps          (20.0f)        // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// only for encoder
#define USER_MOTOR1_ENC_POS_MAX            (USER_MOTOR1_NUM_ENC_SLOTS * 4 - 1)
#define USER_MOTOR1_ENC_POS_OFFSET         (668)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 100.0f

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

// Current and Speed PI Regulators Tuning Coefficient
#define USER_MOTOR1_GAIN_SPEED_LOW_Hz        (60.0f)
#define USER_MOTOR1_GAIN_SPEED_HIGH_Hz       (150.0f)

#define USER_MOTOR1_KP_SPD_START_SF          (1.5f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_START_SF          (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_LOW_SF            (2.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_LOW_SF            (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_SPD_HIGH_SF           (1.0f)       // 0.1~100.0
#define USER_MOTOR1_KI_SPD_HIGH_SF           (1.0f)       // 0.1~10.0

#define USER_MOTOR1_GAIN_IQ_LOW_A            (2.0f)
#define USER_MOTOR1_GAIN_IQ_HIGH_A           (6.0f)

#define USER_MOTOR1_KP_IQ_START_SF           (1.5f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_START_SF           (1.5f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_LOW_SF             (2.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_LOW_SF             (2.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_IQ_HIGH_SF            (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_IQ_HIGH_SF            (1.0f)       // 0.1~10.0

#define USER_MOTOR1_KP_ID_SF                 (1.0f)       // 0.1~10.0
#define USER_MOTOR1_KI_ID_SF                 (1.0f)       // 0.1~10.0

//------------------------------------------------------------------------------
#else
#error No motor type specified
#endif


#ifndef USER_MOTOR1
#error Motor type is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_TYPE
#error The motor type is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_NUM_POLE_PAIRS
#error Number of motor pole pairs is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Rr_Ohm
#error The rotor resistance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Rs_Ohm
#error The stator resistance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Ls_d_H
#error The direct stator inductance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Ls_q_H
#error The quadrature stator inductance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_RATED_FLUX_VpHz
#error The rated flux of motor is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_MAGNETIZING_CURRENT_A
#error The magnetizing current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_RES_EST_CURRENT_A
#error The resistance estimation current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_IND_EST_CURRENT_A
#error The inductance estimation current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_MAX_CURRENT_A
#error The maximum current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_FLUX_EXC_FREQ_Hz
#error The flux excitation frequency is not defined in user_mtr1.h
#endif

#if ((USER_M1_NUM_CURRENT_SENSORS < 2) || (USER_M1_NUM_CURRENT_SENSORS > 3))
#error The number of current sensors must be 2 or 3
#endif

#if (USER_M1_NUM_VOLTAGE_SENSORS != 3)
#error The number of voltage sensors must be 3
#endif

// **************************************************************************
// the globals
extern USER_Params userParams_M1;

// **************************************************************************
// the functions
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_setMotor1Params(userParams_Handle handle);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of USER_MTR1_H definition

