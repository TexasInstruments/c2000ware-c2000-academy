//############################################################################
//
// FILE: lab_main.c
//
// TITLE: MCPWM & eCAP Academy Lab
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_control_peripherals_c2000_lab_pwm&packageId=C2000-ACADEMY
//
//! \addtogroup academy_lab_list
//! <h1> MCPWM & eCAP Academy Lab - Sysconfig </h1>
//!
//! The objective of this lab is to gain familiarity with the multi-channel pulse width modulation (MCPWM) module. A MCPWM module will be configured to generate a PWM signal of a specified frequency and duty cycle. The analog-to-digital converter (ADC) will then be used to sample that PWM signal. This will require a second MCPWM signal to trigger an ADC start-of-conversion (SOC) signal so that the ADC begins sampling. We will also measure the duty cycle and period of the first PWM signal with the enhanced capture (ECAP) module.
//!
//! \b External \b Connections \n
//!  - Connect the MCPWM1_A GPIO and ADCINA0 GPIO
//!  - Refer to Academy Lab instruction for exact pin for your device/board
//!
//! \b Watch \b Variables \n
//!  - eCapPwmDuty
//!  - eCapPwmPeriod
//!  - ePwm_curDuty 
//!  - DutyModOn 
//!
//############################################################################
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com
//
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
//############################################################################


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "mcpwm.h"

//
// Globals
//

uint32_t mcPwm_TimeBase;
uint32_t mcPwm_MinDuty;
uint32_t mcPwm_MaxDuty;
uint32_t mcPwm_curDuty;

//
// Buffer to store ADC samples
//
uint16_t AdcBuf[50];

//
// Pointer to ADC buffer samples
//
uint16_t *AdcBufPtr = AdcBuf;   

//
// Counter to slow down LED toggle in ADC ISR
//
uint16_t LedCtr = 0;   

//
// Flag to turn on/off duty cycle modulation
//
uint16_t DutyModOn = 0;     

//
// Flag to control duty mod direction up/down
//
uint16_t DutyModDir = 0;    

//
// Counter to slow down rate of modulation
//
uint16_t DutyModCtr = 0;    

//
// Percent = (eCapPwmDuty/eCapPwmPeriod)*100
//
int32_t eCapPwmDuty;          

//
// Frequency = DEVICE_SYSCLK_FREQ/eCapPwmPeriod
//
int32_t eCapPwmPeriod;          


__interrupt void adcA1ISR(void)
{
    //
    // Clear interrupt flags
    //
    Interrupt_clearACKGroup(INT_myADC0_1_INTERRUPT_ACK_GROUP);
    ADC_clearInterruptStatus(myADC0_BASE, ADC_INT_NUMBER1);

    //
    // Write contents of the ADC register to a circular buffer
    //
    *AdcBufPtr = ADC_readResult(myADC0_RESULT_BASE, myADC0_SOC0);
    if (AdcBufPtr == (AdcBuf + 49))
    {
        //
        // Force buffer to wrap around
        //
        AdcBufPtr = AdcBuf;
    } else {
        AdcBufPtr += 1;
    }
    if (LedCtr >= 49999) {
        //
        // Divide 50kHz sample rate by 50e3 to toggle LED at a rate of 1Hz
        //
        GPIO_togglePin(myBoardLED0_GPIO);
        LedCtr = 0;
    } else {
        LedCtr += 1;
    }
    if (DutyModOn) {
        //
        // Divide 50kHz sample rate by 16 to slow down duty modulation
        //
        if (DutyModCtr >= 15) {
            if (DutyModDir == 0) {
                //
                // Increment State => Decrease Duty Cycle
                //
                if (mcPwm_curDuty >= mcPwm_MinDuty) {
                    DutyModDir = 1;
                } else {
                    mcPwm_curDuty += 1;
                }
            } else {
                //
                // Decrement State => Increase Duty Cycle
                //
                if (mcPwm_curDuty <= mcPwm_MaxDuty) {
                    DutyModDir = 0;
                } else {
                    mcPwm_curDuty -= 1;
                }
            }
            DutyModCtr = 0;
        } else {
            DutyModCtr += 1;
        }
    }
    //
    // Set the counter compare value
    //
    MCPWM_setCounterCompareShadowValue(myMCPWM0_BASE, MCPWM_COUNTER_COMPARE_1A, mcPwm_curDuty);
}


__interrupt void ecap1ISR(void)
{
    ECAP_clearGlobalInterrupt(myECAP0_BASE);
    ECAP_clearInterrupt(myECAP0_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_3);
    eCapPwmDuty = (int32_t)ECAP_getEventTimeStamp(myECAP0_BASE, ECAP_EVENT_2) -
                  (int32_t)ECAP_getEventTimeStamp(myECAP0_BASE, ECAP_EVENT_1);
    eCapPwmPeriod = (int32_t)ECAP_getEventTimeStamp(myECAP0_BASE, ECAP_EVENT_3) -
                    (int32_t)ECAP_getEventTimeStamp(myECAP0_BASE, ECAP_EVENT_1);
    Interrupt_clearACKGroup(INT_myECAP0_INTERRUPT_ACK_GROUP);
}

//
// Main
//
void main(void)
{
	//
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Initialize all of the required peripherals using SysConfig
    //
    Board_init();

    //
    // Initialize variables for MCPWM Duty Cycle
    //
    mcPwm_TimeBase = MCPWM_getTimeBasePeriodActive(myMCPWM0_BASE);
    mcPwm_MinDuty = (uint32_t)(0.95f * (float)mcPwm_TimeBase);
    mcPwm_MaxDuty = (uint32_t)(0.05f * (float)mcPwm_TimeBase);
    mcPwm_curDuty = MCPWM_getCounterCompareActiveValue(myMCPWM0_BASE, MCPWM_COUNTER_COMPARE_1A);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Loop indefinitely
    //
    for (;;) {
        NOP;
    }
}

//
// End of File
//

