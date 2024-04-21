//#############################################################################
//
// FILE:   lab_main.c
//
// TITLE:  ePWM/eCAP Lab exercise
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_control_peripherals_c2000_lab_pwm&packageId=C2000-ACADEMY
//
// This lab configures ePWM at 1kHz, and eCAP to capture the events
// to measure pwm duty cycle
//
//#############################################################################
//
//
// $Copyright:
// Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
//#############################################################################


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"

uint32_t ePwm_TimeBase;
uint32_t ePwm_MinDuty;
uint32_t ePwm_MaxDuty;
uint32_t ePwm_curDuty;

uint16_t AdcBuf[50];            // Buffer to store ADC samples.
uint16_t *AdcBufPtr = AdcBuf;   // Pointer to ADC buffer samples.
uint16_t LedCtr = 0;            // Counter to slow down LED toggle in ADC ISR.
uint16_t DutyModOn = 0;         // Flag to turn on/off duty cycle modulation.
uint16_t DutyModDir = 0;        // Flag to control duty mod direction up/down.
uint16_t DutyModCtr = 0;        // Counter to slow down rate of modulation.
int32_t eCapPwmDuty;            // Percent = (eCapPwmDuty/eCapPwmPeriod)*100.
int32_t eCapPwmPeriod;          // Frequency = DEVICE_SYSCLK_FREQ/eCapPwmPeriod.


__interrupt void adcA1ISR(void)
{
    // Clear interrupt flags.
    Interrupt_clearACKGroup(INT_myADC0_1_INTERRUPT_ACK_GROUP);
    ADC_clearInterruptStatus(myADC0_BASE, ADC_INT_NUMBER1);
    // Write contents of the ADC register to a circular buffer.
    *AdcBufPtr = ADC_readResult(myADC0_RESULT_BASE, myADC0_SOC0);
    if (AdcBufPtr == (AdcBuf + 49))
    {
        // Force buffer to wrap around.
        AdcBufPtr = AdcBuf;
    } else {
        AdcBufPtr += 1;
    }
    if (LedCtr >= 49999) {
        // Divide 50kHz sample rate by 50e3 to toggle LED at a rate of 1Hz.
        GPIO_togglePin(myBoardLED0_GPIO);
        LedCtr = 0;
    } else {
        LedCtr += 1;
    }
    if (DutyModOn) {
        // Divide 50kHz sample rate by 16 to slow down duty modulation.
        if (DutyModCtr >= 15) {
            if (DutyModDir == 0) {
                // Increment State => Decrease Duty Cycle.
                if (ePwm_curDuty >= ePwm_MinDuty) {
                    DutyModDir = 1;
                } else {
                    ePwm_curDuty += 1;
                }
            } else {
                // Decrement State => Increase Duty Cycle.
                if (ePwm_curDuty <= ePwm_MaxDuty) {
                    DutyModDir = 0;
                } else {
                    ePwm_curDuty -= 1;
                }
            }
            DutyModCtr = 0;
        } else {
            DutyModCtr += 1;
        }
    }
    // Set the counter compare value.
    EPWM_setCounterCompareValue(myEPWM0_BASE, EPWM_COUNTER_COMPARE_A, ePwm_curDuty);
}


__interrupt void ecap1ISR(void)
{
    Interrupt_clearACKGroup(INT_myECAP0_INTERRUPT_ACK_GROUP);
    ECAP_clearGlobalInterrupt(myECAP0_BASE);
    ECAP_clearInterrupt(myECAP0_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_3);
    eCapPwmDuty = (int32_t)ECAP_getEventTimeStamp(myECAP0_BASE, ECAP_EVENT_2) -
                  (int32_t)ECAP_getEventTimeStamp(myECAP0_BASE, ECAP_EVENT_1);
    eCapPwmPeriod = (int32_t)ECAP_getEventTimeStamp(myECAP0_BASE, ECAP_EVENT_3) -
                    (int32_t)ECAP_getEventTimeStamp(myECAP0_BASE, ECAP_EVENT_1);
}

//
// Main
//
void main(void)
{
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    Board_init();

    // Initialize variables for ePWM Duty Cycle
    ePwm_TimeBase = EPWM_getTimeBasePeriod(myEPWM0_BASE);
    ePwm_MinDuty = (uint32_t)(0.95f * (float)ePwm_TimeBase);
    ePwm_MaxDuty = (uint32_t)(0.05f * (float)ePwm_TimeBase);
    ePwm_curDuty = EPWM_getCounterCompareValue(myEPWM0_BASE, EPWM_COUNTER_COMPARE_A);

    EINT;
    ERTM;

    for (;;) {
        NOP;
    }
}

//
// End of File
//

