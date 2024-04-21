//#############################################################################
//
// FILE: lab_main.c
//
// TITLE: Lab - System Design
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_system_control_c2000_lab_system_design&packageId=C2000-ACADEMY
//
//! \addtogroup academy_lab_list
//! <h1> System Design Example Lab </h1>
//!
//! This lab will build on all of the different topics covered thus far. As a system
//! example, the epwm module is setup to output a 50kHz output with a duty cycle
//! that will vary based on an ADC conversion result. A comparator will also be
//! used to monitor the ADC conversion results and cause a one-shot trip to drive
//! the EPWM output low incase the ADC conversion is above a certain threshold.
//!
//! \b External \b Connections \n
//!  - Refer to Academy Lab instruction for exact pin for your device/board
//!
//! \b Watch \b Variables \n
//!  - None.
//!
//#############################################################################
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
//#############################################################################


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"


//
// Defines
//
#define EPWM_PERIOD             1000
#define ADC_SCALE               1/4096.0
#define comp_threshold          2500

#ifdef DACA_BASE
    #define USE_DAC                 //Comment line if not using DAC
#endif

//
// Globals
//
uint16_t adcAResult;          /// value used to store ADC conversion
uint16_t CMPA_result;         /// value used to calculate CMPA for new duty cycle value
uint16_t dacVal = 1000;       /// value used to set the DAC output
uint16_t ledBlinkRate = 50;   /// initialized value for LED blink rate
uint16_t LED_count = 0;       /// LED counter
uint16_t interruptCount=0;
float duty;                   /// value that holds part of the calculation for CMPA

//
// Function Prototypes
//
__interrupt void INT_myADCA_1_ISR(void);

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
    // Disable pin locks and enable internal pullups.
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
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Start ePWM1, enabling SOCA
    //
    EPWM_enableADCTrigger(myEPWM1_BASE, EPWM_SOC_A);

    //
    // Loop indefinitely
    //
    while(1)
    {

       #ifdef USE_DAC
       //
       // Set the value for the DAC
       //
       DAC_setShadowValue(myDACA_BASE, dacVal);
       #endif

       //
       // Trip flag is set when CTRIP signal is asserted
       //
       if((EPWM_getTripZoneFlagStatus(myEPWM1_BASE) & EPWM_TZ_FLAG_OST) != 0U)
        {
            //
            // Wait for comparator CTRIP to de-assert
            //
            while((CMPSS_getStatus(myCMPSS_BASE) & CMPSS_STS_HI_FILTOUT) != 0U)
            {

               #ifdef USE_DAC
               //Update DAC value in case updated during this time
               DAC_setShadowValue(myDACA_BASE, dacVal);
               #endif
            }

            //
            // Clear trip flags
            //
            EPWM_clearTripZoneFlag(myEPWM1_BASE, EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1);
        }
    }
}

__interrupt void INT_myADCA_1_ISR(void)
{
    interruptCount = interruptCount+1;
    //
    // Get the latest ADC conversion result
    //
    adcAResult= ADC_readResult(myADCA_RESULT_BASE, myADCA_SOC0);

    //
    // Update the CMPA value according to the ADC result
    //
    duty = adcAResult*ADC_SCALE;
    CMPA_result = EPWM_PERIOD*(1-duty);

    //
    // Update the CMPA value with the new calculated value
    //
    EPWM_setCounterCompareValue(myEPWM1_BASE, EPWM_COUNTER_COMPARE_A, CMPA_result);

    //
    // Calculate the LED blink rate
    //
    ledBlinkRate = (100-(duty*100))*100;

    //
    // Toggle the LED based upon the duty cycle chosen
    //
    if(LED_count++ > ledBlinkRate)               // Toggle LED to change state
    {
        GPIO_togglePin(myBoardLED0_GPIO);    // Toggle the pin
        LED_count = 0;                           // Reset the counter
    }

    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(myADCA_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if(true == ADC_getInterruptOverflowStatus(myADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(myADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(myADCA_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//
// End of File
//
