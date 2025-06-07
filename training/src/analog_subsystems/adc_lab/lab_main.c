//#############################################################################
//
// FILE: lab_main.c
//
// TITLE: adc lab
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_analog_subsystem_c2000_lab_adc&packageId=C2000-ACADEMY
//
//! \addtogroup academy_lab_list
//! <h1> Analog Subsystems (ADC) Academy Lab - Sysconfig </h1>
//!
//! The objective of this lab exercise is to become familiar with the
//! programming and operation of the on-chip analog-to-digital converter (ADC).
//! The microcontroller (MCU) will be setup to sample a single ADC input
//! channel at a prescribed sampling rate and store the conversion result in a
//! circular memory buffer. In the second part of this lab exercise, the
//! digital-to-analog converter (DAC) will be explored. 
//!
//! \b External \b Connections \n
//!  - Refer to Academy Lab instruction for exact pin for your device/board
//!
//! \b Watch \b Variables \n
//!  - AdcBuf
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
#include "board.h"

//
// Global variables and definitions
//
#define ADC_BUF_LEN         50

//
// Variable used to enable/disable real-time mode
//
uint16_t DEBUG_TOGGLE = 1;

//
// ADC buffer allocation
//
uint16_t AdcBuf[ADC_BUF_LEN];  

//
// Includes/excludes buffered DAC code depending on if a DAC module is
// present on the device. 
//
#ifdef DACB_BASE
uint16_t DacOutput;
uint16_t DacOffset;
uint16_t SINE_ENABLE = 0;

//
// Quadrature look-up table: contains 4 quadrants of sinusoid data points
//
#define SINE_PTS 25
int QuadratureTable[SINE_PTS] = {
        0x0000,         // [0]  0.0
        0x1FD4,         // [1]  14.4
        0x3DA9,         // [2]  28.8
        0x579E,         // [3]  43.2
        0x6C12,         // [4]  57.6
        0x79BB,         // [5]  72.0
        0x7FBE,         // [6]  86.4
        0x7DBA,         // [7]  100.8
        0x73D0,         // [8]  115.2
        0x629F,         // [9]  129.6
        0x4B3B,         // [10] 144.0
        0x2F1E,         // [11] 158.4
        0x100A,         // [12] 172.8
        0xEFF6,         // [13] 187.2
        0xD0E2,         // [14] 201.6
        0xB4C5,         // [15] 216.0
        0x9D61,         // [16] 230.4
        0x8C30,         // [17] 244.8
        0x8246,         // [18] 259.2
        0x8042,         // [19] 273.6
        0x8645,         // [20] 288.0
        0x93EE,         // [21] 302.4
        0xA862,         // [22] 316.8
        0xC257,         // [23] 331.2
        0xE02C          // [24] 345.6
        };

#endif

//
// Function Declarations
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

    // Enable global interrupts and real-time debug
    //
    EINT;
    ERTM;

    //
    // Main Loop
    //
    while(1){}

}

interrupt void INT_myADCA_1_ISR(void)
{
    static uint16_t *AdcBufPtr = AdcBuf;
    static volatile uint16_t LED_count = 0;

    //
    // Read the ADC Result
    //
    *AdcBufPtr++ = ADC_readResult(myADCA_RESULT_BASE, myADCA_SOC0);

    //
    // Brute Force the circular buffer
    //
    if (AdcBufPtr == (AdcBuf + ADC_BUF_LEN))
    {
        AdcBufPtr = AdcBuf;
    }

    //
    // Toggle the pin : real-time mode
    //
    if(DEBUG_TOGGLE == 1)
    {
        GPIO_togglePin(myGPIOToggle);
    }

    //
    // Toggle slowly to see the LED blink
    //
    if(LED_count++ > 25000)                      
    {
        //
        // Toggle the pin
        //
        GPIO_togglePin(myBoardLED0_GPIO);       

        //
        // Reset the counter
        //            
        LED_count = 0;                           
    }

#ifdef DACB_BASE
    //
    // Write to DAC-B to create input to ADC-A0
    //

    //
    // Define Quadrature table index
    //
    static uint16_t iQuadratureTable = 0;        

    if(SINE_ENABLE == 1)
    {
        DacOutput = DacOffset + ((QuadratureTable[iQuadratureTable++] ^ 0x8000) >> 5);
    }
    else
    {
        DacOutput = DacOffset;
    }
    //
    // Wrap the index
    //
    if(iQuadratureTable > (SINE_PTS - 1))        
    {
        iQuadratureTable = 0;
    }
    DAC_setShadowValue(myDACB_BASE, DacOutput);
#endif

    Interrupt_clearACKGroup(INT_myADCA_1_INTERRUPT_ACK_GROUP);
    ADC_clearInterruptStatus(myADCA_BASE, ADC_INT_NUMBER1);
} // End of ADC ISR

//
// End of File
//
