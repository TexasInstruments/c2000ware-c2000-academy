//############################################################################
//
// FILE: lab_main.c
//
// TITLE: Lab - CLA lab
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_advanced_topics_c2000_lab_cla&packageId=C2000-ACADEMY
//
//! \addtogroup academy_lab_list
//! <h1> Lab solution on CLA </h1>
//!
//! In this lab, we will use the Control Law Accelerator (CLA) to implement a
//! FIR lowpass filter. We will apply a PWM waveform to the input of our
//! lowpass filter, which will result in a sinusoidal waveform at the output.
//! The PWM waveform will be generated using one of the ePWM modules and it
//! will be sampled via the ADC at a given sample rate. Both the filtered and
//! unfiltered waveforms will be viewed in a real-time Code Composer Studio
//! (CCS) debug session via internal buffers. [[r! F28002x and F280013x Devices
//! This lab cannot be performed on the F28002x and F280013x devices since they
//! do not have a CLA. ]]
//!
//! \b External \b Connections \n
//!  - Refer to Academy Lab instruction for exact pin for your device/board
//!    https://dev.ti.com/tirex/explore/content/c2000Academy/modules/Module_9_CLA_Deep_Dive/module_9_cla_lab.html#build-and-run-interactive-debug-session
//!
//! \b Watch \b Variables \n
//!  - None.
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

#define BUF_BITS    7                           // Buffer bits <= 16.
#define BUF_LEN     (1 << BUF_BITS)             // Buffer length.
#define BUF_MASK    ((uint16_t)(BUF_LEN - 1))   // Buffer mask.

float       ClaBuf[BUF_LEN];                    // Buffer to store filtered samples.
float       AdcBuf[BUF_LEN];                    // Buffer for un-filtered samples.
uint16_t    ClaBufIdx   = 0;                    // Buffer index for ClaBuf.
uint16_t    AdcBufIdx   = 0;                    // Buffer index for AdcBufIdx.
uint16_t    LedCtr      = 0;

#pragma DATA_SECTION(filter_out,"Cla1ToCpuMsgRAM");
float filter_out;

#pragma DATA_SECTION(filter_in,"Cla1ToCpuMsgRAM");
float filter_in;


__interrupt void cla1Isr1(void)
{
    // Clear interrupt flags.
    ADC_clearInterruptStatus(myADC0_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INT_myCLA01_INTERRUPT_ACK_GROUP);

    // Store raw ADC sample in AdcBuf.
    AdcBuf[AdcBufIdx++] = filter_in;
    AdcBufIdx &= BUF_MASK;

    // Store filtered output in ClaBuf.
    ClaBuf[ClaBufIdx++] = filter_out;
    ClaBufIdx &= BUF_MASK;

    // Toggle LED1 at a rate of 1Hz.
    if (LedCtr++ >= 8000) {
        GPIO_togglePin(myBoardLED0_GPIO);
        LedCtr = 0;
    }
}



//
// Main
//
void main(void)
{
    // Configure system clock and PLL, enable peripherals, and configure
    // flash if used.
    Device_init();

    // Initialize the PIE module and vector table.
    Interrupt_initModule();
    Interrupt_initVectorTable();

	Board_init();

    // Enable global interrupts.
    EINT;

    // Enable real-time debug.
    ERTM;

    for(;;) {
        NOP;  // Do nothing.
    }
}

//
// End of File
//
