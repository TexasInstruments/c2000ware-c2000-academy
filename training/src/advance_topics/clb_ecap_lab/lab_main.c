//#############################################################################
//
// FILE:   lab_main.c
//
// TITLE:  CLB eCAP Example
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_advanced_topics_c2000_lab_clb_ecap&packageId=C2000-ACADEMY
//
// This example is a CLB eCAP setup for SysConfig development. The myEPWM0 is
// used to apply a signal in place of an external signal from a GPIO pin. The
// range of tested frequencies is 50 mHz to 1 Mhz using the ePWM. The range of
// tested duties is 0.0016% to 99.99%. To change the frequency and duty of the
// signal, adjust the time base clock divider, high speed clock divider, time
// base period, and the counter compare A value. The CLB input can be either
// the ePWM or a GPIO pin (routed in using CLBXBAR).
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
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
// Macros and Global Variables
//
volatile float32_t frequency = 0.0;
volatile float32_t period = 0.0;
volatile float32_t duty = 0.0;
volatile float32_t timesHLH[3];
volatile uint32_t regPush[4];
volatile uint32_t firstHigh  = 0;
volatile uint32_t firstLow   = 0;
volatile uint32_t secondHigh = 0;
volatile uint16_t ledCount = 0;
volatile uint16_t  gpioStatus = 0;

//
// Functions
//
__interrupt void INT_myCLB0_ISR(void);

//
// Main
//
void main(void)
{
	//
    // CPU Initialization
    //
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();

    //
    // Enable global interrupts and real-time debug
    //
    EINT;
    ERTM;

    //
    // Local variables
    //
    uint32_t clockFreq = DEVICE_SYSCLK_FREQ;
    uint32_t firstHighCLB = 0;
    uint32_t firstLowCLB = 0;
    uint32_t secondHighCLB = 0;

	while(1)
	{
	    //
	    // Frequency, period, duty, and other timing calculations
	    // NOTE: 1 is added to each count because of zero-indexing counter
	    //
	    firstHighCLB = regPush[0] + 1;
        firstLowCLB = regPush[1] + 1;
        secondHighCLB = regPush[2] + 1;
	    frequency = (float32_t)(clockFreq) / ((float32_t)(firstHighCLB + firstLowCLB));
	    period = 1 / frequency;
        timesHLH[0] = (float32_t)(firstHighCLB) / (float32_t)(clockFreq);
        timesHLH[1] = (float32_t)(firstLowCLB) / (float32_t)(clockFreq);
        timesHLH[2] = (float32_t)(secondHighCLB) / (float32_t)(clockFreq);
        duty = timesHLH[0] * frequency * 100;
	}
}

__interrupt void INT_myCLB0_ISR(void)
{
    CLB_readFIFOs(myCLB0_BASE, (uint32_t *)regPush);

    //
    // Toggle the GPIO pins; increase from 1000 to slow down the toggle
    //
    ledCount = (ledCount + 1) % (1000);

    if(ledCount == 0)
    {
        if(gpioStatus)
        {
           gpioStatus = 0;
        }
        else
        {
           gpioStatus = 1;
        }
        GPIO_writePin(myBoardLED0_GPIO, gpioStatus);
    }

    CLB_clearInterruptTag(myCLB0_BASE);
    Interrupt_clearACKGroup(INT_myCLB0_INTERRUPT_ACK_GROUP);
}

//
// End of File
//
