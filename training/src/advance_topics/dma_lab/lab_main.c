//#############################################################################
//
// FILE:   lab_main.c
//
// TITLE:  DMA lab using SysConfig
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_advanced_topics_c2000_lab_dma&packageId=C2000-ACADEMY 
//
// This example is an lab project for DMA setup using sysconfig
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

#define ADC_BUF_LEN 50              // Buffer length.

#pragma DATA_SECTION(AdcBufRaw, "ramgs0");

uint16_t AdcBufRaw[2*ADC_BUF_LEN];  // The ping-pong buffer.
uint16_t AdcBuf[ADC_BUF_LEN];       // Buffer for CCS plotting.
uint16_t PingPongState = 0;         // ping-pong buffer state.
uint16_t LedCtr = 0;                // Counter to slow LED toggling.
uint16_t TaskDelayUs = 0;           // Delay to simulate data processing task.
uint16_t OverCnt = 0;               // Counter to store DMA overwrites.
uint32_t TimDiff;                   // To measure the DMA ISR time.

const void* AdcAddr = (void*)(ADCARESULT_BASE + ADC_O_RESULT0);
const void* AdcRawBufAddr = (void*)AdcBufRaw;

interrupt void dma_Ch1ISR(void)
{
    // Clear the interrupt flags.
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);

    // Start and reload the timer.
    CPUTimer_startTimer(myCPUTIMER0_BASE);

    uint16_t *AdcBufPtr = AdcBuf;
    uint16_t *AdcBufRawPtr;
    uint16_t i;

    // Blink LED at about 1Hz. ISR is occurring every 1ms.
    if (LedCtr++ >= 1000) {
        GPIO_togglePin(myBoardLED0_GPIO);
        LedCtr = 0;
    }

    if (PingPongState == 0) {
        // Set DMA address to start at ping buffer.
        DMA_configAddresses(DMA_CH1_BASE,
                            (const void *)AdcBufRaw,
                            (const void *)(ADCARESULT_BASE + ADC_O_RESULT0));

        // Fill AdcBuf with contents of the pong buffer.
        AdcBufRawPtr = AdcBufRaw + ADC_BUF_LEN;
        for (i = 0; i < ADC_BUF_LEN; i++) {
            *(AdcBufPtr++) = *(AdcBufRawPtr++);
        }
    } else {
        // Set DMA address to start at pong buffer.
        DMA_configAddresses(DMA_CH1_BASE,
                            (const void *)(AdcBufRaw + ADC_BUF_LEN),
                            (const void *)(ADCARESULT_BASE + ADC_O_RESULT0));

        // Fill AdcBuf with contents on the ping buffer.
        AdcBufRawPtr = AdcBufRaw;
        for (i = 0; i < ADC_BUF_LEN; i++) {
            *(AdcBufPtr++) = *(AdcBufRawPtr++);
        }
    }

    // Toggle PingPongState.
    PingPongState ^= 1;

    // Delay to simulate more data processing.
    for (i = 0; i < TaskDelayUs; i++) {
        DEVICE_DELAY_US(1);
    }

    // Get the time stamp and check to see if interrupt completed within the
    // required time frame of 1ms. Don't worry about overflows.
    CPUTimer_stopTimer(myCPUTIMER0_BASE);
    TimDiff = 0xFFFFFFFF - CPUTimer_getTimerCount(myCPUTIMER0_BASE);
    if (TimDiff >= ((uint32_t)(0.001*DEVICE_SYSCLK_FREQ))) {
        OverCnt++;
    }
}

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
        // Do nothing.
        NOP;
    }
}

//
// End of File
//
