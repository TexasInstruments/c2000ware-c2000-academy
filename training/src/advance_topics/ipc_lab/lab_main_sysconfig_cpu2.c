//############################################################################
//
// FILE: lab_main_sysconfig_cpu2.c
//
// TITLE: Lab - SysConfig Inter Processor Communications
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_advanced_topics_c2000_lab_ipc&packageId=C2000-ACADEMY
//
//! \addtogroup academy_lab_list
//! <h1> Lab Solution on Inter Processor Communication (CPU2)</h1>
//!
//! The objective of this lab exercise is to demonstrate and become familiar
//! with the operation of the IPC module. We will be using the basic IPC
//! features to send data in both directions between CPU1 and CPU2. As in
//! the previous lab exercise, PWM2 will be configured to provide a 50 kHz
//! SOC signal to ADC-A. An End-of-Conversion ISR on CPU1 will read each
//! result and write it into a data register in the IPC. An IPC interrupt
//! will then be triggered on CPU2 which fetches this data and stores it in
//! a circular buffer. The same ISR grabs a data point from a sine table and
//! loads it into a different IPC register for transmission to CPU1. This
//! triggers an interrupt on CPU1 to fetch the sine data and write it into
//! DAC-B. The DAC-B output is connected by a jumper wire to the ADCINA0 pin.
//! If the program runs as expected the sine table and ADC results buffer on
//! CPU2 should contain very similar data.
//!
//! \b External \b Connections \n
//!  - Jumper cable between ADCINA0 pin to the DACB pin
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

#include "driverlib.h"
#include "device.h"
#include "board.h"

#define DAC_OUTPUT_BITS 11                  // Bits of output for the DAC.
#define BUF_BITS 8                          // Buffer bits.
#define BUF_LEN (1 << BUF_BITS)             // Buffer length.
#define BUF_MASK ((uint16_t)(BUF_LEN - 1))  // Buffer index mask.
#define LUT_BITS 6                          // Sin table bits.
#define LUT_LEN (1 << LUT_BITS)             // Sin table length.
#define LUT_MASK ((uint16_t)(LUT_LEN - 1))  // Sin table index mask.

uint16_t AdcBuf[BUF_LEN];                   // ADC buffer to store samples.
uint16_t AdcBufIdx = 0;                     // ADC buffer index.
uint16_t SinPhase = 0;                      // Sin table index, i.e., phase.
uint16_t LedCtr2 = 0;                       // Counter to slow down the toggling of the LED.

// Fixed Point Sin Table
// 16-bits of amplitude resolution
// 6-bits of phase resolution
// phi = (0:2^LUT_BITS-1)*2pi
// SinTable = (sin(phi) + 1)*(2^16 - 1)/2
uint16_t SinTable[LUT_LEN] = {
    0x8000,0x8c8b,0x98f8,0xa527,0xb0fb,0xbc56,0xc71c,0xd133,
    0xda82,0xe2f1,0xea6d,0xf0e2,0xf641,0xfa7c,0xfd89,0xff61,
    0xffff,0xff61,0xfd89,0xfa7c,0xf641,0xf0e2,0xea6d,0xe2f1,
    0xda82,0xd133,0xc71c,0xbc56,0xb0fb,0xa527,0x98f8,0x8c8b,
    0x8000,0x7374,0x6707,0x5ad8,0x4f04,0x43a9,0x38e3,0x2ecc,
    0x257d,0x1d0e,0x1592,0x0f1d,0x09be,0x0583,0x0276,0x009e,
    0x0000,0x009e,0x0276,0x0583,0x09be,0x0f1d,0x1592,0x1d0e,
    0x257d,0x2ecc,0x38e3,0x43a9,0x4f04,0x5ad8,0x6707,0x7374,
};

interrupt void ipc0_ISR(void)
{
    uint32_t cmd, addr, data;

    // Clear interrupt flags.
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    // Read the data from the IPC registers.
    IPC_readCommand(IPC_CPU2_L_CPU1_R, IPC_FLAG0, false, &cmd, &addr, &data);

    // Acknowledge IPC1 flag from remote.
    IPC_ackFlagRtoL(IPC_CPU2_L_CPU1_R, IPC_FLAG0);

    // Get the ADC sample from CPU1 and store it in a circular buffer.
    AdcBuf[AdcBufIdx++ & BUF_MASK] = (uint16_t)data;

    // Send next DAC sample from the sinusoidal table to CPU1.
    IPC_sendCommand(IPC_CPU2_L_CPU1_R, IPC_FLAG1, false, 0, 0,
                    (uint32_t)(SinTable[SinPhase++ & LUT_MASK] >> (16 - DAC_OUTPUT_BITS)));

    // Toggle LED2 at a rate of 1Hz.
    if (LedCtr2++ >= 50000) {
        GPIO_togglePin(DEVICE_GPIO_PIN_LED2);
        LedCtr2 = 0;
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

    //
    // Initialize settings from SysConfig
    //
    Board_init();

    // Clear any IPC flags if set already
    IPC_clearFlagLtoR(IPC_CPU2_L_CPU1_R, IPC_FLAG_ALL);

    // Enable global interrupts.
    EINT;
    // Enable real-time debug.
    ERTM;

    // Synchronize both the cores.
    IPC_sync(IPC_CPU2_L_CPU1_R, IPC_SYNC);

    for (;;) {
        // Do nothing.
        NOP;
    }
}
