//############################################################################
//
// FILE: lab_main_sysconfig_cpu1.c
//
// TITLE: Lab - SysConfig Inter Processor Communications
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/explore/node?node=AOpze8ebskysmgASY3VKSA__jEBbtmC__LATEST
//
//! \addtogroup academy_lab_list
//! <h1> Lab Solution on Inter Processor Communication (CPU1) </h1>
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

uint16_t LedCtr1 = 0;           // Counter to slow down LED toggling.

interrupt void ipc1_ISR(void)
{
    uint32_t cmd, addr, data;

    // Clear the interrupt flags.
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    // Get the next DAC sample from CPU2.
    IPC_readCommand(IPC_CPU1_L_CPU2_R, IPC_FLAG1, false, &cmd, &addr, &data);

    // Acknowledge IPC1 flag from remote.
    IPC_ackFlagRtoL(IPC_CPU1_L_CPU2_R, IPC_FLAG1);

    // Load the new sample on the DAC.
    DAC_setShadowValue(DACB_BASE, (uint16_t)data);

    // Toggle LED1 at a rate of 1Hz.
    if (LedCtr1++ >= 50000) {
        GPIO_togglePin(CPU1_LED);
        LedCtr1 = 0;
    }
}

interrupt void adcA1ISR(void)
{
    uint32_t adcResult;
    // Clear interrupt flags.
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    // Read the sample from the ADC.
    adcResult = (uint32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);

    // Send it over to CPU2.
    IPC_sendCommand(IPC_CPU1_L_CPU2_R, IPC_FLAG0, false, 0, 0,
                    adcResult);
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

    // Short delay to let the DAC and other peripherals start up.
    DEVICE_DELAY_US(1000);

    //
    // Clear any IPC flags if set already
    //
    IPC_clearFlagLtoR(IPC_CPU1_L_CPU2_R, IPC_FLAG_ALL);

    //
    // Synchronize both the cores.
    //
    IPC_sync(IPC_CPU1_L_CPU2_R, IPC_SYNC);

    // Enable global interrupts.
    EINT;
    // Enable real-time debug.
    ERTM;

    for (;;) {
        // Do nothing.
        NOP;
    }
}
