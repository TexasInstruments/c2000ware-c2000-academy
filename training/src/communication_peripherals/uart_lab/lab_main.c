//#############################################################################
//
// FILE:   uart_academy_lab.c
//
// TITLE:  UART Academy Lab
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_analog_subsystem_c2000_lab_adc&packageId=C2000-ACADEMY
//
//! \addtogroup academy_lab_list
//! <h1> UART Academy Lab - Sysconfig </h1>
//!
//! The objective of this lab is to become familiar with the on-board UART
//! (Universal Asychronous Receiver Transmitter) by sending and receiving data
//! between a C2000 device and a computer. We will use the computer to change
//! the frequency of the blinking LED and then the board will echo this value
//! back to the computer. This will allow us to demonstrate both directions of
//! communication. Additionally, Code Composer Studio's terminal feature will
//! be explored and used to interact with the device.
//! The LED will blink at a rate of scaleFactor[Hz], with the scaleFactor being
//! the integer 1-9 input into the UART terminal. This means the LED can blink
//! at a frequency of 1Hz (default) - 9Hz in this example. As you increase the
//! frequency, you should see the LED blink rate increase.
//!
//! \b External \b Connections \n
//!  - None
//!
//! \b Watch \b Variables \n
//!  - \b cpuTimer0IntCount - Counter incremented every 1 second
//!  - \b scaleFactor - Current scaleFactor used to blink the LED 
//!  - \b delayCount - Number of CPU timer ISRs to delay LED toggling
//!
//! Note: Avoid keeping the memory browser open while the execution
//! is in progress.
//
//#############################################################################
//
//
// 
// C2000Ware v5.05.00.00
//
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com
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
// Globals
//

//
// Number of times the TIMER 0 ISR is triggered.
//
uint16_t cpuTimer0IntCount = 0;

//
// Number (1-9) to scale the LED frequency.
//
uint16_t scaleFactor = 1;

//
// Number of CPU timer interrupts to delay toggling.
// For example: 5Hz toggling - toggles every 0.2 seconds - toggle every 2 ISRs.
//
uint16_t delayCount = 9;        

//
// Main
//
int main(void){

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
    // Enable global interrupts and real-time debug
    //
    EINT;
    ERTM;

    //
    // Start CPU Timer 0.
    //
    CPUTimer_startTimer(myCPUTIMER0_BASE);

    //    
    // Define local variables.
    //

    //
    // Message sent through terminal window.
    //
    char* msg;          

    //
    // Variable used to track input from the terminal window.
    //      
    char receivedChar;     

    //
    // Variable used to store the status of the UART RX status register.
    //   
    uint16_t rxStatus = 0U;   

    //
    // Send starting message.
    //
    msg = "\r\n\n\nHello World! Enter a number 1-9 to change the LED blink rate.\0";
    int i;
    for(i = 0; i < 66; i++){
        UART_writeChar(myUART0_BASE, (char)msg[i]);
    }

    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {
        msg = "\r\nEnter a number 1-9: \0";
        for(i = 0; i < 23; i++){
            UART_writeChar(myUART0_BASE, (char)msg[i]);
        }

        //
        // Read a character from the FIFO.
        //
        receivedChar = UART_readChar(myUART0_BASE);

        //
        // Convert the received character into an integer.
        //
        scaleFactor = receivedChar - '0';

        //
        // Amount of timer 0 ISR's to skip before toggling the LED.
        //
        delayCount = (1.0 / (float)scaleFactor) / 0.1;

        rxStatus = UART_getRxError(myUART0_BASE);
        if((rxStatus & UART_RSR_ALL_M) != 0)
        {
            //
            // If Execution stops here there an error.
            // Analyze the rxStatus value.
            //
            ESTOP0;
        }

        //
        // Echo back the received character.
        //
        msg = "\r\nLED set to blink rate \0";
        for(i = 0; i < 25; i++){
            UART_writeChar(myUART0_BASE, (char)msg[i]);
        }
        UART_writeChar(myUART0_BASE, receivedChar);
        msg = " Hz";
        for(i = 0; i < 3; i++){
            UART_writeChar(myUART0_BASE, (char)msg[i]);
        }
    }
}

//
// ISR for CPUTIMER0 to blink the LED based on delayCount.
// Interrupts every 0.1 seconds.
//
void INT_myCPUTIMER0_ISR(void)
{
    cpuTimer0IntCount++;
    if (cpuTimer0IntCount >= delayCount){
        cpuTimer0IntCount = 0;
        GPIO_togglePin(myBoardLED0_GPIO);
    }
    //
    // Acknowledge this interrupt to receive more interrupts from the interrupt group
    //
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}

//
// End of File
//
