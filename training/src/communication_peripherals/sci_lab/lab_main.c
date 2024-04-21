//############################################################################
//
// FILE: lab_main.c
//
// TITLE: Lab - SCI communication
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_communications_lab_c2000_lab_sci&packageId=C2000-ACADEMY
//
//! \addtogroup academy_lab_list
//! <h1> Lab solution on Using Communication Peripherals </h1>
//!
//! The objective of this lab is to become familiar with the on-board SCI
//! (Serial Communication Interface) by sending and receiving data between a
//! C2000 device and a computer. We will use the computer to change the
//! frequency of the blinking LED and then the board will echo this value back
//! to the computer. This will allow us to demonstrate both means of
//! communication. Additionally, Code Composer Studio's terminal feature will
//! be explored and will be used to interact with the device.
//!
//! \b External \b Connections \n
//!  - None.
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

//
// Globals
//
uint16_t cpuTimer0IntCount; //number of times TIMER 0 ISR is triggered
uint16_t delayCount;        //number (0-9) to scale the LED frequency

//
// Function Prototypes
//
__interrupt void INT_myCPUTIMER0_ISR(void);

//
// Main
//
void main(void)
{
    //
    // CPU Initialization
    //
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    //
    // Configure GPIO pins
    //
    Device_initGPIO();

    //
    // Initialize the SCI and Timer Modules
    //
    Board_init();

    //
    // Enable global interrupts and real-time debug
    //
    EINT;
    ERTM;

    //
    // Start CPU Timer 0
    //
    CPUTimer_startTimer(myCPUTIMER0_BASE);

    //
    // Define local variables
    //
    char* msg;                // Message sent through terminal window
    char receivedChar;        // Variable used to track input from the terminal window
    uint16_t rxStatus = 0U;   // Variable used to store the status of the SCI RX Register

    //
    // Send starting message.
    //
    msg = "\r\n\n\nHello World! Enter a number 0-9 to change the LED blink rate.\0";
    SCI_writeCharArray(mySCIA_BASE, (uint16_t*)msg, 65);

    for(;;)
        {
            msg = "\r\nEnter a number 0-9: \0";
            SCI_writeCharArray(mySCIA_BASE, (uint16_t*)msg, 24);

            //
            // Read a character from the FIFO.
            //
            receivedChar = SCI_readCharBlockingFIFO(mySCIA_BASE);


            //Turns character to digit
            delayCount = receivedChar - '0';

            rxStatus = SCI_getRxStatus(mySCIA_BASE);
            if((rxStatus & SCI_RXSTATUS_ERROR) != 0)
            {
                //
                //If Execution stops here there is some error
                //Analyze SCI_getRxStatus() API return value
                //
                ESTOP0;
            }

            //
            // Echo back the character.
            //
            msg = "\r\nLED set to blink rate \0";
            SCI_writeCharArray(mySCIA_BASE, (uint16_t*)msg, 25);
            SCI_writeCharBlockingNonFIFO(mySCIA_BASE, receivedChar);
        }
}

//
// ISR for CPUTIMER0 to change LED blink rate based on input to delayCount
//
__interrupt void INT_myCPUTIMER0_ISR(void)
{
    cpuTimer0IntCount++;
    if (cpuTimer0IntCount >= delayCount){
        cpuTimer0IntCount = 0;
        GPIO_togglePin(myBoardLED0_GPIO);
    }

    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}

//
// End of File
//
