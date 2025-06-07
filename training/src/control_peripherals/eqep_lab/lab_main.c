//#############################################################################
//
// FILE:   lab_main.c
//
// TITLE:  Lab - Enhanced Quadrature Encoder Pulse (eQEP)
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_control_peripherals_c2000_lab_qep&packageId=C2000-ACADEMY
//
//! \addtogroup academy_lab_list
//! <h1> eQEP Academy Lab - Sysconfig </h1>
//!
//! The objective of this lab is to gain familiarity with the Enhanced Quadrature
//! Encoder Pulse (eQEP) module. This lab will illustrate a common use-case of 
//! the eQEP module: connecting quadrature signals to the C2000 device and using
//! the eQEP to calculate motor characteristics.
//!
//! \b External \b Connections \n
//!  - Connect the PWM1_A GPIO and EQEP1_A GPIO
//!  - Connect the PWM1_B GPIO and EQEP1_B GPIO
//!  - Connect the myGPIOIndex GPIO and EQEP1_I GPIO
//!  - Refer to Academy Lab instruction for exact pin for your device/board
//!
//! \b Watch \b Variables \n
//!  - currentEncoderPosition
//!  - frequency
//!  - speed
//!  - direction
//!
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
// Defines
//

//
// LVSERVOMTR is a 1000-line encoder
//
#define ENCODER_SLOTS   1000U      

//
// Unit period in microseconds
//
#define UNIT_PERIOD     10000U 

//
// Globals
//

//
// Stores the previous position counter value
//
uint32_t oldCount = 0; 

//
// Stores the new position counter value for frequency calculation
//
uint32_t newCount = 0;                  

//
// Stores the current encoder position
//
uint32_t currentEncoderPosition = 0;   

//
// Measured quadrature signal frequency of motor using eQEP
//
int32_t frequency = 0;    

//
// Measured speed of motor in rpm
//
float32_t speed = 0.0f;  

//
// Direction of rotation of motor
//
int32_t direction = 0;                  

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
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Loop indefinitely
    //
    while(1)
    {

        //
        // myGPIOIndex pulses high for 200 microseconds every 1000 encoder cycles (400,000 us)
        //
        DEVICE_DELAY_US(400000L);
        GPIO_writePin(myBoardLED0_GPIO, 1);
        DEVICE_DELAY_US(200L);
        GPIO_writePin(myBoardLED0_GPIO, 0);
    }
}

//
// EQEP1 ISR- interrupts upon unit time-out event
//
__interrupt void
INT_myEQEP1_ISR(void)
{
    //
    // Save current encoder position value for monitoring
    //
    currentEncoderPosition = EQEP_getPosition(myEQEP1_BASE);

    //
    // Get position counter value latched on unit time-out event
    //
    newCount = EQEP_getPositionLatch(myEQEP1_BASE);

    //
    // Gets rotation direction of motor
    //
    direction = EQEP_getDirection(myEQEP1_BASE);

    //
    // Calculates the number of position in unit time based on direction
    //
    if (direction > 0 ){
        if (newCount >= oldCount)
            newCount = newCount - oldCount;
        else
            newCount = ((4 * ENCODER_SLOTS - 1) - oldCount) + newCount;
        }
    else {
        if (newCount <= oldCount)
            newCount = oldCount - newCount;
        else
            newCount = ((4 * ENCODER_SLOTS - 1) - newCount) + oldCount;
        }

    //
    // Stores the current position count value to oldCount variable
    //
    oldCount = currentEncoderPosition;

    //
    // Calculate frequency and speed values
    // frequency found by counting external input pulses for UNIT PERIOD
    // speed derived from encoder frequency and encoder resolution
    //
    frequency = (newCount * (uint32_t)1000000U) / ((uint32_t)UNIT_PERIOD);
    speed = (frequency * 60) / ((float)(4 * ENCODER_SLOTS));

    //
    // Clear interrupt flag and issue ACK
    //
    EQEP_clearInterruptStatus(myEQEP1_BASE,EQEP_INT_UNIT_TIME_OUT|EQEP_INT_GLOBAL);
    Interrupt_clearACKGroup(INT_myEQEP1_INTERRUPT_ACK_GROUP);
}

//
// End of File
//
