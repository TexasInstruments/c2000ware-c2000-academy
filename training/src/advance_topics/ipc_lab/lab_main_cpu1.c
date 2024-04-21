//############################################################################
//
// FILE: lab_main_cpu1.c
//
// TITLE: Lab - Inter Processor Communications
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/local?id=source_c2000_academy_labs_advanced_topics_c2000_lab_ipc&packageId=C2000-ACADEMY
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
#include "ipc.h"

#define ADC_SAMPLE_PERIOD 1999  // For 50kHz sampling rate.

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
        GPIO_togglePin(DEVICE_GPIO_PIN_LED1);
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

inline void InitDac(void)
{
    // Set the reference voltage.
    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_ADC_VREFHI);
    // Set the load mode clock to the system clock.
    DAC_setLoadMode(DACB_BASE, DAC_LOAD_SYSCLK);
    // Load zero into the DAC output.
    DAC_setShadowValue(DACB_BASE, 0);
    // Enable DAC output.
    DAC_enableOutput(DACB_BASE);
    // Short delay to let the DAC start up.
    DEVICE_DELAY_US(10);
}

inline void InitEPwm(void)
{
    // Disable clock on all ePWM peripherals.
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    // Reset ePWM2.
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM2);
    // Freeze the internal counter.
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
    // Set the clock prescalers.
    EPWM_setClockPrescaler(EPWM2_BASE, EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    // Set the period count. Use full period for up count mode.
    EPWM_setTimeBasePeriod(EPWM2_BASE, ADC_SAMPLE_PERIOD);
    // Use shadow register to load period count.
    EPWM_setPeriodLoadMode(EPWM2_BASE, EPWM_PERIOD_SHADOW_LOAD);
    // Set the phase shift.
    EPWM_setPhaseShift(EPWM2_BASE, 0);
    // Enable ADC SOCA trigger.
    EPWM_enableADCTrigger(EPWM2_BASE, EPWM_SOC_A);
    // Set the ADC trigger source.
    EPWM_setADCTriggerSource(EPWM2_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);
    // Generate SOCA on first event.
    EPWM_setADCTriggerEventPrescale(EPWM2_BASE, EPWM_SOC_A, 1);
    // Enable up count mode.
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP);
    // Enable clock on all ePWM peripherals.
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

inline void InitAdc(void)
{
    // Reset the ADC.
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_ADCA);
    // Power down the ADC for configuration.
    ADC_disableConverter(ADCA_BASE);
    // Set the ADC clock prescaler.
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    // Generate an interrupt at the end of conversion.
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    // Setup SOC0 to trigger from ePWM2-ADCSOC, input is on ADCINA0.
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM2_SOCA,
                 ADC_CH_ADCIN0, 8);
    // Disable ADC interrupt triggers since ePWM2 is generating them.
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER0,
                               ADC_INT_SOC_TRIGGER_NONE);
    // Set SOC priority mode.
    ADC_setSOCPriority(ADCA_BASE, ADC_PRI_ALL_ROUND_ROBIN);
    // ADC interrupt pulses regardless of flag state.
    ADC_enableContinuousMode(ADCA_BASE, ADC_INT_NUMBER1);
    // Set ADC interrupt source.
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    // Enable the interrupt in the ADC.
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    // Register the ADCA interrupt in the PIE.
    Interrupt_register(INT_ADCA1, &adcA1ISR);
    // Enable the ADCA interrupt in the PIE.
    Interrupt_enable(INT_ADCA1);
#ifdef USE_ADC_REFERENCE_INTERNAL
    // Set the reference voltage.
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
#endif
    // Power up the ADC.
    ADC_enableConverter(ADCA_BASE);
    // Wait 1 ms after power-up before using ADC.
    DEVICE_DELAY_US(1000);
}

inline void InitGpio(void)
{
    // Unlock GPIO pins.
    Device_initGPIO();

    // Configure GPIO to blink LED.
    GPIO_setPinConfig(DEVICE_GPIO_CFG_LED1);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);
    GPIO_writePin(DEVICE_GPIO_PIN_LED1, 1);
    GPIO_setMasterCore(DEVICE_GPIO_PIN_LED1, GPIO_CORE_CPU1);

    // Configure GPIO to blink LED.
    GPIO_setPinConfig(DEVICE_GPIO_CFG_LED2);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED2, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED2, GPIO_DIR_MODE_OUT);
    GPIO_writePin(DEVICE_GPIO_PIN_LED2, 0);
    GPIO_setMasterCore(DEVICE_GPIO_PIN_LED2, GPIO_CORE_CPU2);
}

void main(void)
{
    // Configure system clock and PLL, enable peripherals, and configure
    // flash if used.
    Device_init();

#ifdef _STANDALONE
#ifdef _FLASH
    // TODO check to see if this breaks.
    Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    // TODO this breaks the RAM build.
    Device_bootCPU2(BOOTMODE_BOOT_TO_M0RAM);
#endif
#endif

    // Initialize the PIE module and vector table.
    Interrupt_initModule();
    Interrupt_initVectorTable();

    // Initialize and configure peripherals.
    InitGpio();
    InitAdc();
    InitDac();
    InitEPwm();

    // Register IPC1 interrupt for loading the DAC samples.
    IPC_registerInterrupt(IPC_CPU1_L_CPU2_R, IPC_INT1, ipc1_ISR);

    //
    // Clear any IPC flags if set already
    //
    IPC_clearFlagLtoR(IPC_CPU1_L_CPU2_R, IPC_FLAG_ALL);

    //
    // Synchronize both the cores.
    //
    IPC_sync(IPC_CPU1_L_CPU2_R, IPC_FLAG17);

    // Enable global interrupts.
    EINT;
    // Enable real-time debug.
    ERTM;

    for (;;) {
        // Do nothing.
        NOP;
    }
}
