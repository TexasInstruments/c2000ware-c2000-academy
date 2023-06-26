//############################################################################
//
// FILE: lab_shared.h
//
// TITLE: Structure Shared between .c and .cla files
//
// C2K ACADEMY URL: https://dev.ti.com/tirex/explore/node?node=AOpze8ebskysmgASY3VKSA__jEBbtmC__LATEST
//
// In this lab, we will use the Control Law Accelerator (CLA) to implement a
// FIR lowpass filter. We will apply a PWM waveform to the input of our
// lowpass filter, which will result in a sinusoidal waveform at the output.
// The PWM waveform will be generated using one of the ePWM modules and it
// will be sampled via the ADC at a given sample rate. Both the filtered and
// unfiltered waveforms will be viewed in a real-time Code Composer Studio
// (CCS) debug session via internal buffers. [[r! F28002x and F280013x Devices
// This lab cannot be performed on the F28002x and F280013x devices since they
// do not have a CLA. ]]
//
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

#ifndef LAB_SHARED_H
#define LAB_SHARED_H

#define ADC_BITS 12
#define ADC_VREF ((float)3.3)
#define ADC_VSTEP ((float)(ADC_VREF/((1 << ADC_BITS) - 1)))

extern float filter_out;
extern float filter_in;

__attribute__((interrupt)) void Cla1Task1(void);
__attribute__((interrupt)) void Cla1Task8(void);

#endif /* LAB_SHARED_H */
