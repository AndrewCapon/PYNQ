/******************************************************************************
 *  Copyright (c) 2025, Andrew Capon.
 *
 *  Based on : audio_adau1761.c
 * 
 *  Copyright (c) 2016, Xilinx, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1.  Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 *
 *  2.  Redistributions in binary form must reproduce the above copyright 
 *      notice, this list of conditions and the following disclaimer in the 
 *      documentation and/or other materials provided with the distribution.
 *
 *  3.  Neither the name of the copyright holder nor the names of its 
 *      contributors may be used to endorse or promote products derived from 
 *      this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/


/******************************************************************************
 *
 * @file audio_ssm2603.h
 *
 *  Library for the audio control block.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who          Date     Changes
 * ----- ------------ -------- -----------------------------------------------
 * 1.00  Andrew Capon 13/04/17 Support for audio codec SSM2603
 * 
 * </pre>
 *
******************************************************************************/
/*
 * SSM2603 audio controller parameters
 */
#ifndef _AUDIO_SSM2603_H_
#define _AUDIO_SSM2603_H_

// Slave address for the SSM2603
#define IIC_SLAVE_ADDR          0x1A

// I2C Serial Clock frequency in Hertz
#define IIC_SCLK_RATE           100000

// I2S Register
#define I2S_DATA_RX_L_REG           0x00
#define I2S_DATA_RX_R_REG           0x04
#define I2S_DATA_TX_L_REG           0x08
#define I2S_DATA_TX_R_REG           0x0C
#define I2S_STATUS_REG              0x10

//ADAU internal registers
enum audio_ssm2603_regs {
	R0_LEFT_CHANNEL_ADC_INPUT_VOLUME    = 0x00,
	R1_RIGHT_CHANNEL_ADC_INPUT_VOLUME	= 0x01,
	R2_LEFT_CHANNEL_DAC_VOLUME		    = 0x02,
	R3_RIGHT_CHANNEL_DAC_VOLUME			= 0x03,
	R4_ANALOG_AUDIO_PATH				= 0x04,
	R5_DIGITAL_AUDIO_PATH				= 0x05,
	R6_POWER_MANAGEMENT					= 0x06,
	R7_DIGITAL_AUDIO_I_F				= 0x07,
	R8_SAMPLING_RATE					= 0x08,
	R9_ACTIVE							= 0x09,
	R15_SOFTWARE_RESET					= 0x0F,
	R16_ALC_CONTROL_1					= 0x10,
	R17_ALC_CONTROL_2					= 0x11,
	R18_NOISE_GATE						= 0x12,
};


#endif /* _AUDIO_SSM2603_H_ */