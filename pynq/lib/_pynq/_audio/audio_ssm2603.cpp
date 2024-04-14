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
 * @file audio_ssm2603.c
 *
 * Functions to control audio controller.
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
 *****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#include "i2cps.h"
#include "uio.h"
#include "audio_ssm2603.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
}
#endif

/******************************************************************************
 * Function to write 8 bits to one of the registers from the audio
 * controller.
 * @param   u8RegAddr is the register address.
 * @param   u16Data is the data word to write.
 * @param   iic_fd is the file descriptor for /dev/i2c-x
 * @return  none.
 *****************************************************************************/
void write_audio_reg(unsigned char u8RegAddr, 
                     uint16_t u16Data, int iic_fd) {
    unsigned char u8TxData[2];

    u8TxData[0] = u8RegAddr << 1;
	u8TxData[0] = u8TxData[0] | ((u16Data >> 8) & 0b1);

	u8TxData[1] = u16Data & 0xFF;

    if (writeI2C_asFile(iic_fd, u8TxData, 2) < 0){
        printf("Unable to write audio register.\n");
    }
}

/******************************************************************************
 * Function to configure the audio PLL.
 * @param   iic_index is the i2c index in /dev list.
 * @return  none.
 *****************************************************************************/
extern "C" void config_audio_pll(int iic_index) {
    // Not needed
}

/******************************************************************************
 * Function to configure the audio codec.
 * @param   iic_index is the i2c index in /dev list.
 * @return  none.
 *****************************************************************************/
extern "C" void config_audio_codec(int iic_index) {
    int iic_fd;
    iic_fd = setI2C(iic_index, IIC_SLAVE_ADDR);
    if (iic_fd < 0) {
        printf("Unable to set I2C %d.\n", iic_index);
    }

	useconds_t wait = 75000; // 75 ms for VMID capacitor to settle.

    // Perform Reset
	write_audio_reg(R15_SOFTWARE_RESET, 				0b000000000, iic_fd); 
	usleep(wait);

    // Power Up everything apart from CLKOUT
	write_audio_reg(R6_POWER_MANAGEMENT, 				0b000010000, iic_fd); 
	
    // Default output Volume to 0db both left and right MSB=1
    write_audio_reg(R0_LEFT_CHANNEL_ADC_INPUT_VOLUME, 	0b100010111, iic_fd); 

    // Mute  both left and right
	write_audio_reg(R2_LEFT_CHANNEL_DAC_VOLUME, 		0b100000000, iic_fd); 

    // Allow Mixed DAC, Mute MIC mixing
	write_audio_reg(R4_ANALOG_AUDIO_PATH, 				0b000010010, iic_fd); 

    // 48 kHz Sampling Rate emphasis, no high pass
	write_audio_reg(R5_DIGITAL_AUDIO_PATH, 				0b000000111, iic_fd); 
	
    // 32 bit i2s mode
    write_audio_reg(R7_DIGITAL_AUDIO_I_F, 				0b000001110, iic_fd); 

    // 48 kHz sampling for ADC and DAC
	write_audio_reg(R8_SAMPLING_RATE, 					0b000000000, iic_fd); 

    // Select DAC
	write_audio_reg(R4_ANALOG_AUDIO_PATH, 				0b000010000, iic_fd); 
	
    // Unmute, disable filtering, disable de-emphasis
    write_audio_reg(R5_DIGITAL_AUDIO_PATH, 				0b000000000, iic_fd);

    // 32 bit i2s mode
	write_audio_reg(R7_DIGITAL_AUDIO_I_F, 				0b000001110, iic_fd); 
	
    // 48khz sampling rate
    write_audio_reg(R8_SAMPLING_RATE, 					0b000000000, iic_fd);
	usleep(wait);

	if (unsetI2C(iic_fd) < 0) {
        printf("Unable to unset I2C %d.\n", iic_index);
    }
}

/******************************************************************************
 * Function to select LINE_IN as input.
 * @param  iic_index is the i2c index in /dev list.
 * @return none.
 *****************************************************************************/
extern "C" void select_line_in(int iic_index) {
    int iic_fd;
    iic_fd = setI2C(iic_index, IIC_SLAVE_ADDR);
    if (iic_fd < 0) {
        printf("Unable to set I2C %d.\n", iic_index);
    }

    // Select DAC, Line->ADC, Mute MIC
	write_audio_reg(R4_ANALOG_AUDIO_PATH, 0b000010010, iic_fd); 

    if (unsetI2C(iic_fd) < 0) {
        printf("Unable to unset I2C %d.\n", iic_index);
    }
}

/******************************************************************************
 * Function to select MIC as input.
 * @param  iic_index is the i2c index in /dev list.
 * @return none.
 *****************************************************************************/
extern "C" void select_mic(int iic_index) {
    int iic_fd;
    iic_fd = setI2C(iic_index, IIC_SLAVE_ADDR);
    if (iic_fd < 0) {
        printf("Unable to set I2C %d.\n", iic_index);
    }

    // Select DAC, Mic->ADC
	write_audio_reg(R4_ANALOG_AUDIO_PATH, 0b000010100, iic_fd); 

    if (unsetI2C(iic_fd) < 0) {
        printf("Unable to unset I2C %d.\n", iic_index);
    }
}

/******************************************************************************
 * Function to deselect input, either LINE_IN, or MIC.
 * @param  iic_index is the i2c index in /dev list.
 * @return none.
 *****************************************************************************/
extern "C" void deselect(int iic_index) {
    int iic_fd;
    iic_fd = setI2C(iic_index, IIC_SLAVE_ADDR);
    if (iic_fd < 0) {
        printf("Unable to set I2C %d.\n", iic_index);
    }

    // Select DAC
	write_audio_reg(R4_ANALOG_AUDIO_PATH, 0b000010000, iic_fd); 

    if (unsetI2C(iic_fd) < 0) {
        printf("Unable to unset I2C %d.\n", iic_index);
    }
}


/******************************************************************************
 * Record and play the audio without storing in the DDR3.
 *
 * @param   audio_mmap_size is the address range of the audio codec.
 * @param   nsamples is the number of samples to read and output.
 * @param   uio_index is the uio index in /dev list.
 * @param   iic_index is the i2c index in /dev list.
 * @return  none.
 *****************************************************************************/
extern "C" void bypass(unsigned int audio_mmap_size,
                       unsigned int nsamples, unsigned int volume,
                       int uio_index, int iic_index) {
    int i, status;
    void *uio_ptr;
    int DataL, DataR;
    int iic_fd;

    uio_ptr = setUIO(uio_index, audio_mmap_size);
    iic_fd = setI2C(iic_index, IIC_SLAVE_ADDR);
    if (iic_fd < 0) {
        printf("Unable to set I2C %d.\n", iic_index);
    }

    // // Mute mixer1 and mixer2 input
    // write_audio_reg(R23_PLAYBACK_MIXER_LEFT_CONTROL_1, 0x00, iic_fd);
    // write_audio_reg(R25_PLAYBACK_MIXER_RIGHT_CONTROL_1, 0x00, iic_fd);
    // // Enable Mixer3 and Mixer4
    // write_audio_reg(R22_PLAYBACK_MIXER_LEFT_CONTROL_0, 0x21, iic_fd);
    // write_audio_reg(R24_PLAYBACK_MIXER_RIGHT_CONTROL_0, 0x41, iic_fd);

    // unsigned char vol_register = (unsigned char)volume << 2 | 0x3;
    // // Enable Left/Right Headphone out
    // write_audio_reg(R29_PLAYBACK_HEADPHONE_LEFT_VOLUME_CONTROL,
    //                 vol_register,
    //                 iic_fd);
    // write_audio_reg(R30_PLAYBACK_HEADPHONE_RIGHT_VOLUME_CONTROL,
    //                 vol_register,
    //                 iic_fd);

    // for(i=0; i<nsamples; i++){
    //     //wait for RX data to become available
    //     do {
    //         status = \
    //         *((volatile unsigned *)(((uint8_t *)uio_ptr) + I2S_STATUS_REG));
    //     } while (status == 0);
    //     *((volatile unsigned *)(((uint8_t *)uio_ptr) + I2S_STATUS_REG)) = \
    //         0x00000001;

    //     // Read the sample from the input
    //     DataL = *((volatile int *)(((uint8_t *)uio_ptr) + I2S_DATA_RX_L_REG));
    //     DataR = *((volatile int *)(((uint8_t *)uio_ptr) + I2S_DATA_RX_R_REG));

    //     // Write the sample to output
    //     *((volatile int *)(((uint8_t *)uio_ptr) + I2S_DATA_TX_L_REG)) = DataL;
    //     *((volatile int *)(((uint8_t *)uio_ptr) + I2S_DATA_TX_R_REG)) = DataR;
    // }

    // write_audio_reg(R23_PLAYBACK_MIXER_LEFT_CONTROL_1, 0x00, iic_fd);
    // write_audio_reg(R25_PLAYBACK_MIXER_RIGHT_CONTROL_1, 0x00, iic_fd);
    // write_audio_reg(R22_PLAYBACK_MIXER_LEFT_CONTROL_0, 0x00, iic_fd);
    // write_audio_reg(R24_PLAYBACK_MIXER_RIGHT_CONTROL_0, 0x00, iic_fd);
    // write_audio_reg(R29_PLAYBACK_HEADPHONE_LEFT_VOLUME_CONTROL, 0xE5, iic_fd);
    // write_audio_reg(R30_PLAYBACK_HEADPHONE_RIGHT_VOLUME_CONTROL, 0xE5, iic_fd);

    if (unsetUIO(uio_ptr, audio_mmap_size) < 0){
        printf("Unable to free UIO %d.\n", uio_index);
    }
    if (unsetI2C(iic_fd) < 0) {
        printf("Unable to unset I2C %d.\n", iic_index);
    }
}

/******************************************************************************
 * Function to support audio recording using the audio codec controller.
 *
 * Notice that the buffer has to be twice the size of the number of samples,
 * because both left and right channels are sampled. 
 *
 * @param   audio_mmap_size is the address range of the audio codec.
 * @param   BufAddr is the buffer address.
 * @param   nsamples is the number of samples.
 * @param   uio_index is the uio index in /dev list.
 * @param   iic_index is the i2c index in /dev list.
 * @return  none.
 *****************************************************************************/
extern "C" void record(unsigned int audio_mmap_size,
                       unsigned int* BufAddr, unsigned int nsamples, 
                       int uio_index, int iic_index){
    unsigned int  i, status;
    void *uio_ptr;
    int DataL, DataR;
    int iic_fd;

    uio_ptr = setUIO(uio_index, audio_mmap_size);
    iic_fd = setI2C(iic_index, IIC_SLAVE_ADDR);
    if (iic_fd < 0) {
        printf("Unable to set I2C %d.\n", iic_index);
    }

    for(i=0; i<nsamples; i++) {
        do {
            status = \
            *((volatile unsigned *)(((uint8_t *)uio_ptr) + I2S_STATUS_REG));
        } while (status == 0);
        *((volatile unsigned *)(((uint8_t *)uio_ptr) + I2S_STATUS_REG)) = \
            0x00000001;

        // Read the sample from the input
        DataL = *((volatile int *)(((uint8_t *)uio_ptr) + I2S_DATA_RX_L_REG));
        DataR = *((volatile int *)(((uint8_t *)uio_ptr) + I2S_DATA_RX_R_REG));

        // Write the sample into memory
        *(BufAddr+2*i) = DataL;
        *(BufAddr+2*i+1) = DataR;
    }

    if (unsetUIO(uio_ptr, audio_mmap_size) < 0){
        printf("Unable to free UIO %d.\n", uio_index);
    }
    if (unsetI2C(iic_fd) < 0) {
        printf("Unable to unset I2C %d.\n", iic_index);
    }
}

/******************************************************************************
 * Function to support audio playing using the audio codec controller.
 *
 * Notice that the buffer has to be twice the size of the number of samples,
 * because both left and right channels are sampled.
 *
 * @param   audio_mmap_size is the address range of the audio codec.
 * @param   BufAddr is the buffer address.
 * @param   nsamples is the number of samples.
 * @param   volume is the volume, 0=-57dB, Maximum : 63=+6dB.
 * @param   uio_index is the uio index in /dev list.
 * @param   iic_index is the i2c index in /dev list.
 * @return  none.
 *****************************************************************************/
extern "C" void play(unsigned int audio_mmap_size,
                     unsigned int* BufAddr, unsigned int nsamples, 
                     unsigned int volume, int uio_index, int iic_index){
    unsigned int  i, status;
    void *uio_ptr;
    int DataL, DataR;
    int iic_fd;

    printf("Playing\n");
    
    uio_ptr = setUIO(uio_index, audio_mmap_size);
    iic_fd = setI2C(iic_index, IIC_SLAVE_ADDR);
    if (iic_fd < 0) {
        printf("Unable to set I2C %d.\n", iic_index);
    }

    // Set volume for DAC left and right
    uint16_t vol_register = (unsigned char)volume << 2 | 0xb100000011;

    // update both channels
	write_audio_reg(R2_LEFT_CHANNEL_DAC_VOLUME, vol_register, iic_fd); 

    for(i=0; i<nsamples; i++){
        do {
            status = \
            *((volatile unsigned *)(((uint8_t *)uio_ptr) + I2S_STATUS_REG));
        } while (status == 0);
        *((volatile unsigned *)(((uint8_t *)uio_ptr) + I2S_STATUS_REG)) = \
            0x00000001;

        // Read the sample from memory
        DataL = *(BufAddr+2*i);
        DataR = *(BufAddr+2*i+1);

        // Write the sample to output
        *((volatile int *)(((uint8_t *)uio_ptr) + I2S_DATA_TX_L_REG)) = DataL;
        *((volatile int *)(((uint8_t *)uio_ptr) + I2S_DATA_TX_R_REG)) = DataR;
    }

    // mute DAC left and right
    vol_register = 0b100000000;
	write_audio_reg(R2_LEFT_CHANNEL_DAC_VOLUME, vol_register, iic_fd); 

    if (unsetUIO(uio_ptr, audio_mmap_size) < 0){
        printf("Unable to free UIO %d.\n", uio_index);
    }
    if (unsetI2C(iic_fd) < 0) {
        printf("Unable to unset I2C %d.\n", iic_index);
    }
}
