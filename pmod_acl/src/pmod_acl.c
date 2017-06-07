/***************************************************************************//**
 *   @file   ADXL345.h
 *   @brief  Header file of ADXL345 Driver.
 *   @author ATofan (alexandru.tofan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: $WCREV$
*******************************************************************************/

#include "xparameters.h"
#include "xil_cache.h"
#include "xil_io.h"
#include "pmod.h"

/******************************************************************************/
/* ADXL345                                                                    */
/******************************************************************************/
#define ADXL345_RnW					7
#define ADXL345_MB					6
// ADXL345 Registers
#define ADXL345_DEVID				0x00
#define ADXL345_THRESH_TAP			0x1D
#define ADXL345_OFSX				0x1E
#define ADXL345_OFSY				0x1F
#define ADXL345_OFSZ				0x20
#define ADXL345_DUR					0x21
#define ADXL345_Latent				0x22
#define ADXL345_Window				0x23
#define ADXL345_THRESH_ACT			0x24
#define ADXL345_THRESH_INACT    	0x25
#define ADXL345_TIME_INACT			0x26
#define ADXL345_ACT_INACT_CTL		0x27
#define ADXL345_THRESH_FF			0x28
#define ADXL345_TIME_FF				0x29
#define ADXL345_TAP_AXES			0x2A
#define ADXL345_ACT_TAP_STATUS		0x2B
#define ADXL345_BW_RATE				0x2C
#define ADXL345_POWER_CTL			0x2D
#define ADXL345_INT_ENABLE			0x2E
#define ADXL345_INT_MAP				0x2F
#define ADXL345_INT_SOURCE			0x30
#define ADXL345_DATA_FORMAT			0x31
#define ADXL345_DATA_X0				0x32
#define ADXL345_DATA_X1				0x33
#define ADXL345_DATA_Y0				0x34
#define ADXL345_DATA_Y1				0x35
#define ADXL345_DATA_Z0				0x36
#define ADXL345_DATA_Z1				0x37
#define ADXL345_FIFO_CTL			0x38
#define ADXL345_FIFO_STATUS			0x39

// Register specific bits
// Device ID Register 0x00 - Read value should be 0xE5
// Tap Threshold Register 0x1D - Holds Threshold values for tap interrupts
// Offset X Register 0x1E |
// Offset Y Register 0x1F |- 15.6 mg/LSB, added to the acceleration data
// Offset Z Register 0x20 |
// Tap Duration Register 0x21 - 625us/LSB. Holds time that an event must be above THRESH_TAP to qualify as event
// Latent Register 0x22 - Time from detection of tap to time window. 1.25ms/LSB
// Window Register 0x23 - Time when a second tap can be detected. 1.25ms/LSB
// Activity Threshold Register 0x24 - Threshold value for detecting activity. 62.5mg/LSB
// Inactivity Threshold Register 0x25 - Threshold value for detecting inactivity. 62.5mg/LSB
// Inactivity Time Register 0x26 - Time for which acceleration must be less than THRESH_INACT. 1s/LSB
// Activity/Inactivity Control Register 0x27
#define ADXL345_ACT_ac_dc			7
#define ADXL345_ACT_X_en			6
#define ADXL345_ACT_Y_en			5
#define ADXL345_ACT_Z_en			4
#define ADXL345_INACT_ac_dc			3
#define ADXL345_INACT_X_en			2
#define ADXL345_INACT_Y_en			1
#define ADXL345_INACT_Z_en			0
// Threshold Free Fall Register 0x28 - Value for free fall detection. 62.5mg/LSB 0x05 to 0x09 are recommended
// Free Fall Time Register 0x29 - Time for which axes value must be lower than THRESH_FF to generate fall interrupt. 5ms/LSB. 0x14 to 0x46 recommended
// Axes Control For Single/Double Tap Register 0x2A
#define ADXL345_Suppress 			3
#define ADXL345_TAP_X_en			2
#define ADXL345_TAP_Y_en			1
#define ADXL345_TAP_Z_en			0
// Source Of Single/Double Tap Register 0x2B
#define ADXL345_ACT_X_Source		6
#define ADXL345_ACT_Y_Source		5
#define ADXL345_ACT_Z_Source		4
#define ADXL345_ASleep				3
#define ADXL345_TAP_X_Source		2
#define ADXL345_TAP_Y_Source		1
#define ADXL345_TAP_Z_Source		0
// Data Rate and Power Mode Control Register 0x2C
#define ADXL345_LOW_POWER			4
#define ADXL345_RATE				0
// Power Savings Features Control Register 0x2D
#define ADXL345_Link				5
#define ADXL345_AUTO_SLEEP			4
#define ADXL345_Measure				3
#define ADXL345_Sleep				2
#define ADXL345_Wakeup				0
// Interrupt Enable Control Register 0x2E
// Interrupt Mapping Control Register 0x2F
// Interrupt Source Register 0x30
#define ADXL345_DATA_READY			7
#define ADXL345_SINGLE_TAP			6
#define ADXL345_DOUBLE_TAP			5
#define ADXL345_Activity			4
#define ADXL345_Inactivity			3
#define ADXL345_FREE_FALL			2
#define ADXL345_Watermark			1
#define ADXL345_Overrun				0
// Data Format Control Register 0x31
#define ADXL345_SELF_TEST			7
#define ADXL345_SPI					6
#define ADXL345_INT_INVERT			5
#define ADXL345_FULL_RES			3
#define ADXL345_Justify				2
#define ADXL345_Range				0
// X-Axis Data0 Register 0x32
// X-Axis Data1 Register 0x33
// Y-Axis Data0 Register 0x34
// Y-Axis Data1 Register 0x35
// Z-Axis Data0 Register 0x36
// Z-Axis Data1 Register 0x37
// FIFO Control Register 0x38
#define ADXL345_FIFO_MODE			6
#define ADXL345_Trigger				5
#define ADXL345_Samples				0
// FIFO Status Register	0x39
#define ADXL345_FIFO_TRIG			7
#define ADXL345_Entries				0
//Command in the mal box
#define CAPTURE 0x3
#define XPS_FPGA11_INT_ID 	0

volatile char rxData;
volatile char isRunning = 0;
volatile int timeStepValue = 0;



/******************************************************************************
* @brief Writes to a ADXL345 Internal Register.
*
* @param addr 	- Register address.
* @param txData - Data to be sent.
*
* @return None.
******************************************************************************/
void ADXL345_WriteReg(u8 addr, u8 txData)
{
	u8 txBuffer[2] = {0x00, 0x00};
	
	txBuffer[0] = addr & 0x3F;
	txBuffer[1] = txData;

	spi_transfer(SPI_BASEADDR, 2, NULL, txBuffer);
}
/******************************************************************************
* @brief Reads from a ADXL345 Internal Register.
*
* @param addr 	- Register address.
*
* @return rxData - Data read from ADXL345.
******************************************************************************/
int ADXL345_ReadReg(u8 addr)
{
	u8 txBuffer[2] 	= {0x00, 0x00};
	u8 rxBuffer[2] 	= {0x00, 0x00};
	u8 rxData 		= 0;

	txBuffer[0] = (1 << ADXL345_RnW) | (addr & 0x3F);
	txBuffer[1] = 0x00;

	spi_transfer(SPI_BASEADDR, 2, rxBuffer, NULL);

	rxData = (rxBuffer[1] & 0xFF);

	return(rxData);
}

/******************************************************************************
* @brief Burst reads from a ADXL345 Internal Register.
*
* @param addr 	- First Register address.
*
* @return rxData - Data read from ADXL345.
******************************************************************************/
int ADXL345_BurstReadReg(u8 addr)
{
	u8 txBuffer[3] 	= {0x00, 0x00, 0x00};
	u8 rxBuffer[3] 	= {0x00, 0x00, 0x00};
	int  rxData 	 	= 0;

	txBuffer[0] = (1 << ADXL345_RnW) | (1 << ADXL345_MB) | (addr & 0x3F);
	txBuffer[1] = 0x00;

	spi_transfer(SPI_BASEADDR, 3, txBuffer, NULL);

	rxData = ((rxBuffer[2] & 0xFF) << 8) | (rxBuffer[1] & 0xFF);

	return(rxData);
}
void ADXL345_Init(void)
{
	ADXL345_WriteReg(ADXL345_BW_RATE, (0x07 << ADXL345_RATE)); 			// Data update rate = 100 Hz
	ADXL345_WriteReg(ADXL345_DATA_FORMAT, ((1 << ADXL345_FULL_RES)	|	// Data format = +- 16g, Full Resolution
										   (3 << ADXL345_Range)));
	ADXL345_WriteReg(ADXL345_INT_ENABLE, ((1 << ADXL345_SINGLE_TAP)	|	// Enable Single, Double Tap
										  (1 << ADXL345_DOUBLE_TAP)	|
										  (0 << ADXL345_Activity)	|
										  (0 << ADXL345_FREE_FALL)	|
										  (0 << ADXL345_DATA_READY)));
	ADXL345_WriteReg(ADXL345_THRESH_TAP, 0x20); 						// Tap threshold
	ADXL345_WriteReg(ADXL345_DUR, 0x0D); 								// Tap duration
	ADXL345_WriteReg(ADXL345_Latent, 0x50); 							// Tap latency
	ADXL345_WriteReg(ADXL345_Window, 0xF0); 							// Tap Window
	ADXL345_WriteReg(ADXL345_TAP_AXES, ((1 << ADXL345_TAP_X_en)		|	// Enable tap on all axes
										(1 << ADXL345_TAP_Y_en)		|
										(1 << ADXL345_TAP_Z_en)));
	ADXL345_WriteReg(ADXL345_THRESH_ACT, 0x08); 						// Activity threshold
	ADXL345_WriteReg(ADXL345_ACT_INACT_CTL, ((1 << ADXL345_ACT_X_en)|	// Activity axes
											(1 << ADXL345_ACT_Y_en) |
											(1 << ADXL345_ACT_Z_en)));
	ADXL345_WriteReg(ADXL345_THRESH_FF, 0x08); 							// Free fall threshold
	ADXL345_WriteReg(ADXL345_TIME_FF, 0x0A); 							// Free fall window
	ADXL345_WriteReg(ADXL345_FIFO_CTL, 0x01);
}
/******************************************************************************
* @brief Display g force for desired axis.
*
* @param axis - Which axis to read.
*
* @return None.
******************************************************************************/
int Get_G_Force(char axis)
{
	int 	gForce 		= 0;
	int 	gForceCalc 	= 0;
	// Select which axis to read
	switch(axis)
	{
		case 'x':
			gForce = ADXL345_BurstReadReg(ADXL345_DATA_X0);
			break;
		case 'y':
			gForce = ADXL345_BurstReadReg(ADXL345_DATA_Y0);
			break;
		case 'z':
			gForce = ADXL345_BurstReadReg(ADXL345_DATA_Z0);
			break;
		default:
			break;
	}

	// If read result is negative, apply padding with 0xFFFFF000
	if((gForce & 0x1000) == 0x1000)
	{
		gForceCalc = 0xFFFFF000 | gForce;
	}
	// If read result is positive, do nothing to modify it
	else
	{
		gForceCalc = gForce & 0x0FFF;
	}
	return gForceCalc;
}
/******************************************************************************
* @brief Run measurements.
*
* @param None.
*
* @return None.
******************************************************************************/
void ADXL345_Run(void)
{
	if(isRunning == 0)
	{
		ADXL345_WriteReg(ADXL345_POWER_CTL, (1 << ADXL345_Measure));
		isRunning = 1;
	}
}

/******************************************************************************
* @brief Stop measurements.
*
* @param None.
*
* @return None.
******************************************************************************/
void ADXL345_Stop(void)
{
	if(isRunning == 1)
	{
		ADXL345_WriteReg(ADXL345_POWER_CTL, (0 << ADXL345_Measure));
		isRunning = 0;
	}
}
int main(void)
{
	u8 rxValue = 0x00;
	int cmd;
	
	pmod_init(0,1);
	config_pmod_switch(SS, GPIO_1, MISO, SPICLK, 
                      GPIO_4, GPIO_5, GPIO_6, GPIO_7);
	ADXL345_Init();
	ADXL345_Run();
	while(1)
	{
		while((MAILBOX_CMD_ADDR & 0x01)==0);
		cmd = MAILBOX_CMD_ADDR;

		switch(cmd){
		
		case CAPTURE:
			if(isRunning==1)
			{
				//Start to caputure
				MAILBOX_DATA(0)= Get_G_Force('x');
				MAILBOX_DATA(1)= Get_G_Force('y');
				MAILBOX_DATA(2)= Get_G_Force('z');
			}
			MAILBOX_CMD_ADDR = 0x0;
			break;
		default:
			// reset command
			MAILBOX_CMD_ADDR = 0x0;
			break;
      }
	  ADXL345_Stop();
   }
}
