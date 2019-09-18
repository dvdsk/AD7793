/***************************************************************************//**
 *   @file   AD7793.h
 *   @brief  Header file of AD7793 Driver.
 *   @author Bancisor Mihai
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
 *   SVN Revision: 500
*******************************************************************************

********************************************************************************
 *   Modified and adapted for Arduino by Ph. Sonnet.  March 27, 2019, Version 1.1
*******************************************************************************/

#ifndef __AD7793_H__
#define __AD7793_H__

#include <stdint.h>
#include "Arduino.h"
#include <SPI.h>

/******************************************************************************/
/* AD7793                                                                   */
/******************************************************************************/

/*AD7793 Registers*/
#define AD7793_REG_COMM		0 /* Communications Register(WO, 8-bit) */
#define AD7793_REG_STAT	    0 /* Status Register	    (RO, 8-bit) */
#define AD7793_REG_MODE	    1 /* Mode Register	     	(RW, 16-bit */
#define AD7793_REG_CONF	    2 /* Configuration Register (RW, 16-bit)*/
#define AD7793_REG_DATA	    3 /* Data Register	     	(RO, 16-/24-bit) */
#define AD7793_REG_ID	    4 /* ID Register	     	(RO, 8-bit) */
#define AD7793_REG_IO	    5 /* IO Register	     	(RO, 8-bit) */
#define AD7793_REG_OFFSET   6 /* Offset Register	    (RW, 24-bit */
#define AD7793_REG_FULLSCAL	7 /* Full-Scale Register	(RW, 24-bit */

/* Communications Register Bit Designations (AD7793_REG_COMM) */
#define AD7793_COMM_WEN		(1 << 7) 			/* Write Enable */
#define AD7793_COMM_WRITE	(0 << 6) 			/* Write Operation */
#define AD7793_COMM_READ    (1 << 6) 			/* Read Operation */
#define AD7793_COMM_ADDR(x)	(((x) & 0x7) << 3)	/* Register Address */
#define AD7793_COMM_CREAD	(1 << 2) 			/* Continuous Read of Data Register */

/* Status Register Bit Designation masks(AD7793_REG_STAT) */
#define AD7793_STAT_RDY		(1 << 7) /* Ready */
#define AD7793_STAT_ERR		(1 << 6) /* Error (Overrange, Underrange) */
#define AD7793_STAT_CH3		(1 << 2) /* Channel 3 */
#define AD7793_STAT_CH2		(1 << 1) /* Channel 2 */
#define AD7793_STAT_CH1		(1 << 0) /* Channel 1 */

/* Mode Register Bit Designations (AD7793_REG_MODE) */
#define AD7793_MODE_SEL(x)		(((x) & 0x7) << 13)	/* Operation Mode Select */
#define AD7793_MODE_CLKSRC(x)	(((x) & 0x3) << 6) 	/* ADC Clock Source Select */
#define AD7793_MODE_RATE(x)		((x) & 0xF) 		/* Filter Update Rate Select */

/* AD7793_MODE_SEL(x) options */
#define AD7793_MODE_CONT		 0 /* Continuous Conversion Mode */
#define AD7793_MODE_SINGLE		 1 /* Single Conversion Mode */
#define AD7793_MODE_IDLE		 2 /* Idle Mode */
#define AD7793_MODE_PWRDN		 3 /* Power-Down Mode */
#define AD7793_MODE_CAL_INT_ZERO 4 /* Internal Zero-Scale Calibration */
#define AD7793_MODE_CAL_INT_FULL 5 /* Internal Full-Scale Calibration */
#define AD7793_MODE_CAL_SYS_ZERO 6 /* System Zero-Scale Calibration */
#define AD7793_MODE_CAL_SYS_FULL 7 /* System Full-Scale Calibration */

/* AD7793_MODE_CLKSRC(x) options */
#define AD7793_CLK_INT		0 /* Internal 64 kHz Clk not available at the CLK pin */
#define AD7793_CLK_INT_CO	1 /* Internal 64 kHz Clk available at the CLK pin */
#define AD7793_CLK_EXT		2 /* External 64 kHz Clock */
#define AD7793_CLK_EXT_DIV2	3 /* External Clock divided by 2 */

/* AD7793_MODE_RATE(x) options */
#define AD7793_RATE_NIL    		0
#define AD7793_RATE_470    		1
#define AD7793_RATE_242    		2
#define AD7793_RATE_123    		3
#define AD7793_RATE_62    		4
#define AD7793_RATE_50  		5
#define AD7793_RATE_39			6
#define AD7793_RATE_33_2		7
#define AD7793_RATE_19_6		8 	/* 60 Hz only  */
#define AD7793_RATE_16_7_50		9 	/* 50 Hz only  */
#define AD7793_RATE_16_7_50_60  10	/* 50 and 60 Hz */
#define AD7793_RATE_12_5  		11 	/* 50 and 60 Hz */
#define AD7793_RATE_10    	12 	/* 50 and 60 Hz */
#define AD7793_RATE_8_33  		13 	/* 50 and 60 Hz */
#define AD7793_RATE_6_25  		14 	/* 50 and 60 Hz */
#define AD7793_RATE_4_17  		15 	/* 50 and 60 Hz */

/* Configuration Register Bit Designations (AD7793_REG_CONF) */
#define AD7793_CONF_VBIAS(x)  (((x) & 0x3) << 14) 	/* Bias Voltage Generator Enable */
#define AD7793_CONF_BO_EN	  (1 << 13) 			/* Burnout Current Enable */
#define AD7793_CONF_UNIPOLAR  (1 << 12) 			/* Unipolar/Bipolar Enable */
#define AD7793_CONF_BOOST	  (1 << 11) 			/* Boost Enable */
#define AD7793_CONF_GAIN(x)	  (((x) & 0x7) << 8) 	/* Gain Select */
#define AD7793_CONF_REFSEL(x) (((x) & 0x1) << 7) 	/* INT/EXT Reference Select */
#define AD7793_CONF_BUF		  (1 << 4) 				/* Buffered Mode Enable */
#define AD7793_CONF_CHAN(x)	  ((x) & 0x7) 			/* Channel select */

/* AD7793_CONF_VBIAS(x) options */
#define AD7793_VBIAS_GEN_DISABL    	0	/* Bias voltage generator disabled */
#define AD7793_VBIAS_AIN1_NEG    	1 	/* Bias voltage generator to AIN1(-) */
#define AD7793_VBIAS_AIN2_NEG    	2 	/* Bias voltage generator to AIN2(-) */

/* AD7793_CONF_GAIN(x) options  Gain and ADC input range (2.5V reference)*/
#define AD7793_GAIN_1       0	/* 1X (In-amp nor used)	2.5 */
#define AD7793_GAIN_2       1	/* 2X (In-amp nor used)	1.25 */
#define AD7793_GAIN_4       2	/* 4X 					625mV */
#define AD7793_GAIN_8       3	/* 8X 					312.5 mV*/
#define AD7793_GAIN_16      4	/* 16X					156.2 mV*/
#define AD7793_GAIN_32      5	/* 32X 					78.125 mV */
#define AD7793_GAIN_64      6	/* 64X					39.06 mV  */
#define AD7793_GAIN_128     7	/* 128X					19.53 mV */

/* AD7793_CONF_REFSEL(x) options */
#define AD7793_REFSEL_INT   1	/* Internal reference selected. */
#define AD7793_REFSEL_EXT   0	/* External reference applied between REFIN(+) and REFIN(–). */

/* AD7793_CONF_CHAN(x) options */
#define AD7793_CH_AIN1P_AIN1M	0 /* Select channel AIN1(+) - AIN1(-) */
#define AD7793_CH_AIN2P_AIN2M	1 /* Select channel AIN2(+) - AIN2(-) */
#define AD7793_CH_AIN3P_AIN3M	2 /* Select channel AIN3(+) - AIN3(-) */
#define AD7793_CH_AIN1M_AIN1M	3 /* Select channel AIN1(-) - AIN1(-) */
#define AD7793_CH_TEMP			6 /* Temp Sensor, gain 1, Internal reference */
#define AD7793_CH_AVDD_MONITOR	7 /* AVDD voltage Monitor, gain 1/6, Internal 1.17V reference */

/* ID Register Bit Designations (AD7793_REG_ID) */
#define AD7793_ID			0xB
#define AD7793_ID_MASK		0xF

/* IO (Excitation Current Sources) Register Bit Designations (AD7793_REG_IO) */
#define AD7793_IEXCDIR(x)	(((x) & 0x3) << 2)
#define AD7793_IEXCEN(x)	(((x) & 0x3) << 0)

/* AD7793_IEXCDIR(x) options*/
#define AD7793_DIR_IEXC1_IOUT1_IEXC2_IOUT2	0  /* IEXC1 connect to IOUT1, IEXC2 connect to IOUT2 */
#define AD7793_DIR_IEXC1_IOUT2_IEXC2_IOUT1	1  /* IEXC1 connect to IOUT2, IEXC2 connect to IOUT1 */
#define AD7793_DIR_IEXC1_IEXC2_IOUT1		2  /* Both current sources IEXC1,2 connect to IOUT1  */
#define AD7793_DIR_IEXC1_IEXC2_IOUT2		3  /* Both current sources IEXC1,2 connect to IOUT2 */

/* AD7793_IEXCEN(x) options*/
#define AD7793_EN_IXCEN_DISABLE				0  /* Disable excitation current*/
#define AD7793_EN_IXCEN_10uA				1  /* Excitation Current 10uA */
#define AD7793_EN_IXCEN_210uA				2  /* Excitation Current 210uA */
#define AD7793_EN_IXCEN_1mA					3  /* Excitation Current 1mA */

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/

class AD7793{
	private:
	uint8_t cs_pin;
	uint8_t data_rdy_pin;
	SPIClass* spi_port;

	public:
	/* Initialize AD7793 and check if the device is present*/
	unsigned char begin(uint8_t _cs_pin, uint8_t _data_rdy_pin, SPIClass &_spi_port = SPI);

	/* Sends 32 consecutive 1's on SPI in order to reset the part. */
	void Reset(void);

	/* Reads the Ready Bit for ADC */
	unsigned char Ready();

	/* Reads the ADC Error Bit */
	unsigned char Error();

	/* Indicates that channel 3 is being converted by the ADC */
	unsigned char Channel3();

	/* Indicates that channel 2 is being converted by the ADC */
	unsigned char Channel2();

	/* Indicates that channel 1 is being converted by the ADC */
	unsigned char Channel1();


	/* Reads the value of the selected register. */
	unsigned long GetRegisterValue(unsigned char regAddress,
										unsigned char size,
										unsigned char modifyCS);

	/* Writes a value to the register. */
	void SetRegisterValue(unsigned char regAddress,
								unsigned long regValue,
								unsigned char size,
								unsigned char modifyCS);

	/* Waits for RDY pin to go low. */
	void WaitRdyGoLow(void);

	/* Sets the operating mode of AD7793 */
	void SetMode(unsigned long mode);

	/* Sets the ADC clock source of AD7793 */
	void SetClockSource(unsigned long clockSource);

	/* Sets the filter update rate of AD7793 */
	void SetFilterUpdateRate(unsigned long filterRate);

	/* Sets the direction of the internal excitation current source */
	void SetExcitDirection(unsigned long direction);

	/* Sets the current of the internal excitation current source */
	void SetExcitCurrent(unsigned long current);

	/* Bias voltage generator enable */
	void SetBiasVoltage(unsigned long voltage);

	/* Enable burnout current */
	void EnableBurnoutCurr(void);

	/* Disable burnout current of AD7793 */
	void DisableBurnoutCurr(void);

	/* Enable unipolar coding of AD7793 */
	void EnableUnipolar(void);

	/* Disable bipolar coding of AD7793 */
	void DisableBipolar(void);

	/* Enable bias voltage generator current boost of AD7793 */
	void EnableCurrBoost(void);

	/* Disable bias voltage generator current boost of AD7793 */
	void DisableCurrBoost(void);

	/* Sets the gain of the In-Amp */
	void SetGain(unsigned long gain);

	/* Sets the reference source for the ADC */
	void SetIntReference(unsigned char type);

	/* Enable buffered mode of AD7793*/
	void EnableBufMode(void);

	/* Disable buffered mode of AD7793 */
	void DisableBufMode(void);

	/* Selects the channel of AD7793 */
	void SetChannel(unsigned long channel);

	/* Performs the given calibration to the specified channel. */
	void Calibrate(unsigned char mode, unsigned char channel);

	/* Returns the result of a single conversion. */
	unsigned long SingleConversion(void);

	/* Returns the average of several conversion results. */
	unsigned long ContinuousReadAvg(unsigned char sampleNumber);

	/* Returns a single measurement, provided continuous mesurement mode has been set up. */
	unsigned long ContinuousSingleRead();

	/* Initializes the SPI communication peripheral. */
	unsigned char SPI_Init();

	/* Writes data to SPI. */
	unsigned char SPI_Write(unsigned char* data,
							unsigned char bytesNumber);
	/* Reads data from SPI. */
	unsigned char SPI_Read(unsigned char* data,
						unsigned char bytesNumber);
};

#endif	// _AD7793_H_
