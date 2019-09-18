How to implement the AD7793 library (Ph. Sonnet, Mach 29, 2019)
===================================

Directory AD7793 must be created in the Aduino "libraries" directory 
and must contain the following files : 

AD7793.h
AD7793.cpp
Communication.h
Communication.cpp


How to connect the AD7793 to the Arduino Atmega328 or MEGA
===========================================================

AD7793 pin 16 (DIN), Master Out Slave In (MOSI), must be connected to pin 11 (for Atmega328) or pin 51 (for MEGA). 

AD7793 pin 15 (DOUT/~RDY), Master In Slave Out (MISO), must be connected to pin 12 (for Atmega328) or pin 50 (for MEGA). 
It must also be connected to another digital pin af the Arduino. Suppose that you choose pin 9 for this purpose. 
This pin number must be specified in the Communication.h file by editing the #define ADI_PAR_CS_PIN line in the "GPIO Definitions" section. 

AD7793 pin 1 (CLK), Serial Clock (SCK), must be connected to Pin 13 (for Atmega328) or pin 52 (for MEGA).

AD7793 pin 3(~CS), Chip Select (CS) should not be connected to pin 10 (for Atmega328) or pin 53 (for MEGA), as it is supposed by the SPI library. 
Instead, another digital pin of the Arduino must be used. Suppose that you choose pin 8 for this purpose. 
This pin number must be specified in the Communication.h file by editing the #define ADI_PAR_CS_PIN line in the "GPIO Definitions" section.

From my experience, it is crucial to insert pullup resistors (for instance, 50 KOhm) in the MOSI, CS and CLK circuits, to ensure reliable
communication between the AD7793 and the Arduino. 


Preprocessing instructions 
==========================
  Here are the preprocessing instruction which must be inserted before setup()
  into your sketch.

#include <AD7793.h>
#include <Communication.h>
#include <SPI.h>
    
Initialization 
==============
  This must be inserted as such into setup(). No need to initialize the
  SPI channel, because this is taken care of by the AD7793_Init()function.

unsigned char answer = AD7793_Init();  /* Initializes the AD7793 and checks if the device is present*/
AD7793_Reset();    /* Sends 32 consecutive 1 on SPI in order to reset the part */


Preprogramed register setting functions
========================================
  Just insert these functions into your sketch as they are listed here, and they will
  access the various AD7793 registers (Status, Mode, Configuration, Data, IO and Data) and set parameters 
  according to the needs of your project. 

Status register
...............

unsigned char answer = AD7793_Ready();   /* Reads the Ready Bit for ADC */  
unsigned char answer = AD7793_Error();  /* Reads the ADC Error Bit */ 
unsigned char answer = AD7793_Channel3(); /* Indicates that channel 3 is being converted by the ADC */ 
unsigned char answer = AD7793_Channel2(); /* Indicates that channel 2 is being converted by the ADC */ 
unsigned char answer = AD7793_Channel1(); /* Indicates that channel 1 is being converted by the ADC */    

Mode register
..............

AD7793_SetMode(AD7793_MODE_CONT);  /* Continuous Conversion Mode */
D7793_SetMode(AD7793_MODE_SINGLE);  /* Single Conversion Mode */
AD7793_SetMode(AD7793_MODE_IDLE);  /* Idle Mode */
AD7793_SetMode(AD7793_MODE_PWRDN);  /* Power-Down Mode */
AD7793_SetMode(AD7793_MODE_CAL_INT_ZERO); /* Internal Zero-Scale Calibration */
AD7793_SetMode(AD7793_MODE_CAL_INT_FULL); /* Internal Full-Scale Calibration */
AD7793_SetMode(AD7793_MODE_CAL_SYS_ZERO); /* System Zero-Scale Calibration */
AD7793_SetMode(AD7793_MODE_CAL_SYS_FULL); /* System Full-Scale Calibration */

AD7793_SetClockSource(AD7793_CLK_INT);  /* Internal 64 kHz Clk not available at the CLK pin */
AD7793_SetClockSource(AD7793_CLK_INT_CO); /* Internal 64 kHz Clk available at the CLK pin */
AD7793_SetClockSource(AD7793_CLK_EXT);  /* External 64 kHz Clock */
AD7793_SetClockSource(AD7793_CLK_EXT_DIV2); /* External Clock divided by 2 */

AD7793_SetFilterUpdateRate(AD7793_RATE_NIL); /* Filter Update Rate: nil */
AD7793_SetFilterUpdateRate(AD7793_RATE_470); /* Filter Update Rate: 470Hz */
AD7793_SetFilterUpdateRate(AD7793_RATE_242); /* Filter Update Rate: 242Hz */
AD7793_SetFilterUpdateRate(AD7793_RATE_123); /* Filter Update Rate: 123Hz */
AD7793_SetFilterUpdateRate(AD7793_RATE_62); /* Filter Update Rate: 62Hz
AD7793_SetFilterUpdateRate(AD7793_RATE_50); /* Filter Update Rate: 50Hz */
AD7793_SetFilterUpdateRate(AD7793_RATE_39); /* Filter Update Rate: 39Hz */
AD7793_SetFilterUpdateRate(AD7793_RATE_33_2); /* Filter Update Rate: 33.2Hz */
AD7793_SetFilterUpdateRate(AD7793_RATE_19_6); /* Filter Update Rate: 19.6Hz 90dB (60Hz only) */ 
AD7793_SetFilterUpdateRate(AD7793_RATE_16_7_50_60); /* Filter Update Rate: 16.7Hz 80dB (50Hz only) */
AD7793_SetFilterUpdateRate(AD7793_RATE_12_5);  /* Filter Update Rate: 12.5Hz 66dB (50 and 60Hz) */
AD7793_SetFilterUpdateRate(AD7793_RATE_10);  /* Filter Update Rate: 10Hz 69dB (50 and 60Hz) */
AD7793_SetFilterUpdateRate(AD7793_RATE_8_33);  /* Filter Update Rate: 8.33Hz 70dB (50 and 60Hz) */
AD7793_SetFilterUpdateRate(AD7793_RATE_6_25);  /* Filter Update Rate: 6.25Hz 72dB (50 and 60Hz) */
AD7793_SetFilterUpdateRate(AD7793_RATE_4_17);  /* Filter Update Rate: 4.17Hz 74dB (50 and 60Hz) */

Configuration register
......................

AD7793_SetBiasVoltage(AD7793_VBIAS_GEN_DISABL); /* Bias voltage generator disabled */
AD7793_SetBiasVoltage(AD7793_VBIAS_AIN1_NEG);   /* Bias voltage generator to AIN1(-) * 
AD7793_SetBiasVoltage(AD7793_VBIAS_AIN2_NEG); /* Bias voltage generator to AIN2(-) */

AD7793_EnableBurnoutCurr();  /* Enable burnout current of AD7793 */
AD7793_DisableBurnoutCurr();  /* Disable burnout current of AD7793 */

AD7793_EnableUnipolar();  /* Enable unipolar coding of AD7793 */
AD7793_DisableBipolar();  /* Enable bipolar coding of AD7793 */

AD7793_EnableCurrBoost();  /* Enable bias voltage generator current boost of AD7793 */
AD7793_DisableCurrBoost();  /* Disable bias voltage generator current boost of AD7793 */

AD7793_AD7793_SetGain(AD7793_GAIN_1);  /* Gain of 1, In-amp not used  */ 
AD7793_AD7793_SetGain(AD7793_GAIN_2);  /* Gain of 2, In-amp not used */ 
AD7793_AD7793_SetGain(AD7793_GAIN_4);  /* Set the gain of the In-amp to 4 */ 
AD7793_AD7793_SetGain(AD7793_GAIN_8);  /* Set the gain of the In-amp to 8 */ 
AD7793_AD7793_SetGain(AD7793_GAIN_16);  /* Set the gain of the In-amp to 16 */ 
AD7793_AD7793_SetGain(AD7793_GAIN_32);  /* Set the gain of the In-amp to 32 */ 
AD7793_AD7793_SetGain(AD7793_GAIN_64);  /* Set the gain of the In-amp to 64 */ 
AD7793_AD7793_SetGain(AD7793_GAIN_128); /* Set the gain of the In-amp to 128 */ 

AD7793_SetIntReference(AD7793_REFSEL_INT);  /* Select internal reference voltage source for the ADC */
AD7793_SetIntReference(AD7793_REFSEL_EXT);  /* External voltage reference applied between REFIN(+) and REFIN(–) */

AD7793_EnableBufMode();  /* Enable buffered mode of AD7793 */
AD7793_DisableBufMode();  /* Disable buffered mode of AD7793 */

AD7793_SetChannel(AD7793_CH_AIN1P_AIN1M); /* Select channel AIN1(+) - AIN1(-) */           
AD7793_SetChannel(AD7793_CH_AIN2P_AIN2M); /* Select channel AIN2(+) - AIN2(-) */
AD7793_SetChannel(AD7793_CH_AIN3P_AIN3M); /* Select channel AIN3(+) - AIN3(-) */
AD7793_SetChannel(AD7793_CH_TEMP);    /* Temp Sensor, gain 1, Internal current reference */
AD7793_SetChannel(AD7793_CH_AVDD_MONITOR); /* AVDD Monitor, gain 1/6, internal 1.17V reference */

IO Register
===========

AD7793_SetExcitDirection(AD7793_DIR_IEXC1_IOUT1_IEXC2_IOUT2); /* IEXC1 connect to IOUT1, IEXC2 connect to IOUT2 */
AD7793_SetExcitDirection(AD7793_DIR_IEXC1_IOUT2_IEXC2_IOUT1); /* IEXC1 connect to IOUT2, IEXC2 connect to IOUT1 */
AD7793_SetExcitDirection(AD7793_DIR_IEXC1_IEXC2_IOUT1);  /* Both current sources IEXC1,2 connect to IOUT1  */
AD7793_SetExcitDirection(AD7793_DIR_IEXC1_IEXC2_IOUT2);  /* Both current sources IEXC1,2 connect to IOUT2 */
AD7793_SetExcitCurrent(AD7793_EN_IXCEN_DISABLE); /* Disable excitation current*/
AD7793_SetExcitCurrent(AD7793_EN_IXCEN_10uA);  /* Excitation Current 10uA */
AD7793_SetExcitCurrent(AD7793_EN_IXCEN_210uA);  /* Excitation Current 210uA */
AD7793_SetExcitCurrent(AD7793_EN_IXCEN_1mA);  /* Excitation Current 1mA */


Return the result of conversion
===============================

unsigned long conv = AD7793_SingleConversion(); /* Returns the result of a single conversion. */
unsigned long conv = AD7793_ContinuousReadAvg(unsigned char sampleNumber); /* Returns the average of several conversion results. */
unsigned long conv = AD7793_ContinuousSingleRead(); /* Returns a single measurement, provided continuous mesurement mode has been set up. */


Calibration
===========

void AD7793_Calibrate(unsigned char mode, unsigned char channel); /* Performs the given calibration to the specified channel. */

Values for parameter mode:
For performing an internal Zero-Scale Calibration: AD7793_MODE_CAL_INT_ZERO
For performing an internal Full-Scale Calibration: AD7793_MODE_CAL_INT_FULL
For performing a system Zero-Scale Calibration: AD7793_MODE_CAL_SYS_ZERO
For performing a Full-Scale Calibration: AD7793_MODE_CAL_SYS_FULL

Values for parameter mode:
For selecting channel 1 AIN1(+) - AIN1(-): AD7793_CH_AIN1P_AIN1M
For selecting channel 2 AIN2(+) - AIN2(-): AD7793_CH_AIN2P_AIN2M
For selecting channel 3 AIN3(+) - AIN3(-): AD7793_CH_AIN3P_AIN3M


Power-on/reset default register setting
======================================

After powering on or resetting the AD7793, the registers are set just as if all the following commands had been executed:

Mode register
...............

AD7793_SetMode(AD7793_MODE_CONT);  /* Continuous Conversion Mode */
AD7793_SetClockSource(AD7793_CLK_INT);  /* Internal 64 kHz Clk not available at the CLK pin */
AD7793_SetFilterUpdateRate(AD7793_RATE_NIL); /* Filter Update Rate: nil */

Configuration register
......................

AD7793_SetBiasVoltage(AD7793_VBIAS_GEN_DISABL); /* Bias voltage generator disabled */
AD7793_DisableBurnoutCurr();    /* Disable burnout current of AD7793 */
AD7793_DisableBipolar();    /* Enable bipolar coding of AD7793 */
AD7793_DisableCurrBoost();    /* Disable bias voltage generator current boost of AD7793 */
AD7793_AD7793_SetGain(AD7793_GAIN_128);  /* Set the gain of the In-amp to 128 */
AD7793_SetIntReference(AD7793_REFSEL_EXT); /* External voltage reference applied between REFIN(+) and REFIN(–) */
AD7793_EnableBufMode();    /* Enable buffered mode of AD7793 */
AD7793_SetChannel(AD7793_CH_AIN1P_AIN1M); /* Select channel AIN1(+) - AIN1(-) */

IO register
...........

AD7793_SetExcitDirection(AD7793_DIR_IEXC1_IOUT1_IEXC2_IOUT2); /* IEXC1 connect to IOUT1, IEXC2 connect to IOUT2 */
AD7793_SetExcitCurrent(AD7793_EN_IXCEN_DISABLE);  /* Disable excitation current*/





