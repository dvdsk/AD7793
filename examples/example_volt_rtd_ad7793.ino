/*
 This example shows how to use an AD7793 converter to perform successively two measurements:
 1) a single voltage reading in the range from 0 to 0.58 V on AD7793 channel 2 (0.58 = 1.17V AD7793 internal reference divided by 2);
 2) a repeated temperature reading using a 4-wire PT100 RTD and a ratiometric measurement circuit on AD7793 channel 1 (up to 144 C; 
 for higher temperature or to use PT1000, simply lower the gain). 
 
 Using RTDs in a ratiometric measurement has the advantage in that it eliminates sources of error such as the accuracy and drift of 
 the excitation current source (provided by the AD7793) and the influence of the RTC sensor lead resistance.
 The basics for 4-wire RTD ratiometric measurement and calculation using an ADC can be found at: 
 http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/platinum-rtd-sensors/amplifier-circuit-schematics  
  
 The schematics and part specifications used in this example are according to Figure 7 in 
 "Analog Front-End Design Considerations for RTD Ratiometric Temperature Measurements" by B. Zhang and A. Buda:
 https://www.analog.com/en/analog-dialogue/articles/afe-design-considerations-rtd-ratiometric.html
 However, in the present example, reference resistor is 4.99 Kohm, 0.1%, 10 ppm/C. 
 
 For additional details about RTD signal conditioning, see "Completely Integrated 4-Wire RTD Measurement System 
 "Using a Low Power, Precision, 24-Bit, Sigma-Delta ADC circuit", Note CN-0381 from Analog Devices: 
 https://www.analog.com/media/en/reference-design-documentation/reference-designs/CN0381.pdf  "

 The AD7793 and the Arduino Atmega328 R3 are connected in the following way : 

 Arduino R3 pin#   AD7793 pin#    Pullup resistors
     8   GPIO      3   ~CS             50 KOhm
     9   RDY       15  DOUT/~RDY  
     12  MISO      15  DOUT/~RDY
     11  MOSI      16  DIN             50 KOhm
     10  CS        not connected   
     13  SCK       1   CLK             50 KOhm

 Ph. Sonnet, May 1, 2019
 */
 
#include <AD7793.hpp>
#include <SPI.h>

unsigned long conv; /* The 24 bit output code resulting from a conversion by the ADC and read from the data register */
float Vref; /* The external reference voltage applied between pins REFIN(+) and REFIN(-)and resulting from the excitation current flowing through the reference resistor  */
float GAIN; /* Gain of the AD7793 unternal instrumentation amplifier */
float V; /* The voltage read on the analog input channel 2 (should be between -0.57 +0.57 when gain is set to 1) */
float RREF = 4990.0; /* The reference resistor: here, 4.99 Kohm, 0.1%, 10ppm/C */
float RRTD; /* The measured resistance of the RTD */ 
float temp; /* The temperature read on the analog input channel 1 */
float R0 = 100.0; /* RTD resistance at 0C */
float A = 3.9083E-3; /* Coefficient for t in the Callender-Van Dusen equation for temperature > 0C */
float B = -5.775E-7; /* Coefficient for t squared in the Callender-Van Dusen equation for temperature > 0C */

AD7793 ad7793();

void setup() {
    Serial.begin(9600);      
    
    unsigned char answer = ad7793.Init(); /* Initialize AD7793 and check if the device is present */
    Serial.print("AD7793 status = ");
    if (answer == 1) {
      Serial.println("OK");
    }
    else {
      Serial.println("Device is not present");
    }
    
    ad7793.Reset(); /* Sends 32 consecutive 1's on SPI in order to reset the part */       
    ad7793.SetChannel(AD7793_CH_AIN2P_AIN2M); /* Selects channel 2 of AD7793 */
    ad7793.SetGain(AD7793_GAIN_1); /* Sets the gain to 1 */
    ad7793.SetIntReference(AD7793_REFSEL_INT); /* Sets the reference source for the ADC. */
    conv = ad7793.SingleConversion();
    
    Vref = 1.17; /* This is the internal reference voltage provided by the AD7793, expressed in volt */
    GAIN = 1.0; 
    V = 1000 * (conv - 8388608.0) / (8388608.0 * GAIN/Vref); /* Computes the read voltage from the conversion code, in mV */
    Serial.print("Voltage (mV) = ");
    Serial.println(V);

    Serial.println("Temperature (Celsius)");
    Serial.println("====================");
        
    ad7793.Reset(); /* Sends 32 consecutive 1's on SPI in order to reset the part */   
    ad7793.SetChannel(AD7793_CH_AIN1P_AIN1M); /* Selects channel 1 of AD7793 */
    ad7793.SetGain(AD7793_GAIN_32); /* Sets the gain to 32 */
    GAIN = 32.0;
    
    /* As the gain of the internal instrumentation amplifier has been changed, Analog Devices recommends performing a calibration  */
    ad7793.Calibrate(AD7793_MODE_CAL_INT_ZERO, AD7793_CH_AIN1P_AIN1M); /* Performs Internal Zero calibration to the specified channel. */
    ad7793.Calibrate(AD7793_MODE_CAL_INT_FULL, AD7793_CH_AIN1P_AIN1M); /* Performs Internal Full Calibration to the specified channel. */
    
    ad7793.SetIntReference(AD7793_REFSEL_EXT); /* Sets the voltage reference source for the ADC */
    ad7793.SetExcitDirection(AD7793_DIR_IEXC1_IOUT1_IEXC2_IOUT2); /*Sets the direction of the internal excitation current source */

    ad7793.SetMode(AD7793_MODE_CONT);  /* Continuous Conversion Mode */
    ad7793.SetExcitCurrent(AD7793_EN_IXCEN_210uA);/* Sets the current of the AD7793 internal excitation current source */
    delay(1000); /* Allows excitation current to settle before starting conversions*/    
}

void loop() {
    conv = ad7793.ContinuousSingleRead();
    RRTD = RREF * (conv - 8388608.0) / (8388608.0 * GAIN); /* Computes the RTD resistance from the conversion code */
    temp = (sqrt(sq(A) - 4*B*(1.0 - RRTD/R0))-A)/(2*B);    /* Callender-Van Dusen equation temp > 0C */
    Serial.println(temp);
    delay(1000);
    }

