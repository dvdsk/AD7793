/*
This is an example sketch for testing communication between the Arduino and the
AD7793 using the AD7793 library on the Analog Devices web site adapted for the Arduino
by Ph. Sonnet, April 8, 2019

The AD7793 and the Arduino Atmega328 R3 are connected in the following way : 

Arduino R3 pin#    AD7793 pin#     Pullup resistors
    8   GPIO      3   ~CS             50 KOhm
    9   RDY       15  DOUT/~RDY  
    12  MISO      15  DOUT/~RDY
    11  MOSI      16  DIN             50 KOhm
    10  CS        not connected   
    13  SCK       1   CLK             50 KOhm

The sketch reads the ID number of the AD7793, then uses the ADC to
measure the voltage on the analog voltage pin 13 and the temperature
inside the chip. These are the 3 functions that can be performed by the
barebone chip (not counting in the pullup resistors which must be present).  


*/
#include <AD7793.hpp>
#include <SPI.h>

float Vref = 1.17; /* AD7783 internal reference voltage */ 
AD7793 ad7793();

void setup() {
  
  Serial.begin(9600);
  while (!Serial);
  
  unsigned char answer = ad7793.Init();   /* Initializes the AD7793 and checks if the device is present*/
  Serial.print("AD7793 status = ");
  Serial.println(answer); /* Answer is 1 when the device is initialized and the ID is read and recognized */ 
  Serial.println("");
}

void loop() {
  
  ad7793.SetChannel(AD7793_CH_AVDD_MONITOR);  /* AVDD Monitor, gain 1/6, internal 1.17V reference */
  unsigned long conv = ad7793.SingleConversion();  /* Returns the result of a single conversion. */                                          
  float AVDD = ((conv - 8388608.0) / 8388608.0) * 1.17 / (1/6.0) ; /* Note: 8388608 = 2exp(23) = 0x8000000 = the output code of 0 V in bipolar mode */
  Serial.print("Analog supply voltage (AVDD) = ");
  Serial.print(AVDD, 4);
  Serial.println(" V");

  ad7793.SetChannel(AD7793_CH_TEMP); /* Temp Sensor, gain 1, Internal current reference */
  conv = ad7793.SingleConversion();  /* Returns the result of a single conversion. */
  float Temp = (((conv - 8388608.0) / 8388608.0) * 1.17 * 1000 / 0.810) - 273;  /* Sentitivity is approximately 0.81 mV/Â°K, according to AD7793 datasheet */
                                                                                /* To improve precision, it should be further calibrated by the user. */
  Serial.print("Chip temperature = ");
  Serial.print(Temp, 2);
  Serial.println(" C");
   
  delay(1000);
}
