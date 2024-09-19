//
//    FILE: force_tendons.ino
//  AUTHOR: Daniel Gomez - Rob Tillaart (original)
// PURPOSE: HX711 for Exoskeleton Tendons
//     URL: https://github.com/dagv21/ankle_exoskeleton.git - https://github.com/RobTillaart/HX711
// LIBRARY: 0.5.0


#include "HX711.h"

HX711 scale, scale1;

// Posterior Sensor PINs
uint8_t dataPin1 = 11;
uint8_t clockPin1 = 10;

// Frontal Sensor PINs
uint8_t dataPin2 = 12;
uint8_t clockPin2 = 13;

volatile float f_frontal, f_posterior;

double offset_frontal = 19005;
double offset_posterior = 51400;

void setup()
{
  Serial.begin(250000);

  scale1.begin(dataPin2, clockPin2, 1);
  scale.begin(dataPin1, clockPin1, 1);
  
  scale.tare();
  scale1.tare();

  offset_frontal = scale1.read_medavg(15);
  offset_posterior = scale.read_medavg(15);  
}


void loop()
{
  f_posterior = -(scale.read() - offset_posterior)*15.0/124000.0;
  f_frontal = -(scale1.read() - offset_frontal)*15.0/182000.0;
  Serial.print(f_frontal);
  Serial.print(",");
  Serial.println(f_posterior);
}