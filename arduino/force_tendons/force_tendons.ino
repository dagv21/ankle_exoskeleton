//
//    FILE: force_tendons.ino
//  AUTHOR: Daniel Gomez - Rob Tillaart (original)
// PURPOSE: HX711 for Exoskeleton Tendons
//     URL: https://github.com/dagv21/ankle_exoskeleton.git - https://github.com/RobTillaart/HX711
// LIBRARY: 0.5.0


#include "HX711.h"

HX711 scale, scale1;

// Posterior Sensor PINs
uint8_t dataPin1 = 8; //11;
uint8_t clockPin1 = 9; //10;

// Frontal Sensor PINs
uint8_t dataPin2 = 10; //12;
uint8_t clockPin2 = 11; //13;

volatile float f_frontal, f_posterior;

double offset_frontal = 0 ;//19005;
double offset_posterior = 0; //51400;

void setup()
{
  Serial.begin(250000);

  scale1.begin(dataPin2, clockPin2, 1);
  scale.begin(dataPin1, clockPin1, 1);
  
  scale.tare();
  scale1.tare();

  offset_frontal = scale1.read_medavg(30);
  offset_posterior = scale.read_medavg(30);  
}


void loop()
{
  f_posterior = -(scale.read() - offset_posterior) * (19.6 / 75000);
  f_frontal   = -(scale1.read() - offset_frontal) * (19.6 / 73500);
  Serial.print(f_frontal);
  Serial.print(",");
  Serial.println(f_posterior);
}
