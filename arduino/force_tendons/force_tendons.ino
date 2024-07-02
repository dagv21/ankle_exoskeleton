//
//    FILE: HX_read_median.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: HX711 demo
//     URL: https://github.com/RobTillaart/HX711


#include "HX711.h"

HX711 scale, scale1;

uint8_t dataPin1 = 6;
uint8_t clockPin1 = 7;
uint8_t dataPin2 = 3;
uint8_t clockPin2 = 2;

uint32_t start, stop;
volatile float f1, f2;


void setup()
{
  Serial.begin(250000);

  scale.begin(dataPin1, clockPin1);
  scale1.begin(dataPin2, clockPin2);

  // TODO find a nice solution for this calibration..
  // load cell factor 20 KG
  scale.set_scale(127.15);       // TODO you need to calibrate this yourself.
  scale1.set_scale(127.15);       // TODO you need to calibrate this yourself.

  // load cell factor 5 KG
  // scale.set_scale(420.0983);
  // reset the scale to zero = 0
  scale.tare();
  scale1.tare();
}


void loop()
{
  f1 = scale.read();
  f2 = scale1.read();
  Serial.print(f1);
  Serial.print(",");
  Serial.println(f2);
}
