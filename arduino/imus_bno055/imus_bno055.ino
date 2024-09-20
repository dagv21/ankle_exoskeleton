#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* 
   Connections
   ===========
   Check the I2C ports and adjust Wire
   Library Adafruit_BNO055 version 1.6.3 (write8 must be changed in the .h file from private to public)
*/

// Define configuration registers
#define BNO055_PAGE_ID 0x07
#define BNO055_ACCEL_CONFIG 0x08
#define BNO055_MAG_CONFIG 0x09
#define BNO055_GYRO_CONFIG_0 0x0A
#define BNO055_GYRO_CONFIG_1 0x0B


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1;

Adafruit_BNO055 bno_shank = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055 bno_foot = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  Serial.begin(1000000);

  /* Initialise the sensor */
  if (!bno_foot.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("-28,-28,-28,-28");
    while (1);
  }
  /* Initialise the sensor */
  if (!bno_shank.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("-29,-29,-29,-29");
    while (1);
  }

  bno_foot.setExtCrystalUse(true);
  bno_shank.setExtCrystalUse(true);
  
  // Configure sensor settings
  bno_foot.setMode(OPERATION_MODE_NDOF);
  bno_shank.setMode(OPERATION_MODE_NDOF);
  // Normal Power Mode
  bno_foot.write8(Adafruit_BNO055::BNO055_PWR_MODE_ADDR, 0x00);
  bno_shank.write8(Adafruit_BNO055::BNO055_PWR_MODE_ADDR, 0x00);

  bno_foot.write8(BNO055_PAGE_ID, 0x01); //Change Page
  bno_shank.write8(BNO055_PAGE_ID, 0x01); //Change Page
  delay(10);
  // Set gyroscope range (125DPS) AND bandwidth (47Hz)
  bno_foot.write8(BNO055_GYRO_CONFIG_0, 0x1C);
  bno_shank.write8(BNO055_GYRO_CONFIG_0, 0x1C);
  delay(10);
  // Set gyroscope normal mode
  bno_foot.write8(BNO055_GYRO_CONFIG_1, 0x00);
  bno_shank.write8(BNO055_GYRO_CONFIG_1, 0x00);
  delay(10);
  // Set accelerometer range (2G) bandwidth (31.25Hz) and normal mode
  bno_foot.write8(BNO055_ACCEL_CONFIG, 0x08);
  bno_shank.write8(BNO055_ACCEL_CONFIG, 0x08);
  delay(10);
  // Set magnetometer rate 20 Hz, operating mode (enhaced regular), power mode (forced mode)
  bno_foot.write8(BNO055_MAG_CONFIG, 0x75);
  bno_shank.write8(BNO055_MAG_CONFIG, 0x75);
  delay(10);
  bno_foot.write8(BNO055_PAGE_ID, 0x00); //Change Page
  bno_shank.write8(BNO055_PAGE_ID, 0x00); //Change Page
  delay(10);
  
  /* Wait for the sensor to be fully calibrated */
  uint8_t systemF, gyroF, accelF, magF = 0;
  uint8_t systemS, gyroS, accelS, magS = 0;
  while (true) {
    bno_foot.getCalibration(&systemF, &gyroF, &accelF, &magF);
    bno_shank.getCalibration(&systemS, &gyroS, &accelS, &magS);
    Serial.print(systemF, DEC);
    Serial.print(",");
    Serial.print(gyroF, DEC);
    Serial.print(",");
    Serial.print(accelF, DEC);
    Serial.print(",");
    Serial.print(magF, DEC);
    Serial.print(",");
    Serial.print(systemS, DEC);
    Serial.print(",");
    Serial.print(gyroS, DEC);
    Serial.print(",");
    Serial.print(accelS, DEC);
    Serial.print(",");
    Serial.println(magS, DEC);
    delay(500);
    if (systemF == 3 && gyroF == 3 && accelF == 3 && magF == 3 && systemS == 3 && gyroS == 3 && accelS == 3 && magS == 3) {
      break;
    }
    
  }

  
}

void loop() {

  sensors_event_t orientationDataF, angVelocityDataF, linearAccelDataF, accelerometerDataF, gravityDataF, orientationDataS, angVelocityDataS, linearAccelDataS, accelerometerDataS, gravityDataS;
  //bno_foot.getEvent(&orientationDataF, Adafruit_BNO055::VECTOR_EULER);
  bno_foot.getEvent(&angVelocityDataF, Adafruit_BNO055::VECTOR_GYROSCOPE);
  //bno_foot.getEvent(&linearAccelDataF, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno_foot.getEvent(&accelerometerDataF, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno_foot.getEvent(&gravityDataF, Adafruit_BNO055::VECTOR_GRAVITY);

  // Read quaternion data
  imu::Quaternion quatF = bno_foot.getQuat();

  //bno_shank.getEvent(&orientationDataS, Adafruit_BNO055::VECTOR_EULER);
  bno_shank.getEvent(&angVelocityDataS, Adafruit_BNO055::VECTOR_GYROSCOPE);
  //bno_shank.getEvent(&linearAccelDataS, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno_shank.getEvent(&accelerometerDataS, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno_shank.getEvent(&gravityDataS, Adafruit_BNO055::VECTOR_GRAVITY);

  // Read quaternion data
  imu::Quaternion quatS = bno_shank.getQuat();

  // Build and send the CSV data string
  String dataString = //String(orientationDataF.orientation.x) + "," +
                      //String(orientationDataF.orientation.y) + "," +
                      //String(orientationDataF.orientation.z) + "," +
                      String(angVelocityDataF.gyro.x) + "," +
                      String(angVelocityDataF.gyro.y) + "," +
                      String(angVelocityDataF.gyro.z) + "," +
                      //String(linearAccelDataF.acceleration.x) + "," +
                      //String(linearAccelDataF.acceleration.y) + "," +
                      //String(linearAccelDataF.acceleration.z) + "," +
                      String(accelerometerDataF.acceleration.x) + "," +
                      String(accelerometerDataF.acceleration.y) + "," +
                      String(accelerometerDataF.acceleration.z) + "," +
                      String(gravityDataF.acceleration.x) + "," +
                      String(gravityDataF.acceleration.y) + "," +
                      String(gravityDataF.acceleration.z) + "," +
                      String(quatF.x()) + "," +
                      String(quatF.y()) + "," +
                      String(quatF.z()) + "," +
                      String(quatF.w()) + "," +
                      //String(orientationDataS.orientation.x) + "," +
                      //String(orientationDataS.orientation.y) + "," +
                      //String(orientationDataS.orientation.z) + "," +
                      String(angVelocityDataS.gyro.x) + "," +
                      String(angVelocityDataS.gyro.y) + "," +
                      String(angVelocityDataS.gyro.z) + "," +
                      //String(linearAccelDataS.acceleration.x) + "," +
                      //String(linearAccelDataS.acceleration.y) + "," +
                      //String(linearAccelDataS.acceleration.z) + "," +
                      String(accelerometerDataS.acceleration.x) + "," +
                      String(accelerometerDataS.acceleration.y) + "," +
                      String(accelerometerDataS.acceleration.z) + "," +
                      String(gravityDataS.acceleration.x) + "," +
                      String(gravityDataS.acceleration.y) + "," +
                      String(gravityDataS.acceleration.z) + "," +
                      String(quatS.x()) + "," +
                      String(quatS.y()) + "," +
                      String(quatS.z()) + "," +
                      String(quatS.w());

  Serial.println(dataString);

  //delay(BNO055_SAMPLERATE_DELAY_MS);
}