#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* 
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
*/

// Define configuration registers
#define BNO055_PAGE_ID 0x07
#define BNO055_ACCEL_CONFIG 0x08
#define BNO055_MAG_CONFIG 0x09
#define BNO055_GYRO_CONFIG_0 0x0A
#define BNO055_GYRO_CONFIG_1 0x0B



/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  Serial.begin(250000);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("-1,-1,-1,-1");
    while (1);
  }

  bno.setExtCrystalUse(true);
  
  // Configure sensor settings
  bno.setMode(OPERATION_MODE_NDOF);
  // Normal Power Mode
  bno.write8(Adafruit_BNO055::BNO055_PWR_MODE_ADDR, 0x00);

  bno.write8(BNO055_PAGE_ID, 0x01); //Change Page
  delay(10);
  // Set gyroscope range (125DPS) AND bandwidth (47Hz)
  bno.write8(BNO055_GYRO_CONFIG_0, 0x1C);
  delay(10);
  // Set gyroscope normal mode
  bno.write8(BNO055_GYRO_CONFIG_1, 0x00);
  delay(10);
  // Set accelerometer range (2G) bandwidth (31.25Hz) and normal mode
  bno.write8(BNO055_ACCEL_CONFIG, 0x08);
  delay(10);
  // Set magnetometer rate 20 Hz, operating mode (enhaced regular), power mode (forced mode)
  bno.write8(BNO055_MAG_CONFIG, 0x75);
  delay(10);
  bno.write8(BNO055_PAGE_ID, 0x00); //Change Page
  delay(10);
  
  /* Wait for the sensor to be fully calibrated */
  uint8_t system, gyro, accel, mag = 0;
  while (true) {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print(system, DEC);
    Serial.print(",");
    Serial.print(gyro, DEC);
    Serial.print(",");
    Serial.print(accel, DEC);
    Serial.print(",");
    Serial.println(mag, DEC);
    delay(500);
    if (system == 3 && gyro == 3 && accel == 3 && mag == 3) {
      break;
    }
    
  }

  
}

void loop() {

  sensors_event_t orientationData, angVelocityData, linearAccelData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  //bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  // Read quaternion data
  imu::Quaternion quat = bno.getQuat();

  // Build and send the CSV data string
  String dataString = String(orientationData.orientation.x) + "," +
                      String(orientationData.orientation.y) + "," +
                      String(orientationData.orientation.z) + "," +
                      String(angVelocityData.gyro.x) + "," +
                      String(angVelocityData.gyro.y) + "," +
                      String(angVelocityData.gyro.z) + "," +
                      //String(linearAccelData.acceleration.x) + "," +
                      //String(linearAccelData.acceleration.y) + "," +
                      //String(linearAccelData.acceleration.z) + "," +
                      String(accelerometerData.acceleration.x) + "," +
                      String(accelerometerData.acceleration.y) + "," +
                      String(accelerometerData.acceleration.z) + "," +
                      String(gravityData.acceleration.x) + "," +
                      String(gravityData.acceleration.y) + "," +
                      String(gravityData.acceleration.z) + "," +
                      String(quat.x()) + "," +
                      String(quat.y()) + "," +
                      String(quat.z()) + "," +
                      String(quat.w());

  Serial.println(dataString);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}