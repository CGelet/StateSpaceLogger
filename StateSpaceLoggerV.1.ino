#include <Wire.h>
#include "SparkFunBME280.h"
#include "SparkFunLSM6DSO.h"
#include "MLX90393.h"

MLX90393 mlx;
MLX90393::txyz data;
LSM6DSO myIMU;    // Default constructor is I2C, addr 0x6B
BME280 mySensorA; // Uses default I2C address 0x77

void setup()
{
  Serial.begin(115200);

  Wire.begin();
  mySensorA.beginI2C();
  
  if (myIMU.begin())
    Serial.println("Ready.");
  else
  {
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if (myIMU.initialize(BASIC_SETTINGS))
    Serial.println("Loaded Settings.");
  
  mlx.begin(); //Assumes I2C jumpers are GND. No DRDY pin used.
  mlx.setOverSampling(0);
  mlx.setDigitalFiltering(0);
}

void loop()
{
  /*
  Serial.print("HumidityA: ");
  Serial.print(mySensorA.readFloatHumidity(), 0);

  Serial.print(" PressureA: ");
  Serial.print(mySensorA.readFloatPressure(), 0);

  Serial.print(" TempA: ");
  // Serial.print(mySensorA.readTempC(), 2);
  Serial.print(mySensorA.readTempF(), 2);

  Serial.println();
  delay(500);
  // Get all parameters
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatAccelX(), 3);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatAccelY(), 3);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatAccelZ(), 3);

  Serial.print("\nGyroscope:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatGyroX(), 3);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatGyroY(), 3);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatGyroZ(), 3);
  */
  Serial.println();
  delay(500);
  
  mlx.readData(data); //Read the values from the sensor
  Serial.print("magX[");
  Serial.print(data.x);
  Serial.print("] magY[");
  Serial.print(data.y);
  Serial.print("] magZ[");
  Serial.print(data.z);
  delay(500);
  Serial.println();
}