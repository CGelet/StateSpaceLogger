#include <Wire.h>
#include "src/SparkFunBME280.h"
#include "src/ICM_20948.h"

BME280 ES_Sens;      // Uses default I2C address 0x77
ICM_20948_I2C myICM; // Uses default I2C address 0x69
#define WIRE_PORT Wire
#define AD0_VAL 1

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  };

  Wire.begin();
  ES_Sens.beginI2C();
  myICM.begin(WIRE_PORT, AD0_VAL);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

bool initialized = false;
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

  Serial.print(F("Initialization of the sensor returned: "));
  Serial.println(myICM.statusString());
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.println("Trying again...");
    delay(500);
  }
  else
  {
    Serial.println("Success!");
    initialized = true;
  }
 //success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok); 
}

void loop()
{

  // es_sensor_readings();
  // Serial.println();
  // delay(1000);
  if (myICM.dataReady())
  {
  myICM.getAGMT();
  icm_20948_readings(&myICM);
  delay(30);
  }
  else
  {
    Serial.println("Data not ready!");
    delay(500);
  }

}

void es_sensor_readings()
{
  Serial.print("Temperature: ");
  Serial.print(ES_Sens.readTempC());
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(ES_Sens.readFloatHumidity());
  Serial.println(" %");

  Serial.print("Pressure: ");
  Serial.print(ES_Sens.readFloatPressure());
  Serial.println(" Pa");

  Serial.print("Altitude: ");
  Serial.print(ES_Sens.readFloatAltitudeMeters());
  Serial.println(" m");

  Serial.println();
}

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    Serial.print(" ");
    if (val < 10000)
    {
      Serial.print("0");
    }
    if (val < 1000)
    {
      Serial.print("0");
    }
    if (val < 100)
    {
      Serial.print("0");
    }
    if (val < 10)
    {
      Serial.print("0");
    }
  }
  else
  {
    Serial.print("-");
    if (abs(val) < 10000)
    {
      Serial.print("0");
    }
    if (abs(val) < 1000)
    {
      Serial.print("0");
    }
    if (abs(val) < 100)
    {
      Serial.print("0");
    }
    if (abs(val) < 10)
    {
      Serial.print("0");
    }
  }
  Serial.print(abs(val));
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    Serial.print("-");
  }
  else
  {
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      Serial.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    Serial.print(-val, decimals);
  }
  else
  {
    Serial.print(val, decimals);
  }
}

void icm_20948_readings(ICM_20948_I2C *sensor)
{
  Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  Serial.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  Serial.print(" ]");
  Serial.println();
}

