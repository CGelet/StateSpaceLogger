#include "electricui.h"
#include <eui_config.h>
#include <eui_macro.h>
#include <eui_types.h>
#include <eui_utilities.h>
#include <Wire.h>
#include "src/SparkFunBME280.h"
#include "src/ICM_20948.h"

BME280 ES_Sens;      // Uses default I2C address 0x77
ICM_20948_I2C myICM; // Uses default I2C address 0x69
#define WIRE_PORT Wire
#define AD0_VAL 1

// EUI Initialization
eui_interface_t serial_comms = EUI_INTERFACE( &serial_write ); // Create an interface for Serial
double q1;
double q2;
double q3;
double q0;
eui_message_t tracked_variables[] =
{
  EUI_DOUBLE("Q1", q1),
  EUI_DOUBLE("Q2", q2),
  EUI_DOUBLE("Q3", q3),
  EUI_DOUBLE("Q0", q0),
};

void setup()
{
  // Serial and EUI
  Serial.begin(115200);
  while (!Serial)
  {
  };

  eui_setup_interface( &serial_comms );
  EUI_TRACK( tracked_variables );
  eui_setup_identifier( "MRT SSL", 13 );

  // Others
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
  // DMP Enabling
  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  if (success)
  {
#ifndef QUAT_ANIMATION
    Serial.println(F("DMP enabled!"));
#endif
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    while (1)
      ; // Do nothing more
  }
}

void loop()
{
  int old_time = 0;
  int new_time = millis();
  int dt = new_time - old_time;
  serial_rx_handler();
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
  serial_rx_handler();
  dmp_readings();
}
// BME Functions
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

// Print Functions

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

// ICM Functions

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

// DMP Functions

void dmp_readings()
{
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
#ifndef QUAT_ANIMATION

     Serial.print(F("Q0:"));
      Serial.print(q0, 3);
     Serial.print(F("Q1:"));
      Serial.print(q1, 3);
      Serial.print(F(" Q2:"));
      Serial.print(q2, 3);
      Serial.print(F(" Q3:"));
      Serial.print(q3, 3);
      Serial.print(F(" Accuracy:"));
      Serial.println(data.Quat9.Data.Accuracy);
#else
      // Output the Quaternion data in the format expected by ZaneL's Node.js Quaternion animation tool
      Serial.print(F("{\"quat_w\":"));
      Serial.print(q0, 3);
      Serial.print(F(", \"quat_x\":"));
      Serial.print(q1, 3);
      Serial.print(F(", \"quat_y\":"));
      Serial.print(q2, 3);
      Serial.print(F(", \"quat_z\":"));
      Serial.print(q3, 3);
      Serial.println(F("}"));
#endif
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}

// EUI Functions

void serial_rx_handler()
{
  // While we have data, we will pass those bytes to the ElectricUI parser
  while( Serial.available() > 0 )  
  {  
    eui_parse( Serial.read(), &serial_comms );  // Ingest a byte
  }
}

void serial_write( uint8_t *data,  uint16_t len )
{
  Serial.write( data, len ); //output on the main serial port
}
