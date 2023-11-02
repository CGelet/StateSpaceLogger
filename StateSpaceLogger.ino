// Initializing Laibraries
#include <Wire.h>
#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>
#include "src/SparkFunBME280.h"
#include "src/ICM_20948.h"
// Will need UV Sensor Setup

// Analog Pin Setup
// GUVAS12SD uv(A0);

// Digital Pin Setup
BME280 ES_Sens;         // Uses default I2C address 0x77
ICM_20948_I2C IMU_Sens; // Uses default I2C address 0x69
OpenLog LOGGER;         // Uses default I2C address 0x32
#define AD0_VAL 1

// Analog Pin Setup
int SyncPin = A0;
int UV1 = A2;
// Variable Setup
int Iteration = 0;
int KeepOn;
void setup()
{
    Serial.begin(9600); // Setting Baud Rate to 115200
    while (!Serial)
    {
    }; // Waiting for Serial to register

    // Sensor / I2C Setup
    Wire.begin();          // Starting I2C
    Wire.setClock(400000); // Setting I2C Clock Rate to 400kHz
    ES_Sens.beginI2C();    // Starting BME280
    LOGGER.begin();        // Starting OpenLog
    
    // Output Setup
    pinMode(SyncPin,OUTPUT);

    // IMU Setup and Testing
    IMU_Sens.enableDebugging();
    bool IMU_Init = false;

    while (!IMU_Init)
    {
        IMU_Sens.begin(Wire, AD0_VAL); // Starting ICM20948
        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(IMU_Sens.statusString());
        if (IMU_Sens.status != ICM_20948_Stat_Ok)
        {
            Serial.println(F("Trying again..."));
            delay(500);
        }
        else
        {
            IMU_Init = true;
        }
    }
    Serial.println("IMU Sucessfully Connected!");
    /* Future DMP Setup
    // DMP Setup and Testing
    bool DMP_Init = true;
    DMP_Init =&
    */
   // Sensor Range Setup
}
void loop()
{
    Iteration = Iteration + 1;
    //IMU_Sens.getAGMT(); // Getting Accelerometer, Gyroscope, Magnetometer, and Temperature Readings
    //AGMT_Readings(&IMU_Sens); // Calling AGMT Function to read values
    UVSensors();
    delay(1000);
    if (Iteration == 20) {
        analogWrite(SyncPin,255);
        Iteration = 0;
        KeepOn = random(3,8);
    }
    if (Iteration == KeepOn){
        analogWrite(SyncPin,0);
    }
    //Serial.println(Iteration);
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

void AGMT_Readings(ICM_20948_I2C *sensor)
{
    if (IMU_Sens.dataReady())
    {
        // Accelerometer, Gyroscope, Magnetometer, and Temperature Readings
        float Accel[3];
        float Gyro[3];
        float Mag[3];
        float Temp;
        Accel[0] = IMU_Sens.accX();
        Accel[1] = IMU_Sens.accY();
        Accel[2] = IMU_Sens.accZ();
        Gyro[0] = IMU_Sens.gyrX();
        Gyro[1] = IMU_Sens.gyrY();
        Gyro[2] = IMU_Sens.gyrZ();
        Mag[0] = IMU_Sens.magX();
        Mag[1] = IMU_Sens.magY();
        Mag[2] = IMU_Sens.magZ();

        /*Serial.print("Accel: ");
        Serial.print(Accel[0]);
        Serial.print(", ");
        Serial.print(Accel[1]);
        Serial.print(", ");
        Serial.print(Accel[2]);
        Serial.print(" | Gyro: ");
        Serial.print(Gyro[0]);
        Serial.print(", ");
        Serial.print(Gyro[1]);
        Serial.print(", ");
        Serial.print(Gyro[2]);
        Serial.print(" | Mag: ");
        printFormattedFloat(Mag[0], 5, 2);
        Serial.print(", ");
        printFormattedFloat(Mag[1], 5, 2);
        Serial.print(", ");
        printFormattedFloat(Mag[2], 5, 2);
        delay(30);*/
        Serial.print("Scaled. Acc (mg) [ ");
        printFormattedFloat(Accel[0], 5, 2);
        Serial.print(", ");
        printFormattedFloat(Accel[1], 5, 2);
        Serial.print(", ");
        printFormattedFloat(Accel[2], 5, 2);
        Serial.print(" ], Gyr (DPS) [ ");
        printFormattedFloat(Gyro[0], 5, 2);
        Serial.print(", ");
        printFormattedFloat(Gyro[1], 5, 2);
        Serial.print(", ");
        printFormattedFloat(Gyro[2], 5, 2);
        Serial.print(" ], Mag (uT) [ ");
        printFormattedFloat(Mag[0], 5, 2);
        Serial.print(", ");
        printFormattedFloat(Mag[1], 5, 2);
        Serial.print(", ");
        printFormattedFloat(Mag[2], 5, 2);
        Serial.print(" ]");
        Serial.println();
    }
    else
    {
        Serial.println("Data not Ready");
        delay(500);
    }
}

void UVSensors(){
    int sensorValue;
    long sum1;
    for (int i =0; i<100; i++){
        sensorValue = analogRead(A2);
        //Serial.println(sensorValue);
        sum1 = sensorValue+sum1;
        delay(2);
    }
    long meanVal = sum1/1024;  // get mean value
    Serial.print("The current UV index is:");
    Serial.print((meanVal*1000/4.3-83)/21);// get a detailed calculating expression for UV index in schematic files.
    Serial.print("\n");
}