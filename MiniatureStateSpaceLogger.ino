// Initializing Laibraries
#include <Wire.h>
#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>
#include "src/SparkFunBME280.h"
#include "src/ICM_20948.h"

// Digital Pin Setup
BME280 ES_Sens;         // Uses default I2C address 0x77
ICM_20948_I2C IMU_Sens; // Uses default I2C address 0x69
OpenLog LOGGER;         // Uses default I2C address 0x32
#define AD0_VAL 1

// Analog Pin Setup
int UV1 = A1;

// IMU Sensors
float Accel[3];
float Gyro[3];
float Mag[3];

// UV Sensors
float meanVal;
float UV1Val;
float sum1;
int UV1Volt;


// ES Sensors
float Temp;
float Humidity;
float Pressure;
float Altitude;

void setup()
{
    Serial.begin(9600); // Setting Baud Rate to 9600
    /*while (!Serial)
    {
    };*/ // Waiting for Serial to register

    // Sensor / I2C / Logger Setup
    Wire.begin();          // Starting I2C
    Wire.setClock(400000); // Setting I2C Clock Rate to 400kHz
    ES_Sens.beginI2C();    // Starting BME280
    LOGGER.begin();        // Starting OpenLog

    // UV Setup
    pinMode(UV1, INPUT);

    // IMU Setup and Testing
    IMU_Sens.enableDebugging();
    bool IMU_Init = false;

    // Logger Setup
    LOGGER.append("FlightData.txt");
    LOGGER.println("AX AY AZ GX GY GZ MX MY MZ T P H U1 U2 U3 SP"); // Data order Accel - Gyro - Mag - Temp - Press - Humid - (UV) 1 - 2 - 3 - 4
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
}
void loop()
{
    Serial.println();                     // Printing a blank line to Serial Monitor (For Debugging)
    Serial.println("Loop Start");         // Printing Loop Start to Serial Monitor (For Debugging)
    unsigned long millisStart = millis(); // Setting up an old time in milliseconds
    Serial.println(millisStart);         // Printing the old time in milliseconds (For Debugging)

    AGMT_Readings(&IMU_Sens);                   // Calling AGMT Function to read values
    Serial.println("AGMT Complete");            // Printing AGMT Complete to Serial Monitor (For Debugging
    UVSensors();                                // Calling UV Sensor Function to read values
    Serial.println("UV Complete");              // Printing UV Complete to Serial Monitor (For Debugging
    ESSensor();                                 // Calling Environmental Sensor Function to read values
    Serial.println("ES Complete");              // Printing ES Complete to Serial Monitor (For Debugging
    LogData();                                  // Calling Log Data Function to log data
    Serial.println("Data Logged");              // Printing Data Logged to Serial Monitor (For Debugging
    unsigned long millisEnd = millis();         // Setting the new time in milliseconds
    long dt = millisEnd - millisStart; // Finding howm much time elapsed to collect data
    Serial.println(millisEnd);         // Printing the old time in milliseconds (For Debugging)
    Serial.println(dt);
    if ((1000 - dt) > 0)
    {
        delay(1000 - dt);
        Serial.println(1000-dt);
    }
    else
    {
        Serial.println("Data Collection Time Exceeded");
    }
    // Serial.println(Iteration);
    Serial.println("Loop End");         // Printing Loop End to Serial Monitor (For Debugging)
    Serial.println();                    // Printing a blank line to Serial Monitor (For Debugging)
}

// Function Definitions

// Function to print floats with leading zeros
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

// Function to read AGMT Values
void AGMT_Readings(ICM_20948_I2C *sensor)
{
    IMU_Sens.getAGMT(); // Getting Accelerometer, Gyroscope, Magnetometer, and Temperature Readings
    if (IMU_Sens.dataReady())
    {
        // Accelerometer, Gyroscope, Magnetometer, and Temperature Readings
        Accel[0] = IMU_Sens.accX() * -1;
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

// Function to read UV Sensor Values
void UVSensors()
{
    int sensorValue;
    sum1 = 0;
    for (int i = 0; i < 100; i++)
    {
        sensorValue = analogRead(UV1);
        sum1 = sensorValue + sum1;
        delay(2);
    }
    UV1Volt = sum1 / 100; // get mean value
    Serial.print("The current UV1 Voltage is:");
    Serial.print(UV1Volt); // get a detailed calculating expression for UV index in schematic files.
    Serial.print("\n");
}

// Function to read Environmental Sensor Values
void ESSensor()
{
    // Temperature Readings
    Serial.print("Temperature: ");
    Temp = ES_Sens.readTempC();
    Serial.print(Temp);
    Serial.println(" C");
    // Humidity Readings
    Serial.print("Humidity: ");
    Humidity = ES_Sens.readFloatHumidity();
    Serial.print(Humidity);
    Serial.println(" %");
    // Pressure Readings
    Serial.print("Pressure: ");
    Pressure = ES_Sens.readFloatPressure();
    Serial.print(Pressure);
    Serial.println(" Pa");
}

void LogData()
{
    LOGGER.append("FlightData.txt");
    for (int i = 0; i < 3; i++)
    {
        LOGGER.print(Accel[i]);
        LOGGER.print(" ");
    }
    for (int i = 0; i < 3; i++)
    {
        LOGGER.print(Gyro[i]);
        LOGGER.print(" ");
    }
    for (int i = 0; i < 3; i++)
    {
        LOGGER.print(Mag[i]);
        LOGGER.print(" ");
    }
    LOGGER.print(Temp);
    LOGGER.print(" ");
    LOGGER.print(Pressure);
    LOGGER.print(" ");
    LOGGER.print(Humidity);
    LOGGER.print(" ");

    LOGGER.print(UV1Volt); // These UV values are the raw analog readings averaged over 100 samples

    LOGGER.println();
}