// Libraries needed.
//https://github.com/adafruit/Adafruit_BusIO v1.16.1
//https://github.com/adafruit/Adafruit-GFX-Library v1.11.11
//https://github.com/adafruit/Adafruit_SCD30 v1.0.11
//https://github.com/adafruit/Adafruit_SSD1306 v2.5.12
//https://github.com/adafruit/Adafruit_Sensor v1.1.14
//https://github.com/Seeed-Studio/Grove_Barometer_HP20x v1.0l0
//https://github.com/MonsieurV/ArduinoPocketGeiger v0.6.4



#include "RadiationWatch.h"
#include <Adafruit_SCD30.h>
#include <Multichannel_Gas_GMXXX.h>
#include <HP20x_dev.h>
#include <KalmanFilter.h>
#include <SD.h>
#include <SPI.h>

RadiationWatch radiationWatch;
Adafruit_SCD30 scd30;
GAS_GMXXX<TwoWire> gas;
HP20x_dev hp20x;

KalmanFilter t_filter;    // temperature filter
KalmanFilter p_filter;    // pressure filter
KalmanFilter a_filter;    // altitude filter

const byte sdCardPin = 53;
File dataFile;
unsigned long csvStartTime;

unsigned long previousMillis = 0;
const long sensorInterval = 50;  // 100 milliseconds for all sensors

void csvKeys(Print &print) {
    csvStartTime = millis();
    // Define the CSV header with separate columns for each data point
    print.println("time(ms),radiation_count,cpm,uSv/h,uSv/hError,temperature,relative_humidity,CO2,NO2,C2H5OH,VOC,CO,pressure,altitude");
}

void csvStatus(Print &print, int radiationCount = -1, float cpm = -1, float uSvh = -1, float uSvhError = -1, 
               float temperature = -999, float humidity = -999, float CO2 = -999,
               float NO2 = -999, float C2H5OH = -999, float VOC = -999, float CO = -999,
               float pressure = -999, float altitude = -999) {
    // Write the time
    print.print(millis() - csvStartTime);
    print.print(',');

    // Radiation data
    if (radiationCount != -1) {
        print.print(radiationCount);
    }
    print.print(',');

    if (cpm != -1) {
        print.print(cpm, 3);
    }
    print.print(',');

    if (uSvh != -1) {
        print.print(uSvh, 3);
    }
    print.print(',');

    if (uSvhError != -1) {
        print.print(uSvhError, 3);
    }
    print.print(',');

    // SCD30 sensor data
    if (temperature != -999) {
        print.print(temperature, 2);
    }
    print.print(',');

    if (humidity != -999) {
        print.print(humidity, 2);
    }
    print.print(',');

    if (CO2 != -999) {
        print.print(CO2, 3);
    }
    print.print(',');

    // Multichannel Gas Sensor data
    if (NO2 != -999) {
        print.print(NO2, 3);
    }
    print.print(',');

    if (C2H5OH != -999) {
        print.print(C2H5OH, 3);
    }
    print.print(',');

    if (VOC != -999) {
        print.print(VOC, 3);
    }
    print.print(',');

    if (CO != -999) {
        print.print(CO, 3);
    }
    print.print(',');

    // HP20x sensor data
    if (pressure != -999) {
        print.print(pressure, 2);
    }
    print.print(',');

    if (altitude != -999) {
        print.print(altitude, 2);
    }
    print.println("");  // End the line
}

void onRadiationPulse() {
    // Prepare SCD30 sensor data
    float temperature = -999;  // Default invalid value
    float humidity = -999;     // Default invalid value
    float CO2 = -999;          // Default invalid value

    if (scd30.dataReady()) {
        if (scd30.read()) {
            temperature = scd30.temperature;
            humidity = scd30.relative_humidity;
            CO2 = scd30.CO2;
        }
    }

    // Prepare Multichannel Gas Sensor data (fetch latest values)
    float NO2 = gas.measure_NO2();
    float C2H5OH = gas.measure_C2H5OH();
    float VOC = gas.measure_VOC();
    float CO = gas.measure_CO();

    // Prepare HP20x sensor data
    float pressure = p_filter.Filter(hp20x.ReadPressure() / 100.0);
    float altitude = a_filter.Filter(hp20x.ReadAltitude() / 100.0);

    // Log data with all sensor values
    csvStatus(dataFile, radiationWatch.radiationCount(), radiationWatch.cpm(), radiationWatch.uSvh(), radiationWatch.uSvhError(),
              temperature, humidity, CO2, NO2, C2H5OH, VOC, CO, pressure, altitude);
    dataFile.flush();

    // Optionally, log to Serial if connected
    if (Serial) {
        csvStatus(Serial, radiationWatch.radiationCount(), radiationWatch.cpm(), radiationWatch.uSvh(), radiationWatch.uSvhError(),
                  temperature, humidity, CO2, NO2, C2H5OH, VOC, CO, pressure, altitude);
    }
}

void setup() {
    // Initialize SD card
    if (!SD.begin(sdCardPin)) {
        // Handle SD card initialization failure
        return;
    }
    dataFile = SD.open("data.csv", FILE_WRITE);
    if (!dataFile) {
        // Handle file opening failure
        return;
    }

    // Initialize RadiationWatch
    radiationWatch.setup();

    // Initialize SCD30 sensor
    if (!scd30.begin()) {
        // Handle SCD30 initialization failure
        return;
    }

    // Initialize Multichannel Gas Sensor
    gas.begin(Wire, 0x08);  // Use the correct I2C address

    // Initialize HP20x sensor
    delay(150);
    HP20x.begin();
    delay(100);

    // Log CSV headers to SD card
    csvKeys(dataFile);

    // Optionally, log CSV headers to Serial if connected
    if (Serial) {
        Serial.begin(115200);
        csvKeys(Serial);
    }

    // Register the radiation pulse callback
    radiationWatch.registerRadiationCallback(&onRadiationPulse);
}

void loop() {
    radiationWatch.loop();  // Handle radiation sensor events

    unsigned long currentMillis = millis();

    // Poll and log data every 100 ms
    if (currentMillis - previousMillis >= sensorInterval) {
        previousMillis = currentMillis;

        // Check SCD30 data
        float temperature = -999;
        float humidity = -999;
        float CO2 = -999;

        if (scd30.dataReady()) {
            if (scd30.read()) {
                temperature = scd30.temperature;
                humidity = scd30.relative_humidity;
                CO2 = scd30.CO2;
            }
        }

        // Poll Multichannel Gas Sensor data
        float NO2 = gas.measure_NO2();
        float C2H5OH = gas.measure_C2H5OH();
        float VOC = gas.measure_VOC();
        float CO = gas.measure_CO();

        // Poll HP20x sensor data
        float pressure = p_filter.Filter(hp20x.ReadPressure() / 100.0);
        float altitude = a_filter.Filter(hp20x.ReadAltitude() / 100.0);

        // Log all sensor data to SD card
        csvStatus(dataFile, -1, -1, -1, -1, temperature, humidity, CO2, NO2, C2H5OH, VOC, CO, pressure, altitude);
        dataFile.flush();

        // Optionally, log to Serial if connected
        if (Serial) {
            csvStatus(Serial, -1, -1, -1, -1, temperature, humidity, CO2, NO2, C2H5OH, VOC, CO, pressure, altitude);
        }
    }
}
