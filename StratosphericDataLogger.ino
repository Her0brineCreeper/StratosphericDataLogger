#include <Wire.h>
#include "esp_sleep.h"
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SI1145.h"
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include "SparkFun_SCD4x_Arduino_Library.h"
#include "MS5611.h"
#include <FastLED.h>

#define SEALEVELPRESSURE_HPA (1013.25)

//Define all I2C addreses
#define BME280_ADDR 0x76
#define GY280_ADDR 0x77
#define SI1145_ADDR 0x60
#define MPU6050_ADDR 0x68
#define SCD41_ADDR 0x62
#define MS5611_ADDR 0x77

//Define custom UART pins
#define GNSS_RX D6
#define GNSS_TX D7

//Pull-up to change addres to 0x77 for the GY-280
#define outputPin D10

//Define custom I2C pins
#define SDA_PIN D12  // Change as needed
#define SCL_PIN D11  // Change as needed

//Define SPI communication pins
#define SD_CS D9

//Define filename
#define FILENAME "/logdata.txt"

//Define for RGB LED
#define NUM_LEDS 1         //Number of RGB LED beads
#define DATA_PIN D8        //The pin for controlling RGB LED
#define LED_TYPE NEOPIXEL  //RGB LED strip type

CRGB leds[NUM_LEDS];  //Instantiate RGB LED

File dataFile;

TinyGPSPlus gps;
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_MPU6050 mpu;
SCD4x scd41;
MS5611 ms5611(0x77, &Wire1);

HardwareSerial GNSS(2);

bool bme1Present = false;
bool bme2Present = false;
bool uvPresent = false;
bool mpuPresent = false;
bool scd41Present = false;
bool ms5611Present = false;
bool sdPresent = false;

bool checkI2CDevice(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return (bus.endTransmission() == 0);
}

void blinkLED() {
  leds[0] = CRGB::HotPink;
  FastLED.setBrightness(40);
  FastLED.show();
  delay(500);
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);
}

void SD_CheckLED(CRGB color) {
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(100);
  leds[0] = color;
  FastLED.setBrightness(50);
  FastLED.show();
  delay(1000);
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(100);
}

void setup() {
  Serial.begin(115200);

  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, HIGH);

  Wire.begin();
  Wire1.begin(SDA_PIN, SCL_PIN);  //Second I2C buss

  FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, NUM_LEDS);  //Initialize RGB LED
  leds[0] = CRGB::Black;
  FastLED.show();
  FastLED.clear(true);

  GNSS.begin(115200, SERIAL_8N1, GNSS_RX, GNSS_TX);
  Serial.println("GNSS module initialized.");

  if (checkI2CDevice(Wire, BME280_ADDR) && bme1.begin(BME280_ADDR)) {
    Serial.println("BME280 detected.");
    blinkLED();
    bme1Present = true;
  }

  if (checkI2CDevice(Wire, GY280_ADDR) && bme2.begin(GY280_ADDR)) {
    Serial.println("GY-280 detected.");
    blinkLED();
    bme2Present = true;
  }

  if (checkI2CDevice(Wire, SI1145_ADDR) && uv.begin()) {
    Serial.println("SI1145 detected.");
    blinkLED();
    uvPresent = true;
  }

  if (checkI2CDevice(Wire, MPU6050_ADDR) && mpu.begin()) {
    Serial.println("MPU6050 detected.");
    blinkLED();
    mpuPresent = true;
  }

  if (checkI2CDevice(Wire, SCD41_ADDR) && scd41.begin()) {
    Serial.println("SCD41 detected.");
    scd41.startPeriodicMeasurement();
    blinkLED();
    scd41Present = true;
  }

  if (checkI2CDevice(Wire1, MS5611_ADDR) && ms5611.begin()) {
    Serial.println("MS5611 detected.");
    blinkLED();
    ms5611Present = true;
  }

  SPI.begin(18, 19, 23, SD_CS);  // Set SPI pins: SCK, MISO, MOSI, CS
  if (!SD.begin(SD_CS)) {
    sdPresent = false;
    leds[0] = CRGB::Red;
    FastLED.setBrightness(40);
    FastLED.show();
    Serial.println("Card Mount Failed");
    delay(3000);
  } else {
    delay(100);
    sdPresent = true;
    leds[0] = CRGB::Green;  //LED shows green light
    FastLED.setBrightness(40);
    FastLED.show();
    Serial.println("Card mount works!");
    delay(3000);
  }

  // Try opening file for read first to check if it exists
  if (!SD.exists(FILENAME)) {
    Serial.println("File does not exist. Creating and writing header...");
    dataFile = SD.open(FILENAME, FILE_WRITE);
    if (dataFile) {
      dataFile.println("Time(ms),GPS_Date(DD/MM/YYYY),GPS_Time(HH:MM:SS),BME_T(C),BME_H(%),BME_P(hPa),GY_T(C),GY_H(%),GY_P(hPa),Vis,IR,UV,Ax,Ay,Az,Gx,Gy,Gz,MPU_T(C),CO2(ppm),SCD_T(C),SCD_H(%),MS_T(C),MS_P(hPa),Lat,Lon,Alt,Sats,HDOP");
      dataFile.flush();  // Force header to be written
      dataFile.close();
      Serial.println("Header written.");
    } else {
      Serial.println("Failed to create file. Disabling SD logging.");
      sdPresent = false;
    }
  }
}
void loop() {
  // Start CSV line with timestamp
  unsigned long timeStamp = millis();
  String csvLine = String(timeStamp);

  String gpsDate = ",,";
  if (gps.date.isValid() && gps.time.isValid()) {
    gpsDate = String(",") + String(gps.date.day()) + "/" + String(gps.date.month()) + "/" + String(gps.date.year());
    gpsDate += "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
  }
  csvLine += gpsDate;


  while (GNSS.available()) {
    gps.encode(GNSS.read());
  }

  Serial.println("-----------\n");

  // ---------- BME280 ----------
  Serial.println("Sensor BME280");
  if (checkI2CDevice(Wire, BME280_ADDR)) {
    if (!bme1Present && bme1.begin(BME280_ADDR)) {
      Serial.println("BME280 reconnected.");
      bme1Present = true;
    }
    if (bme1Present) {
      printBME280(bme1);
      csvLine += "," + String(bme1.readTemperature(), 2);
      csvLine += "," + String(bme1.readHumidity(), 2);
      csvLine += "," + String(bme1.readPressure() / 100.0F, 2);
    } else {
      csvLine += ",,,";
    }
  } else {
    if (bme1Present) Serial.println("BME280 disconnected!");
    bme1Present = false;
    Serial.println("Skipping BME280 readings... Not connected");
    csvLine += ",,,";
  }

  Serial.println("-----------\n");

  // ---------- GY-280 ----------
  Serial.println("Sensor GY-280");
  if (checkI2CDevice(Wire, GY280_ADDR)) {
    if (!bme2Present && bme2.begin(GY280_ADDR)) {
      Serial.println("GY-280 reconnected.");
      bme2Present = true;
    }
    if (bme2Present) {
      printBME280(bme2);
      csvLine += "," + String(bme2.readTemperature(), 2);
      csvLine += "," + String(bme2.readHumidity(), 2);
      csvLine += "," + String(bme2.readPressure() / 100.0F, 2);
    } else {
      csvLine += ",,,";
    }
  } else {
    if (bme2Present) Serial.println("GY-280 disconnected!");
    bme2Present = false;
    Serial.println("Skipping GY-280 readings... Not connected");
    csvLine += ",,,";
  }

  Serial.println("-----------\n");

  // ---------- SI1145 ----------
  Serial.println("Sensor SI1145");
  if (checkI2CDevice(Wire, SI1145_ADDR)) {
    if (!uvPresent && uv.begin()) {
      Serial.println("SI1145 reconnected.");
      uvPresent = true;
    }
    if (uvPresent) {
      printSI1145(uv);
      csvLine += "," + String(uv.readVisible());
      csvLine += "," + String(uv.readIR());
      csvLine += "," + String(uv.readUV() / 100.0, 2);
    } else {
      csvLine += ",,,";
    }
  } else {
    if (uvPresent) Serial.println("SI1145 disconnected!");
    uvPresent = false;
    Serial.println("Skipping SI1145 readings... Not connected");
    csvLine += ",,,";
  }

  Serial.println("-----------\n");

  // ---------- MPU6050 ----------
  Serial.println("Sensor MPU6050");
  if (checkI2CDevice(Wire, MPU6050_ADDR)) {
    if (!mpuPresent && mpu.begin()) {
      Serial.println("MPU6050 reconnected.");
      mpuPresent = true;
    }
    if (mpuPresent) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      printMPU6050(mpu);
      csvLine += "," + String(a.acceleration.x, 2);
      csvLine += "," + String(a.acceleration.y, 2);
      csvLine += "," + String(a.acceleration.z, 2);
      csvLine += "," + String(g.gyro.x, 2);
      csvLine += "," + String(g.gyro.y, 2);
      csvLine += "," + String(g.gyro.z, 2);
      csvLine += "," + String(temp.temperature, 2);
    } else {
      csvLine += ",,,,,,,";
    }
  } else {
    if (mpuPresent) Serial.println("MPU6050 disconnected!");
    mpuPresent = false;
    Serial.println("Skipping MPU6050 readings... Not connected");
    csvLine += ",,,,,,,";
  }

  Serial.println("-----------\n");

  // ---------- SCD41 ----------
  Serial.println("Sensor SCD41");
  if (checkI2CDevice(Wire, SCD41_ADDR)) {
    if (!scd41Present && scd41.begin()) {
      Serial.println("SCD41 reconnected.");
      scd41Present = true;
    }
    if (scd41Present && scd41.readMeasurement()) {
      printSCD41();
      csvLine += "," + String(scd41.getCO2());
      csvLine += "," + String(scd41.getTemperature(), 1);
      csvLine += "," + String(scd41.getHumidity(), 1);
    } else {
      csvLine += ",,,";
    }
  } else {
    if (scd41Present) Serial.println("SCD41 disconnected!");
    scd41Present = false;
    Serial.println("Skipping SCD41 readings... Not connected");
    csvLine += ",,,";
  }

  Serial.println("-----------\n");

  // ---------- MS5611 ----------
  Serial.println("Sensor MS5611");
  if (checkI2CDevice(Wire1, MS5611_ADDR)) {
    if (!ms5611Present && ms5611.begin()) {
      Serial.println("MS5611 reconnected.");
      ms5611Present = true;
    }
    if (ms5611Present) {
      ms5611.read();
      printMS5611();
      csvLine += "," + String(ms5611.getTemperature(), 2);
      csvLine += "," + String(ms5611.getPressure(), 2);
    } else {
      csvLine += ",,";
    }
  } else {
    if (ms5611Present) Serial.println("MS5611 disconnected!");
    ms5611Present = false;
    Serial.println("Skipping MS5611 readings... Not connected");
    csvLine += ",,";
  }

  Serial.println("-----------\n");

  // ---------- GNSS ----------
  Serial.println("Sensor GNSS");
  printGPS();
  if (gps.location.isValid()) {
    csvLine += "," + String(gps.location.lat(), 6);
    csvLine += "," + String(gps.location.lng(), 6);
    csvLine += "," + String(gps.altitude.meters(), 2);
    csvLine += "," + String(gps.satellites.value());
    csvLine += "," + String(gps.hdop.hdop(), 1);
  } else {
    csvLine += ",,,,";
  }

  Serial.println("-----------\n");

  if (!sdPresent) {
    Serial.println("SD-Card not detected... No Data Logging");
    SD_CheckLED(CRGB::Red);
  }

  Serial.println("-----------\n");

  // Write the final line to the SD card
  logToSD(csvLine);

  // Set up a timer to wake us after 10 seconds
  esp_sleep_enable_timer_wakeup(10 * 1000000ULL - (micros() - timeStamp*1000));  // 10 seconds in microseconds

  Serial.println("Entering light sleep mode...");
  Serial.flush();  // Ensure serial output is flushed

  // Enter light sleep
  esp_light_sleep_start();

  Serial.println("Woke up from light sleep.");
}

void printBME280(Adafruit_BME280 &sensor) {
  Serial.print("Temp: ");
  Serial.print(sensor.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(sensor.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humidity: ");
  Serial.print(sensor.readHumidity());
  Serial.println(" %");
}

void printSI1145(Adafruit_SI1145 &sensor) {
  Serial.print("Vis: ");
  Serial.println(sensor.readVisible());

  Serial.print("IR: ");
  Serial.println(sensor.readIR());

  float UVindex = sensor.readUV();
  UVindex /= 100.0;
  Serial.print("UV: ");
  Serial.println(UVindex);
}

void printMPU6050(Adafruit_MPU6050 &sensor) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");
}

void printSCD41() {
  Serial.println();
  Serial.print(F("CO2(ppm):"));
  Serial.print(scd41.getCO2());
  Serial.print(F("\tTemperature(C):"));
  Serial.print(scd41.getTemperature(), 1);
  Serial.print(F("\tHumidity(%RH):"));
  Serial.print(scd41.getHumidity(), 1);
  Serial.println();
}

void printMS5611() {
  // Read data from MS5611
  //ms5611.read();  // Note: no error checking here
  Serial.print("T:\t");
  Serial.print(ms5611.getTemperature(), 2);
  Serial.print("\tP:\t");
  Serial.print(ms5611.getPressure(), 2);
  Serial.println();
}

void printGPS() {
  if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" m");
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    Serial.print("HDOP: ");
    Serial.println(gps.hdop.hdop());
    Serial.print("Time (UTC): ");
    if (gps.time.isValid()) {
      Serial.printf("%02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
      Serial.println("Invalid");
    }
    Serial.print("Date: ");
    if (gps.date.isValid()) {
      Serial.printf("%02d/%02d/%d\n", gps.date.day(), gps.date.month(), gps.date.year());
      leds[0] = CRGB::Green;
      FastLED.setBrightness(40);
      FastLED.show();
      delay(1000);
      leds[0] = CRGB::Black;
    } else {
      Serial.println("Invalid");
    }
  } else {
    Serial.println("Waiting for GPS fix...");
    leds[0] = CRGB::Blue;
    FastLED.setBrightness(40);
    FastLED.show();
    delay(1000);
    leds[0] = CRGB::Black;
    FastLED.show();
  }
}

void logToSD(String csvLine) {
  if (!sdPresent) return;

  dataFile = SD.open(FILENAME, FILE_APPEND);
  if (dataFile) {
    dataFile.println(csvLine);
    dataFile.flush();  // Force writing to card
    dataFile.close();  // Ensure data is saved
    Serial.println("Data written to SD.");
    delay(200);
    SD_CheckLED(CRGB::Green);
  } else {
    Serial.println("Failed to open file for writing. SD logging disabled.");
    sdPresent = false;
  }
}
