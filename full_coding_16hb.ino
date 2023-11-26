#include <Wire.h>
#include <SoftwareSerial.h>
#include "Adafruit_PM25AQI.h"
#include <WiFi.h>
#include "ThingSpeak.h"

#define SECRET_SSID "4G-MIFI-DEDB"
#define SECRET_PASS "12345678"
#define SECRET_CH_ID 2344718
#define SECRET_WRITE_APIKEY "1RS7BQ7JIV6OLPJ4"

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
WiFiClient client;

unsigned long myChannelNumber = SECRET_CH_ID;
const char* myWriteAPIKey = SECRET_WRITE_APIKEY;
String myStatus = "";

SoftwareSerial pmSerial(16, 17); // SoftwareSerial for PM2.5 sensor
Adafruit_PM25AQI aqi = Adafruit_PM25AQI(); // PM2.5 sensor

// Pin definitions
#define SHT30_I2C_ADDR 0x44
#define SHT30_CMD_HIGH_PRECISION 0x2C06
#define SHT30_CMD_READ_DATA 0x2400

const int mq7Pin = 35; // Pin to which AOUT of MQ-7 is connected
float getPPM(float voltage);

void setup() {
  Serial.begin(115200);
  pmSerial.begin(9600);
  Wire.begin();

  if (!aqi.begin_UART(&pmSerial)) {
    Serial.println("Could not find PM 2.5 sensor!");
    while (1) delay(10);
  }

  Serial.println("PM25 found!");

  delay(1000);

  WiFi.begin(ssid, pass);
  ThingSpeak.begin(client);
}

void loop() {
  PM25_AQI_Data data;
  
  if (! aqi.read(&data)) {
    Serial.println("Could not read from AQI");
    delay(500);  // try again in a bit!
    return;
  }
  Serial.println("AQI reading success");

  Serial.println();
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Concentration Units (standard)"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_standard);
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_standard);
  Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_standard);
  Serial.println(F("Concentration Units (environmental)"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_env);
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_env);
  Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_env);
  Serial.println(F("---------------------------------------"));
  Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(data.particles_03um);
  Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(data.particles_05um);
  Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(data.particles_10um);
  Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(data.particles_25um);
  Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(data.particles_50um);
  Serial.print(F("Particles > 10 um / 0.1L air:")); Serial.println(data.particles_100um);
  Serial.println(F("---------------------------------------"));
  

  Wire.beginTransmission(SHT30_I2C_ADDR);
  Wire.write(SHT30_CMD_HIGH_PRECISION >> 8);
  Wire.write(SHT30_CMD_HIGH_PRECISION & 0xFF);
  Wire.endTransmission();
  delay(1000);

  Wire.requestFrom(SHT30_I2C_ADDR, 6);
  while (Wire.available() < 6);

  uint16_t rawTemperature = Wire.read() << 8 | Wire.read();
  Wire.read();
  uint16_t rawHumidity = Wire.read() << 8 | Wire.read();
  Wire.read();

  float temperature = -45 + 175 * (rawTemperature / 65535.0);
  float humidity = 100 * (rawHumidity / 65535.0);

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("°C, Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  int sensorValue = analogRead(mq7Pin);
  float voltage = sensorValue * (5.0 / 1023.0); // Convert analog reading to voltage
  float ppm = getPPM(voltage); // Convert voltage to PPM (parts per million)

  Serial.print("CO Concentration (PPM): ");
  Serial.println(ppm);

  // WiFi & ThingSpeak
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, pass);
      Serial.print(".");
      delay(5000);
    }
    Serial.println("\nConnected.");
  }

  ThingSpeak.setField(1, data.pm25_env); // PM2.5 environmental concentration
  ThingSpeak.setField(2, temperature);   // Temperature
  ThingSpeak.setField(3, humidity);      // Humidity
  ThingSpeak.setField(4, ppm);           // CO2 concentration

  ThingSpeak.setStatus(myStatus);

  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial.println("Channel update successful.");
  } else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  delay(15000);
}

float getPPM(float voltage) {
  // You'll need to calibrate your sensor to get an accurate conversion from voltage to PPM
  // This is just a simple example, and you may need to adjust these values based on your sensor's characteristics
  float slope = -0.423; // Adjust this based on your calibration
  float intercept = 1.25; // Adjust this based on your calibration

  float ppm = slope * voltage + intercept;
  return ppm;
}
