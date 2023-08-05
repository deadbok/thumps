#include <Wire.h>
#include <ArduinoHA.h>
#include <ESP8266WiFi.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include "version.h"
#include "secrets.h"

#define I2C_SDA             D2
#define I2C_SCL             D1
#define SENSOR_MAP_ENABLED  0b00000011

WiFiClient                  client;
HADevice                    device;
HAMqtt                      mqtt(client, device);
Adafruit_BMP280             bmp(&Wire);
Adafruit_AHTX0              aht;
Adafruit_Sensor             *aht_humidity,
                            *aht_temp,
                            *bmp_temp,
                            *bmp_pres;
HASensorNumber              temp_sensor("Temperature", HASensorNumber::PrecisionP1);
HASensorNumber              hum_sensor("Humidity", HASensorNumber::PrecisionP1);
HASensorNumber              pres_sensor("Pressure", HASensorNumber::PrecisionP1);
unsigned long               cycles = 0;

void scan_I2C();

void setup()
{
  uint8_t init_map = 0xff;
  byte mac[WL_MAC_ADDR_LENGTH];
  
  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));

  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  Serial.printf("\n\nTHUMPRESS v%s\n\n", VERSION_STRING);

  Serial.printf("Initialising sensors:\n");
  while (init_map > 0)
  {
    if (0b00000001 && init_map)
    {
      if (!aht.begin())
      {
        Serial.printf("ERROR initialising the AHT20 sensor.\n");
      }
      else
      {
        Serial.printf("The AHT20 sensor was initialised.\n");
        init_map &= 0b11111110;
      }
    }

    if (00000010 && init_map)
    {
      if (!bmp.begin())
      {
        Serial.printf("ERROR initialising the BMP280 sensor.\n");
      }
      else
      {
        Serial.printf("The BMP280 sensor was initialised.\n");
        init_map &= 0b1111101;
      }
    }

    // Zero out unused entries in the map
    init_map &= SENSOR_MAP_ENABLED;

    if (init_map > 0)
    {
      Serial.printf("Failed initialising sensors!\n");
      delay(15000);
      scan_I2C();
    }
    else
    {
      Serial.printf("Sensors initialised.\n");
    }
  }

  device.setUniqueId(mac, sizeof(mac));
  device.setName("Temperature, humidity, and pressure sensor");
  device.setSoftwareVersion(VERSION_STRING);
  device.setManufacturer("deadbok");
  device.setModel("THUMPS-1");
  device.enableSharedAvailability();
  device.enableLastWill();

  temp_sensor.setIcon("mdi:thermometer");
  temp_sensor.setUnitOfMeasurement("C");
  
  hum_sensor.setIcon("mdi:water-percent");
  hum_sensor.setUnitOfMeasurement("rH");

  pres_sensor.setIcon("mdi:gauge");
  pres_sensor.setUnitOfMeasurement("hPa");

  cycles = 0;  
  printf("Connecting to WiFi network: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while ((WiFi.status() != WL_CONNECTED) && (cycles < 300))
  {
    delay(1000);
    cycles += 1;
    printf(".");
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    printf("Connected!\n");
  }
  else
  {
    printf("NOT connected!\n");
  }

  mqtt.begin(MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD);

  aht_temp = aht.getTemperatureSensor();
  aht_temp->printSensorDetails();
  Serial.printf("\n");

  aht_humidity = aht.getHumiditySensor();
  aht_humidity->printSensorDetails();
  Serial.printf("\n");

  bmp_temp = bmp.getTemperatureSensor();
  bmp_temp->printSensorDetails();
  Serial.printf("\n");

  bmp_pres = bmp.getPressureSensor();
  bmp_pres->printSensorDetails();
  Serial.printf("\n");
  printf("--------------- Entering main loop ---------------\n\n");
}

void loop()
{
  sensors_event_t humidity;
  sensors_event_t temp;
  sensors_event_t temp2;
  sensors_event_t pressure;
  float           temp_avg;

  if ((cycles < 1) || (cycles > 9000000))
  {
    aht_humidity->getEvent(&humidity);
    aht_temp->getEvent(&temp);
    bmp_temp->getEvent(&temp2);
    bmp_pres->getEvent(&pressure);

    Serial.printf("------------------------------\n");
    Serial.printf("Temperature (AHT) %.2f deg C\n", temp.temperature);
    Serial.printf("Temperature (BMP) %.2f deg C\n", temp2.temperature);
    temp_avg = (temp.temperature + temp2.temperature) / 2;
    Serial.printf("Temperature (avg) %.2f deg C\n", temp_avg);
    Serial.printf("Humidity: %.2f %% rH\n", humidity.relative_humidity);
    Serial.printf("Pressure: %.2f hPa\n", pressure.pressure);

    Serial.printf("Publishing data to MQTT.\n");
    temp_sensor.setValue(temp_avg);
    hum_sensor.setValue(humidity.relative_humidity);
    pres_sensor.setValue(pressure.pressure);
    mqtt.loop();

    cycles = 1;
  }
  mqtt.loop();
  cycles += 1;
}

void scan_I2C()
{
  uint8_t error, address, nDevices;

  Serial.printf("Scanning...\n");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("0x%02X", address);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.printf(" E: 0x%02X ", address);
    }
    else
    {
      Serial.printf(".");
    }
  }
  if (nDevices == 0)
  {
    Serial.printf("No I2C devices found\n");
  }
  else
  {
    Serial.printf("done.\n%d devices found.\n",nDevices);
  }
}
