/*
  AWS IoT WiFi

  This sketch securely connects to an AWS IoT using MQTT over WiFi.
  It uses a private key stored in the ATECC508A and a public
  certificate for SSL/TLS authetication.

  It publishes a message every 5 seconds to arduino/outgoing
  topic and subscribes to messages on the arduino/incoming
  topic.

  The circuit:
  - Arduino MKR WiFi 1010 or MKR1000

  The following tutorial on Arduino Project Hub can be used
  to setup your AWS account and the MKR board:

  https://create.arduino.cc/projecthub/132016/securely-connecting-an-arduino-mkr-wifi-1010-to-aws-iot-core-a9f365

  This example code is in the public domain.
*/
#include "arduino_secrets_mbank.h"

#include "ArduinoJson.h"
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h> // change to #include <WiFi101.h> for MKR1000
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
//#include "Adafruit_BME680.h"
#include "bsec.h"
#include <ClosedCube_OPT3001.h>
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30

#define OPT3001_ADDRESS 0x45
#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)
#define SEALEVELPRESSURE_HPA (1013.25)
#define JSON_BUFFER 512
// Variables to find the peak-to-peak amplitude of AUD output
#define AUDIO_INTEGRAL_TIME_MS 500
// Sensors objecbs

Adafruit_BMP280 bmp280; // I2C
//Adafruit_BME680 bme680; // I2C
Bsec iaqSensor;
ClosedCube_OPT3001 opt3001;
SCD30 airSensorSCD30;

// Connect the INMP401 AUD output to the Arduino A0 pin
int mic = A0;

int micOutput = 0;
/////// Enter your sensitive data in arduino_secrets.h
const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;
const char broker[] = SECRET_BROKER;
const char *certificate = SECRET_CERTIFICATE;

WiFiClient wifiClient;               // Used for the TCP socket connection
BearSSLClient sslClient(wifiClient); // Used for SSL/TLS connection, integrates with ECC508
MqttClient mqttClient(sslClient);

unsigned long lastMillis = 0;

StaticJsonDocument<JSON_BUFFER> jsonObject;
void setup()
{
  Serial.begin(115200);
  delay(1000);
  if (!ECCX08.begin())
  {
    Serial.println("No ECCX08 present!");
    while (1)
      ;
  }

  // Set a callback to get the current time
  // used to validate the servers certificate
  ArduinoBearSSL.onGetTime(getTime);

  // Set the ECCX08 slot to use for the private key
  // and the accompanying public certificate for it
  sslClient.setEccSlot(0, certificate);

  // Optional, set the client id used for MQTT,
  // each device that is connected to the broker
  // must have a unique client id. The MQTTClient will generate
  // a client id for you based on the millis() value if not set
  //
  // mqttClient.setId("clientId");

  // Set the message callback, this function is
  // called when the MQTTClient receives a message
  mqttClient.onMessage(onMessageReceived);
  sensorInit();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    connectWiFi();
  }

  if (!mqttClient.connected())
  {
    // MQTT client is disconnected, connect
    connectMQTT();
  }

  // poll for new MQTT messages and send keep alives
  mqttClient.poll();
  dataUpdate();
  // publish a message roughly every 5 seconds.
  if (millis() - lastMillis > 20000)
  {
    lastMillis = millis();
    publishMessage();
    sensorReset();
  }
}

unsigned long getTime()
{
  // get the current time from the WiFi module
  return WiFi.getTime();
}

void connectWiFi()
{
  Serial.print("Attempting to connect to SSID: ");
  Serial.print(ssid);
  Serial.print(" ");

  while (WiFi.begin(ssid, pass) != WL_CONNECTED)
  {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println();

  Serial.println("You're connected to the network");
  Serial.println();
}

void connectMQTT()
{
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(broker);
  Serial.println(" ");

  while (!mqttClient.connect(broker, 8883))
  {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe("arduino/incoming");
}
void dataUpdate()
{

  jsonObject["node_id"] = "USN1";
  if (airSensorSCD30.dataAvailable())
  {
    jsonObject["scd30_temperature"] = airSensorSCD30.getTemperature();
    jsonObject["scd30_co2"] = airSensorSCD30.getCO2();
    jsonObject["scd30_humidity"] = airSensorSCD30.getHumidity();
  }
  else
  {
    Serial.println("Waiting for new data");
  }
  OPT3001 result = opt3001.readResult();
  if (result.error == NO_ERROR)
  {
    jsonObject["opt3001_light"] = result.lux;
  }

  while (!iaqSensor.run())
  { // If new data is available
    delay(500);
  }
  jsonObject["bme680_iaq"] = iaqSensor.iaq;
  jsonObject["bme680_iaq_accuracy"] = iaqSensor.iaqAccuracy;
  jsonObject["bme680_co2equivalent"] = iaqSensor.co2Equivalent;
  jsonObject["bme680_breath_voc_equivalent"] = iaqSensor.breathVocEquivalent;

  micOutput = micOutput + readVolumeLevel();
  jsonObject["inmp401_mic_output_integral"] = micOutput;

  jsonObject["bmp280_temperature"] = bmp280.readTemperature();
  jsonObject["bmp280_pressure"] = bmp280.readPressure();

  Serial.println("Data Updated");
}
void publishMessage()
{
  Serial.println("Publishing message");
  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage("mbank/topic");
  serializeJson(jsonObject, Serial);
  serializeJson(jsonObject, mqttClient);
  mqttClient.endMessage();
}

void onMessageReceived(int messageSize)
{
  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available())
  {
    Serial.print((char)mqttClient.read());
  }
  Serial.println();

  Serial.println();
}

void sensorInit(void)
{

  Wire.begin();

  if (airSensorSCD30.begin() == false)
  {
    Serial.println("SCD30 Air sensor not detected. Please check wiring. Freezing...");
  }
  while (!bmp280.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }
  opt3001.begin(OPT3001_ADDRESS);
  OPT3001_Config newConfig;

  newConfig.RangeNumber = B1100;
  newConfig.ConvertionTime = B0;
  newConfig.Latch = B1;
  newConfig.ModeOfConversionOperation = B11;

  OPT3001_ErrorCode errorConfig = opt3001.writeConfig(newConfig);
  if (errorConfig == NO_ERROR)
  {
    OPT3001_Config sensorConfig = opt3001.readConfig();
  }

  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  bsec_virtual_sensor_t sensorList[10] = {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };
  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  /* Default settings from datasheet. */
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
void sensorReset(void)
{
  micOutput = 0;
}
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK)
  {
    if (iaqSensor.status < BSEC_OK)
    {
      Serial.print("BSEC error code : ");
    }
    else
    {
      Serial.print("BSEC warning code : ");
    }
    Serial.println(String(iaqSensor.status));
  }

  if (iaqSensor.bme680Status != BME680_OK)
  {
    if (iaqSensor.bme680Status < BME680_OK)
    {
      Serial.print("BME680 error code : ");
    }
    else
    {
      Serial.print("BME680 warning code : ");
    }
    Serial.println(String(iaqSensor.bme680Status));
  }
}
// Find the VolumeLevel
int readVolumeLevel()
{

  // Time variables to find the peak-to-peak amplitude
  unsigned long startTime = millis(); // Start of sample window
  unsigned int PTPAmp = 0;
  unsigned int micOut = 0;
  // Signal variables to find the peak-to-peak amplitude
  unsigned int maxAmp = 0;
  unsigned int minAmp = 1023;

  // Find the max and min of the mic output within the 50 ms timeframe
  while (millis() - startTime < AUDIO_INTEGRAL_TIME_MS)
  {
    micOut = analogRead(mic);
    if (micOut < 1023) //prevent erroneous readings
    {
      if (micOut > maxAmp)
      {
        maxAmp = micOut; //save only the max reading
      }
      else if (micOut < minAmp)
      {
        minAmp = micOut; //save only the min reading
      }
    }
  }

  PTPAmp = maxAmp - minAmp; // (max amp) - (min amp) = peak-to-peak amplitude

  double micOut_Volts = (PTPAmp * 3.3) / 1024; // Convert ADC into voltage

  //Uncomment this line for help debugging (be sure to also comment out the VUMeter function)
  //Serial.println(PTPAmp);
  //Return the PTP amplitude to use in the soundLevel function.
  // You can also return the micOut_Volts if you prefer to use the voltage level.
  return PTPAmp;
}
