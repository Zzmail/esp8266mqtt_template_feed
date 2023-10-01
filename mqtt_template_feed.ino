
/*
   MQTT Sensor - Temperature and Humidity (DHT22) for Home-Assistant - NodeMCU (ESP8266)
   https://home-assistant.io/components/sensor.mqtt/
   Libraries :
    - ESP8266 core for Arduino : https://github.com/esp8266/Arduino
    - PubSubClient : https://github.com/knolleary/pubsubclient
    - DHT : https://github.com/adafruit/DHT-sensor-library
    - ArduinoJson : https://github.com/bblanchon/ArduinoJson
   Sources :
    - File > Examples > ES8266WiFi > WiFiClient
    - File > Examples > PubSubClient > mqtt_auth
    - File > Examples > PubSubClient > mqtt_esp8266
    - File > Examples > DHT sensor library > DHTtester
    - File > Examples > ArduinoJson > JsonGeneratorExample
    - http://www.jerome-bernard.com/blog/2015/10/04/wifi-temperature-sensor-with-nodemcu-esp8266/
   Schematic :
    - https://github.com/mertenats/open-home-automation/blob/master/ha_mqtt_sensor_dht22/Schematic.png
    - DHT22 leg 1 - VCC
    - DHT22 leg 2 - D1/GPIO5 - Resistor 4.7K Ohms - GND
    - DHT22 leg 4 - GND
    - D0/GPIO16 - RST (wake-up purpose)
   Configuration (HA) :
    sensor 1:
      platform: mqtt
      state_topic: 'office/sensor1'
      name: 'Temperature'
      unit_of_measurement: '°C'
      value_template: '{{ value_json.temperature }}'
    
    sensor 2:
      platform: mqtt
      state_topic: 'office/sensor1'
      name: 'Humidity'
      unit_of_measurement: '%'
      value_template: '{{ value_json.humidity }}'
   Samuel M. - v1.1 - 08.2016
   If you like this example, please add a star! Thank you!
   https://github.com/mertenats/open-home-automation
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <ArduinoJson.h>
#include <OneWire.h>

#define MQTT_VERSION MQTT_VERSION_3_1_1

int DS18S20_Pin = 4; //定义数字口3
OneWire ds(DS18S20_Pin); 
// Wifi: SSID and password
const char* WIFI_SSID = "Gen";
const char* WIFI_PASSWORD = "15254961333";

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "bedroom_tem";
const PROGMEM char* MQTT_SERVER_IP = "101.43.237.55";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char* MQTT_USER = "";
const PROGMEM char* MQTT_PASSWORD = "";

// MQTT: topic
const PROGMEM char* MQTT_SENSOR_TOPIC = "office/sensor1";
const PROGMEM char* FISH_SWITCH_STATUS = "home/fish/fishswitch";
const PROGMEM char* FISH_SWITCH_SET = "home/fish/fishswitch/set";
const PROGMEM char* FISH_SWITCH_ONLINE_STATUS = "home/fish/fishswitch/available";


const char* SWITCH_ON = "ON";
const char* SWITCH_OFF = "OFF";
boolean m_switch_state = false; // light is turned off by default
int loop_num=0;
const PROGMEM uint8_t FEED_PIN = 2;
// sleeping time
const PROGMEM uint16_t SLEEPING_TIME_IN_SECONDS = 600; // 10 minutes x 60 seconds


WiFiClient wifiClient;
PubSubClient client(wifiClient);

// function called to publish the temperature and the humidity
void publishData(float p_temperature) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference

  StaticJsonDocument<200> jsonBuffer;
  // JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  jsonBuffer["temperature"] = (String)p_temperature;

  char data[200];
  serializeJson(jsonBuffer, data);

  /*
     {
        "temperature": "23.20" ,
        "humidity": "43.70"
     }
  */
  Serial.print(data);

  client.publish(MQTT_SENSOR_TOPIC, data, true);
  client.publish(FISH_SWITCH_ONLINE_STATUS, "online", true);
  yield();
}

void publishLightState() {
  if (m_switch_state) {
    client.publish(FISH_SWITCH_STATUS, SWITCH_ON, true);
  } else {
    client.publish(FISH_SWITCH_STATUS, SWITCH_OFF, true);
  }
}

// function called to turn on/off the light
void setLightState() {
  if (m_switch_state) {
    digitalWrite(FEED_PIN, HIGH);
    // Serial.println("INFO: Turn light on...");
  } else {
    digitalWrite(FEED_PIN, LOW);
    // Serial.println("INFO: Turn light off...");
  }
}

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
    // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }
  // handle message topic
  if (String(FISH_SWITCH_SET).equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    Serial.println("INFO: Attempting MQTT connection...");
    if (payload.equals(String(SWITCH_ON))) {
      if (m_switch_state != true) {
        m_switch_state = true;
        setLightState();
        publishLightState();
      }
    } else if (payload.equals(String(SWITCH_OFF))) {
      if (m_switch_state != false) {
        m_switch_state = false;
        setLightState();
        publishLightState();
      }
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("INFO: connected");
      client.subscribe(FISH_SWITCH_SET);
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.print(client.state());
      Serial.println("DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(10000);
    }
  }
}

void setup() {
  // init the serial
  Serial.begin(115200);


  // 连wifi
  Serial.println();
  Serial.println();
  Serial.print("INFO: Connecting to ");
  WiFi.mode(WIFI_STA);
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("INFO: WiFi connected");
  Serial.println("INFO: IP address: ");
  Serial.println(WiFi.localIP());
  pinMode(FEED_PIN, OUTPUT);
  analogWriteRange(255);

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  if (!client.connected()) {
    reconnect();
  }
  client.setCallback(callback);
  digitalWrite(FEED_PIN, LOW);
  setLightState();
  publishLightState();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  // Read temperature as Celsius (the default)
  // if(false){
    if(loop_num>100000){
    float t = getTemp(); //调用函数计算温度值

    if (isnan(t)) {
      Serial.println("ERROR: Failed to read from DHT sensor!");
      return;
    } else {
      // Serial.println(h);
      Serial.println(t);
    
        publishData(t);
        loop_num=0;
    }
    
  
  }
  loop_num++;
  // Serial.println("INFO: Closing the MQTT connection");
  // client.disconnect();

  // Serial.println("INFO: Closing the Wifi connection");
  // WiFi.disconnect();
  // loop_num++;
  // ESP.deepSleep(SLEEPING_TIME_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);
  // delay(50000); // wait for deep sleep to happen
}


float getTemp(){

  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];

  byte addr[8];

  if ( !ds.search(addr)) {

      //no more sensors on chain, reset search

      ds.reset_search();

      return -0;

  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {

      Serial.println("CRC is not valid!");

      return -0;

  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {

      Serial.print("Device is not recognized");

      return -0;

  }

  ds.reset();

  ds.select(addr);

  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();

  ds.select(addr);   

  ds.write(0xBE); // Read Scratchpad  

  for (int i = 0; i < 9; i++) { // we need 9 bytes

    data[i] = ds.read();

  }

  ds.reset_search();  

  byte MSB = data[1];

  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment

  float TemperatureSum = tempRead / 16;  

  return TemperatureSum;   

}