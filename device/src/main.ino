#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <stdlib.h>

#include "config_main.h"
#include "config_device.h"

// mqtt

WiFiClientSecure wifiClient;
PubSubClient mqttClient(mqttHost, mqttPort, wifiClient);
char mqttPrefix[50];

long mqttLastConnectAttempt = 0;

void mqttSubscribe(const char *topic) {
  if (mqttClient.connected()) {
    char t[100];
    sprintf(t, "%s%s", mqttPrefix, topic);
    mqttClient.subscribe(t);
  }
}

void mqttPublish(const char *topic, const char *payload, bool retain = true) {
  if (mqttClient.connected()) {
    char t[100];
    sprintf(t, "%s%s", mqttPrefix, topic);
    mqttClient.publish(t, payload, retain);
  }
}

void mqttPublishLong(const char *topic, long value, const char *uom, bool retain = true) {
  char payload[100];
  sprintf(payload, "{\"val\":%d,\"uom\":\"%s\"}", value, uom);
  mqttPublish(topic, payload, retain);
}

void mqttPublishFloat(const char *topic, float value, const char *uom, int precision = 2, bool retain = true) {
  char val[50];
  dtostrf(value, 0, precision, val);
  char payload[100];
  sprintf(payload, "{\"val\":%s,\"uom\":\"%s\"}", val, uom);
  mqttPublish(topic, payload, retain);
}

void mqttPublishBool(const char *topic, bool value, bool retain = true) {
  char payload[100];
  sprintf(payload, "{\"val\":%s,\"uom\":\"\"}", value ? "true" : "false");
  mqttPublish(topic, payload, retain);
}

void mqttAnnounce() {
//  mqttSubscribe("#");
  mqttPublish("$Online", "true");
  mqttPublish("$Name", deviceName);
  mqttPublish("$MAC", WiFi.macAddress().c_str());
  mqttPublish("$IP", WiFi.localIP().toString().c_str());
}

void mqttConnect() {
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - mqttLastConnectAttempt > 2000) {
      Serial.println("MQTT Connect");
      mqttLastConnectAttempt = now;
      char lwt[100];
      sprintf(lwt, "%s%s", mqttPrefix, "$Online");
      mqttClient.connect(deviceName, mqttUser, mqttPass, lwt, 0, true, "false");
      if (mqttClient.connected()) {
        mqttLastConnectAttempt = 0;
        Serial.println("MQTT Connected.");
        mqttAnnounce();
      }
    }
  }
}

void mqttLoop() {
  mqttClient.loop();
}

// wifi

void wifiConnect() {
  Serial.println("WiFi Connect");
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPassword);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.print("WiFi Connected: IP: ");
  Serial.println(WiFi.localIP());
}

// ota

void otaInit() {
  ArduinoOTA.setPort(otaPort);
  ArduinoOTA.setHostname(deviceName);
  ArduinoOTA.setPassword(otaPass);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else
      type = "filesystem";
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    delay(2000);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void otaLoop() {
  ArduinoOTA.handle();
}

// main

void mainSetup();
void mainLoop();

void setup() {
  Serial.begin(115200);
  Serial.println("\r\n\nHoCo ESP Setup\n");
  sprintf(mqttPrefix, "hoco/esp_pio/%s/", deviceName);
  wifiConnect();
  otaInit();
  mqttConnect();
  mainSetup();
  Serial.println("Setup complete.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED)
    wifiConnect();
  else {
    if (!mqttClient.connected())
      mqttConnect();
    else {
      mainLoop();
      mqttLoop();
    }
    otaLoop();
  }
}

#ifdef TOSAESP_ONEWIRE
OneWire ow(OW_PIN);

#ifdef TOSAESP_DS18B20
DallasTemperature ds(&ow);

const long dsDelay = 1 * 60 * 1000;
long dsLast = 0;
bool dsFetch = false;
const long dsFetchDelay = 1 * 1000;

void dsSetup() {
  ds.begin();
  DeviceAddress address;
  for (byte i = 0; i < ds.getDeviceCount(); i++) {
    if (ds.getAddress(address, i)) {
      bool known = false;
      for (byte s = 0; s < DS_SENSORS; s++) {
        if (memcmp(address, dsAddress[s], 8) == 0) {
          known = true;
          Serial.print("Temperature sensor found: ");
          Serial.println(dsName[s]);
//          mqttPublish("", true);
        }
      }
      if (!known) {
        char da[30];
        sprintf(da, "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", address[0], address[1], address[2], address[3], address[4], address[5], address[6], address[7]);
        mqttPublish("$UnknownSensor", da);
      }
    }
  }
}

void dsLoop() {
  long now = millis();
  if (!dsFetch && (now - dsLast > dsDelay)) {
    ds.requestTemperatures();
    dsFetch = true;
    dsLast = now;
  } else if (dsFetch && (now - dsLast > dsFetchDelay)) {
    for (byte i = 0; i < DS_SENSORS; i++) {
      float t = ds.getTempC(dsAddress[i]);
      char topic[40];
      sprintf(topic, "%sTemperatur", dsName[i]);
      mqttPublishFloat(topic, t, "°C", 3);
    }
    dsFetch = false;
    dsLast = now;
  }
}
#endif // TOSAESP_DS18B20
#endif // TOSAESP_ONEWIRE

#ifdef TOSAESP_HCSR04
NewPing us(US_TRIGGER_PIN, US_ECHO_PIN);

const long usDelay = 1 * 60 * 1000;
long usLast = 0;
const long usSampleDelay = 1 * 1000;
long usSampleLast = 0;
long usSample[US_SAMPLES];
byte usSamplePos = 0;

void usSetup() {
}

void usLoop() {
  long now = millis();
  if (now - usLast > usDelay) {
    if (now - usSampleLast > usSampleDelay) {
      usSample[usSamplePos] = us.ping_cm();
      usSampleLast = now;
      usSamplePos++;
      if (usSamplePos == US_SAMPLES) {
        long dist = 0;
        for (byte i = 0; i < US_SAMPLES; i++)
          dist += usSample[i];
        dist = dist / US_SAMPLES;
        mqttPublishLong("Entfernung", dist, "cm");
        usSamplePos = 0;
        usLast = now;
      }
    }
  }
}
#endif // TOSAESP_HCSR04

#ifdef TOSAESP_PCF8574
PCF8574 pcf8574(PCF8574_ADDR);

const long pcfDelay = 1 * 60 * 1000;
long pcfLast = 0;

void pcf8574Setup() {
  pcf8574.begin();
}

void pcf8574Loop() {
  long now = millis();
  if (now - pcfLast > pcfDelay) {
    for (byte i = 0; i < PCF8574_INPUTS; i++) {
      bool v = pcf8574.readButton(pcfInputPin[i]);
      if (pcfInputInv[i])
        v = !v;
      mqttPublishBool(pcfInputName[i], v);
    }
    pcfLast = now;
  }
}
#endif // TOSAESP_PCF8574

void mainSetup() {
#ifdef TOSAESP_DS18B20
  dsSetup();
#endif
#ifdef TOSAESP_HCSR04
  usSetup();
#endif
#ifdef TOSAESP_PCF8574
  pcf8574Setup();
#endif
}

void mainLoop() {
#ifdef TOSAESP_DS18B20
  dsLoop();
#endif
#ifdef TOSAESP_HCSR04
  usLoop();
#endif
#ifdef TOSAESP_PCF8574
  pcf8574Loop();
#endif
}

