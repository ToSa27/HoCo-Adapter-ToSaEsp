#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <stdlib.h>
#include <EEPROM.h>
#include <hoco-base.h>

#include "device.h"

#include <TimeLib.h> 
int32_t timeOffset = 0;

#define PING_INTERVAL	(60 * 1000)

#define ULONG_MAX   4294967295

const char* deviceName = TOSAESP_DEVICE_NAME;

const char* wifiSsid = TOSAESP_WIFI_SSID;
const char* wifiPassword = TOSAESP_WIFI_PASS;
const int otaPort = TOSAESP_OTA_PORT;
const char* otaPass = TOSAESP_OTA_PASS;
const char* mqttHost = TOSAESP_MQTT_HOST;
const int mqttPort = TOSAESP_MQTT_PORT;
const char* mqttUser = TOSAESP_MQTT_USER;
const char* mqttPass = TOSAESP_MQTT_PASS;

const char* mqttRootPrefix = "hoco/";

// logging

#ifdef TOSAESP_TELNET
WiFiServer telnet(23);
WiFiClient telnetClients;
int disconnectedClient = 1;
bool telnetEnabled = false;
#define DEBUG_LOG_LN(x) { Serial.println(x); if(telnetClients) { telnetClients.println(x); } }
#define DEBUG_LOG_F(x, ...) { Serial.printf(x, ##__VA_ARGS__); if(telnetClients) { telnetClients.printf(x, ##__VA_ARGS__); } }
#define DEBUG_LOG(x) { Serial.print(x); if(telnetClients) { telnetClients.print(x); } }
#else
#define DEBUG_LOG_LN(x) { Serial.println(x); }
#define DEBUG_LOG_F(x, ...) { Serial.printf(x, ##__VA_ARGS__); }
#define DEBUG_LOG(x) { Serial.print(x); }
#endif
// mqtt

//StaticJsonBuffer<400> jsonBuffer;
WiFiClientSecure wifiClient;

long mqttLastConnectAttempt = 0;

bool handleMessage(char* topic, JsonObject& msg) {
  bool handled = false;
//  JsonObject& msg = jsonBuffer.parseObject(payload);
#ifdef TOSAESP_SCHEDULER
  if (!handled) handled = schedCallback(topic, msg);
#endif
#ifdef TOSAESP_SERIAL
  if (!handled) handled = serialCallback(topic, msg);
#endif
#ifdef TOSAESP_OUTPUTS
  if (!handled) handled = outCallback(topic, msg);
#endif
#ifdef TOSAESP_HLW8012
  if (!handled) handled = hlw8012Callback(topic, msg);
#endif
  return handled;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  bool handled = false;
//  payload[length] = '\0';
//  DEBUG_LOG_F("RX: %s <= %s\r\n", topic, payload);
  if (strncmp(topic, mqttRootPrefix, strlen(mqttRootPrefix)) != 0)
    return;
  char* dtopic = topic + strlen(mqttRootPrefix);
//  char pl[length + 1];
//  sprintf(pl, "%*.*s", length, length, payload);
//  DEBUG_LOG_F("RX: %s <= %*.*s\r\n", topic, length, length, payload);
//  DEBUG_LOG_F("RX: %s <= %s\r\n", topic, pl);
//  DEBUG_LOG_F("heap size: %u\r\n", ESP.getFreeHeap());
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& msg = jsonBuffer.parseObject((char *)payload);
  DEBUG_LOG("JSON MSG: ");
  msg.printTo(Serial);
  DEBUG_LOG_LN("");
  if (strncmp(dtopic, "set/", 4) == 0) {
    dtopic += 4;
    if (strlen(dtopic) >= 10 && strncmp(dtopic, "broadcast/", 10) == 0) {
      // handle broadcast
      dtopic += 10;
      char subtopic[50];
      sprintf(subtopic, "%s", dtopic);
//      JsonObject& msg = jsonBuffer.parseObject((char*)payload);
      if (!handled) handled = timeSetTime(subtopic, msg);
    } else if (strlen(dtopic) >= (strlen(deviceName) + 1) && strncmp(dtopic, deviceName, strlen(deviceName)) == 0) {
      // handle device topic
      dtopic += (strlen(deviceName) + 1);
      char subtopic[50];
      sprintf(subtopic, "%s", dtopic);
      if (strncmp(subtopic, "$reboot", 7) == 0) {
        ESP.reset();
#ifdef TOSAESP_TELNET
      } else if (strncmp(subtopic, "$telnet", 7) == 0) {
//        JsonObject& msg = jsonBuffer.parseObject((char*)payload);
        telnetEnabled = msg["val"];
#endif
      } else {
        handled = handleMessage(subtopic, msg);
      }
    }
  }
}

PubSubClient mqttClient(wifiClient);

void mqttPublish(const char *topic, const char *payload, time_t ts = 0, unsigned long ms = 0, bool retain = true) {
  if (mqttClient.connected()) {
    char t[100];
    sprintf(t, "%sstatus/%s/%s", mqttRootPrefix, deviceName, topic);
    if ((timeStatus() != timeNotSet) && (payload[0] == '{') && (strstr(payload, "\"ts\"") == NULL)) {
      if (ts == 0)
        ts = now() + timeOffset;
      unsigned int msonly = ms - (((unsigned long)(ms / 1000)) * 1000);
      char payloadts[220];
      sprintf(payloadts, "{\"ts\":%lu%03d%s%s", ts, msonly, (strlen(payload) > 2 ? "," : ""), payload + 1);
      mqttClient.publish(t, payloadts, retain);
      DEBUG_LOG_F("TX: %s => %s\r\n", topic, payloadts);
    } else {
      mqttClient.publish(t, payload, retain);
      DEBUG_LOG_F("TX: %s => %s\r\n", topic, payload);
    }
  }
}

void mqttPublishLong(const char *topic, long value, const char *uom, time_t ts = 0, unsigned long ms = 0, bool retain = true) {
  char payload[100];
  sprintf(payload, "{\"val\":%ld,\"uom\":\"%s\"}", value, uom);
  mqttPublish(topic, payload, ts, ms, retain);
}

void mqttPublishByteArray(const char *topic, byte value[], unsigned int len, time_t ts = 0, unsigned long ms = 0, bool retain = true) {
  char payload[len * 2 + 1];
  for (unsigned int i = 0; i < len; i++) {
    byte nib1 = (value[i] >> 4) & 0x0F;
    byte nib2 = (value[i] >> 0) & 0x0F;
    payload[i*2+0] = nib1 < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    payload[i*2+1] = nib2 < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
  }
  payload[len*2] = '\0';
  sprintf(payload, "{\"val\":%ld}", value);
  mqttPublish(topic, payload, ts, ms, retain);
}

void mqttPublishTimestamp(const char *topic, time_t ts = 0, unsigned long ms = 0, bool retain = true) {
  char payload[100];
  sprintf(payload, "{\"val\":1}");
  mqttPublish(topic, payload, ts, ms, retain);
}

void mqttPublishFloat(const char *topic, float value, const char *uom, int precision = 2, time_t ts = 0, unsigned long ms = 0, bool retain = true) {
  char val[20];
  dtostrf(value, 0, precision, val);
  char payload[100];
  sprintf(payload, "{\"val\":%s,\"uom\":\"%s\"}", val, uom);
  mqttPublish(topic, payload, ts, ms, retain);
}

void mqttPublishFloatMMA(const char *topic, float value, float min, float avg, float max, const char *uom, int precision = 2, time_t ts = 0, unsigned long ms = 0, bool retain = true) {
  char val[20];
  dtostrf(value, 0, precision, val);
  char valmin[20];
  dtostrf(min, 0, precision, valmin);
  char valavg[20];
  dtostrf(avg, 0, precision, valavg);
  char valmax[20];
  dtostrf(max, 0, precision, valmax);
  char payload[200];
  sprintf(payload, "{\"val\":%s,\"min\":%s,\"avg\":%s,\"max\":%s,\"uom\":\"%s\"}", val, valmin, valavg, valmax, uom);
  mqttPublish(topic, payload, ts, ms, retain);
}

void mqttPublishBool(const char *topic, bool value, time_t ts = 0, unsigned long ms = 0, bool retain = true) {
  char payload[100];
  sprintf(payload, "{\"val\":%s}", value ? "true" : "false");
  mqttPublish(topic, payload, ts, ms, retain);
}

void mqttAnnounce() {
  char t[100];
  sprintf(t, "%sset/broadcast/#", mqttRootPrefix);
  mqttClient.subscribe(t);
  sprintf(t, "%sset/%s/#", mqttRootPrefix, deviceName);
  mqttClient.subscribe(t);
  if (mqttClient.connected()) {
    sprintf(t, "%sset/root/sendtime", mqttRootPrefix);
    mqttClient.publish(t, deviceName, false);
    mqttPublish("$Online", "true");
    mqttPublish("$Name", deviceName);
    mqttPublish("$MAC", WiFi.macAddress().c_str());
    mqttPublish("$IP", WiFi.localIP().toString().c_str());
  }
}

void mqttConnect() {
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - mqttLastConnectAttempt > 2000) {
      DEBUG_LOG_LN("MQTT Connect");
      mqttLastConnectAttempt = now;
      char lwt[100];
      sprintf(lwt, "%sstatus/%s/%s", mqttRootPrefix, deviceName, "$Online");
      if (mqttClient.connect(deviceName, mqttUser, mqttPass, lwt, 0, true, "false")) {
        mqttLastConnectAttempt = 0;
        DEBUG_LOG_LN("MQTT Connected.");
        mqttAnnounce();
      } else {
        DEBUG_LOG_LN("MQTT Connection Failed!");
      }
    }
  }
}

void mqttSetup() {
  mqttClient.setServer(mqttHost, mqttPort);
  mqttClient.setCallback(mqttCallback);
}

void mqttLoop() {
  if (!mqttClient.connected())
    mqttConnect();
  mqttClient.loop();
}

// wifi

void wifiConnect() {
  DEBUG_LOG_LN("WiFi Connect");
  WiFi.mode(WIFI_STA);
/*
  byte mac[6];
  WiFi.macAddress(mac);
  DEBUG_LOG("WiFi Connect: MAC: ");
  for (int i = 0; i < 5; i++)
    DEBUG_LOG(String(mac[i],HEX) + ":");
  DEBUG_LOG_LN(String(mac[5],HEX));
*/
  delay(10);
  WiFi.begin(wifiSsid, wifiPassword);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    DEBUG_LOG_LN("WiFi Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  DEBUG_LOG("WiFi Connected: IP: ");
  DEBUG_LOG_LN(WiFi.localIP());
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
    DEBUG_LOG_LN("OTA started " + type);
  });
  ArduinoOTA.onEnd([]() {
    DEBUG_LOG_LN("OTA finished");
    delay(2000);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_LOG_F("OTA: %u%%\r\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_LOG_F("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      DEBUG_LOG_LN("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      DEBUG_LOG_LN("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      DEBUG_LOG_LN("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      DEBUG_LOG_LN("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      DEBUG_LOG_LN("End Failed");
    }
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
#ifdef TOSAESP_TELNET
  telnet.begin();
  telnet.setNoDelay(true);
#endif
  DEBUG_LOG_LN("\r\n\nHoCo ESP Setup\n");

  //HoCo::Init();
  
  wifiConnect();
  otaInit();
  mqttSetup();
  mainSetup();
  DEBUG_LOG_LN("Setup complete.");
}

#ifdef TOSAESP_TELNET
void telnetLoop() {
  if (telnet.hasClient()) {
    if (!telnetClients || !telnetClients.connected()) {
      if (telnetClients) {
        telnetClients.stop();
        DEBUG_LOG_LN("Telnet Client Stop");
        telnetEnabled = false;
      }
      telnetClients = telnet.available();
      DEBUG_LOG_LN("New Telnet client");
      telnetClients.flush();  // clear input buffer, else you get strange characters 
      disconnectedClient = 0;
    }
  }
  else {
    if(!telnetClients.connected()) {
      if(disconnectedClient == 0) {
        DEBUG_LOG_LN("Client Not connected");
        telnetClients.stop();
        disconnectedClient = 1;
      }
    }
  }
}
#endif

#ifdef TOSAESP_PING
unsigned long pingLast = 0;

void pingLoop() {
  /*
  unsigned long now = millis();
  if (now - pingLast > PING_INTERVAL) {
    mqttPublish("$ping", "{}");
    pingLast = now;
  }
  */
}
#endif

void loop() {
  yield();
  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_LOG_LN("WiFi reconnect");
    wifiConnect();
  } else {
    if (!mqttClient.connected()) {
      yield();
      mqttConnect();
    }
    else {
      yield();
      mainLoop();
      yield();
      mqttLoop();
    }
#ifdef TOSAESP_TELNET
    yield();
    if (telnetEnabled)
      telnetLoop();
#endif
#ifdef TOSAESP_PING
    yield();
    pingLoop();
#endif
    yield();
    otaLoop();
  }
}

void commonSetup() {
  yield();
}

void commonLoop() {
  yield();
  otaLoop();
}

bool timeSetTime(char* subtopic, JsonObject& msg) {
  if (strncmp(subtopic, "$time", 5) != 0)
    return false;
  unsigned long epoch = msg["epoch"];
  if (epoch > 0) {
    timeOffset = msg["offset"];
    setTime(epoch);
  }
  return true;
}

#ifdef TOSAESP_OUTPUTS
bool outPulseEndVal[TOSAESP_OUTPUTS];
int outPulse[TOSAESP_OUTPUTS];
unsigned long outPulseStart[TOSAESP_OUTPUTS];

void outSetVal(byte i, bool v) {
  digitalWrite(outputPin[i], (outputInv[i] ? (!v) : v) ? HIGH : LOW);
  mqttPublishBool(outputName[i], v);
}

void outSetup() {
  for (byte i = 0; i < TOSAESP_OUTPUTS; i++) {
    outPulse[i] = 0;
    pinMode(outputPin[i], OUTPUT);
    outSetVal(i, outputDefault[i]);
  }
}

void outLoop() {
  for (byte i = 0; i < TOSAESP_OUTPUTS; i++)
    if (outPulse[i] > 0) {
      unsigned long cmillis = millis();
      if (millis() - outPulseStart[i] >= outPulse[i] * 1000) {
        outPulse[i] = 0;
        outSetVal(i, outPulseEndVal[i]);
      }
    }
}

bool outCallback(char* subtopic, JsonObject& msg) {
  DEBUG_LOG("outCallback val: ");
  bool mv = msg["val"];
  if (mv) {
    DEBUG_LOG_LN("ON");
  } else {
    DEBUG_LOG_LN("OFF");
  }
  for (byte i = 0; i < TOSAESP_OUTPUTS; i++) {
    if (strlen(subtopic) >= strlen(outputName[i])) {
      if (strncmp(subtopic, outputName[i], strlen(outputName[i])) == 0) {
        bool v = msg["val"];
        outPulseEndVal[i] = outputDefault[i];
        outPulse[i] = outputMaxPulse[i];
        if (msg.containsKey("pulse")) {
          outPulseEndVal[i] = !v;
          outPulse[i] = msg["pulse"];
        }
        outPulseStart[i] = millis();
        outSetVal(i, v);
        return true;
      }
    }
  }
  return false;
}
#endif // TOSAESP_OUTPUTS

#ifdef TOSAESP_INPUTS
volatile time_t inTimestamp[TOSAESP_INPUTS];
volatile unsigned long inLast[TOSAESP_INPUTS];
volatile boolean inValue[TOSAESP_INPUTS];

void ICACHE_RAM_ATTR inTriggered(byte input) {
  if (timeStatus() != timeNotSet) {
    time_t t = now() + timeOffset;
    unsigned long ms = millis();
    if (inLast[input] + inputDebounce[input] < ms) {
      inValue[input] = digitalRead(inputPin[input]);
      inTimestamp[input] = t;
    }
    inLast[input] = ms;
  } else {
    DEBUG_LOG_LN("trigger but time not set");
  }
}

void ICACHE_RAM_ATTR inTriggered_0() {
  inTriggered(0);
}

void ICACHE_RAM_ATTR inTriggered_1() {
  inTriggered(1);
}

void ICACHE_RAM_ATTR inTriggered_2() {
  inTriggered(2);
}

void ICACHE_RAM_ATTR inTriggered_3() {
  inTriggered(3);
}

void ICACHE_RAM_ATTR inTriggered_4() {
  inTriggered(4);
}

void ICACHE_RAM_ATTR inTriggered_5() {
  inTriggered(5);
}

void ICACHE_RAM_ATTR inTriggered_6() {
  inTriggered(6);
}

void ICACHE_RAM_ATTR inTriggered_7() {
  inTriggered(7);
}

void inSetup() {
  for (byte i = 0; i < TOSAESP_INPUTS; i++) {
    inTimestamp[i] = 0;
    inLast[i] = 0;
    pinMode(inputPin[i], inputType[i]);
    if (inputTrigger[i] != 255)
      switch (i) {
        case 0:
          attachInterrupt(inputPin[i], inTriggered_0, inputTrigger[i]);
          break;
        case 1:
          attachInterrupt(inputPin[i], inTriggered_1, inputTrigger[i]);
          break;
        case 2:
          attachInterrupt(inputPin[i], inTriggered_2, inputTrigger[i]);
          break;        
        case 3:
          attachInterrupt(inputPin[i], inTriggered_3, inputTrigger[i]);
          break;        
        case 4:
          attachInterrupt(inputPin[i], inTriggered_4, inputTrigger[i]);
          break;        
        case 5:
          attachInterrupt(inputPin[i], inTriggered_5, inputTrigger[i]);
          break;        
        case 6:
          attachInterrupt(inputPin[i], inTriggered_6, inputTrigger[i]);
          break;        
        case 7:
          attachInterrupt(inputPin[i], inTriggered_7, inputTrigger[i]);
          break;        
      }
  }
}

void inLoop() {
  for (byte i = 0; i < TOSAESP_INPUTS; i++) {
    if (inTimestamp[i] > 0) {
      time_t t = inTimestamp[i];
      inTimestamp[i] = 0;
//      mqttPublishTimestamp(inputName[i], t, inLast[i], false);
      mqttPublishBool(inputName[i], inValue[i], t, inLast[i], true);
    }
  }
}
#endif // TOSAESP_INPUTS

#ifdef TOSAESP_SERIAL
#include <SoftwareSerial.h>

SoftwareSerial sSerial(serialRxPin, serialTxPin);

void serialSetup() {
  sSerial.begin(serialBaud);
  pinMode(serialRxPin, INPUT);
  pinMode(serialTxPin, OUTPUT);
}

void serialLoop() {
  if (sSerial.available()) {
    byte serialBuf[serialBufLen];
    byte serialBufIdx = 0;
    while (sSerial.available()) {
      serialBuf[serialBufIdx] = sSerial.read();
      serialBufIdx++;
    }
    mqttPublishByteArray("serial", serialBuf, serialBufIdx);
  }
}

bool serialCallback(char* subtopic, JsonObject& msg) {
  DEBUG_LOG("serialCallback val: ");
  if (strncmp(subtopic, "serial", 6) == 0) {
    String mv = msg["val"];
    byte mp = msg["pause"];
    unsigned int pos = 0;
    byte mvb[mv.length() >> 1];
    while (pos < mv.length()) {
      char nib1 = mv[pos++];
      char nib2 = mv[pos++];
      byte v = (nib1 <= '9' ? nib1 - '0' : nib1 - 'A' + 0xA) * 0x10;
      v += nib2 <= '9' ? nib2 - '0' : nib2 - 'A' + 0xA;
      mqttPublishLong("serialsend", v, "");
      sSerial.write(v);
      if (mp)
        delay(mp);
    }
    return true;
  }
  return false;
}
#endif // TOSAESP_SERIAL

#ifdef TOSAESP_ONEWIRE
#include <Wire.h>
#include <OneWire.h>

OneWire ow(OW_PIN);

#ifdef TOSAESP_DS18B20
#include <DallasTemperature.h>

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
          DEBUG_LOG("Temperature sensor found: ");
          DEBUG_LOG_LN(dsName[s]);
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
#include <NewPing.h>

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
#include <pcf8574_esp.h>
#include <Wire.h>

PCF857x pcf8574(PCF8574_ADDR, &Wire);

unsigned long pcf8574InputLast[PCF8574_INPUTEND - PCF8574_INPUTSTART + 1];

#ifdef PCF8574_INTPIN
volatile bool pcf8574InterruptFlag = false;

void ICACHE_RAM_ATTR pcf8574Interrupt() {
  pcf8574InterruptFlag = true;
}
#endif

void pcf8574Setup() {
  Wire.begin(I2C_SDAPIN, I2C_SCLPIN);
  Wire.setClock(100000L);
  for (uint8_t i = PCF8574_INPUTSTART; i <= PCF8574_INPUTEND; i++)
    pcf8574InputLast[i - PCF8574_INPUTSTART] = 0;
  pcf8574.begin();
#ifdef PCF8574_INTPIN
  pinMode(PCF8574_INTPIN, INPUT_PULLUP);
  pcf8574.resetInterruptPin();
  attachInterrupt(PCF8574_INTPIN, pcf8574Interrupt, FALLING);
#endif
}

void pcf8574Loop() {
  if (pcf8574InterruptFlag) {
    uint8_t d = pcf8574.read8();
    pcf8574InterruptFlag = false;
    if (timeStatus() != timeNotSet) {
      unsigned long ms = millis();
      time_t t = now() + timeOffset;
      for (uint8_t i = PCF8574_INPUTSTART; i <= PCF8574_INPUTEND; i++) {
        if (bitRead(d, i) == 0) {
          if (pcf8574InputLast[i - PCF8574_INPUTSTART] + pcf8574InputDebounce[i - PCF8574_INPUTSTART] < ms)
            mqttPublishTimestamp(pcf8574InputName[i - PCF8574_INPUTSTART], t, ms, false);
          pcf8574InputLast[i - PCF8574_INPUTSTART] = ms;
        }
      }
    } else {
      DEBUG_LOG_LN("trigger but time not set");
    }
    pcf8574InterruptFlag = false;
  }
}
#endif // TOSAESP_PCF8574

#ifdef TOSAESP_MPU6050
#include <tosaMPU6050.h>

const long mpu6050Delay = 100;
long mpu6050Last = 0;
float mpu6050LastAngles[3] = { 0, 0, 0 };

void mpu6050Setup() {
  Wire.begin(I2C_SDAPIN, I2C_SCLPIN);
  Wire.setClock(400000L);
  uint8_t c;
  //MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  //MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  calibrate_sensors();  
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}

void mpu6050Loop() {
  long now = millis();
  if (now - mpu6050Last > mpu6050Delay) {
    int error;
    double dT;
    accel_t_gyro_union accel_t_gyro;
    read_gyro_accel_vals((uint8_t*) &accel_t_gyro);
    unsigned long t_now = millis();
    float FS_SEL = 131;
    float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
    float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
    float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;
    float accel_x = accel_t_gyro.value.x_accel;
    float accel_y = accel_t_gyro.value.y_accel;
    float accel_z = accel_t_gyro.value.z_accel;
    float RADIANS_TO_DEGREES = 180/3.14159;
    float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
    float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
    float accel_angle_z = 0;
    float dt =(t_now - get_last_time())/1000.0;
    float gyro_angle_x = gyro_x*dt + get_last_x_angle();
    float gyro_angle_y = gyro_y*dt + get_last_y_angle();
    float gyro_angle_z = gyro_z*dt + get_last_z_angle();
    float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
    float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
    float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
    float alpha = 0.96;
    float angles[3];
    angles[0] = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
    angles[1] = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
    angles[2] = gyro_angle_z;  //Accelerometer doesn't give z-angle
    set_last_read_angle_data(t_now, angles[0], angles[1], angles[2], unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
    for (uint8_t i = 0; i < 3; i++) {
      if (mpu6050Publish[i]) {
        if (abs(mpu6050LastAngles[i] - angles[i]) > mpu6050Threshold[i]) {
          mqttPublishFloat(mpu6050Name[i], angles[i], "°", 0, now);
          mpu6050LastAngles[i] = angles[i];
        }
      }
    }
//    DEBUG_LOG(F("DEL:")); DEBUG_LOG(dt, DEC); DEBUG_LOG(F("#ACC:")); DEBUG_LOG(accel_angle_x, 2); DEBUG_LOG(F(",")); DEBUG_LOG(accel_angle_y, 2); DEBUG_LOG(F(",")); DEBUG_LOG(accel_angle_z, 2); DEBUG_LOG(F("#GYR:")); DEBUG_LOG(unfiltered_gyro_angle_x, 2); DEBUG_LOG(F(",")); DEBUG_LOG(unfiltered_gyro_angle_y, 2); DEBUG_LOG(F(",")); DEBUG_LOG(unfiltered_gyro_angle_z, 2); DEBUG_LOG(F("#FIL:")); DEBUG_LOG(angles[0], 2); DEBUG_LOG(F(",")); DEBUG_LOG(angles[1], 2); DEBUG_LOG(F(",")); DEBUG_LOG(angles[2], 2); DEBUG_LOG_LN(F(""));
    mpu6050Last = now;
  }
}
#endif // TOSAESP_MPU6050

#ifdef TOSAESP_MPU9250
#include <quaternionFilters.h>
#include <MPU9250.h>

MPU9250 mpu9250;

void mpu9250Setup() {
  Wire.begin(I2C_SDAPIN, I2C_SCLPIN);
  Wire.setClock(400000L);
  pinMode(MPU9250_INTPIN, INPUT);
  digitalWrite(MPU9250_INTPIN, LOW);
  byte c = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  DEBUG_LOG("MPU9250 is: "); DEBUG_LOG(c, HEX); DEBUG_LOG(" should be: "); DEBUG_LOG_LN(0x71, HEX);
  if (c == 0x71) {
    DEBUG_LOG_LN("MPU9250 is online...");
    mpu9250.MPU9250SelfTest(mpu9250.SelfTest);
    /*
    DEBUG_LOG("x-axis self test: acceleration trim within : ");
    DEBUG_LOG(mpu9250.SelfTest[0],1); DEBUG_LOG_LN("% of factory value");
    DEBUG_LOG("y-axis self test: acceleration trim within : ");
    DEBUG_LOG(mpu9250.SelfTest[1],1); DEBUG_LOG_LN("% of factory value");
    DEBUG_LOG("z-axis self test: acceleration trim within : ");
    DEBUG_LOG(mpu9250.SelfTest[2],1); DEBUG_LOG_LN("% of factory value");
    DEBUG_LOG("x-axis self test: gyration trim within : ");
    DEBUG_LOG(mpu9250.SelfTest[3],1); DEBUG_LOG_LN("% of factory value");
    DEBUG_LOG("y-axis self test: gyration trim within : ");
    DEBUG_LOG(mpu9250.SelfTest[4],1); DEBUG_LOG_LN("% of factory value");
    DEBUG_LOG("z-axis self test: gyration trim within : ");
    DEBUG_LOG(mpu9250.SelfTest[5],1); DEBUG_LOG_LN("% of factory value");
    */
    mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias);
    mpu9250.initMPU9250();
    DEBUG_LOG_LN("MPU9250 initialized for active data mode....");
    byte d = mpu9250.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    DEBUG_LOG("AK8963 is: "); DEBUG_LOG(d, HEX); DEBUG_LOG(" should be: "); DEBUG_LOG_LN(0x48, HEX);
    mpu9250.initAK8963(mpu9250.magCalibration);
    DEBUG_LOG_LN("AK8963 initialized for active data mode....");
    DEBUG_LOG("X-Axis sensitivity adjustment value ");
    DEBUG_LOG_LN(mpu9250.magCalibration[0], 2);
    DEBUG_LOG("Y-Axis sensitivity adjustment value ");
    DEBUG_LOG_LN(mpu9250.magCalibration[1], 2);
    DEBUG_LOG("Z-Axis sensitivity adjustment value ");
    DEBUG_LOG_LN(mpu9250.magCalibration[2], 2);
  } else {
    DEBUG_LOG("Could not connect to MPU9250: 0x"); DEBUG_LOG_LN(c, HEX);
//    while(1) ; // Loop forever if communication doesn't happen
  }
}

void mpu9250Loop() {
  if (mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  
    mpu9250.readAccelData(mpu9250.accelCount);  // Read the x/y/z adc values
    mpu9250.getAres();
    mpu9250.ax = (float)mpu9250.accelCount[0]*mpu9250.aRes; // - accelBias[0];
    mpu9250.ay = (float)mpu9250.accelCount[1]*mpu9250.aRes; // - accelBias[1];
    mpu9250.az = (float)mpu9250.accelCount[2]*mpu9250.aRes; // - accelBias[2];
    mpu9250.readGyroData(mpu9250.gyroCount);  // Read the x/y/z adc values
    mpu9250.getGres();
    mpu9250.gx = (float)mpu9250.gyroCount[0]*mpu9250.gRes;
    mpu9250.gy = (float)mpu9250.gyroCount[1]*mpu9250.gRes;
    mpu9250.gz = (float)mpu9250.gyroCount[2]*mpu9250.gRes;
    mpu9250.readMagData(mpu9250.magCount);  // Read the x/y/z adc values
    mpu9250.getMres();
    mpu9250.magbias[0] = +470.;
    mpu9250.magbias[1] = +120.;
    mpu9250.magbias[2] = +125.;
    mpu9250.mx = (float)mpu9250.magCount[0]*mpu9250.mRes*mpu9250.magCalibration[0] - mpu9250.magbias[0];
    mpu9250.my = (float)mpu9250.magCount[1]*mpu9250.mRes*mpu9250.magCalibration[1] - mpu9250.magbias[1];
    mpu9250.mz = (float)mpu9250.magCount[2]*mpu9250.mRes*mpu9250.magCalibration[2] - mpu9250.magbias[2];
  }
  mpu9250.updateTime();
  MahonyQuaternionUpdate(mpu9250.ax, mpu9250.ay, mpu9250.az, mpu9250.gx*DEG_TO_RAD, mpu9250.gy*DEG_TO_RAD, mpu9250.gz*DEG_TO_RAD, mpu9250.my, mpu9250.mx, mpu9250.mz, mpu9250.deltat);
  mpu9250.delt_t = millis() - mpu9250.count;
  if (mpu9250.delt_t > 500) {
/*
    DEBUG_LOG("ax="); DEBUG_LOG((int)1000*mpu9250.ax);
    DEBUG_LOG(" ay="); DEBUG_LOG((int)1000*mpu9250.ay);
    DEBUG_LOG(" az="); DEBUG_LOG((int)1000*mpu9250.az);
    DEBUG_LOG(" gx="); DEBUG_LOG( mpu9250.gx, 2);
    DEBUG_LOG(" gy="); DEBUG_LOG( mpu9250.gy, 2);
    DEBUG_LOG(" gz="); DEBUG_LOG( mpu9250.gz, 2);
    DEBUG_LOG(" mx="); DEBUG_LOG( (int)mpu9250.mx );
    DEBUG_LOG(" my="); DEBUG_LOG( (int)mpu9250.my );
    DEBUG_LOG(" mz="); DEBUG_LOG( (int)mpu9250.mz );
    DEBUG_LOG(" q0="); DEBUG_LOG(*getQ());
    DEBUG_LOG(" qx="); DEBUG_LOG(*(getQ() + 1));
    DEBUG_LOG(" qy="); DEBUG_LOG(*(getQ() + 2));
    DEBUG_LOG(" qz="); DEBUG_LOG_LN(*(getQ() + 3));
*/
    mpu9250.yaw = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    mpu9250.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() * *(getQ()+2)));
    mpu9250.roll = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
    mpu9250.pitch *= RAD_TO_DEG;
    mpu9250.yaw *= RAD_TO_DEG;
    mpu9250.yaw -= 8.5;
    mpu9250.roll *= RAD_TO_DEG;
    DEBUG_LOG("Y,P,R: ");
    DEBUG_LOG(mpu9250.yaw, 2);
    DEBUG_LOG(", ");
    DEBUG_LOG(mpu9250.pitch, 2);
    DEBUG_LOG(", ");
    DEBUG_LOG(mpu9250.roll, 2);
    DEBUG_LOG(" rate: ");
    DEBUG_LOG((float)mpu9250.sumCount/mpu9250.sum, 2);
    DEBUG_LOG_LN("Hz");
    mpu9250.count = millis();
    mpu9250.sumCount = 0;
    mpu9250.sum = 0;
  }
}
#endif // TOSAESP_MPU9250

#ifdef TOSAESP_COUNTER
const long cntDelay = 1 * 60 * 1000;
long cntLast = 0;
volatile long cntVal = 0;

void ICACHE_RAM_ATTR cntIntCallback() {
  cntVal++;
}

void cntSetup() {
//  pinMode(cntPin, INPUT_PULLUP);
  pinMode(cntPin, INPUT);
  int edge = CHANGE;
  if (cntEdge == 'r')
    edge = RISING;
  else if (cntEdge == 'f')
    edge = FALLING;
  cntVal = 0;
  attachInterrupt(digitalPinToInterrupt(cntPin), cntIntCallback, edge);
}

void cntLoop() {
  long now = millis();
  if (now - cntLast > cntDelay) {
    long v = cntVal;
    float vl = (v * cntFactor) + cntOffset;
    mqttPublishFloat(cntName, vl, "l");
    cntVal -= v;
    cntLast = now;
  }
}
#endif // TOSAESP_COUNTER

#ifdef TOSAESP_HLW8012
/*
#define V_REF               2.43
#define R_CURRENT           0.001
#define R_VOLTAGE           2821
#define F_OSC               3579000
_current_multiplier = ( 1000000.0 * 512 * V_REF / R_CURRENT / 24.0 / F_OSC );                     // 14484
_voltage_multiplier = ( 1000000.0 * 512 * V_REF * R_VOLTAGE / 2.0 / F_OSC );                      // 490
_power_multiplier = ( 1000000.0 * 128 * V_REF * V_REF * R_VOLTAGE / R_CURRENT / 48.0 / F_OSC );   // 12411
*/
#define HLW8012_PREF            10000    // 1000.0W
#define HLW8012_UREF             2200    // 220.0V
#define HLW8012_IREF             4545    // 4.545A
#define HLW8012_PREF_PULSE      12530    // was 4975us = 201Hz = 1000W
#define HLW8012_UREF_PULSE       1950    // was 1666us = 600Hz = 220V
#define HLW8012_IREF_PULSE       3500    // was 1666us = 600Hz = 4.545A
unsigned long hlw8012PCal = HLW8012_PREF_PULSE;
unsigned long hlw8012UCal = HLW8012_UREF_PULSE;
unsigned long hlw8012ICal = HLW8012_IREF_PULSE;
unsigned long hlw8012Last = 0;
unsigned long hlw8012LastSec = 0;
unsigned long hlw8012LastSend = 0;
float hlw8012Voltage = 0;
float hlw8012Current = 0;
float hlw8012Power = 0;
float hlw8012PowerMin = 0;
float hlw8012PowerMax = 0;
float hlw8012PowerSum = 0;
unsigned int hlw8012PowerCount = 0;
float hlw8012ApparentPower = 0;
float hlw8012ApparentPowerMin = 0;
float hlw8012ApparentPowerMax = 0;
float hlw8012ApparentPowerSum = 0;
unsigned int hlw8012ApparentPowerCount = 0;
float hlw8012PowerFactor = 0;
volatile boolean hlw8012Off = true;
volatile unsigned long hlw8012CFPulseLast = 0;
volatile unsigned long hlw8012CFPulseLen = 0;
volatile unsigned long hlw8012CFPulseCount = 0;
byte hlw8012CF1Sel = 0;
volatile unsigned long hlw8012CF1PulseLast = 0;
volatile unsigned long hlw8012CF1PulseLen = 0;
volatile unsigned long hlw8012CF1PulseSum = 0;
volatile unsigned long hlw8012CF1PulseCount = 0;
volatile byte hlw8012CF1State = 0;
unsigned long hlw8012Energy_kWh = 0;

void ICACHE_RAM_ATTR hlw8012CFInterrupt() {
  unsigned long us = micros();
  if (hlw8012Off) {
    hlw8012CFPulseLast = us;
    hlw8012Off = false;
  } else {
    hlw8012CFPulseLen = us - hlw8012CFPulseLast;
    hlw8012CFPulseLast = us;
    hlw8012CFPulseCount++;
  }
}

void ICACHE_RAM_ATTR hlw8012CF1Interrupt() {
  unsigned long us = micros();
  hlw8012CF1PulseLen = us - hlw8012CF1PulseLast;
  hlw8012CF1PulseLast = us;
  if ((hlw8012CF1State > 2) && (hlw8012CF1State < 8)) {
    hlw8012CF1PulseSum += hlw8012CF1PulseLen;
    hlw8012CF1PulseCount++;
    if (10 == hlw8012CF1PulseCount)
      hlw8012CF1State = 8;
  }
}

void hlw8012Setup() {
  pinMode(hlw8012SelPin, OUTPUT);
  digitalWrite(hlw8012SelPin, hlw8012CF1Sel);
  pinMode(hlw8012CF1Pin, INPUT_PULLUP);
  attachInterrupt(hlw8012CF1Pin, hlw8012CF1Interrupt, FALLING);
  pinMode(hlw8012CFPin, INPUT_PULLUP);
  attachInterrupt(hlw8012CFPin, hlw8012CFInterrupt, FALLING);
}

void hlw8012Loop() {
  unsigned long now = millis();
  
  if (now - hlw8012Last > 200) {

    if (hlw8012SendSeconds > 0) {
      if (now - hlw8012LastSend > 1000 * hlw8012SendSeconds) {
        mqttPublishFloatMMA("ActivePower", hlw8012Power, hlw8012PowerMin, hlw8012PowerCount ? hlw8012PowerSum / hlw8012PowerCount : 0, hlw8012PowerMax, "W", 1);
        mqttPublishFloatMMA("ApparentPower", hlw8012ApparentPower, hlw8012ApparentPowerMin, hlw8012ApparentPowerCount ? hlw8012ApparentPowerSum / hlw8012ApparentPowerCount : 0, hlw8012ApparentPowerMax, "VA", 1);
        mqttPublishFloat("PowerFactor", hlw8012PowerFactor, "", 5);
        if (hlw8012CFPulseCount)
          mqttPublishLong("PowerTicks", hlw8012CFPulseCount, "#");
        mqttPublishFloat("Voltage", hlw8012Voltage, "V", 1);
        mqttPublishFloat("Current", hlw8012Current, "A", 3);
        hlw8012CFPulseCount = 0;
        hlw8012PowerMin = 0;
        hlw8012PowerMax = 0;
        hlw8012PowerSum = 0;
        hlw8012PowerCount = 0;
        hlw8012ApparentPowerMin = 0;
        hlw8012ApparentPowerMax = 0;
        hlw8012ApparentPowerSum = 0;
        hlw8012ApparentPowerCount = 0;
        hlw8012LastSend = now;
      }
    }

    if (micros() - hlw8012CFPulseLast > 10000000) {
      hlw8012CFPulseLen = 0;
      hlw8012Off = true;
    }

    if (hlw8012CFPulseLen && !hlw8012Off) {
      unsigned long hlw_w = (HLW8012_PREF * hlw8012PCal) / hlw8012CFPulseLen;
      hlw8012Power = (float)hlw_w / 10;
    } else
      hlw8012Power = 0;

    hlw8012PowerSum += hlw8012Power;
    hlw8012PowerCount++;
    if ((hlw8012Power < hlw8012PowerMin) || (hlw8012PowerMin == 0))
      hlw8012PowerMin = hlw8012Power;
    if (hlw8012Power > hlw8012PowerMax)
      hlw8012PowerMax = hlw8012Power;

    hlw8012CF1State++;
    if (hlw8012CF1State >= 8) {
      hlw8012CF1State = 0;
      hlw8012CF1Sel = (hlw8012CF1Sel) ? 0 : 1;
      digitalWrite(hlw8012SelPin, hlw8012CF1Sel);
      if (hlw8012CF1PulseCount)
        hlw8012CF1PulseLen = hlw8012CF1PulseSum / hlw8012CF1PulseCount;
      else
        hlw8012CF1PulseLen = 0;
      if (hlw8012CF1Sel) {
        if (hlw8012CF1PulseLen) {
          unsigned long hlw_u = (HLW8012_UREF * hlw8012UCal) / hlw8012CF1PulseLen;
          hlw8012Voltage = (float)hlw_u / 10;
        } else
          hlw8012Voltage = 0;
      } else {
        if (hlw8012CF1PulseLen) {
          unsigned long hlw_i = (HLW8012_IREF * hlw8012ICal) / hlw8012CF1PulseLen;
          hlw8012Current = (float)hlw_i / 1000;
        } else
          hlw8012Current = 0;
      }
      
      hlw8012ApparentPower = hlw8012Voltage * hlw8012Current;
      hlw8012ApparentPowerSum += hlw8012ApparentPower;
      hlw8012ApparentPowerCount++;
      if ((hlw8012ApparentPower < hlw8012ApparentPowerMin) || (hlw8012ApparentPowerMin == 0))
        hlw8012ApparentPowerMin = hlw8012ApparentPower;
      if (hlw8012ApparentPower > hlw8012ApparentPowerMax)
        hlw8012ApparentPowerMax = hlw8012ApparentPower;
      hlw8012CF1PulseSum = 0;

      hlw8012CF1PulseCount = 0;
    }

    if (hlw8012Voltage && hlw8012Current && hlw8012Power) {
      if (hlw8012Power > hlw8012Voltage * hlw8012Current)
        hlw8012PowerFactor = 1;
      else
        hlw8012PowerFactor = hlw8012Power / (hlw8012Voltage * hlw8012Current);
    } else
      hlw8012PowerFactor = 0;
    
    hlw8012Last = now;
  }
}

bool hlw8012Callback(char* subtopic, JsonObject& msg) {
  /*
  if (strncmp(subtopic, "$calibrate", 10) == 0) {
    double ep = msg["power"];
    double ev = msg["voltage"];
    hlw8012.expectedActivePower(ep);
    hlw8012.expectedVoltage(ev);
    hlw8012.expectedCurrent(ep / ev);
    double cm = hlw8012.getCurrentMultiplier();
    mqttPublishFloat("CurrentMultiplier", cm, "");    
    double vm = hlw8012.getVoltageMultiplier();
    mqttPublishFloat("VoltageMultiplier", vm, "");    
    double pm = hlw8012.getPowerMultiplier();
    mqttPublishFloat("PowerMultiplier", pm, "");    
    // ToDo : persist multipliers
    return true;
  } else if (strncmp(subtopic, "$multipliers", 12) == 0) {
    double cm = msg["current"];
    hlw8012.setCurrentMultiplier(cm);
    double vm = msg["voltage"];
    hlw8012.setVoltageMultiplier(vm);
    double pm = msg["power"];
    hlw8012.setPowerMultiplier(pm);
    // ToDo : persist multipliers
    return true;
  }
  */
  return false;
}
#endif // TOSAESP_HLW8012

#ifdef TOSAESP_SCHEDULER
const long schedDelay = 15 * 1000;
long schedLast = 0;
unsigned long LastExecution[TOSAESP_SCHEDULER_EVENTS];
#define EVT_MAX_ACTION_LEN    64
#define EVT_CONFIG_MAGIC      0x2334ae68

typedef struct{
	uint16_t MinuteOfDay;
	uint8_t WeekdayMask;
	uint8_t Enabled:1;
	uint8_t Holiday:1;
	uint8_t Vacation:1;
	uint8_t Interval:1;
	uint8_t Reserved:2;
	uint8_t Random:2;
	char Action[EVT_MAX_ACTION_LEN];
} EVENT;

typedef struct {
	uint32 magic;
	EVENT event[TOSAESP_SCHEDULER_EVENTS];
	uint32_t NextHoliday[TOSAESP_SCHEDULER_HOLIDAYS];
	uint32_t NextVacationStart;
	uint32_t NextVacationEnd;
} EVENTS;

EVENTS events;

void schedLoadEvents(bool reset) {
  uint8_t* bevents = (uint8_t*)&events;
  for (size_t i = 0; i <= sizeof(events); i++) 
	  bevents[i] = (uint8_t)0;
  for (size_t i = 0; i <= sizeof(events); i++) 
    bevents[i] = (uint8_t)EEPROM.read(i); 
	if(events.magic != EVT_CONFIG_MAGIC || reset) {
    for (size_t i = 0; i <= sizeof(events); i++) 
	    bevents[i] = (uint8_t)0;
		// ToDo
		schedSaveEvents();
	}
}

void schedSaveEvents() {
	events.magic = EVT_CONFIG_MAGIC;
  uint8_t* bevents = (uint8_t*)&events;
  for (size_t i = 0; i <= sizeof(events); i++) 
    EEPROM.write(i, (uint8_t)bevents[i]); 
  EEPROM.commit();
}

bool schedCallback(char* subtopic, JsonObject& msg) {
  if (strncmp(subtopic, "$schedule", 9) == 0)
    schedUpdateEvent(msg);
}

void schedUpdateEvent(JsonObject& msg) {
	// schedule json syntax:
	// {"i":...,en":...,"mod":...,"wd":...,"h":...,"v":...,"r":...,"a":...}
  uint8_t index = msg["i"];
	EVENT e = events.event[index];
  if (msg.containsKey("en"))
	  e.Enabled = msg["en"];
  if (msg.containsKey("int"))
    e.Interval = msg["int"];
  if (msg.containsKey("mod"))
    e.MinuteOfDay = msg["mod"];
  if (msg.containsKey("wd"))
    e.WeekdayMask = msg["wd"];
  if (msg.containsKey("h"))
    e.Holiday = msg["h"];
  if (msg.containsKey("v"))
    e.Vacation = msg["v"];
  if (msg.containsKey("r"))
	  e.Random = msg["r"];
  if (msg.containsKey("a")) {
    const char* a = msg["a"];
    bool end = false;
    for (uint8_t i = 0; i < EVT_MAX_ACTION_LEN; i++) {
      if (end || a[i] == 0) {
        e.Action[i] = 0;
        end = true;
      } else if (a[i] == '\'')
        e.Action[i] = '\"';
      else
        e.Action[i] = a[i];
    }
  }
	schedSaveEvents();
	schedPublishEvent(index);
}

void schedPublishEvent(uint8_t index) {
	EVENT e = events.event[index];
	if (e.Enabled > 0) {
		char t[12];
		sprintf(t, "$event/%d", index);
		char a[EVT_MAX_ACTION_LEN];
		memcpy(a, e.Action, EVT_MAX_ACTION_LEN);
		for (uint8_t i = 0; i < EVT_MAX_ACTION_LEN; i++)
			if (a[i] == '\"')
				a[i] = '\'';
		char jt[80 + EVT_MAX_ACTION_LEN];
		sprintf(jt, "{\"en\":%d,\"mod\":%d,\"wd\":%d,\"h\":%d,\"v\":%d,\"r\":%d,\"a\":\"%s\"}", e.Enabled, e.MinuteOfDay, e.WeekdayMask, e.Holiday, e.Vacation, e.Random, a);
		mqttPublish(t, jt, true);
	}
}

void schedCheckEvents() {
	if (timeStatus() != timeNotSet) {
		time_t t = now() + timeOffset;
		uint16_t mod = minute(t) + (hour(t) * 60);
		uint8_t wd = weekday(t);
		bool IsHoliday = false;
		for (uint8_t i = 0; i < TOSAESP_SCHEDULER_HOLIDAYS; i++)
			if ((t > events.NextHoliday[i]) && (t < events.NextHoliday[i] + SECS_PER_DAY))
				IsHoliday = true;
		bool IsVacation = ((t > events.NextVacationStart) && (t < events.NextVacationEnd + SECS_PER_DAY));
		for (uint8_t i = 0; i < TOSAESP_SCHEDULER_EVENTS; i++) {
			EVENT e = events.event[i];
			if (e.Enabled > 0) {
				if (e.Interval > 0) {
					if (LastExecution[i] <= t - (e.MinuteOfDay * 60)) {
						schedExecEvent(e.Action);
						LastExecution[i] = t;
					}
				} else {
					if ((mod >= e.MinuteOfDay) && (mod < e.MinuteOfDay + 30))
					{
						if (LastExecution[i] < t - SECS_PER_HOUR)
						{
							bool exec = false;
							if (IsHoliday && e.Holiday)
								exec = true;
							else if (IsVacation && e.Vacation)
								exec = true;
							else if ((e.WeekdayMask & (1 << wd)) > 0)
								exec = true;
							if (exec) {
								schedExecEvent(e.Action);
								LastExecution[i] = t;
							}
						}
					}
				}
			}
		}
	}
}

void schedExecEvent(char* action) {
	char *s = action;
	while (s[0] != '|')
		s++;
	uint8_t l = s - action;
	char *topic = new char[l + 1];
	ets_memcpy(topic, action, l);
	topic[l] = '\0';
	s++;
	l = ets_strlen(action) - (s - action);
	char *data = new char[l + 1];
	ets_memcpy(data, s, l);
	data[l] = '\0';
  handleMessage(topic, data);
	delete data;
	delete topic;
}

void schedSetup() {
  schedLoadEvents(false);
	for (uint8_t i = 0; i < TOSAESP_SCHEDULER_EVENTS; i++)
		LastExecution[i] = 0;
}

void schedLoop() {
  long now = millis();
  if (now - schedLast > schedDelay) {
    schedCheckEvents();
    schedLast = now;
  }
}
#endif // TOSAESP_SCHEDULER

void mainSetup() {
#ifdef TOSAESP_OUTPUTS
  commonSetup();
  outSetup();
#endif
#ifdef TOSAESP_INPUTS
  commonSetup();
  inSetup();
#endif
#ifdef TOSAESP_SERIAL
  commonSetup();
  serialSetup();
#endif
#ifdef TOSAESP_COUNTER
  commonSetup();
  cntSetup();
#endif
#ifdef TOSAESP_DS18B20
  commonSetup();
  dsSetup();
#endif
#ifdef TOSAESP_HCSR04
  commonSetup();
  usSetup();
#endif
#ifdef TOSAESP_PCF8574
  commonSetup();
  pcf8574Setup();
#endif
#ifdef TOSAESP_MPU6050
  commonSetup();
  mpu6050Setup();
#endif
#ifdef TOSAESP_MPU9250
  commonSetup();
  mpu9250Setup();
#endif
#ifdef TOSAESP_HLW8012
  commonSetup();
  hlw8012Setup();
#endif
#ifdef TOSAESP_SCHEDULER
  commonSetup();
  schedSetup();
#endif
}

void mainLoop() {
#ifdef TOSAESP_OUTPUTS
  commonLoop();
  outLoop();
#endif
#ifdef TOSAESP_INPUTS
  commonLoop();
  inLoop();
#endif
#ifdef TOSAESP_SERIAL
  commonLoop();
  serialLoop();
#endif
#ifdef TOSAESP_COUNTER
  commonLoop();
  cntLoop();
#endif
#ifdef TOSAESP_DS18B20
  commonLoop();
  dsLoop();
#endif
#ifdef TOSAESP_HCSR04
  commonLoop();
  usLoop();
#endif
#ifdef TOSAESP_PCF8574
  commonLoop();
  pcf8574Loop();
#endif
#ifdef TOSAESP_MPU6050
  commonLoop();
  mpu6050Loop();
#endif
#ifdef TOSAESP_MPU9250
  commonLoop();
  mpu9250Loop();
#endif
#ifdef TOSAESP_HLW8012
  commonLoop();
  hlw8012Loop();
#endif
#ifdef TOSAESP_SCHEDULER
  commonLoop();
  schedLoop();
#endif
}
