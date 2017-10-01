#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <stdlib.h>
#include <EEPROM.h>
#include <TimeLib.h> 

#include <Wire.h>
#include <PCF8574.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NewPing.h>
#include <PCF8574.h>

#include "device.h"

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

const char* mqttSysPrefix = "hoco/esp_pio/";

// mqtt

StaticJsonBuffer<200> jsonBuffer;
WiFiClientSecure wifiClient;
char mqttPrefix[50];

long mqttLastConnectAttempt = 0;

bool handleMessage(char* topic, char* payload) {
  bool handled = false;
  JsonObject& msg = jsonBuffer.parseObject(payload);
#ifdef TOSAESP_SCHEDULER
  if (!handled) handled = schedCallback(topic, msg);
#endif
#ifdef TOSAESP_OUTPUTS
  if (!handled) handled = outCallback(topic, msg);
#endif
  return handled;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  bool handled = false;
  if (strncmp(topic, mqttSysPrefix, strlen(mqttSysPrefix)) != 0)
    return;
  char* dtopic = topic + strlen(mqttSysPrefix);
  if (strncmp(dtopic, "broadcast", 9) == 0) {
    // handle broadcast
    char subtopic[50];
    sprintf(subtopic, "%s", dtopic + 9 + 1);
    JsonObject& msg = jsonBuffer.parseObject((char*)payload);
#ifdef TOSAESP_SCHEDULER
    if (!handled) handled = schedSetTime(subtopic, msg);
#endif
  } else if (strncmp(dtopic, deviceName, strlen(deviceName)) == 0) {
    // handle device topic
    char subtopic[50];
    sprintf(subtopic, "%s", dtopic + strlen(deviceName) + 1);
    if (strncmp(subtopic, "$reboot", 7) == 0) {
      ESP.reset();
    } else {
      handled = handleMessage(subtopic, (char*)payload);
    }
  }
}

PubSubClient mqttClient(wifiClient);

void mqttSubscribe(const char *topic) {
    char t[100];
    sprintf(t, "%s%s", mqttPrefix, topic);
    mqttClient.subscribe(t);
}

bool mqttBroadcastSubscribed = false;

void mqttSubscribeBroadcast() {
    if (!mqttBroadcastSubscribed) {
      char t[30];
      sprintf(t, "%sbroadcast/#", mqttSysPrefix);
      mqttClient.subscribe(t);
      mqttBroadcastSubscribed = true;
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
  mqttSubscribe("$reboot");
#ifdef TOSAESP_SCHEDULER
  schedSubscribe();
#endif
#ifdef TOSAESP_OUTPUTS
  outSubscribe();
#endif
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
      if (mqttClient.connect(deviceName, mqttUser, mqttPass, lwt, 0, true, "false")) {
        mqttLastConnectAttempt = 0;
        Serial.println("MQTT Connected.");
        mqttAnnounce();
      } else {
        Serial.println("MQTT Connection Failed!");
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
  Serial.print("WiFi Connect");
  WiFi.mode(WIFI_STA);
/*
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("WiFi Connect: MAC: ");
  for (int i = 0; i < 5; i++)
    Serial.print(String(mac[i],HEX) + ":");
  Serial.println(String(mac[5],HEX));
*/
  delay(10);
  WiFi.begin(wifiSsid, wifiPassword);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Connection Failed! Rebooting...");
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
    Serial.println("OTA started " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA finished");
    delay(2000);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA: %u%%\r", (progress / (total / 100)));
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
  sprintf(mqttPrefix, "%s%s/", mqttSysPrefix, deviceName);
  wifiConnect();
  otaInit();
  mqttSetup();
  mainSetup();
  Serial.println("Setup complete.");
}

void loop() {
  yield();
  if (WiFi.status() != WL_CONNECTED)
    wifiConnect();
  else {
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

void outSubscribe() {
  char t[100];
  for (byte i = 0; i < TOSAESP_OUTPUTS; i++) {
    sprintf(t, "%s/$set", outputName[i]);
    mqttSubscribe(t);
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
  for (byte i = 0; i < TOSAESP_OUTPUTS; i++) {
    if (strlen(subtopic) > strlen(outputName[i])) {
      if (strncmp(subtopic, outputName[i], strlen(outputName[i])) == 0) {
        char* cmd = subtopic + strlen(outputName[i]);
        if (cmd[0] == '/') {
          cmd++;
          if (strncmp(cmd, "$set", 4) == 0) {
            bool v = msg["val"];
            outPulseEndVal[i] = outputDefault[i];
            outPulse[i] = outputMaxPulse[i];
            if (msg.containsKey("pulse")) {
              outPulseEndVal[i] = !v;
              outPulse[i] = msg["pulse"];
            }
            outPulseStart[i] = millis();
            outSetVal(i, v);
          }
          return true;
        }
      }
    }
  }
  return false;
}
#endif // TOSAESP_OUTPUTS

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

#ifdef TOSAESP_SCHEDULER
const long schedDelay = 15 * 1000;
long schedLast = 0;
unsigned long LastExecution[TOSAESP_SCHEDULER_EVENTS];
int32_t schedOffset = 0;
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

bool schedSetTime(char* subtopic, JsonObject& msg) {
	if (strncmp(subtopic, "$time", 5) != 0)
    return false;
  schedOffset = msg["offset"];
  unsigned long epoch = msg["epoch"];
  setTime(epoch);
  return true;
}

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
		time_t t = now() + schedOffset;
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

void schedSubscribe() {
  mqttSubscribeBroadcast();
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
#ifdef TOSAESP_SCHEDULER
  commonLoop();
  schedLoop();
#endif
}
