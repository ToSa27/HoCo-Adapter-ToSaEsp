#include <OneWire.h>
#include <DallasTemperature.h>

OneWire ow(OW_PIN);
DallasTemperature ds(&ow);

extern const char* dsName[DS_SENSORS];
extern DeviceAddress dsAddress[DS_SENSORS];

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
