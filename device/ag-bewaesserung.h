const char* deviceName = "ag-bewaesserung";

#define TOSAESP_ONEWIRE
#include <OneWire.h>
#define OW_PIN            2

#define TOSAESP_DS18B20
#include <DallasTemperature.h>
#define DS_SENSORS        1
const char* dsName[DS_SENSORS] = {
  ""
};
DeviceAddress dsAddress[DS_SENSORS] = {
  {0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??}
};

#define TOSAESP_COUNTER
#define CNT_PIN           5
