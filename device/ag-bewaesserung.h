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
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
};

#define TOSAESP_COUNTER
byte cntPin = 5;
char cntEdge = 'r';
const char* cntName = "Wasserzaehler";

#define TOSAESP_OUTPUTS   3
byte outputPin[TOSAESP_OUTPUTS] = { 13, 12, 14 };
bool outputInv[TOSAESP_OUTPUTS] = { false, false, false };
const char* outputName[TOSAESP_OUTPUTS] = {
  "VentilBlumen",
  "VentilRasen",
  "VentilVeranda"
};
