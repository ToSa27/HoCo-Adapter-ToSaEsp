//env:TOSAESP_BOARD=nodemcuv2
//env:TOSAESP_LIB_DEPS="OneWire, DallasTemperature, PCF8574, NewPing, HLW8012"

#define TOSAESP_ONEWIRE
#define OW_PIN            2

#define TOSAESP_DS18B20
#define DS_SENSORS        1
const char* dsName[DS_SENSORS] = {
  ""
};
DeviceAddress dsAddress[DS_SENSORS] = {
  {0x28,0xff,0xca,0x31,0x86,0x16,0x05,0xda}
};
/*
#define TOSAESP_COUNTER
byte cntPin = 5;
char cntEdge = 'r';
const char* cntName = "Wasserzaehler";
float cntFactor = 1 / 4.5;
float cntOffset = 0;
*/
#define TOSAESP_OUTPUTS   3
byte outputPin[TOSAESP_OUTPUTS] = { 14, 12, 13 };
bool outputInv[TOSAESP_OUTPUTS] = { true, true, true };
bool outputDefault[TOSAESP_OUTPUTS] = { false, false, false };
int outputMaxPulse[TOSAESP_OUTPUTS] = { 1800, 1800, 1800 };
const char* outputName[TOSAESP_OUTPUTS] = {
  "VentilBlumen",
  "VentilVeranda",
  "VentilRasen"
};
/*
#define TOSAESP_SCHEDULER
#define TOSAESP_SCHEDULER_EEPROMADDR  0
#define TOSAESP_SCHEDULER_EVENTS      (TOSAESP_OUTPUTS * 10)
#define TOSAESP_SCHEDULER_HOLIDAYS    3
#define TOSAESP_SCHEDULER_VACATIONS   1
*/