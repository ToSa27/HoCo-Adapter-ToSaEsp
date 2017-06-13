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

#define TOSAESP_COUNTER
byte cntPin = 5;
char cntEdge = 'r';
const char* cntName = "Wasserzaehler";
float cntFactor = 1 / 4.5;
float cntOffset = 0;

#define TOSAESP_OUTPUTS   3
byte outputPin[TOSAESP_OUTPUTS] = { 13, 12, 14 };
bool outputInv[TOSAESP_OUTPUTS] = { true, true, true };
const char* outputName[TOSAESP_OUTPUTS] = {
  "VentilBlumen",
  "VentilRasen",
  "VentilVeranda"
};
