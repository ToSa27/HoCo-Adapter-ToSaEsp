//env:TOSAESP_BOARD=nodemcuv2
//env:TOSAESP_LIB_DEPS="OneWire, DallasTemperature, PCF8574, NewPing, HLW8012"

#define TOSAESP_TIME

#define TOSAESP_SERIAL
byte serialRxPin = 4;
byte serialTxPin = 5;
long serialBaud = 115200;
int serialBufLen = 100;


#define TOSAESP_OUTPUTS   1
byte outputPin[TOSAESP_OUTPUTS] = { 2 };
bool outputInv[TOSAESP_OUTPUTS] = { true };
bool outputDefault[TOSAESP_OUTPUTS] = { false };
int outputMaxPulse[TOSAESP_OUTPUTS] = { 0 };
const char* outputName[TOSAESP_OUTPUTS] = {
  "Wake"
};
