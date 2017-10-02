//env:TOSAESP_BOARD=nodemcuv2
//env:TOSAESP_LIB_DEPS="OneWire, DallasTemperature, PCF8574, NewPing, HLW8012"

#define TOSAESP_INPUTS   1
byte inputPin[TOSAESP_INPUTS] = { 5 };
byte inputTrigger[TOSAESP_INPUTS] = { RISING };
byte inputType[TOSAESP_INPUTS] = { INPUT_PULLUP };
byte inputDebounce[TOSAESP_INPUTS] = { 100 };
const char* inputName[TOSAESP_INPUTS] = {
  "Drehstromzähler"
};
const char* inputUoM[TOSAESP_INPUTS] = {
  "Wh"
};
  