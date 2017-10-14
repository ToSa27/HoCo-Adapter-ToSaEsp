//env:TOSAESP_BOARD=nodemcuv2
//env:TOSAESP_LIB_DEPS="OneWire, DallasTemperature, PCF8574, NewPing, HLW8012"

#define TOSAESP_INPUTS   4
byte inputPin[TOSAESP_INPUTS] = { 5, 4, 0, 2 };
byte inputTrigger[TOSAESP_INPUTS] = { RISING, RISING, RISING, RISING };
byte inputType[TOSAESP_INPUTS] = { INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP };
byte inputDebounce[TOSAESP_INPUTS] = { 100, 100, 100, 100 };
const char* inputName[TOSAESP_INPUTS] = {
  "Drehstromzähler",
  "Waschmaschine",
  "Serverschrank",
  "Heizung"
};
const char* inputUoM[TOSAESP_INPUTS] = {
  "Wh",
  "1/2 Wh",
  "1/2 Wh",
  "1/2 Wh"
};
  