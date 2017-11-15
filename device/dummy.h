//env:TOSAESP_BOARD=nodemcuv2
//env:TOSAESP_LIB_DEPS="OneWire, DallasTemperature, PCF8574, NewPing, HLW8012"

#define TOSAESP_TIME  

#define TOSAESP_INPUTS   8
byte inputPin[TOSAESP_INPUTS] = { 5, 4, 0, 2, 14, 12, 13, 15 };
byte inputTrigger[TOSAESP_INPUTS] = { RISING, RISING, RISING, RISING, RISING, RISING, RISING, RISING };
byte inputType[TOSAESP_INPUTS] = { INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP };
unsigned int inputDebounce[TOSAESP_INPUTS] = { 250, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
const char* inputName[TOSAESP_INPUTS] = {
  "Drehstromzähler",
  "Kühlschrank",
  "Serverschrank",
  "Heizung",
  "Geschirrspüler",
  "Waschmaschine",
  "Unbekannt1",
  "Unbekannt2"
};
const char* inputUoM[TOSAESP_INPUTS] = {
  "Wh",
  "1/2 Wh",
  "1/2 Wh",
  "1/2 Wh",
  "1/2 Wh",
  "1/2 Wh",
  "1/2 Wh",
  "1/2 Wh"
};
  