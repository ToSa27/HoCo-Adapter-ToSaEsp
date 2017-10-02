//env:TOSAESP_BOARD=esp8285
//env:TOSAESP_BOARDFLASHMODE=dout

#define TOSAESP_OUTPUTS   5
byte outputPin[TOSAESP_OUTPUTS] = { 12, 5, 4, 15, 13 };
bool outputInv[TOSAESP_OUTPUTS] = { false, false, false, false, true };
bool outputDefault[TOSAESP_OUTPUTS] = { false, false, false, false, false };
int outputMaxPulse[TOSAESP_OUTPUTS] = { 0, 0, 0, 0, 0 };
const char* outputName[TOSAESP_OUTPUTS] = {
  "Relais1",
  "Relais2",
  "Relais3",
  "Relais4",
  "LED"
};

#define TOSAESP_INPUTS   5
byte inputPin[TOSAESP_INPUTS] = { 2, 0, 9, 10, 14 };
byte inputTrigger[TOSAESP_INPUTS] = { RISING, 255, 255, 255, 255 };
byte inputType[TOSAESP_INPUTS] = { INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP };
byte inputDebounce[TOSAESP_INPUTS] = { 100, 100, 100, 100, 100 };
const char* inputName[TOSAESP_INPUTS] = {
  "Drehstromzähler",
  "Taster1",
  "Taster2",
  "Taster3",
  "Taster4"
};
const char* inputUoM[TOSAESP_INPUTS] = {
  "Wh",
  "",
  "",
  "",
  ""
};
