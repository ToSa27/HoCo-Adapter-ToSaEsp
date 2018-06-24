//env:TOSAESP_BOARD=esp01_1m
//env:TOSAESP_BOARDFLASHMODE=dout

#define TOSAESP_INPUTS   1
byte inputPin[TOSAESP_INPUTS] = { 0 };
byte inputTrigger[TOSAESP_INPUTS] = { CHANGE };
byte inputType[TOSAESP_INPUTS] = { INPUT };
unsigned int inputDebounce[TOSAESP_INPUTS] = { 250 };
const char* inputName[TOSAESP_INPUTS] = {
  "Taster"
};
const char* inputUoM[TOSAESP_INPUTS] = {
  ""
};

#define TOSAESP_OUTPUTS   2
byte outputPin[TOSAESP_OUTPUTS] = { 12, 13 };
bool outputInv[TOSAESP_OUTPUTS] = { false, true };
bool outputDefault[TOSAESP_OUTPUTS] = { false, false };
int outputMaxPulse[TOSAESP_OUTPUTS] = { 0, 0 };
const char* outputName[TOSAESP_OUTPUTS] = {
  "Relais",
  "LED"
};

// GPIO 0 : pushbutton
// GPIO 1 : nc (RX)
// GPIO 2 : nc
// GPIO 3 : nc (TX)
// GPIO 4 : nc
// GPIO 5 : nc
// GPIO 6 : nc (flash)
// GPIO 7 : nc (flash)
// GPIO 8 : nc (flash)
// GPIO 9 : nc (flash)
// GPIO10 : nc (flash)
// GPIO11 : nc (flash)
// GPIO12 : Relay (0 = Off, 1 = On)
// GPIO13 : LED green (0 = On, 1 = Off)
// GPIO14 : nc
// GPIO15 : nc
// GPIO16 : nc
// ADC    : nc

