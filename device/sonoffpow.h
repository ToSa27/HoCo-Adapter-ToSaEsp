//env:TOSAESP_BOARD=esp01_1m
//env:TOSAESP_BOARDFLASHMODE=dout
//env:TOSAESP_LIB_DEPS=HLW8012

/*
#define TOSAESP_INPUTS   1
byte inputPin[TOSAESP_INPUTS] = { 0 };
byte inputTrigger[TOSAESP_INPUTS] = { NONE };
byte inputType[TOSAESP_INPUTS] = { INPUT };
unsigned int inputDebounce[TOSAESP_INPUTS] = { 250 };
const char* inputName[TOSAESP_INPUTS] = {
  "Taster"
};
const char* inputUoM[TOSAESP_INPUTS] = {
  ""
};
*/

#define TOSAESP_OUTPUTS   2
byte outputPin[TOSAESP_OUTPUTS] = { 12, 15 };
bool outputInv[TOSAESP_OUTPUTS] = { false, false };
bool outputDefault[TOSAESP_OUTPUTS] = { true, false };
int outputMaxPulse[TOSAESP_OUTPUTS] = { 0, 0 };
const char* outputName[TOSAESP_OUTPUTS] = {
  "Relais",
  "LED"
};

#define TOSAESP_HLW8012
byte hlw8012SelPin = 5;
byte hlw8012CF1Pin = 13;
byte hlw8012CFPin = 14;
double hlw8012CurrentMultiplier = 16880.00;
double hlw8012VoltageMultiplier = 443332.06;
double hlw8012PowerMultiplier = 13791482.00;
byte hlw8012Samples = 3;
long hlw8012SamplesInterval = 1000;

// GPIO 0 : pushbutton
// GPIO 1 : nc
// GPIO 2 : nc
// GPIO 3 : nc
// GPIO 4 : nc
// GPIO 5 : HLW8012 Select Output
// GPIO 6 : nc (flash)
// GPIO 7 : nc (flash)
// GPIO 8 : nc (flash)
// GPIO 9 : nc (flash)
// GPIO10 : nc (flash)
// GPIO11 : nc (flash)
// GPIO12 : Relay (0 = Off, 1 = On)
// GPIO13 : HLW8012 CF1 Voltage / Current
// GPIO14 : HLW8012 CF Power
// GPIO15 : LED green (0 = On, 1 = Off)
// GPIO16 : nc
// ADC    : nc

