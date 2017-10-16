//env:TOSAESP_BOARD=esp01_1m
//env:TOSAESP_BOARDFLASHMODE=dout
//env:TOSAESP_LIB_DEPS=HLW8012

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
