//env:TOSAESP_BOARD=esp01_1m
//env:TOSAESP_BOARDFLASHMODE=dout
//env:TOSAESP_LIB_DEPS=HLW8012

#define TOSAESP_OUTPUTS   2
byte outputPin[TOSAESP_OUTPUTS] = { 12, 15 };
bool outputInv[TOSAESP_OUTPUTS] = { false, false };
bool outputDefault[TOSAESP_OUTPUTS] = { false, false };
int outputMaxPulse[TOSAESP_OUTPUTS] = { 0, 0 };
const char* outputName[TOSAESP_OUTPUTS] = {
  "Relais",
  "LED"
};
