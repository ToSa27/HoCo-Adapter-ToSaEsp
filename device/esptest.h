#define TOSAESP_OUTPUTS   1
byte outputPin[TOSAESP_OUTPUTS] = { 12 };
bool outputInv[TOSAESP_OUTPUTS] = { true };
bool outputDefault[TOSAESP_OUTPUTS] = { false };
int outputMaxPulse[TOSAESP_OUTPUTS] = { 1800 };
const char* outputName[TOSAESP_OUTPUTS] = {
  "LED"
};
/*
#define TOSAESP_SCHEDULER
#define TOSAESP_SCHEDULER_EEPROMADDR  0
#define TOSAESP_SCHEDULER_EVENTS      (TOSAESP_OUTPUTS * 10)
#define TOSAESP_SCHEDULER_HOLIDAYS    3
#define TOSAESP_SCHEDULER_VACATIONS   1
*/