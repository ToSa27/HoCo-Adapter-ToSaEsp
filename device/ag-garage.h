//env:TOSAESP_BOARD=nodemcuv2
//env:TOSAESP_LIB_DEPS=OneWire\nDallasTemperature

/*
#define TOSAESP_ONEWIRE
#define OW_PIN            2

#define TOSAESP_DS18B20
#define DS_SENSORS        1
const char* dsName[DS_SENSORS] = {
  ""
};

typedef uint8_t DeviceAddress[8];
DeviceAddress dsAddress[DS_SENSORS] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
};
*/

/*
#define TOSAESP_INPUTS   1
byte inputPin[TOSAESP_INPUTS] = { 4 };
byte inputTrigger[TOSAESP_INPUTS] = { FALLING };
byte inputType[TOSAESP_INPUTS] = { INPUT };
unsigned int inputDebounce[TOSAESP_INPUTS] = { 250 };
const char* inputName[TOSAESP_INPUTS] = {
  "TorZu"
};
const char* inputUoM[TOSAESP_INPUTS] = {
  ""
};
*/

#define TOSAESP_I2C
#define I2C_SDAPIN              4
#define I2C_SCLPIN              5

#define TOSAESP_MPU6050
#define MPU6050_ADDRESS         0x68
//#define MPU6050_INTPIN          14
bool mpu6050Publish[3] = { true, false, false };
int16_t mpu6050Threshold[3] = { 10, 0, 0 };
const char* mpu6050Name[3] = { "Position", "", "" };

/*
#define TOSAESP_MPU9250
#define MPU9250_ADDR            0x68
#define MPU9250_INTPIN          14
const char* mpu9250Name = "Position";
*/

#define TOSAESP_OUTPUTS   2
byte outputPin[TOSAESP_OUTPUTS] = { 12, 13 };
bool outputInv[TOSAESP_OUTPUTS] = { false, true };
bool outputDefault[TOSAESP_OUTPUTS] = { false, false };
int outputMaxPulse[TOSAESP_OUTPUTS] = { 1, 0 };
const char* outputName[TOSAESP_OUTPUTS] = {
  "Torantrieb",
  "Licht"
};

/*
#define TOSAESP_SCHEDULER
#define TOSAESP_SCHEDULER_EEPROMADDR  0
#define TOSAESP_SCHEDULER_EVENTS      (TOSAESP_OUTPUTS * 10)
#define TOSAESP_SCHEDULER_HOLIDAYS    3
#define TOSAESP_SCHEDULER_VACATIONS   1
*/
