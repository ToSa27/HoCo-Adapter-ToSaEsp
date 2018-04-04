//env:TOSAESP_BOARD=nodemcuv2
//env:TOSAESP_LIB_DEPS="OneWire, DallasTemperature, PCF8574, NewPing, HLW8012"

#define TOSAESP_TIME

#define TOSAESP_I2C
#define TOSAESP_PCF8574
#define PCF8574_ADDR		0x20
#define PCF8574_INTPIN		14 // WeMos D5 = GPIO14
#define PCF8574_INPUTSTART	0
#define PCF8574_INPUTEND	7
unsigned int pcf8574InputDebounce[PCF8574_INPUTEND - PCF8574_INPUTSTART + 1] = { 250, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
const char* pcf8574InputName[PCF8574_INPUTEND - PCF8574_INPUTSTART + 1] = {
  "Drehstromzähler",
  "Kühlschrank",
  "Serverschrank",
  "Heizung",
  "Geschirrspüler",
  "Waschmaschine",
  "Unbekannt1",
  "Unbekannt2"
};


//#define TOSAESP_INPUTS   8
//byte inputPin[TOSAESP_INPUTS] = { 5, 4, 0, 2, 14, 12, 13, 15 };
//byte inputTrigger[TOSAESP_INPUTS] = { RISING, RISING, RISING, RISING, RISING, RISING, RISING, RISING };
//byte inputType[TOSAESP_INPUTS] = { INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP, INPUT_PULLUP };
//unsigned int inputDebounce[TOSAESP_INPUTS] = { 250, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
//const char* inputName[TOSAESP_INPUTS] = {
//  "Drehstromzähler",
//  "Kühlschrank",
//  "Serverschrank",
//  "Heizung",
//  "Geschirrspüler",
//  "Waschmaschine",
//  "Unbekannt1",
//  "Unbekannt2"
//};
//const char* inputUoM[TOSAESP_INPUTS] = {
//  "Wh",
//  "1/2 Wh",
//  "1/2 Wh",
//  "1/2 Wh",
//  "1/2 Wh",
//  "1/2 Wh",
//  "1/2 Wh",
//  "1/2 Wh"
//};

#define TOSAESP_ONEWIRE
#define OW_PIN            2

#define TOSAESP_DS18B20
#define DS_SENSORS        1
const char* dsName[DS_SENSORS] = {
  "Temperatur"
};

typedef uint8_t DeviceAddress[8];
DeviceAddress dsAddress[DS_SENSORS] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
};
