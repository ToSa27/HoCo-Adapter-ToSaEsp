const char* deviceName = "dg-heizung";

#define TOSAESP_ONEWIRE
#define OW_PIN            2

#define TOSAESP_DS18B20
#define DS_SENSORS        4

const char* dsName[DS_SENSORS] = {
  "Vorlauf",
  "Bad",
  "Schlafzimmer1",
  "Schlafzimmer2"
};

DeviceAddress dsAddress[DS_SENSORS] = {
  {0x28,0xFF,0x94,0xAD,0x85,0x16,0x04,0xA4},
  {0x28,0xFF,0x9C,0x8B,0x85,0x16,0x04,0xE3},
  {0x28,0xFF,0xB2,0x0F,0x85,0x16,0x03,0xED},
  {0x28,0xFF,0x13,0x6D,0x86,0x16,0x05,0xBD}
};

#define TOSAESP_I2C

#define TOSAESP_PCF8574
#define PCF8574_ADDR	0x20
#define PCF8574_INPUTS	2
byte pcfInputPin[PCF8574_INPUTS] = { 0, 1 };
bool pcfInputInv[PCF8574_INPUTS] = { true, true };
const char* pcfInputName[PCF8574_INPUTS] = {
  "VentilBad",
  "VentilSchlafzimmer"
};
