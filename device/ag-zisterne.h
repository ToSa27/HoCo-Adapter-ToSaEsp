#define TOSAESP_ONEWIRE
#define OW_PIN            2

#define TOSAESP_DS18B20
#define DS_SENSORS        1
const char* dsName[DS_SENSORS] = {
  ""
};
DeviceAddress dsAddress[DS_SENSORS] = {
  {0x28,0xFF,0xE5,0x60,0x86,0x16,0x05,0x1A}
};

#define TOSAESP_HCSR04
#define US_TRIGGER_PIN    5
#define US_ECHO_PIN       4
#define US_SAMPLES        10
