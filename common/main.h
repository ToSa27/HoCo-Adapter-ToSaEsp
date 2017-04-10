void mainSetup();
void mainLoop();

void setup() {
  Serial.begin(115200);
  Serial.println("\r\n\nHoCo ESP Setup\n");
  sprintf(mqttPrefix, "hoco/esp_pio/%s/", deviceName);
  wifiConnect();
  otaInit();
  mqttConnect();
  mainSetup();
  Serial.println("Setup complete.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED)
    wifiConnect();
  else {
    if (!mqttClient.connected())
      mqttConnect();
    else {
      mainLoop();
      mqttLoop();
    }
    otaLoop();
  }
}
