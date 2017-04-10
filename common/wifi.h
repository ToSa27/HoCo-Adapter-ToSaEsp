void wifiConnect() {
  Serial.println("WiFi Connect");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.print("WiFi Connected: IP: ");
  Serial.println(WiFi.localIP());
}
