WiFiClientSecure wifiClient;
PubSubClient mqttClient(mqttHost, mqttPort, wifiClient);
char mqttPrefix[50];

long mqttLastConnectAttempt = 0;

void mqttSubscribe(const char *topic) {
  if (mqttClient.connected()) {
    char t[100];
    sprintf(t, "%s%s", mqttPrefix, topic);
    mqttClient.subscribe(t);
  }
}

void mqttPublish(const char *topic, const char *payload, bool retain = true) {
  if (mqttClient.connected()) {
    char t[100];
    sprintf(t, "%s%s", mqttPrefix, topic);
    mqttClient.publish(t, payload, retain);
  }
}

void mqttPublishLong(const char *topic, long value, const char *uom, bool retain = true) {
  char payload[100];
  sprintf(payload, "{\"val\":%d,\"uom\":\"%s\"}", value, uom);
  mqttPublish(topic, payload, retain);
}

void mqttPublishFloat(const char *topic, float value, const char *uom, int precision = 2, bool retain = true) {
  char val[50];
  dtostrf(value, 0, precision, val);
  char payload[100];
  sprintf(payload, "{\"val\":%s,\"uom\":\"%s\"}", val, uom);
  mqttPublish(topic, payload, retain);
}

void mqttAnnounce() {
//  mqttSubscribe("#");
  mqttPublish("$Online", "true");
  mqttPublish("$Name", deviceName);
  mqttPublish("$MAC", WiFi.macAddress().c_str());
  mqttPublish("$IP", WiFi.localIP().toString().c_str());
}

void mqttConnect() {
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - mqttLastConnectAttempt > 2000) {
      Serial.println("MQTT Connect");
      mqttLastConnectAttempt = now;
      char lwt[100];
      sprintf(lwt, "%s%s", mqttPrefix, "$Online");
      mqttClient.connect(deviceName, mqttUser, mqttPass, lwt, 0, true, "false");
      if (mqttClient.connected()) {
        mqttLastConnectAttempt = 0;
        Serial.println("MQTT Connected.");
        mqttAnnounce();
      }
    }
  }
}

void mqttLoop() {
  mqttClient.loop();
}
