#include <NewPing.h>

NewPing us(US_TRIGGER_PIN, US_ECHO_PIN);

const long usDelay = 1 * 60 * 1000;
long usLast = 0;
const long usSampleDelay = 1 * 1000;
long usSampleLast = 0;
long usSample[US_SAMPLES];
byte usSamplePos = 0;

void usSetup() {
}

void usLoop() {
  long now = millis();
  if (now - usLast > usDelay) {
    if (now - usSampleLast > usSampleDelay) {
      usSample[usSamplePos] = us.ping_cm();
      usSampleLast = now;
      usSamplePos++;
      if (usSamplePos == US_SAMPLES) {
        long dist = 0;
        for (byte i = 0; i < US_SAMPLES; i++)
          dist += usSample[i];
        dist = dist / US_SAMPLES;
        mqttPublishLong("Entfernung", dist, "cm");
        usSamplePos = 0;
        usLast = now;
      }
    }
  }
}
