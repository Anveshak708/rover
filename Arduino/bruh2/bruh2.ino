#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup()
{
  Serial.begin(115200);
  bno.begin(OPERATION_MODE_COMPASS);
}

void loop()
{
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.println(event.orientation.x);
  delay(100);
}
