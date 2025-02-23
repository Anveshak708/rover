#include <SoftwareSerial.h>

SoftwareSerial GPS_SoftSerial(7, 9);  // RX, TX

void setup() {
  Serial.begin(9600);
  GPS_SoftSerial.begin(9600);
  Serial.println("Testing GPS - Waiting for NMEA Sentences...");
}

void loop() {
  if (GPS_SoftSerial.available()) {
    Serial.write(GPS_SoftSerial.read());  // Print raw GPS data
  }
}
