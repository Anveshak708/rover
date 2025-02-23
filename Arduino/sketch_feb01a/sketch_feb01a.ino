#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial GPS_SoftSerial(7, 9);  // RX, TX
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  GPS_SoftSerial.begin(9600);
  Serial.println("Testing GPS - Waiting for valid data...");
}

void loop() {
  while (GPS_SoftSerial.available()) {
    gps.encode(GPS_SoftSerial.read());
  }

  if (gps.location.isUpdated()) {
    Serial.println("\n===== GPS Data =====");

    // Print Latitude & Longitude
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print("° ");
    Serial.println(gps.location.isValid() ? "(Valid)" : "(Invalid)");

    Serial.print("Longitude: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print("° ");
    Serial.println(gps.location.isValid() ? "(Valid)" : "(Invalid)");

    // Print Altitude
    Serial.print("Altitude: ");
    if (gps.altitude.isValid()) {
      Serial.print(gps.altitude.meters());
    } else {
      Serial.print("N/A");
    }
    Serial.println(" meters");

    // Print Speed
    Serial.print("Speed: ");
    Serial.print(gps.speed.kmph());
    Serial.println(" km/h");

    Serial.print("Speed (Knots): ");
    Serial.print(gps.speed.knots());
    Serial.println(" knots");

    // Print Course (Heading)
    Serial.print("Course (Heading): ");
    if (gps.course.isValid()) {
      Serial.print(gps.course.deg());
    } else {
      Serial.print("N/A");
    }
    Serial.println("°");

    // Print Time
    Serial.print("Time (UTC): ");
    if (gps.time.isValid()) {
      if (gps.time.hour() < 10) Serial.print("0");
      Serial.print(gps.time.hour());
      Serial.print(":");
      if (gps.time.minute() < 10) Serial.print("0");
      Serial.print(gps.time.minute());
      Serial.print(":");
      if (gps.time.second() < 10) Serial.print("0");
      Serial.println(gps.time.second());
    } else {
      Serial.println("Invalid");
    }

    // Print Date
    Serial.print("Date (DD-MM-YYYY): ");
    if (gps.date.isValid()) {
      if (gps.date.day() < 10) Serial.print("0");
      Serial.print(gps.date.day());
      Serial.print("-");
      if (gps.date.month() < 10) Serial.print("0");
      Serial.print(gps.date.month());
      Serial.print("-");
      Serial.println(gps.date.year());
    } else {
      Serial.println("Invalid");
    }

    // Print Number of Satellites
    Serial.print("Satellites Used: ");
    if (gps.satellites.isValid()) {
      Serial.println(gps.satellites.value());
    } else {
      Serial.println("N/A");
    }

    // Print HDOP (Horizontal Dilution of Precision)
    Serial.print("HDOP (Accuracy): ");
    if (gps.hdop.isValid()) {
      Serial.println(gps.hdop.value() / 10.0);
    } else {
      Serial.println("N/A");
    }

    // Print Fix Status
    Serial.print("Fix Status: ");
    if (gps.hdop.isValid()) {
      if (gps.hdop.value() < 10) {
        Serial.println("3D Fix");
      } else {
        Serial.println("2D Fix");
      }
    } else {
      Serial.println("No Fix");
    }

    Serial.println("====================");

    delay(3000);  // Wait 3 seconds before next update
  }
}
