#include <Adafruit_BNO055.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

Adafruit_BNO055 bno = Adafruit_BNO055();
SoftwareSerial GPS_SoftSerial(4, 3); // (Rx, Tx)
TinyGPSPlus gps;

volatile float minutes, seconds;
volatile int degree, secs, mins;

void setup() {
    Serial.begin(9600);
    GPS_SoftSerial.begin(9600);
    
    if (!bno.begin(OPERATION_MODE_COMPASS)) {
        Serial.println("BNO055 not detected. Check connections!");
        while (1);
    }
}

void loop() {
    sensors_event_t event;
    bno.getEvent(&event);
    float heading = event.orientation.x;
    
    smartDelay(1000);
    double lat_val, lng_val, alt_m_val;
    uint8_t hr_val, min_val, sec_val;
    bool loc_valid, alt_valid, time_valid;
    
    lat_val = gps.location.lat();
    loc_valid = gps.location.isValid();
    lng_val = gps.location.lng();
    alt_m_val = gps.altitude.meters();
    alt_valid = gps.altitude.isValid();
    hr_val = gps.time.hour();
    min_val = gps.time.minute();
    sec_val = gps.time.second();
    time_valid = gps.time.isValid();
    
    Serial.print("Heading: ");
    Serial.println(heading);
    
    if (!loc_valid) {
        Serial.println("Latitude: *****");
        Serial.println("Longitude: *****");
    } else {
        DegMinSec(lat_val);
        Serial.print("Latitude: ");
        Serial.println(lat_val, 6);
        Serial.print("Longitude: ");
        Serial.println(lng_val, 6);
    }
    
    if (!alt_valid) {
        Serial.println("Altitude: *****");
    } else {
        Serial.print("Altitude: ");
        Serial.println(alt_m_val, 6);
    }
    
    if (!time_valid) {
        Serial.println("Time: *****");
    } else {
        Serial.print("Time: ");Serial.print(hr_val);
        Serial.print(":");
        Serial.print(min_val);
        Serial.print(":");
        Serial.println(sec_val);

    }
    
    Serial.println("---------------------------");
    delay(1000);
}

static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (GPS_SoftSerial.available()) {
            gps.encode(GPS_SoftSerial.read());
        }
    } while (millis() - start < ms);
}

void DegMinSec(double tot_val) {
    degree = (int)tot_val;
    minutes = tot_val - degree;
    seconds = 60 * minutes;
    minutes = (int)seconds;
    mins = (int)minutes;
    seconds = seconds - minutes;
    seconds = 60 * seconds;
    secs = (int)seconds;
}
