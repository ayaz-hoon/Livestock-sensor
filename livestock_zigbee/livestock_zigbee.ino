#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;
SoftwareSerial XBee(2, 3); // RX, TX

// Create an ADXL345 instance
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup() {
  Serial.begin(9600);
  XBee.begin(9600);

  // Initialize the ADXL345
  if (!accel.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    while (1);
  }

  // Set the range to +/- 4g (default is +/- 2g)
  accel.setRange(ADXL345_RANGE_4_G);
}

void loop() {
  // Read accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);

  // Get GPS data
  if (gps.encode(Serial.read())) {
    if (gps.location.isValid()) {
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
    }
  }

  // Print accelerometer data to Serial Monitor
  Serial.print("L=");
  Serial.print(event.acceleration.x);
  Serial.print(" ");
  Serial.print(event.acceleration.y);
  Serial.print(" ");
  Serial.println(event.acceleration.z);
  Serial.print(" ");

  // Transmit data via XBee
  if (event.acceleration.x != 0 && event.acceleration.y != 0 && event.acceleration.z != 0) {
    // Send accelerometer data with "L=" prefix
 
 
    XBee.print(event.acceleration.x);
    XBee.print(event.acceleration.y);
    XBee.println(event.acceleration.z);
  }

  // Introduce a delay for the desired sampling frequency (5Hz)
  delay(100);
}
