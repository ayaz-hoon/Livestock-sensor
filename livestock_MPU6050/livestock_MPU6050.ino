#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MPU6050.h>  // Include MPU6050 library
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
 
TinyGPSPlus gps;  // The TinyGPS++ object
SoftwareSerial ss(2, 0); // The serial connection to the GPS device, (Rx,Tx)

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "*****"
#define WIFI_PASSWORD "*****"

// Insert Firebase project API Key
#define API_KEY "enter your api key"

// Insert RTDB URL define the RTDB URL
#define DATABASE_URL "database url "

// Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

String uid;

// Database main path (to be updated in setup with the user UID)
String databasePath;
String databasePath2;
// Database child nodes
String xPath = "/x";
String yPath = "/y";
String zPath = "/z";
String gxPath = "/gx"; // Gyroscope x
String gyPath = "/gy"; // Gyroscope y
String gzPath = "/gz"; // Gyroscope z
String timePath = "/timestamp";
String driverId = "/driverId";
String latPath = "/lat";
String lonPath = "/lng";

// Parent Node (to be updated in every loop)
String parentPath;
String parentPath2;

FirebaseJson json;
FirebaseJson json2;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Variable to save current epoch time
int timestamp;

// MPU6050 sensor
Adafruit_MPU6050 mpu;

// BME280 sensor
//Adafruit_BME280 bme; // I2C
double latitude;
double longitude;
double lat_str;
double lon_str;

unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 1000;

int count = 0;
bool signupOK = false;

// Function that gets current epoch time
unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}

void setup() {
  ss.begin(9600);
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found.");
    while (1);
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  timeClient.begin();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }

  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);

  // Update database path
  databasePath = "/UsersData/0001";
  databasePath2 = "/Recordings/0001";
}

void loop() {
  if (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        lat_str = latitude;
        longitude = gps.location.lng();
        lon_str = longitude;
      }
    }
  }

  // Send new readings to database
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    //Get current timestamp
    timestamp = getTime();
    Serial.print("time: ");
    Serial.println(timestamp);

    parentPath = databasePath;
    parentPath2 = databasePath2 + "/" + String(timestamp);

  
  // Read accelerometer and gyroscope data separately
  sensors_event_t accelEvent, gyroEvent;

  mpu.getAccelerometerSensor()->getEvent(&accelEvent);
  mpu.getGyroSensor()->getEvent(&gyroEvent);

  // Extract accelerometer and gyroscope data
  float x = accelEvent.acceleration.x;
  float y = accelEvent.acceleration.y;
  float z = accelEvent.acceleration.z;

  float gx = gyroEvent.gyro.x;
  float gy = gyroEvent.gyro.y;
  float gz = gyroEvent.gyro.z;


  // Create a JSON object with the data
  json.set(xPath.c_str(), String(x));
  json.set(yPath.c_str(), String(y));
  json.set(zPath.c_str(), String(z));
  json.set(latPath.c_str(), lat_str);
  json.set(lonPath.c_str(), lon_str);
  json.set(driverId, "0001");
  json.set(timePath, String(timestamp));
  json.set(gxPath.c_str(), String(gx)); // Gyroscope x
  json.set(gyPath.c_str(), String(gy)); // Gyroscope y
  json.set(gzPath.c_str(), String(gz)); // Gyroscope z
  Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());

  json2.set(xPath.c_str(), String(x));
  json2.set(yPath.c_str(), String(y));
  json2.set(zPath.c_str(), String(z));
  json2.set(latPath.c_str(), lat_str);
  json2.set(lonPath.c_str(), lon_str);
  json2.set(driverId, "0001");
  json2.set(timePath, String(timestamp));
  json2.set(gxPath.c_str(), String(gx)); // Gyroscope x
  json2.set(gyPath.c_str(), String(gy)); // Gyroscope y
  json2.set(gzPath.c_str(), String(gz)); // Gyroscope z
  Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath2.c_str(), &json2) ? "ok" : fbdo.errorReason().c_str());

  }
}
