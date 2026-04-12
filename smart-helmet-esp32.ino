#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <TM1637Display.h>
#include <math.h>

// ====================== WIFI CONFIG ======================
const char *WIFI_SSID = "SmartHelmetDemo";
const char *WIFI_PASSWORD = "helmet123";

// ====================== FIREBASE CONFIG ======================
const char *FIREBASE_BASE_URL = "https://smart-helmet-demo-8f1d9-default-rtdb.asia-southeast1.firebasedatabase.app";
const char *HELMET_ID = "HLM001";
const char *HELMET_TAG_ID = "QR-HLM001";

// ====================== RIDER INFO ======================
const char *RIDER_NAME = "Yuvraj";
const char *RIDER_BLOOD_GROUP = "B+";
const char *RIDER_EMERGENCY_CONTACT = "+919876543210";
const char *RIDER_CITY = "Delhi, India";
const char *RIDER_MEDICAL_NOTE = "No Known Allergies";

// ====================== PIN CONFIG ======================
const int MQ3_PIN = 34;
const int HELMET_WEAR_PIN = 27; // LOW = worn

// TM1637 display
const int TM1637_CLK_PIN = 18;
const int TM1637_DIO_PIN = 19;
TM1637Display display(TM1637_CLK_PIN, TM1637_DIO_PIN);

// GPS UART2
const int GPS_RX_PIN = 16; // ESP32 RX <- GPS TX
const int GPS_TX_PIN = 17; // ESP32 TX -> GPS RX
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// MPU6050 I2C
const uint8_t MPU_ADDR = 0x68;
const int MPU_SDA_PIN = 21;
const int MPU_SCL_PIN = 22;

// ====================== GPS FALLBACK ======================
float fallbackLat = 28.6139f;
float fallbackLng = 77.2090f;

// ====================== LAST VALID GPS ======================
float lastValidLat = fallbackLat;
float lastValidLng = fallbackLng;
float lastValidAltitudeM = 0.0f;
float lastValidSpeedKmph = 0.0f;
float lastValidCourseDeg = 0.0f;
uint32_t lastFixAgeMsSnapshot = 0;
bool hasEverHadFix = false;

// ====================== THRESHOLDS ======================
const int MQ3_THRESHOLD = 1800;
const unsigned long UPDATE_INTERVAL = 3000;

const float MOVING_ACCEL_DELTA = 3500.0f;
const float ACCIDENT_ACCEL_DELTA = 12000.0f;
const float TILT_THRESHOLD_DEG = 55.0f;
const unsigned long ACCIDENT_HOLD_MS = 1500;
const unsigned long HELMET_REMOVED_ALERT_MS = 2000;

// GPS timing
const unsigned long GPS_FIX_FRESH_MS = 5000;
const unsigned long GPS_CONNECTED_TIMEOUT_MS = 4000;

// ====================== STATE ======================
unsigned long lastUpdate = 0;
unsigned long accidentStartTime = 0;
unsigned long helmetRemovedStartTime = 0;
unsigned long lastGpsDebugTime = 0;
unsigned long lastGpsCharTime = 0;

bool accidentDetected = false;
bool movingNow = false;
bool helmetRemovedWhileMoving = false;
bool gpsSentenceSeen = false;

// Raw MPU values
int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
int16_t prevAx = 0, prevAy = 0, prevAz = 0;

// ====================== HELPERS ======================
String boolToJson(bool value)
{
  return value ? "true" : "false";
}

float absf(float x)
{
  return (x < 0) ? -x : x;
}

String jsonEscape(const String &input)
{
  String out;
  out.reserve(input.length() + 8);

  for (size_t i = 0; i < input.length(); i++)
  {
    char c = input[i];
    switch (c)
    {
    case '\"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\r':
      out += "\\r";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      out += c;
      break;
    }
  }
  return out;
}

String formatTwoDigits(int value)
{
  if (value < 10)
    return "0" + String(value);
  return String(value);
}

String getGpsUtcIso()
{
  if (gps.date.isValid() && gps.time.isValid())
  {
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    return String(year) + "-" +
           formatTwoDigits(month) + "-" +
           formatTwoDigits(day) + "T" +
           formatTwoDigits(hour) + ":" +
           formatTwoDigits(minute) + ":" +
           formatTwoDigits(second) + "Z";
  }
  return "";
}

String buildLatLngString(float lat, float lng)
{
  return String(lat, 6) + "," + String(lng, 6);
}

String buildGoogleMapsUrl(float lat, float lng)
{
  return "https://www.google.com/maps?q=" + buildLatLngString(lat, lng);
}

bool connectWiFi()
{
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("\nWiFi connection failed!");
  return false;
}

// ====================== MPU6050 ======================
void writeMPURegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

bool initMPU6050()
{
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);

  writeMPURegister(0x6B, 0x00);
  delay(100);

  writeMPURegister(0x1C, 0x00); // accel ±2g
  writeMPURegister(0x1B, 0x00); // gyro ±250°/s

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  Wire.requestFrom((int)MPU_ADDR, 1, true);
  if (Wire.available())
  {
    uint8_t who = Wire.read();
    return (who == 0x68 || who == 0x70);
  }

  return false;
}

bool readMPU6050Raw()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  Wire.requestFrom((int)MPU_ADDR, 14, true);
  if (Wire.available() < 14)
  {
    return false;
  }

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

float estimateTiltDeg()
{
  float axf = (float)ax;
  float ayf = (float)ay;
  float azf = (float)az;

  float denominator = sqrtf(ayf * ayf + azf * azf);
  if (denominator < 1.0f)
    denominator = 1.0f;

  float tiltRad = atanf(axf / denominator);
  return absf(tiltRad * 180.0f / PI);
}

float accelDelta()
{
  float dx = (float)(ax - prevAx);
  float dy = (float)(ay - prevAy);
  float dz = (float)(az - prevAz);
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

// ====================== DISPLAY ======================
void showSafe()
{
  uint8_t data[] = {
      display.encodeDigit(5),
      0x77,
      0x71,
      0x79};
  display.setSegments(data);
}

void showHelp()
{
  uint8_t data[] = {
      0x76,
      0x79,
      0x38,
      0x73};
  display.setSegments(data);
}

void showStop()
{
  uint8_t data[] = {
      display.encodeDigit(5),
      0x78,
      0x3F,
      0x73};
  display.setSegments(data);
}

void updateDisplay(bool helmetWorn, bool alcoholSafe, bool accidentDetected)
{
  if (accidentDetected)
  {
    showHelp();
  }
  else if (helmetWorn && alcoholSafe)
  {
    showSafe();
  }
  else
  {
    showStop();
  }
}

// ====================== GPS ======================
void readGPS()
{
  while (gpsSerial.available() > 0)
  {
    char c = gpsSerial.read();
    Serial.write(c);

    lastGpsCharTime = millis();

    if (c == '$')
    {
      gpsSentenceSeen = true;
    }

    gps.encode(c);
  }

  if (gps.location.isUpdated() && gps.location.isValid())
  {
    lastValidLat = gps.location.lat();
    lastValidLng = gps.location.lng();
    hasEverHadFix = true;

    if (gps.altitude.isValid())
      lastValidAltitudeM = gps.altitude.meters();

    if (gps.speed.isValid())
      lastValidSpeedKmph = gps.speed.kmph();

    if (gps.course.isValid())
      lastValidCourseDeg = gps.course.deg();

    lastFixAgeMsSnapshot = gps.location.age();

    Serial.println();
    Serial.println("GPS FIX UPDATED");
    Serial.print("LAT: ");
    Serial.println(lastValidLat, 6);
    Serial.print("LNG: ");
    Serial.println(lastValidLng, 6);
    Serial.print("SATELLITES: ");
    Serial.println(gps.satellites.isValid() ? gps.satellites.value() : 0);
    Serial.print("HDOP: ");
    Serial.println(gps.hdop.isValid() ? gps.hdop.hdop() : 0.0);
    Serial.print("UTC: ");
    Serial.println(getGpsUtcIso());
  }
}

// ====================== FIREBASE UPDATE ======================
void updateHelmetData(
    bool helmetWorn,
    bool alcoholSafe,
    bool gpsConnected,
    bool gpsHasFix,
    const String &locationSource,
    bool moving,
    bool accident,
    bool removedWhileMoving,
    float lat,
    float lng,
    float altitudeM,
    float speedKmph,
    float courseDeg,
    int satellites,
    float hdop,
    unsigned long locationAgeMs,
    int mq3Value,
    float tiltDeg,
    float accelChange)
{
  if (WiFi.status() != WL_CONNECTED)
    return;

  HTTPClient http;
  String url = String(FIREBASE_BASE_URL) + "/helmets/" + HELMET_ID + ".json";

  String latLng = buildLatLngString(lat, lng);
  String googleMapsUrl = buildGoogleMapsUrl(lat, lng);
  String gpsUtcIso = getGpsUtcIso();

  String json = "{";

  json += "\"helmetId\":\"" + jsonEscape(String(HELMET_ID)) + "\",";
  json += "\"helmetTagId\":\"" + jsonEscape(String(HELMET_TAG_ID)) + "\",";

  json += "\"rider\":{";
  json += "\"name\":\"" + jsonEscape(String(RIDER_NAME)) + "\",";
  json += "\"bloodGroup\":\"" + jsonEscape(String(RIDER_BLOOD_GROUP)) + "\",";
  json += "\"emergencyContact\":\"" + jsonEscape(String(RIDER_EMERGENCY_CONTACT)) + "\",";
  json += "\"city\":\"" + jsonEscape(String(RIDER_CITY)) + "\",";
  json += "\"medicalNote\":\"" + jsonEscape(String(RIDER_MEDICAL_NOTE)) + "\"";
  json += "},";

  json += "\"status\":{";
  json += "\"helmetWorn\":" + boolToJson(helmetWorn) + ",";
  json += "\"alcoholSafe\":" + boolToJson(alcoholSafe) + ",";
  json += "\"gpsConnected\":" + boolToJson(gpsConnected) + ",";
  json += "\"gpsHasFix\":" + boolToJson(gpsHasFix) + ",";
  json += "\"moving\":" + boolToJson(moving) + ",";
  json += "\"accidentDetected\":" + boolToJson(accident) + ",";
  json += "\"helmetRemovedWhileRiding\":" + boolToJson(removedWhileMoving) + ",";
  json += "\"rideAllowed\":" + boolToJson(helmetWorn && alcoholSafe) + ",";
  json += "\"sosActive\":false,";
  json += "\"lastUpdated\":\"Live from ESP32\"";
  json += "},";

  json += "\"location\":{";
  json += "\"lat\":" + String(lat, 6) + ",";
  json += "\"lng\":" + String(lng, 6) + ",";
  json += "\"latLng\":\"" + latLng + "\",";
  json += "\"googleMapsUrl\":\"" + googleMapsUrl + "\",";
  json += "\"source\":\"" + jsonEscape(locationSource) + "\",";
  json += "\"hasFix\":" + boolToJson(gpsHasFix) + ",";
  json += "\"connected\":" + boolToJson(gpsConnected) + ",";
  json += "\"ageMs\":" + String(locationAgeMs) + ",";
  json += "\"satellites\":" + String(satellites) + ",";
  json += "\"hdop\":" + String(hdop, 2) + ",";
  json += "\"altitudeM\":" + String(altitudeM, 2) + ",";
  json += "\"speedKmph\":" + String(speedKmph, 2) + ",";
  json += "\"courseDeg\":" + String(courseDeg, 2) + ",";
  json += "\"gpsUtc\":\"" + jsonEscape(gpsUtcIso) + "\",";
  json += "\"geoPoint\":{";
  json += "\"latitude\":" + String(lat, 6) + ",";
  json += "\"longitude\":" + String(lng, 6);
  json += "}";
  json += "},";

  json += "\"telemetry\":{";
  json += "\"mq3Value\":" + String(mq3Value) + ",";
  json += "\"tiltDeg\":" + String(tiltDeg, 2) + ",";
  json += "\"accelDelta\":" + String(accelChange, 2) + ",";
  json += "\"mpuRaw\":{";
  json += "\"ax\":" + String(ax) + ",";
  json += "\"ay\":" + String(ay) + ",";
  json += "\"az\":" + String(az) + ",";
  json += "\"gx\":" + String(gx) + ",";
  json += "\"gy\":" + String(gy) + ",";
  json += "\"gz\":" + String(gz);
  json += "}";
  json += "}";

  json += "}";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.PUT(json);

  Serial.println("----- HELMET DATA UPDATE -----");
  Serial.println(url);
  Serial.println(json);
  Serial.print("HTTP Code: ");
  Serial.println(httpCode);

  if (httpCode > 0)
  {
    Serial.println("Response: " + http.getString());
  }
  else
  {
    Serial.println("Update failed.");
  }

  http.end();
}

// ====================== SETUP ======================
void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(HELMET_WEAR_PIN, INPUT_PULLUP);

  display.setBrightness(0x0f, true);
  showStop();

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  bool mpuOk = initMPU6050();
  Serial.println(mpuOk ? "MPU6050 ready" : "MPU6050 not detected");

  connectWiFi();

  Serial.println("Smart Helmet ESP32 Ready!");
  Serial.println("GPS wiring must be:");
  Serial.println("GPS TX -> ESP32 GPIO16");
  Serial.println("GPS RX -> ESP32 GPIO17");
  Serial.println("Waiting for GPS data...");
}

// ====================== LOOP ======================
void loop()
{
  // ALWAYS process GPS, no restriction
  readGPS();

  int mq3Value = analogRead(MQ3_PIN);
  bool alcoholSafe = (mq3Value < MQ3_THRESHOLD);

  bool helmetWorn = (digitalRead(HELMET_WEAR_PIN) == LOW);

  bool mpuReadOk = readMPU6050Raw();
  float tiltDeg = 0.0f;
  float accelChange = 0.0f;

  if (mpuReadOk)
  {
    tiltDeg = estimateTiltDeg();
    accelChange = accelDelta();

    movingNow = (accelChange > MOVING_ACCEL_DELTA);

    bool crashCondition = (accelChange > ACCIDENT_ACCEL_DELTA) ||
                          (tiltDeg > TILT_THRESHOLD_DEG && movingNow);

    if (crashCondition)
    {
      if (accidentStartTime == 0)
      {
        accidentStartTime = millis();
      }
      else if (millis() - accidentStartTime >= ACCIDENT_HOLD_MS)
      {
        accidentDetected = true;
      }
    }
    else
    {
      accidentStartTime = 0;
      accidentDetected = false;
    }

    if (!helmetWorn && movingNow)
    {
      if (helmetRemovedStartTime == 0)
      {
        helmetRemovedStartTime = millis();
      }
      else if (millis() - helmetRemovedStartTime >= HELMET_REMOVED_ALERT_MS)
      {
        helmetRemovedWhileMoving = true;
      }
    }
    else
    {
      helmetRemovedStartTime = 0;
      helmetRemovedWhileMoving = false;
    }

    prevAx = ax;
    prevAy = ay;
    prevAz = az;
  }

  // -------- GPS STATE --------
  bool gpsConnected = false;
  bool gpsHasFix = false;
  String locationSource = "fallback";

  float lat = fallbackLat;
  float lng = fallbackLng;
  float altitudeM = 0.0f;
  float speedKmph = 0.0f;
  float courseDeg = 0.0f;
  int satellites = 0;
  float hdop = 99.99f;
  unsigned long locationAgeMs = 0;

  if (gpsSentenceSeen && (millis() - lastGpsCharTime) <= GPS_CONNECTED_TIMEOUT_MS)
  {
    gpsConnected = true;
  }

  if (gps.location.isValid() && gps.location.age() < GPS_FIX_FRESH_MS)
  {
    gpsHasFix = true;
    locationSource = "live_gps";

    lat = gps.location.lat();
    lng = gps.location.lng();
    locationAgeMs = gps.location.age();

    if (gps.altitude.isValid())
      altitudeM = gps.altitude.meters();

    if (gps.speed.isValid())
      speedKmph = gps.speed.kmph();

    if (gps.course.isValid())
      courseDeg = gps.course.deg();

    if (gps.satellites.isValid())
      satellites = gps.satellites.value();

    if (gps.hdop.isValid())
      hdop = gps.hdop.hdop();

    lastValidLat = lat;
    lastValidLng = lng;
    lastValidAltitudeM = altitudeM;
    lastValidSpeedKmph = speedKmph;
    lastValidCourseDeg = courseDeg;
    hasEverHadFix = true;
  }
  else if (hasEverHadFix)
  {
    locationSource = "last_known";
    lat = lastValidLat;
    lng = lastValidLng;
    altitudeM = lastValidAltitudeM;
    speedKmph = lastValidSpeedKmph;
    courseDeg = lastValidCourseDeg;
    locationAgeMs = gps.location.isValid() ? gps.location.age() : lastFixAgeMsSnapshot;

    if (gps.satellites.isValid())
      satellites = gps.satellites.value();

    if (gps.hdop.isValid())
      hdop = gps.hdop.hdop();
  }
  else
  {
    locationSource = "fallback";
    lat = fallbackLat;
    lng = fallbackLng;
    altitudeM = 0.0f;
    speedKmph = 0.0f;
    courseDeg = 0.0f;
    locationAgeMs = 0;
    satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
    hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.99f;
  }

  if (millis() - lastGpsDebugTime >= 3000)
  {
    lastGpsDebugTime = millis();

    Serial.println();
    Serial.println("========== GPS STATUS ==========");
    Serial.print("Chars Processed: ");
    Serial.println(gps.charsProcessed());
    Serial.print("Sentences Seen: ");
    Serial.println(gpsSentenceSeen ? "YES" : "NO");
    Serial.print("GPS Connected: ");
    Serial.println(gpsConnected ? "YES" : "NO");
    Serial.print("GPS Has Fix: ");
    Serial.println(gpsHasFix ? "YES" : "NO");
    Serial.print("Location Source: ");
    Serial.println(locationSource);
    Serial.print("Location Age(ms): ");
    Serial.println(gps.location.isValid() ? gps.location.age() : 4294967295UL);
    Serial.print("Satellites: ");
    Serial.println(satellites);
    Serial.print("HDOP: ");
    Serial.println(hdop, 2);

    if (!gpsSentenceSeen)
    {
      Serial.println("No GPS serial data. Check wiring / baud.");
    }
    else if (!gpsConnected)
    {
      Serial.println("GPS sentences were seen earlier, but no recent serial data is arriving.");
    }
    else if (!gpsHasFix)
    {
      Serial.println("GPS connected but no valid fix yet. Go outdoors.");
    }
    else
    {
      Serial.print("Live Lat: ");
      Serial.println(lat, 6);
      Serial.print("Live Lng: ");
      Serial.println(lng, 6);
      Serial.print("Maps URL: ");
      Serial.println(buildGoogleMapsUrl(lat, lng));
    }
  }

  updateDisplay(helmetWorn, alcoholSafe, accidentDetected);

  if (millis() - lastUpdate >= UPDATE_INTERVAL)
  {
    lastUpdate = millis();

    Serial.println("==================================");
    Serial.print("Helmet Tag ID: ");
    Serial.println(HELMET_TAG_ID);

    Serial.print("MQ3 Value: ");
    Serial.println(mq3Value);
    Serial.print("Alcohol Safe: ");
    Serial.println(alcoholSafe ? "YES" : "NO");

    Serial.print("Helmet Worn: ");
    Serial.println(helmetWorn ? "YES" : "NO");

    Serial.print("Ride Allowed: ");
    Serial.println((helmetWorn && alcoholSafe) ? "YES" : "NO");

    Serial.print("Moving: ");
    Serial.println(movingNow ? "YES" : "NO");

    Serial.print("Helmet Removed While Riding: ");
    Serial.println(helmetRemovedWhileMoving ? "YES" : "NO");

    Serial.print("Accident Detected: ");
    Serial.println(accidentDetected ? "YES" : "NO");

    Serial.print("Tilt(deg): ");
    Serial.println(tiltDeg, 2);

    Serial.print("Accel Delta: ");
    Serial.println(accelChange, 2);

    Serial.print("GPS Connected: ");
    Serial.println(gpsConnected ? "YES" : "NO");

    Serial.print("GPS Has Fix: ");
    Serial.println(gpsHasFix ? "YES" : "NO");

    Serial.print("Location Source: ");
    Serial.println(locationSource);

    Serial.print("Lat: ");
    Serial.println(lat, 6);

    Serial.print("Lng: ");
    Serial.println(lng, 6);

    Serial.print("Google Maps URL: ");
    Serial.println(buildGoogleMapsUrl(lat, lng));

    updateHelmetData(
        helmetWorn,
        alcoholSafe,
        gpsConnected,
        gpsHasFix,
        locationSource,
        movingNow,
        accidentDetected,
        helmetRemovedWhileMoving,
        lat,
        lng,
        altitudeM,
        speedKmph,
        courseDeg,
        satellites,
        hdop,
        locationAgeMs,
        mq3Value,
        tiltDeg,
        accelChange);
  }

  delay(50);
}
