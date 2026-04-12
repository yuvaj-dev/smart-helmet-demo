#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <TM1637Display.h>

// ====================== WIFI CONFIG ======================
const char *WIFI_SSID = "SmartHelmetDemo";
const char *WIFI_PASSWORD = "helmet123";

// ====================== FIREBASE / SERVER CONFIG ======================
const char *FIREBASE_BASE_URL = "https://smart-helmet-demo-8f1d9-default-rtdb.asia-southeast1.firebasedatabase.app";
const char *HELMET_ID = "HLM001";
const char *HELMET_TAG_ID = "QR-HLM001";

// ====================== PIN CONFIG ======================
const int MQ3_PIN = 34;
const int HELMET_WEAR_PIN = 27;

// Battery measurement
const int BATTERY_ADC_PIN = 35;

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
float fallbackLat = 28.6139;
float fallbackLng = 77.2090;

// Store last good GPS fix
float lastValidLat = fallbackLat;
float lastValidLng = fallbackLng;
bool hasEverHadFix = false;

// ====================== THRESHOLDS ======================
const int MQ3_THRESHOLD = 1800;
const unsigned long UPDATE_INTERVAL = 3000;

// Movement / accident thresholds
const float MOVING_ACCEL_DELTA = 3500.0;
const float ACCIDENT_ACCEL_DELTA = 12000.0;
const float TILT_THRESHOLD_DEG = 55.0;
const unsigned long ACCIDENT_HOLD_MS = 1500;
const unsigned long HELMET_REMOVED_ALERT_MS = 2000;

// Battery divider ratio for 10k(top) + 20k(bottom)
const float BATTERY_DIVIDER_RATIO = 3.0;
const float ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX = 4095;

// Single-cell Li-ion percentage estimate
const float BATTERY_EMPTY_V = 3.30;
const float BATTERY_FULL_V = 4.20;

// ====================== STATE ======================
unsigned long lastUpdate = 0;
unsigned long accidentStartTime = 0;
unsigned long helmetRemovedStartTime = 0;
unsigned long lastGpsDebugTime = 0;

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

int clampi(int value, int minV, int maxV)
{
  if (value < minV)
    return minV;
  if (value > maxV)
    return maxV;
  return value;
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

// ====================== MPU6050 RAW ACCESS ======================
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

  writeMPURegister(0x1C, 0x00);
  writeMPURegister(0x1B, 0x00);

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

  float denominator = sqrt(ayf * ayf + azf * azf);
  if (denominator < 1.0f)
    denominator = 1.0f;

  float tiltRad = atan(axf / denominator);
  return absf(tiltRad * 180.0f / PI);
}

float accelDelta()
{
  float dx = (float)(ax - prevAx);
  float dy = (float)(ay - prevAy);
  float dz = (float)(az - prevAz);
  return sqrt(dx * dx + dy * dy + dz * dz);
}

// ====================== BATTERY ======================
float readBatteryVoltage()
{
  uint32_t sum = 0;
  const int samples = 16;

  for (int i = 0; i < samples; i++)
  {
    sum += analogRead(BATTERY_ADC_PIN);
    delay(2);
  }

  float adcRaw = (float)sum / samples;
  float vadc = (adcRaw / ADC_MAX) * ADC_REF_VOLTAGE;
  float vbattery = vadc * BATTERY_DIVIDER_RATIO;
  return vbattery;
}

int batteryPercentFromVoltage(float vbattery)
{
  float pct = (vbattery - BATTERY_EMPTY_V) * 100.0f / (BATTERY_FULL_V - BATTERY_EMPTY_V);
  return clampi((int)(pct + 0.5f), 0, 100);
}

void showBatteryPercent(int batteryPercent)
{
  display.showNumberDec(batteryPercent, false, 4, 0);
}

// ====================== GPS ======================
void readGPS()
{
  while (gpsSerial.available() > 0)
  {
    char c = gpsSerial.read();

    // Raw GPS debug output
    Serial.write(c);

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
  }
}

// ====================== SERVER UPDATE ======================
void updateHelmetData(
    bool helmetWorn,
    bool alcoholSafe,
    bool gpsOnline,
    bool moving,
    bool accident,
    bool removedWhileMoving,
    float lat,
    float lng,
    int mq3Value,
    float tiltDeg,
    float accelChange,
    float batteryVoltage,
    int batteryPercent)
{
  if (WiFi.status() != WL_CONNECTED)
    return;

  HTTPClient http;
  String url = String(FIREBASE_BASE_URL) + "/helmets/" + HELMET_ID + ".json";

  String json = "{";
  json += "\"helmetId\":\"" + String(HELMET_ID) + "\",";
  json += "\"helmetTagId\":\"" + String(HELMET_TAG_ID) + "\",";
  json += "\"helmetWorn\":" + boolToJson(helmetWorn) + ",";
  json += "\"alcoholSafe\":" + boolToJson(alcoholSafe) + ",";
  json += "\"gpsOnline\":" + boolToJson(gpsOnline) + ",";
  json += "\"moving\":" + boolToJson(moving) + ",";
  json += "\"accidentDetected\":" + boolToJson(accident) + ",";
  json += "\"helmetRemovedWhileRiding\":" + boolToJson(removedWhileMoving) + ",";
  json += "\"rideAllowed\":" + boolToJson(helmetWorn && alcoholSafe) + ",";
  json += "\"mq3Value\":" + String(mq3Value) + ",";
  json += "\"tiltDeg\":" + String(tiltDeg, 2) + ",";
  json += "\"accelDelta\":" + String(accelChange, 2) + ",";
  json += "\"batteryVoltage\":" + String(batteryVoltage, 3) + ",";
  json += "\"batteryPercent\":" + String(batteryPercent) + ",";
  json += "\"location\":{";
  json += "\"lat\":" + String(lat, 6) + ",";
  json += "\"lng\":" + String(lng, 6);
  json += "},";
  json += "\"mpuRaw\":{";
  json += "\"ax\":" + String(ax) + ",";
  json += "\"ay\":" + String(ay) + ",";
  json += "\"az\":" + String(az) + ",";
  json += "\"gx\":" + String(gx) + ",";
  json += "\"gy\":" + String(gy) + ",";
  json += "\"gz\":" + String(gz);
  json += "},";
  json += "\"lastUpdated\":\"Live from ESP32\"";
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
  analogReadResolution(12);

  display.setBrightness(0x0f, true);
  display.showNumberDec(0, false, 4, 0);

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
  readGPS();

  int mq3Value = analogRead(MQ3_PIN);
  bool alcoholSafe = (mq3Value < MQ3_THRESHOLD);

  bool helmetWorn = (digitalRead(HELMET_WEAR_PIN) == LOW);

  bool mpuReadOk = readMPU6050Raw();
  float tiltDeg = 0.0;
  float accelChange = 0.0;

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

  bool gpsOnline = false;
  float lat = fallbackLat;
  float lng = fallbackLng;

  if (gps.location.isValid() && gps.location.age() < 5000)
  {
    gpsOnline = true;
    lat = gps.location.lat();
    lng = gps.location.lng();

    lastValidLat = lat;
    lastValidLng = lng;
    hasEverHadFix = true;
  }
  else if (hasEverHadFix)
  {
    // Use last known valid coordinates if temporary signal loss happens
    lat = lastValidLat;
    lng = lastValidLng;
  }
  else
  {
    // No valid fix ever received
    lat = fallbackLat;
    lng = fallbackLng;
  }

  if (millis() - lastGpsDebugTime >= 3000)
  {
    lastGpsDebugTime = millis();

    Serial.println();
    Serial.println("========== GPS STATUS ==========");
    Serial.print("Chars Processed: ");
    Serial.println(gps.charsProcessed());
    Serial.print("Sentences With Fix Data Seen: ");
    Serial.println(gpsSentenceSeen ? "YES" : "NO");
    Serial.print("Location Valid: ");
    Serial.println(gps.location.isValid() ? "YES" : "NO");
    Serial.print("Location Age(ms): ");
    Serial.println(gps.location.age());
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.isValid() ? gps.satellites.value() : 0);
    Serial.print("HDOP: ");
    Serial.println(gps.hdop.isValid() ? gps.hdop.hdop() : 0.0);

    if (!gpsSentenceSeen)
    {
      Serial.println("No GPS serial data received. Check TX/RX wiring or baud rate.");
    }
    else if (!gps.location.isValid())
    {
      Serial.println("GPS data present, but no location fix yet. Move outdoors/open sky.");
    }
  }

  float batteryVoltage = readBatteryVoltage();
  int batteryPercent = batteryPercentFromVoltage(batteryVoltage);
  showBatteryPercent(batteryPercent);

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

    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage, 3);

    Serial.print("Battery Percent: ");
    Serial.println(batteryPercent);

    Serial.print("GPS Online: ");
    Serial.println(gpsOnline ? "YES" : "NO");
    Serial.print("Lat: ");
    Serial.println(lat, 6);
    Serial.print("Lng: ");
    Serial.println(lng, 6);

    updateHelmetData(
        helmetWorn,
        alcoholSafe,
        gpsOnline,
        movingNow,
        accidentDetected,
        helmetRemovedWhileMoving,
        lat,
        lng,
        mq3Value,
        tiltDeg,
        accelChange,
        batteryVoltage,
        batteryPercent);
  }

  delay(50);
}
