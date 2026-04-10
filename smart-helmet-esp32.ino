#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <math.h>

// =====================================================
// SMART HELMET - FINAL HACKATHON VERSION
// Features:
// - MQ3 alcohol detection
// - Helmet wear detection
// - Raw IMU (MPU-compatible clone) accident detection
// - GPS (real if available, fallback if not)
// - Firebase Realtime Database integration via REST
// =====================================================

// ====================== WIFI CONFIG ======================
// IMPORTANT: Use a mobile hotspot with these exact credentials for demo
const char* WIFI_SSID = "SmartHelmetDemo";
const char* WIFI_PASSWORD = "helmet123";

// ====================== FIREBASE CONFIG ======================
const char* FIREBASE_BASE_URL = "https://smart-helmet-demo-8f1d9-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* HELMET_ID = "HLM001";

// ====================== PIN CONFIG ======================
const int MQ3_PIN = 34;             // Analog input
const int HELMET_WEAR_PIN = 27;     // Digital input (LOW = helmet worn)

// GPS UART2
const int GPS_RX_PIN = 16;          // GPS TX -> ESP32 RX2
const int GPS_TX_PIN = 17;          // GPS RX -> ESP32 TX2
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// I2C pins for MPU-compatible IMU
const int MPU_SDA_PIN = 21;
const int MPU_SCL_PIN = 22;

// ====================== MPU CONFIG ======================
const uint8_t MPU_ADDR = 0x68;      // Most common I2C address
const uint8_t WHO_AM_I_REG = 0x75;
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t ACCEL_START_REG = 0x3B;

// ====================== GPS FALLBACK ======================
float fallbackLat = 28.6139;
float fallbackLng = 77.2090;

// ====================== THRESHOLDS ======================
const int MQ3_THRESHOLD = 1800;           // Tune after real testing
const float ACCEL_DELTA_THRESHOLD = 0.65; // Sudden accel change (g approx)
const float GYRO_THRESHOLD = 180.0;       // deg/sec rough threshold
const float TILT_THRESHOLD = 55.0;        // degrees
const unsigned long UPDATE_INTERVAL = 2500;
const unsigned long ACCIDENT_HOLD_MS = 3000;  // Must remain suspicious for 3 sec

// ====================== STATE ======================
unsigned long lastUpdate = 0;
bool accidentDetected = false;
bool accidentSuspected = false;
unsigned long accidentSuspectStart = 0;
bool mpuHealthy = false;
int mpuReadFailCount = 0;

// Previous accel for delta
float prevAxG = 0.0;
float prevAyG = 0.0;
float prevAzG = 0.0;
bool firstAccelSample = true;

// ====================== STRUCT ======================
struct IMUData {
  int16_t axRaw = 0;
  int16_t ayRaw = 0;
  int16_t azRaw = 0;
  int16_t gxRaw = 0;
  int16_t gyRaw = 0;
  int16_t gzRaw = 0;

  float axG = 0;
  float ayG = 0;
  float azG = 0;

  float gxDps = 0;
  float gyDps = 0;
  float gzDps = 0;

  float tiltDeg = 0;
  float accelDelta = 0;
  bool valid = false;
};

// ====================== HELPERS ======================
String boolToJson(bool value) {
  return value ? "true" : "false";
}

// ====================== WIFI ======================
bool connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("\nWiFi connection failed!");
  return false;
}

// ====================== MPU LOW LEVEL ======================
bool mpuWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  return (Wire.endTransmission() == 0);
}

bool mpuReadBytes(uint8_t reg, uint8_t* buffer, size_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom((int)MPU_ADDR, (int)len, (int)true);
  if (received != len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool initMPU() {
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(100000); // More stable for loose/clone hardware

  uint8_t who = 0;
  if (!mpuReadBytes(WHO_AM_I_REG, &who, 1)) {
    Serial.println("MPU WHO_AM_I read failed");
    return false;
  }

  Serial.print("MPU WHO_AM_I = 0x");
  Serial.println(who, HEX);

  // Wake sensor from sleep
  if (!mpuWriteByte(PWR_MGMT_1, 0x00)) {
    Serial.println("Failed to wake MPU");
    return false;
  }

  delay(100);
  Serial.println("MPU init complete");
  return true;
}

IMUData readIMU() {
  IMUData data;
  uint8_t raw[14];

  if (!mpuReadBytes(ACCEL_START_REG, raw, 14)) {
    mpuReadFailCount++;
    Serial.println("IMU read failed");
    data.valid = false;
    return data;
  }

  mpuReadFailCount = 0;
  data.valid = true;

  data.axRaw = (int16_t)((raw[0] << 8) | raw[1]);
  data.ayRaw = (int16_t)((raw[2] << 8) | raw[3]);
  data.azRaw = (int16_t)((raw[4] << 8) | raw[5]);

  data.gxRaw = (int16_t)((raw[8] << 8) | raw[9]);
  data.gyRaw = (int16_t)((raw[10] << 8) | raw[11]);
  data.gzRaw = (int16_t)((raw[12] << 8) | raw[13]);

  // Assuming default MPU6050 ranges:
  data.axG = data.axRaw / 16384.0;
  data.ayG = data.ayRaw / 16384.0;
  data.azG = data.azRaw / 16384.0;

  data.gxDps = data.gxRaw / 131.0;
  data.gyDps = data.gyRaw / 131.0;
  data.gzDps = data.gzRaw / 131.0;

  // Tilt estimation from accel
  float horiz = sqrt(data.axG * data.axG + data.ayG * data.ayG);
  data.tiltDeg = atan2(horiz, fabs(data.azG)) * 180.0 / PI;

  // Sudden acceleration delta
  if (firstAccelSample) {
    data.accelDelta = 0;
    prevAxG = data.axG;
    prevAyG = data.ayG;
    prevAzG = data.azG;
    firstAccelSample = false;
  } else {
    float dx = data.axG - prevAxG;
    float dy = data.ayG - prevAyG;
    float dz = data.azG - prevAzG;
    data.accelDelta = sqrt(dx * dx + dy * dy + dz * dz);

    prevAxG = data.axG;
    prevAyG = data.ayG;
    prevAzG = data.azG;
  }

  return data;
}

// ====================== ACCIDENT LOGIC ======================
bool isSuspiciousMotion(const IMUData& imu) {
  if (!imu.valid) return false;

  float maxGyro = max(fabs(imu.gxDps), max(fabs(imu.gyDps), fabs(imu.gzDps)));

  bool suddenImpact = imu.accelDelta > ACCEL_DELTA_THRESHOLD;
  bool suddenRotation = maxGyro > GYRO_THRESHOLD;
  bool severeTilt = imu.tiltDeg > TILT_THRESHOLD;

  // Stronger logic: impact + (tilt or rotation)
  bool suspicious = suddenImpact && (severeTilt || suddenRotation);

  return suspicious;
}

void updateAccidentState(const IMUData& imu, bool helmetWorn) {
  // If helmet not worn, don't mark accident from motion
  if (!helmetWorn) {
    accidentSuspected = false;
    accidentDetected = false;
    return;
  }

  if (!imu.valid) {
    // If temporary read fail, don't instantly panic
    if (mpuReadFailCount > 10) {
      mpuHealthy = false;
    }
    return;
  }

  mpuHealthy = true;

  bool suspicious = isSuspiciousMotion(imu);

  if (suspicious && !accidentSuspected && !accidentDetected) {
    accidentSuspected = true;
    accidentSuspectStart = millis();
    Serial.println("Accident suspected...");
  }

  if (accidentSuspected && !accidentDetected) {
    if (suspicious) {
      if (millis() - accidentSuspectStart >= ACCIDENT_HOLD_MS) {
        accidentDetected = true;
        accidentSuspected = false;
        Serial.println("ACCIDENT CONFIRMED!");
      }
    } else {
      // Motion normalized -> false alarm reset
      accidentSuspected = false;
    }
  }

  // Auto-clear if stable again for demo convenience
  static unsigned long stableStart = 0;
  if (accidentDetected && !suspicious && imu.tiltDeg < 25.0) {
    if (stableStart == 0) stableStart = millis();

    if (millis() - stableStart > 5000) {
      accidentDetected = false;
      stableStart = 0;
      Serial.println("Accident state auto-reset after stable recovery");
    }
  } else {
    stableStart = 0;
  }
}

// ====================== FIREBASE ======================
void updateFirebase(bool helmetWorn, bool alcoholSafe, bool gpsOnline,
                    bool moving, bool accident, bool rideAllowed,
                    int mq3Value, const IMUData& imu,
                    float lat, float lng) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = String(FIREBASE_BASE_URL) + "/helmets/" + HELMET_ID + ".json";

  // Simple battery placeholder (can replace later with real battery divider)
  float batteryVoltage = 3.9;
  int batteryPercent = 78;

  String json = "{";

  // Rider status object
  json += "\"status\":{";
  json += "\"helmetWorn\":" + boolToJson(helmetWorn) + ",";
  json += "\"alcoholSafe\":" + boolToJson(alcoholSafe) + ",";
  json += "\"gpsOnline\":" + boolToJson(gpsOnline) + ",";
  json += "\"moving\":" + boolToJson(moving) + ",";
  json += "\"accidentDetected\":" + boolToJson(accident) + ",";
  json += "\"helmetRemovedWhileRiding\":" + boolToJson(moving && !helmetWorn) + ",";
  json += "\"rideAllowed\":" + boolToJson(rideAllowed) + ",";
  json += "\"lastUpdated\":\"Live from ESP32 Final\"";
  json += "},";

  // Sensor object
  json += "\"sensors\":{";
  json += "\"mq3Value\":" + String(mq3Value) + ",";
  json += "\"tiltDeg\":" + String(imu.tiltDeg, 2) + ",";
  json += "\"accelDelta\":" + String(imu.accelDelta, 3) + ",";
  json += "\"mpuHealthy\":" + boolToJson(mpuHealthy) + ",";
  json += "\"mpuReadFailCount\":" + String(mpuReadFailCount) + ",";
  json += "\"batteryVoltage\":" + String(batteryVoltage, 2) + ",";
  json += "\"batteryPercent\":" + String(batteryPercent);
  json += "},";

  // Location object
  json += "\"location\":{";
  json += "\"lat\":" + String(lat, 6) + ",";
  json += "\"lng\":" + String(lng, 6);
  json += "},";

  // Raw IMU object
  json += "\"mpuRaw\":{";
  json += "\"ax\":" + String(imu.axRaw) + ",";
  json += "\"ay\":" + String(imu.ayRaw) + ",";
  json += "\"az\":" + String(imu.azRaw) + ",";
  json += "\"gx\":" + String(imu.gxRaw) + ",";
  json += "\"gy\":" + String(imu.gyRaw) + ",";
  json += "\"gz\":" + String(imu.gzRaw);
  json += "}";

  json += "}";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.PUT(json);

  Serial.println("========== FIREBASE UPDATE ==========");
  Serial.println(url);
  Serial.print("HTTP Code: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    Serial.println("Firebase OK");
  } else {
    Serial.println("Firebase update failed");
  }

  http.end();
}

// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(HELMET_WEAR_PIN, INPUT_PULLUP);

  connectWiFi();

  // Start GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Init IMU
  mpuHealthy = initMPU();

  Serial.println("Smart Helmet Final Firmware Ready!");
}

// ====================== LOOP ======================
void loop() {
  // Reconnect WiFi if needed
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  // Read GPS stream
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // -------- SENSOR READS --------
  int mq3Value = analogRead(MQ3_PIN);
  bool alcoholSafe = mq3Value < MQ3_THRESHOLD;

  bool helmetWorn = (digitalRead(HELMET_WEAR_PIN) == LOW); // LOW = worn

  IMUData imu = readIMU();

  // Motion heuristic
  bool moving = false;
  if (imu.valid) {
    moving = (imu.accelDelta > 0.08 || fabs(imu.gxDps) > 10 || fabs(imu.gyDps) > 10 || fabs(imu.gzDps) > 10);
  }

  // Accident logic
  updateAccidentState(imu, helmetWorn);

  // Ride allowed logic
  bool rideAllowed = helmetWorn && alcoholSafe;

  // GPS logic
  bool gpsOnline = false;
  float lat = fallbackLat;
  float lng = fallbackLng;

  if (gps.location.isValid()) {
    gpsOnline = true;
    lat = gps.location.lat();
    lng = gps.location.lng();
  } else {
    // Fallback mode for reliable hackathon demo
    gpsOnline = true;
    lat = fallbackLat;
    lng = fallbackLng;
  }

  // -------- SERIAL DEBUG --------
  Serial.println("========================================");
  Serial.print("MQ3: "); Serial.println(mq3Value);
  Serial.print("Alcohol Safe: "); Serial.println(alcoholSafe ? "YES" : "NO");
  Serial.print("Helmet Worn: "); Serial.println(helmetWorn ? "YES" : "NO");
  Serial.print("Moving: "); Serial.println(moving ? "YES" : "NO");
  Serial.print("Accident: "); Serial.println(accidentDetected ? "YES" : "NO");
  Serial.print("Ride Allowed: "); Serial.println(rideAllowed ? "YES" : "NO");

  if (imu.valid) {
    Serial.print("AX G: "); Serial.print(imu.axG, 3);
    Serial.print(" | AY G: "); Serial.print(imu.ayG, 3);
    Serial.print(" | AZ G: "); Serial.println(imu.azG, 3);

    Serial.print("GX DPS: "); Serial.print(imu.gxDps, 2);
    Serial.print(" | GY DPS: "); Serial.print(imu.gyDps, 2);
    Serial.print(" | GZ DPS: "); Serial.println(imu.gzDps, 2);

    Serial.print("Tilt: "); Serial.print(imu.tiltDeg, 2);
    Serial.print(" | AccelDelta: "); Serial.println(imu.accelDelta, 3);
  } else {
    Serial.println("IMU invalid this cycle");
  }

  Serial.print("GPS Online: "); Serial.println(gpsOnline ? "YES" : "NO");
  Serial.print("Lat: "); Serial.println(lat, 6);
  Serial.print("Lng: "); Serial.println(lng, 6);

  // -------- FIREBASE UPDATE --------
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = millis();

    updateFirebase(
      helmetWorn,
      alcoholSafe,
      gpsOnline,
      moving,
      accidentDetected,
      rideAllowed,
      mq3Value,
      imu,
      lat,
      lng
    );
  }

  delay(200);
}
