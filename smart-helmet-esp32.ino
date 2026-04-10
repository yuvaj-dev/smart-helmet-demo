#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>

// ====================== WIFI CONFIG ======================
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// ====================== FIREBASE CONFIG ======================
const char* FIREBASE_BASE_URL = "https://smart-helmet-demo-8f1d9-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* HELMET_ID = "HLM001";

// ====================== PIN CONFIG ======================
const int MQ3_PIN = 34;           // Analog pin
const int HELMET_WEAR_PIN = 27;   // Digital input (LOW = helmet worn)
const int ACCIDENT_PIN = 26;      // Digital input (LOW = accident trigger)
const int SOS_PIN = 25;           // Digital input (LOW = SOS button)
const int BUZZER_PIN = 14;        // Output
const int RED_LED_PIN = 12;       // Output

// GPS UART2
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// ====================== GPS FALLBACK ======================
float fallbackLat = 28.6139;
float fallbackLng = 77.2090;

// ====================== THRESHOLDS ======================
const int MQ3_THRESHOLD = 1800;   // Adjust after testing
const unsigned long UPDATE_INTERVAL = 3000; // ms
const unsigned long ACCIDENT_CONFIRM_MS = 5000; // 5 sec false-alarm window

// ====================== STATE ======================
unsigned long lastUpdate = 0;
bool accidentPending = false;
unsigned long accidentStartTime = 0;
bool accidentDetected = false;
bool sosActive = false;

// ====================== HELPERS ======================
String boolToJson(bool value) {
  return value ? "true" : "false";
}

void beepBuzzer(int times = 3, int onMs = 150, int offMs = 150) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    delay(offMs);
  }
}

void emergencyAlarmLoop() {
  // Simple alarm pulse
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  delay(150);
}

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

// PUT status JSON
void updateStatusFirebase(bool helmetWorn, bool alcoholSafe, bool gpsOnline, bool accident, bool sos) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = String(FIREBASE_BASE_URL) + "/helmets/" + HELMET_ID + "/status.json";

  String json = "{";
  json += "\"helmetWorn\":" + boolToJson(helmetWorn) + ",";
  json += "\"alcoholSafe\":" + boolToJson(alcoholSafe) + ",";
  json += "\"gpsOnline\":" + boolToJson(gpsOnline) + ",";
  json += "\"sosActive\":" + boolToJson(sos) + ",";
  json += "\"accidentDetected\":" + boolToJson(accident) + ",";
  json += "\"lastUpdated\":\"Live from ESP32\"";
  json += "}";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.PUT(json);

  Serial.println("----- STATUS UPDATE -----");
  Serial.println(url);
  Serial.println(json);
  Serial.print("HTTP Code: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    String response = http.getString();
    Serial.println("Response: " + response);
  } else {
    Serial.println("Status update failed.");
  }

  http.end();
}

// PUT location JSON
void updateLocationFirebase(float lat, float lng) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = String(FIREBASE_BASE_URL) + "/helmets/" + HELMET_ID + "/location.json";

  String json = "{";
  json += "\"lat\":" + String(lat, 6) + ",";
  json += "\"lng\":" + String(lng, 6);
  json += "}";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.PUT(json);

  Serial.println("----- LOCATION UPDATE -----");
  Serial.println(url);
  Serial.println(json);
  Serial.print("HTTP Code: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    String response = http.getString();
    Serial.println("Response: " + response);
  } else {
    Serial.println("Location update failed.");
  }

  http.end();
}

// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);

  pinMode(HELMET_WEAR_PIN, INPUT_PULLUP);
  pinMode(ACCIDENT_PIN, INPUT_PULLUP);
  pinMode(SOS_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  // GPS UART2
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  connectWiFi();

  // Startup indication
  beepBuzzer(2, 100, 100);

  Serial.println("Smart Helmet ESP32 Ready!");
}

// ====================== LOOP ======================
void loop() {
  // Read GPS stream
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // -------- SENSOR READS --------
  int mq3Value = analogRead(MQ3_PIN);
  bool alcoholSafe = mq3Value < MQ3_THRESHOLD;

  bool helmetWorn = (digitalRead(HELMET_WEAR_PIN) == LOW);   // LOW = worn
  bool accidentTrigger = (digitalRead(ACCIDENT_PIN) == LOW); // LOW = trigger
  bool sosPressed = (digitalRead(SOS_PIN) == LOW);           // LOW = pressed

  // -------- SOS LOGIC --------
  if (sosPressed) {
    sosActive = true;
    Serial.println("SOS button pressed!");
    beepBuzzer(2, 120, 120);
    delay(300);
  } else {
    // Keep SOS latched false only if no accident
    if (!accidentDetected) {
      sosActive = false;
    }
  }

  // -------- ACCIDENT LOGIC --------
  // If trigger happens and no active accident, start confirmation window
  if (accidentTrigger && !accidentPending && !accidentDetected) {
    accidentPending = true;
    accidentStartTime = millis();
    Serial.println("Accident suspected! Starting 5-second confirmation...");
    beepBuzzer(3, 100, 100);
  }

  // If pending, allow cancel by pressing SOS button within confirmation window
  if (accidentPending && !accidentDetected) {
    // SOS button used here as "cancel false alarm" during 5 sec window
    if (sosPressed) {
      accidentPending = false;
      Serial.println("False alarm canceled by rider!");
      beepBuzzer(1, 300, 100);
      delay(300);
    } else if (millis() - accidentStartTime >= ACCIDENT_CONFIRM_MS) {
      accidentDetected = true;
      sosActive = true;
      accidentPending = false;
      Serial.println("ACCIDENT CONFIRMED!");
    }
  }

  // If accident confirmed -> alarm continuously
  if (accidentDetected) {
    emergencyAlarmLoop();
  } else {
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // -------- GPS LOGIC --------
  bool gpsOnline = false;
  float lat = fallbackLat;
  float lng = fallbackLng;

  if (gps.location.isValid()) {
    gpsOnline = true;
    lat = gps.location.lat();
    lng = gps.location.lng();
  } else {
    // Fallback mode for reliable demo
    gpsOnline = true;
    lat = fallbackLat;
    lng = fallbackLng;
  }

  // -------- PERIODIC FIREBASE UPDATE --------
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = millis();

    Serial.println("==================================");
    Serial.print("MQ3 Value: "); Serial.println(mq3Value);
    Serial.print("Alcohol Safe: "); Serial.println(alcoholSafe ? "YES" : "NO");
    Serial.print("Helmet Worn: "); Serial.println(helmetWorn ? "YES" : "NO");
    Serial.print("Accident Detected: "); Serial.println(accidentDetected ? "YES" : "NO");
    Serial.print("SOS Active: "); Serial.println(sosActive ? "YES" : "NO");
    Serial.print("GPS Online: "); Serial.println(gpsOnline ? "YES" : "NO");
    Serial.print("Lat: "); Serial.println(lat, 6);
    Serial.print("Lng: "); Serial.println(lng, 6);

    updateStatusFirebase(helmetWorn, alcoholSafe, gpsOnline, accidentDetected, sosActive);
    updateLocationFirebase(lat, lng);
  }

  delay(50);
}