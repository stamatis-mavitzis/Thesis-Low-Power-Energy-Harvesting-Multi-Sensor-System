#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------------- LCD (I2C 20x4) ----------------
// Most 2004 I2C backpacks are 0x27 or 0x3F. Try 0x27 first.
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4); //SDA -> G21 || SCL -> G22

// Keep a reference to the real hardware Serial BEFORE we override it
HardwareSerial &HWSerial = Serial;

// A tiny "terminal" that prints to a 20x4 LCD with wrapping + scrolling
class LcdTerminal : public Print {
public:
  LcdTerminal(LiquidCrystal_I2C &d) : _lcd(d) { clearBuffer(); }

  void begin() {
    _lcd.init();
    _lcd.backlight();
    clear();
  }

  void clear() {
    clearBuffer();
    redraw();
  }

  size_t write(uint8_t c) override {
    if (c == '\r') return 1;          // ignore CR
    if (c == '\n') {                  // newline
      newLine();
      redraw();
      return 1;
    }
    if (c == '\t') c = ' ';           // tabs -> space

    _buf[_row][_col] = (char)c;
    _col++;

    if (_col >= 20) newLine();        // wrap

    redraw();
    return 1;
  }

private:
  LiquidCrystal_I2C &_lcd;
  char _buf[4][21];
  int _row = 0;
  int _col = 0;

  void clearBuffer() {
    for (int r = 0; r < 4; r++) {
      for (int c = 0; c < 20; c++) _buf[r][c] = ' ';
      _buf[r][20] = '\0';
    }
    _row = 0;
    _col = 0;
  }

  void newLine() {
    _col = 0;
    _row++;
    if (_row >= 4) {
      // scroll up
      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 20; c++) _buf[r][c] = _buf[r + 1][c];
      }
      // clear last row
      for (int c = 0; c < 20; c++) _buf[3][c] = ' ';
      _row = 3;
    }
  }

  void redraw() {
    for (int r = 0; r < 4; r++) {
      _lcd.setCursor(0, r);
      _lcd.print(_buf[r]);
    }
    _lcd.setCursor(_col, _row);
  }
};

// Tee printer: duplicates everything written to A and B
class TeePrint : public Print {
public:
  TeePrint(Print &a, Print &b) : _a(a), _b(b) {}
  size_t write(uint8_t c) override {
    _a.write(c);
    _b.write(c);
    return 1;
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    _a.write(buffer, size);
    _b.write(buffer, size);
    return size;
  }
private:
  Print &_a;
  Print &_b;
};

LcdTerminal lcdTerm(lcd);
TeePrint DebugOut(HWSerial, lcdTerm);

// From here on, every Serial.print goes to Serial Monitor + LCD
#define Serial DebugOut

// ---------------- ADC ----------------
#define USE_11DB 1   // set to 1 for 11dB, 0 for 6dB (Attenuation)

#if USE_11DB
const adc_attenuation_t ADC_ATTEN = ADC_11db;
#else
const adc_attenuation_t ADC_ATTEN = ADC_6db;
#endif

// ---------------- WiFi ----------------
const char* WIFI_SSID = "Armor 24";
const char* WIFI_PASS = "12312345";

// ---------------- ThingSpeak ----------------
// Channel: "Multi-sensor system - Threshold Status"
const char* TS_KEY_THRESHOLD = "O5M9XQFMHBMU93KZ";
// Channel: "Multi-sensor system - Measurements"
const char* TS_KEY_MEASUREMENTS = "FBXCEQHNAZFGLVEK";

// Upload period (ThingSpeak limit ~15s/channel). Use 30s to be safe.
const unsigned long TS_UPLOAD_PERIOD_MS = 30000;
unsigned long lastUploadMs = 0;

// ------- Analog INPUT pins (KEEP YOUR CURRENT PINS) -------
const int PIN_TEMPERATURE_VALUE     = 12;   // G12  (ADC2)
const int PIN_HUMIDITY_VALUE        = 14;   // G14  (ADC2)
const int PIN_SOIL_VALUE            = 27;   // G27  (ADC2)
const int PIN_LIGHT                 = 26;   // G26  (ADC2)

// ------- Digital INPUT pins -------
const int PIN_TEMPERATURE_COMP      = 19;   // G19
const int PIN_HUMIDITY_COMP         = 18;   // G18
const int PIN_SOIL_COMP             = 5;    // G5
const int PIN_FIRE_COMP             = 17;   // G17
const int PIN_LDR_COMP              = 16;   // G16
const int PIN_RAIN_COMP             = 4;    // G4

// OUTPUT PINS
const int PIN_MCU_DONE = 13;  // G13

// Variable of how many samples to take when reading analog
const int samplesAnalogReading = 100;
// Variable of delay between taking the samples when reading analog
const int samplesDelay = 10;

// --------- Calibration (EDIT THESE after you measure) ---------
const float SOIL_DRY_V = 2.80;   // dry
const float SOIL_WET_V = 1.20;   // wet

const float LIGHT_DARK_V   = 0.20;
const float LIGHT_BRIGHT_V = 0.02;

const bool LIGHT_INVERT = true;

// ---------------- Helpers ----------------
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float mapToPercent(float x, float inMin, float inMax) {
  if (inMax == inMin) return 0.0f;
  float t = (x - inMin) / (inMax - inMin);
  t = clampf(t, 0.0f, 1.0f);
  return t * 100.0f;
}

// Average RAW ADC reading (debug)
int analogRead_Debounce_Raw(int pin) {
  long sum = 0;
  for (int i = 0; i < samplesAnalogReading; i++) {
    sum += analogRead(pin);
    delay(samplesDelay);
  }
  return (int)(sum / samplesAnalogReading);
}

// Average CALIBRATED voltage using analogReadMilliVolts (recommended)
float analogReadVolts_Avg(int pin) {
  long sumMv = 0;
  for (int i = 0; i < samplesAnalogReading; i++) {
    sumMv += (long)analogReadMilliVolts(pin);
    delay(samplesDelay);
  }
  float mv = (float)sumMv / (float)samplesAnalogReading;
  return mv / 1000.0f;
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi already connected to: ");
    Serial.print(WiFi.SSID());
    return;
  }

  Serial.print("Connecting to WiFi SSID: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  const unsigned long timeoutMs = 15000;
  unsigned long t0 = millis();

  Serial.print("Waiting");
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeoutMs) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connection SUCCESSFUL!");
  } else {
    Serial.println("WiFi connection FAILED!");
    Serial.print("Status code: ");
    Serial.println((int)WiFi.status()); // 0..6
    Serial.println("Will retry later.");
  }
}

// Simple ThingSpeak update via HTTP GET
bool thingSpeakUpdate(const char* writeKey, const String& fieldsQuery) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  String url = "http://api.thingspeak.com/update?api_key=" + String(writeKey) + fieldsQuery;

  http.begin(url);
  int httpCode = http.GET();
  String payload = http.getString();
  http.end();

  if (httpCode == 200 && payload.toInt() > 0) return true;

  Serial.print("ThingSpeak update failed. HTTP=");
  Serial.print(httpCode);
  Serial.print(" payload=");
  Serial.println(payload);
  return false;
}

// IMPORTANT: ADC2 works reliably only when WiFi is OFF
void wifiOffForADC2() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(50);
}

// ---------------- Setup ----------------
void setup() {
  HWSerial.begin(115200);

  // ESP32 I2C pins: SDA = G21(GPIO21), SCL = G22(GPIO22)
  Wire.begin(21, 22);

  // Start LCD
  lcdTerm.begin();

  delay(300);

  // ADC setup
  analogReadResolution(12);

  // Set attenuation for each analog input pin:
  analogSetPinAttenuation(PIN_TEMPERATURE_VALUE, ADC_ATTEN);
  analogSetPinAttenuation(PIN_HUMIDITY_VALUE,    ADC_ATTEN);
  analogSetPinAttenuation(PIN_SOIL_VALUE,        ADC_ATTEN);
  analogSetPinAttenuation(PIN_LIGHT,             ADC_ATTEN);

  pinMode(PIN_TEMPERATURE_COMP, INPUT);
  pinMode(PIN_HUMIDITY_COMP,    INPUT);
  pinMode(PIN_SOIL_COMP,        INPUT);
  pinMode(PIN_FIRE_COMP,        INPUT);
  pinMode(PIN_LDR_COMP,         INPUT);
  pinMode(PIN_RAIN_COMP,        INPUT);

  pinMode(PIN_MCU_DONE, OUTPUT);
  digitalWrite(PIN_MCU_DONE, LOW);

  // Start with WiFi OFF so ADC2 reads work
  wifiOffForADC2();

  Serial.println("\nESP32 setup finished\n");
}

// ---------------- Loop ----------------
void loop() {
  digitalWrite(PIN_MCU_DONE, LOW);

  // 1) READ MEASUREMENTS FIRST (WiFi OFF -> ADC2 OK)
  wifiOffForADC2();
  lcdTerm.clear();

  // ---- Analog reads ----
  int a_temp_raw          = analogRead_Debounce_Raw(PIN_TEMPERATURE_VALUE);
  int a_humidity_raw      = analogRead_Debounce_Raw(PIN_HUMIDITY_VALUE);
  int a_soil_raw          = analogRead_Debounce_Raw(PIN_SOIL_VALUE);
  int a_light_raw         = analogRead_Debounce_Raw(PIN_LIGHT);

  // Calibrated volts
  float temp_value_v      = analogReadVolts_Avg(PIN_TEMPERATURE_VALUE);
  float humidity_value_v  = analogReadVolts_Avg(PIN_HUMIDITY_VALUE);
  float soil_value_v      = analogReadVolts_Avg(PIN_SOIL_VALUE);
  float light_value_v     = analogReadVolts_Avg(PIN_LIGHT);

  // --- TMP36 (C): (V - 0.5) * 100 ---
  float tempC = (temp_value_v - 0.5f) * 100.0f;

  // --- HIH-5030 (%RH): RH = (Vout/Vs - 0.16) / 0.0062 ---
  const float Vs = 3.3f;
  float humidityRH = ((humidity_value_v / Vs) - 0.16f) / 0.0062f;
  humidityRH = clampf(humidityRH, 0.0f, 100.0f);

  // --- Soil moisture % (needs calibration) ---
  float soilPct = mapToPercent(soil_value_v, SOIL_DRY_V, SOIL_WET_V);

  // --- Light % (needs calibration) ---
  float lightPct = mapToPercent(light_value_v, LIGHT_DARK_V, LIGHT_BRIGHT_V);
  if (LIGHT_INVERT) lightPct = 100.0f - lightPct;

  // ---- Digital reads ----
  int temperature_comp  = digitalRead(PIN_TEMPERATURE_COMP);
  int humidity_comp     = digitalRead(PIN_HUMIDITY_COMP);
  int soil_comp         = digitalRead(PIN_SOIL_COMP);
  int fire_comp         = digitalRead(PIN_FIRE_COMP);
  int ldr_comp          = digitalRead(PIN_LDR_COMP);
  int rain_comp         = digitalRead(PIN_RAIN_COMP);

  // ---- Print measurements ----
  Serial.println("Measurements taken");

  // 2) THEN CONNECT TO WIFI AND UPLOAD (ONLY EVERY 30s)
  // ======== CHANGED: removed the 30s wait gate; upload every loop ========
  connectWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Uploading to ThingSpeak now...");

    // Measurements channel (UPLOAD REAL VALUES, NOT VOLTS)
    String qMeas = "";
    qMeas += "&field1=" + String(tempC, 2);
    qMeas += "&field2=" + String(humidityRH, 1);
    qMeas += "&field3=" + String(soilPct, 1);
    qMeas += "&field4=" + String(lightPct, 1);

    bool ok1 = thingSpeakUpdate(TS_KEY_MEASUREMENTS, qMeas);
    Serial.print("ThingSpeak Measurements: ");
    Serial.println(ok1 ? "OK" : "FAIL");

    // Thresholds channel
    String qThr = "";
    qThr += "&field1=" + String(temperature_comp);
    qThr += "&field2=" + String(humidity_comp);
    qThr += "&field3=" + String(soil_comp);
    qThr += "&field4=" + String(fire_comp);
    qThr += "&field5=" + String(ldr_comp);
    qThr += "&field6=" + String(rain_comp);

    bool ok2 = thingSpeakUpdate(TS_KEY_THRESHOLD, qThr);
    Serial.print("ThingSpeak Thresholds: ");
    Serial.println(ok2 ? "OK" : "FAIL");
  } else {
    Serial.println("WiFi not connected -> upload skipped.");
  }
  // ======== END CHANGED BLOCK ========

  digitalWrite(PIN_MCU_DONE, HIGH);
  delay(10);
  digitalWrite(PIN_MCU_DONE, LOW);
  delay(TS_UPLOAD_PERIOD_MS);
}