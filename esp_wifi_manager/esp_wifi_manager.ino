#include <WiFi.h>
#include <WiFiManager.h>
#include <Firebase_ESP_Client.h>
#include <DHT.h>
#include <SDS011.h>
#include <AverageValue.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

// ================= LCD I2C =================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================= FIREBASE =================
#define API_KEY       "AIzaSyCrJHKtij74HFNHLtOxADIz-cHlnfCVoZY"
#define DATABASE_URL  "https://airguard-b7ef4-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL    "airguard1@gmail.com"
#define USER_PASSWORD "airguard123"

// ================= BATERAI =================
#define BATTERY_PIN 32
float batteryVoltage     = 0;
int   batteryPercent     = 0;
int   batteryPercentLast = -1;
unsigned long lastBatRead = 0;
#define BAT_READ_INTERVAL 5000
#define BAT_HYSTERESIS    2
float R1_bat = 30000.0;
float R2_bat = 7500.0;
const float bat_a = 0.977;
const float bat_b = 0.4524;

// ================= DHT =================
#define DHTPIN  4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float dht_a = 0.9568;
float dht_b = 1.9735;

// ================= MQ SENSOR =================
#define MQ7_PIN   35
#define MQ135_PIN 34
#define VD_FACTOR 2.0
#define VREF      3.3
#define VCC       5.0
#define ADC_MAX   4095.0

#define MQ7_RLOAD 10.0
float mq7_ro = 3.40;
float mq7_a  = 100.23;
float mq7_b  = -1.534;

#define MQ135_RLOAD 4.0
float mq135_r0 = 9.87;
float mq135_a  = 5.88;
float mq135_b  = -2.346;

AverageValue<float> avgCO(10);
AverageValue<float> avgVOC(10);
AverageValue<float> avgBattery(50);

// ================= SDS011 =================
SDS011 sds;
float pm25_raw, pm10_raw;
float pm25, pm10;
float a_pm25 = 0.9581;
float b_pm25 = 0.4552;
float a_pm10 = 0.9741;
float b_pm10 = 1.255;

// ===== VARIABEL GLOBAL SENSOR =====
float co_final   = 0;
float voc_final  = 0;
float suhu_final = 0;

// ===== FLAG STATUS =====
bool nodeInisialisasi = false;
bool wifiTerhubung    = false;

// ===== SPEED KIPAS TERAKHIR =====
String speedTerakhir = "";

// ===== SIMPAN KREDENSIAL WIFI =====
String savedSSID = "";
String savedPass = "";

// ===== TIMER LCD =====
unsigned long lastLcdSwitch = 0;
int lcdPage = 0;
#define LCD_SWITCH_INTERVAL 3000

// ===== TIMER CEK WIFI =====
unsigned long lastWifiCheck = 0;
#define WIFI_CHECK_INTERVAL 5000

// ===== TIMER CEK COMMAND =====
unsigned long lastCmdCheck = 0;
#define CMD_CHECK_INTERVAL 500

// ===== TIMER UPLOAD SENSOR =====
unsigned long lastSensorUpload = 0;
#define SENSOR_UPLOAD_INTERVAL 3000

// ================= ACCESS POINT =================
#define AP_SSID "AirGuard-Setup"
#define AP_IP   "192.168.4.1"
WebServer apServer(80);
DNSServer dnsServer;

// ================= SERIAL2 KE MEGA =================
#define RXD2 26
#define TXD2 27

// ================= FIREBASE OBJ =================
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

WiFiManager wm;

// ================= KIRIM INFO WIFI KE MEGA =================
void kirimInfoWifi() {
  String info = "WIFI," + savedSSID + "," + savedPass;
  Serial2.println(info);
  Serial.println("[WIFI INFO] Kirim ke Mega: " + info);
}

// ================= REINIT LCD =================
void reinitLCD() {
  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(50);
  lcd.init();
  delay(50);
  lcd.backlight();
  delay(50);
}

// ================= HALAMAN WEB AP =================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>AirGuard WiFi Setup</title>";
  html += "<style>";
  html += "body{font-family:sans-serif;max-width:400px;margin:30px auto;padding:20px;background:#f5f5f5;}";
  html += "h2{color:#2196F3;text-align:center;}";
  html += "select,input{width:100%;padding:10px;margin:8px 0;border:1px solid #ddd;border-radius:6px;box-sizing:border-box;font-size:15px;}";
  html += "button{width:100%;padding:12px;background:#2196F3;color:#fff;border:none;border-radius:6px;font-size:16px;cursor:pointer;}";
  html += "button:hover{background:#1976D2;}";
  html += ".info{background:#fff;padding:12px;border-radius:6px;margin-bottom:16px;border-left:4px solid #2196F3;}";
  html += "</style></head><body>";
  html += "<h2>AirGuard WiFi Setup</h2>";
  html += "<div class='info'><b>Status WiFi:</b> ";
  html += wifiTerhubung ? "Terhubung ke " + WiFi.SSID() : "Tidak terhubung";
  html += "</div>";
  html += "<form action='/save' method='POST'>";
  html += "<label>Pilih WiFi:</label><select name='ssid'>";
  int n = WiFi.scanNetworks(false, false, false, 500);
  for (int i = 0; i < n; i++) {
    html += "<option value='" + WiFi.SSID(i) + "'>";
    html += WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
  }
  html += "</select>";
  html += "<label>Password:</label>";
  html += "<input type='password' name='pass' placeholder='Kosongkan jika tidak ada password'>";
  html += "<button type='submit'>Sambungkan</button>";
  html += "</form></body></html>";
  apServer.send(200, "text/html", html);
}

void handleSave() {
  savedSSID = apServer.arg("ssid");
  savedPass = apServer.arg("pass");

  // simpan ke Preferences
  Preferences prefs;
  prefs.begin("wifi", false);
  prefs.putString("ssid", savedSSID);
  prefs.putString("pass", savedPass);
  prefs.end();

  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<style>body{font-family:sans-serif;max-width:400px;margin:30px auto;padding:20px;text-align:center;}";
  html += "h2{color:#4CAF50;}</style></head><body>";
  html += "<h2>Menyambungkan...</h2>";
  html += "<p>Mencoba konek ke <b>" + savedSSID + "</b></p>";
  html += "<p>Alat akan restart sebentar.</p></body></html>";
  apServer.send(200, "text/html", html);
  delay(500);
  reinitLCD();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Ganti WiFi...");
  lcd.setCursor(0, 1); lcd.print(savedSSID.substring(0, 16));
  delay(1000);
  wm.resetSettings();
  WiFi.persistent(true);
  WiFi.begin(savedSSID.c_str(), savedPass.c_str());
  delay(2000);
  ESP.restart();
}

void handleNotFound() {
  apServer.sendHeader("Location", "http://" + String(AP_IP), true);
  apServer.send(302, "text/plain", "");
}

// ================= FUNGSI BATERAI =================
float bacaTegangan() {
  int   adc      = analogRead(BATTERY_PIN);
  float vADC     = (adc * 3.3) / 4095.0;
  float vDivider = vADC / (R2_bat / (R1_bat + R2_bat));
  return (bat_a * vDivider) + bat_b;
}

int voltageToPercent(float v) {
  int pct = (int)((v - 12.0) / (16.8 - 12.0) * 100.0);
  return constrain(pct, 0, 100);
}

// ================= FUNGSI SENSOR =================
float koreksiPM25(float raw) { return (a_pm25 * raw) + b_pm25; }
float koreksiPM10(float raw) { return (a_pm10 * raw) + b_pm10; }
float koreksiSuhu(float raw) { return (dht_a  * raw) + dht_b;  }

float bacaCO() {
  int adc = analogRead(MQ7_PIN);
  if (adc <= 0) adc = 1;
  float vout_esp  = (adc / ADC_MAX) * VREF;
  float vout_real = vout_esp * VD_FACTOR;
  if (vout_real <= 0) vout_real = 0.01;
  float rs_mq7 = ((VCC - vout_real) / vout_real) * MQ7_RLOAD;
  return mq7_a * pow(rs_mq7 / mq7_ro, mq7_b);
}

float bacaVOC() {
  int adc = analogRead(MQ135_PIN);
  if (adc <= 0) adc = 1;
  float vout_esp  = (adc / ADC_MAX) * VREF;
  float vout_real = vout_esp * VD_FACTOR;
  if (vout_real <= 0) vout_real = 0.01;
  float rs_mq135 = ((VCC - vout_real) / vout_real) * MQ135_RLOAD;
  return mq135_a * pow(rs_mq135 / mq135_r0, mq135_b);
}

// ================= BACA STATUS KIPAS DARI MEGA =================
void bacaStatusMega() {
  while (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.startsWith("FAN:") || line == "OFF") {
      if (wifiTerhubung && Firebase.ready()) {
        Firebase.RTDB.setString(&fbdo, "/Status/kipas", line);
      }
      Serial.println("[FAN STATUS] " + line);
    }
  }
}

// ================= CEK COMMAND FIREBASE =================
void cekCommandFirebase() {
  if (!wifiTerhubung || !Firebase.ready()) return;
  if (millis() - lastCmdCheck < CMD_CHECK_INTERVAL) return;
  lastCmdCheck = millis();

  if (Firebase.RTDB.getString(&fbdo, "/Command/speed")) {
    String speed = fbdo.stringData();
    speed.trim();
    speed.toUpperCase();
    if ((speed == "HIGH" || speed == "NORMAL" ||
         speed == "LOW"  || speed == "OFF") &&
         speed != speedTerakhir) {
      if (speed == "OFF") Serial2.println("MANUAL:OFF");
      else                Serial2.println("MANUAL:" + speed);
      Serial.println("[KIPAS] Kirim ke Mega: MANUAL:" + speed);
      Firebase.RTDB.setString(&fbdo, "/Status/kipas", speed);
      speedTerakhir = speed;
    }
  }
}

// ================= UPLOAD SENSOR KE FIREBASE =================
void uploadSensor() {
  if (!wifiTerhubung || !Firebase.ready()) return;
  if (millis() - lastSensorUpload < SENSOR_UPLOAD_INTERVAL) return;
  lastSensorUpload = millis();

  Firebase.RTDB.setFloat(&fbdo, "/Udara/PM25",       pm25);
  Firebase.RTDB.setFloat(&fbdo, "/Udara/PM10",       pm10);
  Firebase.RTDB.setFloat(&fbdo, "/Udara/CO",         co_final);
  Firebase.RTDB.setFloat(&fbdo, "/Udara/VOC",        voc_final);
  Firebase.RTDB.setFloat(&fbdo, "/Udara/Suhu",       suhu_final);
  Firebase.RTDB.setFloat(&fbdo, "/Udara/Tegangan",   batteryVoltage);
  Firebase.RTDB.setInt  (&fbdo, "/Udara/Persentase", batteryPercent);
  Serial.println("[SENSOR] Data terkirim ke Firebase");
}

// ================= FUNGSI TAMPIL LCD =================
void tampilLCD() {
  lcd.clear();
  switch (lcdPage) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("PM2.5:");
      lcd.print(pm25, 1);
      lcd.print("ug/m3");
      lcd.setCursor(0, 1);
      lcd.print("Status:");
      if      (pm25 <= 35.4)  lcd.print("Baik     ");
      else if (pm25 <= 125.4) lcd.print("Perhatian");
      else                    lcd.print("Bahaya   ");
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("PM10 :");
      lcd.print(pm10, 1);
      lcd.print("ug/m3");
      lcd.setCursor(0, 1);
      lcd.print("Status:");
      if      (pm10 <= 154.0) lcd.print("Baik     ");
      else if (pm10 <= 354.0) lcd.print("Perhatian");
      else                    lcd.print("Bahaya   ");
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("CO  :");
      lcd.print(co_final, 1);
      lcd.print(" ppm");
      lcd.setCursor(0, 1);
      lcd.print("VOC :");
      lcd.print(voc_final, 1);
      lcd.print(" ppm");
      break;
    case 3:
      lcd.setCursor(0, 0);
      lcd.print("Suhu:");
      lcd.print(suhu_final, 1);
      lcd.print((char)223);
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print("Bat :");
      lcd.print(batteryPercent);
      lcd.print("% ");
      lcd.print(batteryVoltage, 2);
      lcd.print("V");
      break;
    case 4:
      lcd.setCursor(0, 0);
      if (wifiTerhubung) lcd.print("WiFi: Connected");
      else               lcd.print("WiFi: Offline  ");
      lcd.setCursor(0, 1);
      if      (wifiTerhubung && Firebase.ready()) lcd.print("Firebase: OK   ");
      else if (wifiTerhubung)                     lcd.print("Firebase: Wait ");
      else                                        lcd.print("Firebase: Off  ");
      break;
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(100);
  lcd.init();
  delay(50);
  lcd.clear();
  delay(50);
  lcd.backlight();
  delay(100);
  lcd.setCursor(0, 0); lcd.print("AirGuard v1.0");
  lcd.setCursor(0, 1); lcd.print("Inisialisasi...");
  delay(1500);

  // load kredensial WiFi dari Preferences
  Preferences prefs;
  prefs.begin("wifi", true);
  savedSSID = prefs.getString("ssid", "");
  savedPass = prefs.getString("pass", "");
  prefs.end();
  Serial.println("[PREFS] SSID: " + savedSSID);

  pinMode(MQ7_PIN,   INPUT);
  pinMode(MQ135_PIN, INPUT);
  dht.begin();
  sds.begin(16, 17);

  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Sensor siap");
  lcd.setCursor(0, 1); lcd.print("Konek WiFi...");

  wm.setConnectTimeout(20);
  wm.setConfigPortalTimeout(60);
  bool res = wm.autoConnect("AirGuard-Setup");
  reinitLCD();

  if (!res) {
    wifiTerhubung = false;
    Serial.println("WiFi gagal, mode OFFLINE");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi gagal!");
    lcd.setCursor(0, 1); lcd.print("Mode Offline");
    delay(2000);
  } else {
    wifiTerhubung = true;
    Serial.println("WiFi Connected");
    kirimInfoWifi();  // kirim SSID+Pass ke Mega → Orange Pi
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi Connected!");
    lcd.setCursor(0, 1); lcd.print("Init Firebase...");
    config.api_key      = API_KEY;
    config.database_url = DATABASE_URL;
    auth.user.email     = USER_EMAIL;
    auth.user.password  = USER_PASSWORD;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    delay(1500);
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID);
  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  dnsServer.start(53, "*", apIP);
  apServer.on("/",     handleRoot);
  apServer.on("/save", HTTP_POST, handleSave);
  apServer.onNotFound(handleNotFound);
  apServer.begin();
  reinitLCD();

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("AP: AirGuard");
  lcd.setCursor(0, 1); lcd.print("192.168.4.1");
  delay(1500);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Cek baterai...");
  for (int i = 0; i < 50; i++) {
    avgBattery.push(bacaTegangan());
    delay(10);
  }
  batteryVoltage     = avgBattery.average();
  batteryPercent     = voltageToPercent(batteryVoltage);
  batteryPercentLast = batteryPercent;
  lcd.setCursor(0, 1);
  lcd.print("Bat:");
  lcd.print(batteryPercent);
  lcd.print("% ");
  lcd.print(batteryVoltage, 2);
  lcd.print("V");
  delay(1500);

  if (wifiTerhubung && Firebase.ready()) {
    Firebase.RTDB.setString(&fbdo, "/Status/kipas",  "");
    Firebase.RTDB.setString(&fbdo, "/Command/speed", "");
    nodeInisialisasi = true;
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Sistem siap!");
  delay(1000);
}

// ================= LOOP =================
void loop() {
  dnsServer.processNextRequest();
  apServer.handleClient();

  // === Cek status WiFi ===
  if (millis() - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
    bool statusBaru = (WiFi.status() == WL_CONNECTED);
    if (wifiTerhubung && !statusBaru) {
      wifiTerhubung    = false;
      nodeInisialisasi = false;
      Serial.println("WiFi putus, mode offline");
      reinitLCD();
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("WiFi: Offline  ");
      lcd.setCursor(0, 1); lcd.print("Mode Offline   ");
      delay(1000);
    }
    if (!wifiTerhubung && statusBaru) {
      wifiTerhubung = true;
      Serial.println("WiFi konek kembali");
      kirimInfoWifi();  // kirim ulang saat reconnect
      reinitLCD();
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("WiFi: Connected");
      lcd.setCursor(0, 1); lcd.print("Reconnected!   ");
      delay(1000);
    }
    lastWifiCheck = millis();
  }

  if (wifiTerhubung && Firebase.ready() && !nodeInisialisasi) {
    Firebase.RTDB.setString(&fbdo, "/Status/kipas",  "");
    Firebase.RTDB.setString(&fbdo, "/Command/speed", "");
    nodeInisialisasi = true;
  }

  // === Baca status kipas dari Mega ===
  bacaStatusMega();

  // === Cek command kipas ===
  cekCommandFirebase();

  // === Baca sensor ===
  if (sds.read(&pm25_raw, &pm10_raw) == 0) {
    pm25 = koreksiPM25(pm25_raw);
    pm10 = koreksiPM10(pm10_raw);
  }

  float suhu_raw = dht.readTemperature();
  if (!isnan(suhu_raw)) suhu_final = koreksiSuhu(suhu_raw);

  avgCO.push(bacaCO());
  co_final = avgCO.average();

  avgVOC.push(bacaVOC());
  voc_final = avgVOC.average();

  if (millis() - lastBatRead >= BAT_READ_INTERVAL) {
    avgBattery.push(bacaTegangan());
    batteryVoltage = avgBattery.average();
    int newPct = voltageToPercent(batteryVoltage);
    if (abs(newPct - batteryPercentLast) >= BAT_HYSTERESIS) {
      batteryPercent     = newPct;
      batteryPercentLast = newPct;
    }
    lastBatRead = millis();
  }

  // === Update LCD ===
  if (millis() - lastLcdSwitch >= LCD_SWITCH_INTERVAL) {
    lcdPage = (lcdPage + 1) % 5;
    tampilLCD();
    lastLcdSwitch = millis();
  }

  // === Upload sensor ===
  if (wifiTerhubung && Firebase.ready()) {
    uploadSensor();
  } else {
    String autoCmd = "AUTO:" + String(pm25,     1) + ":" +
                               String(pm10,     1) + ":" +
                               String(co_final, 1) + ":" +
                               String(voc_final,1);
    Serial2.println(autoCmd);
  }

  Serial.printf("PM2.5:%.2f PM10:%.2f CO:%.2f VOC:%.2f Suhu:%.2f Bat:%.2fV(%d%%)\n",
    pm25, pm10, co_final, voc_final, suhu_final,
    batteryVoltage, batteryPercent);

  delay(10);
}