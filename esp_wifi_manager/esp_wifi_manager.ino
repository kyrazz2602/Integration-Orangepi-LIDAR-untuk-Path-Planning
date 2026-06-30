#include <AverageValue.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <SDS011.h>
#include <Wire.h>
#include <math.h>

// ================= LCD I2C =================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================= BATERAI =================
#define BATTERY_PIN 32
float batteryVoltage = 0;
int batteryPercent = 0;
int batteryPercentLast = -1;
unsigned long lastBatRead = 0;
#define BAT_READ_INTERVAL 5000
#define BAT_HYSTERESIS 2
float R1_bat = 30000.0;
float R2_bat = 7500.0;
const float bat_a = 0.977;
const float bat_b = 0.4524;

// ================= DHT =================
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float dht_a = 0.9319;
float dht_b = -0.2831;

// ================= MQ SENSOR =================
#define MQ7_PIN 35
#define MQ135_PIN 34
#define VD_FACTOR 2.0
#define VREF 3.3
#define VCC_MQ 5.0
#define ADC_MAX 4095.0

#define MQ7_RLOAD 10.0
float mq7_ro = 3.40;
float mq7_a = 100.23;
float mq7_b = -1.534;

#define MQ135_RLOAD 4.0
float mq135_r0 = 46.44;
float mq135_a = 0.0334;
float mq135_b = -1.732;

AverageValue<float> avgCO(10);
AverageValue<float> avgVOC(10);
AverageValue<float> avgBattery(50);

// ================= SDS011 =================
SDS011 sds;
float pm25_raw, pm10_raw;
float pm25 = 0, pm10 = 0;
float a_pm25 = 0.9581;
float b_pm25 = 0.4552;
float a_pm10 = 0.9741;
float b_pm10 = 1.255;

// ===== VARIABEL GLOBAL SENSOR =====
float co_final = 0;
float voc_final = 0;
float suhu_final = 0;

// ===== SERIAL2 KE ORANGE PI =====
#define RXD2 26
#define TXD2 27

// ===== TIMER LCD =====
unsigned long lastLcdSwitch = 0;
int lcdPage = 0;
#define LCD_SWITCH_INTERVAL 3000

// ===== TIMER BACA & TRANSMI SENSOR =====
unsigned long lastSensorRead = 0;
#define SENSOR_READ_INTERVAL 1000

// ===== OVERRIDE LCD DARI MASTER =====
bool isLcdOverridden = false;
unsigned long lastLcdOverride = 0;
#define LCD_OVERRIDE_TIMEOUT 5000

String inputBuffer = "";
String inputBuffer2 = "";

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

// ================= FUNGSI BATERAI =================
float bacaTegangan() {
  int adc = analogRead(BATTERY_PIN);
  float vADC = (adc * 3.3) / 4095.0;
  vADC *= 1.10; // kompensasi resistor tambahan 10k-100k
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
float koreksiSuhu(float raw) { return (dht_a * raw) + dht_b; }

float bacaCO() {
  int adc = analogRead(MQ7_PIN);
  if (adc <= 0)
    adc = 1;
  float vout_esp = (adc / ADC_MAX) * VREF;
  float vout_real = vout_esp * VD_FACTOR;
  if (vout_real <= 0)
    vout_real = 0.01;
  float rs_mq7 = ((VCC_MQ - vout_real) / vout_real) * MQ7_RLOAD;
  return mq7_a * pow(rs_mq7 / mq7_ro, mq7_b);
}

float bacaVOC() {
  int adc = analogRead(MQ135_PIN);
  if (adc <= 0)
    adc = 1;
  float vout_esp = (adc / ADC_MAX) * VREF;
  float vout_real = vout_esp * VD_FACTOR;
  if (vout_real <= 0)
    vout_real = 0.01;
  float rs_mq135 = ((VCC_MQ - vout_real) / vout_real) * MQ135_RLOAD;
  return mq135_a * pow(rs_mq135 / mq135_r0, mq135_b);
}

// ================= FUNGSI TAMPIL LCD DEFAULT =================
void tampilLCD() {
  if (isLcdOverridden)
    return;
  lcd.clear();
  switch (lcdPage) {
  case 0:
    lcd.setCursor(0, 0);
    lcd.print("PM2.5:");
    lcd.print(pm25, 1);
    lcd.print("ug/m3");
    lcd.setCursor(0, 1);
    lcd.print("Status:");
    if (pm25 <= 35.4)
      lcd.print("Baik     ");
    else if (pm25 <= 125.4)
      lcd.print("Perhatian");
    else
      lcd.print("Bahaya   ");
    break;
  case 1:
    lcd.setCursor(0, 0);
    lcd.print("PM10 :");
    lcd.print(pm10, 1);
    lcd.print("ug/m3");
    lcd.setCursor(0, 1);
    lcd.print("Status:");
    if (pm10 <= 154.0)
      lcd.print("Baik     ");
    else if (pm10 <= 354.0)
      lcd.print("Perhatian");
    else
      lcd.print("Bahaya   ");
    break;
  case 2:
    lcd.setCursor(0, 0);
    lcd.print("CO  :");
    lcd.print(co_final, 1);
    lcd.print(" ppm");
    lcd.setCursor(0, 1);
    lcd.print("VOC :");
    lcd.print(voc_final, 3);
    lcd.print(" mg/m3");
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
    lcd.print("WiFi: Offline");
    lcd.setCursor(0, 1);
    lcd.print("Firebase: Off");
    break;
  }
}

// ================= PROSES SERIAL COMMANDS =================
void parseLcdCommand(String line) {
  line.trim();
  if (line.startsWith("$LCD,0,")) {
    String text = line.substring(7);
    lcd.setCursor(0, 0);
    lcd.print("                "); // clear line
    lcd.setCursor(0, 0);
    lcd.print(text.substring(0, 16));
    lastLcdOverride = millis();
    isLcdOverridden = true;
  } else if (line.startsWith("$LCD,1,")) {
    String text = line.substring(7);
    lcd.setCursor(0, 1);
    lcd.print("                "); // clear line
    lcd.setCursor(0, 1);
    lcd.print(text.substring(0, 16));
    lastLcdOverride = millis();
    isLcdOverridden = true;
  }
}

void bacaSerialInputs() {
  // Baca dari USB Serial
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      parseLcdCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // Baca dari hardware Serial2 (Orange Pi UART7)
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      parseLcdCommand(inputBuffer2);
      inputBuffer2 = "";
    } else {
      inputBuffer2 += c;
    }
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

  lcd.setCursor(0, 0);
  lcd.print("AirGuard v1.0");
  lcd.setCursor(0, 1);
  lcd.print("Slave Sensor AP");
  delay(1500);

  pinMode(MQ7_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);
  dht.begin();
  sds.begin(16, 17);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sensor siap");
  lcd.setCursor(0, 1);
  lcd.print("Cek baterai...");
  delay(500);

  for (int i = 0; i < 50; i++) {
    avgBattery.push(bacaTegangan());
    delay(10);
  }
  batteryVoltage = avgBattery.average();
  batteryPercent = voltageToPercent(batteryVoltage);
  batteryPercentLast = batteryPercent;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistem siap!");
  delay(1000);
}

// ================= LOOP =================
void loop() {
  // Cek input dari master (Orange Pi)
  bacaSerialInputs();

  // Cek override timeout
  if (isLcdOverridden && (millis() - lastLcdOverride > LCD_OVERRIDE_TIMEOUT)) {
    isLcdOverridden = false;
    lcdPage = 0;
    tampilLCD();
  }

  // Baca dan kirim sensor secara periodik
  if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();

    // Read SDS011
    if (sds.read(&pm25_raw, &pm10_raw) == 0) {
      pm25 = koreksiPM25(pm25_raw);
      pm10 = koreksiPM10(pm10_raw);
    }

    // Read DHT22
    float suhu_raw = dht.readTemperature();
    if (!isnan(suhu_raw))
      suhu_final = koreksiSuhu(suhu_raw);

    // Read MQ7 & MQ135
    avgCO.push(bacaCO());
    co_final = avgCO.average();

    avgVOC.push(bacaVOC());
    voc_final = avgVOC.average();

    // Read Battery
    if (millis() - lastBatRead >= BAT_READ_INTERVAL) {
      avgBattery.push(bacaTegangan());
      batteryVoltage = avgBattery.average();
      int newPct = voltageToPercent(batteryVoltage);
      if (abs(newPct - batteryPercentLast) >= BAT_HYSTERESIS) {
        batteryPercent = newPct;
        batteryPercentLast = newPct;
      }
      lastBatRead = millis();
    }

    // Send sensor data to Orange Pi
    // Format: $DATA,pm25,pm10,co,voc,suhu,voltage,percent
    String dataMsg = "$DATA," + String(pm25, 1) + "," + String(pm10, 1) + "," +
                     String(co_final, 1) + "," + String(voc_final, 4) + "," +
                     String(suhu_final, 1) + "," + String(batteryVoltage, 2) + "," +
                     String(batteryPercent);
    
    Serial.println(dataMsg);
    Serial2.println(dataMsg);
  }

  // Auto switch LCD page if not overridden
  if (!isLcdOverridden && (millis() - lastLcdSwitch >= LCD_SWITCH_INTERVAL)) {
    lcdPage = (lcdPage + 1) % 5;
    tampilLCD();
    lastLcdSwitch = millis();
  }
}