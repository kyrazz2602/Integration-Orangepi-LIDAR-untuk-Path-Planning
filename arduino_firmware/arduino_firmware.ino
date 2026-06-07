// ============================================
// KODE PID - 2 RODA + KENDALI KIPAS (FINAL + ROS2)
// Arduino Mega + 2x BTS7960 + 2x Encoder H12
// Serial2 pin 16/17 → Orange Pi (path planning)
// Serial3 pin 14/15 → ESP32 (kendali manual)
// ============================================

// === PIN ENCODER ===
#define ENC_KANAN_A   3
#define ENC_KANAN_B   2
#define ENC_KIRI_A   18
#define ENC_KIRI_B   19

// === PIN BTS7960 KIRI ===
#define RPWM_KIRI    8
#define LPWM_KIRI    9
#define R_EN_KIRI   24
#define L_EN_KIRI   25

// === PIN BTS7960 KANAN ===
#define RPWM_KANAN   5
#define LPWM_KANAN   4
#define R_EN_KANAN  22
#define L_EN_KANAN  23

// === PIN KIPAS ===
#define FAN_PWM_PIN  6
#define FAN_IN1_PIN 26
#define FAN_IN2_PIN 27

// === KECEPATAN KIPAS ===
#define FAN_OFF    0
#define FAN_LOW    150
#define FAN_NORMAL 200
#define FAN_HIGH   255

// === PID PARAMETER ===
const float Kp      = 0.15;
const float Ki      = 0.8;
const float Kd      = 0.01;
const float pwmBase = 80.0;

// === TARGET ===
float targetRPM      = 50.0;
float targetRPMKanan = 0.0;
float targetRPMKiri  = 0.0;

// === ARAH RODA INDEPENDEN (UNTUK ROS 2 CMD_VEL) ===
bool dirKiriMaju = true;
bool dirKananMaju = true;

// === PWM BASE PER RODA ===
float pwmBaseKanan = 80.0;
float pwmBaseKiri  = 80.0;

// === ENCODER ===
volatile long pulseKanan = 0;
volatile long pulseKiri  = 0;
const float PPR = 241.0;

// === PID VARIABLE - KANAN ===
float errorKanan     = 0;
float lastErrorKanan = 0;
float integralKanan  = 0;
float outputKanan    = 0;
float rpmKanan       = 0;

// === PID VARIABLE - KIRI ===
float errorKiri     = 0;
float lastErrorKiri = 0;
float integralKiri  = 0;
float outputKiri    = 0;
float rpmKiri       = 0;

// === KIPAS ===
int   fanSpeed  = 0;
bool  modeAuto  = false;
float last_pm25 = 0, last_pm10 = 0, last_co = 0, last_voc = 0;
unsigned long lastAutoUpdate = 0;
const unsigned long intervalAuto = 1000;

// === TIMING ===
unsigned long lastPID   = 0;
unsigned long lastPrint = 0;
unsigned long lastOdom  = 0;
const int intervalPID   = 50;
const int intervalPrint = 100;
const int intervalOdom  = 50;

// === STATUS ===
bool motorJalan     = false;
int  modeGerak      = 0;
int  sumberPerintah = 0;

// ============================================
// INTERRUPT ENCODER
// ============================================
void encoderKananA() {
  if (digitalRead(ENC_KANAN_B) == HIGH) pulseKanan++;
  else pulseKanan--;
}

void encoderKiriA() {
  if (digitalRead(ENC_KIRI_B) == HIGH) pulseKiri++;
  else pulseKiri--;
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);  // Orange Pi
  Serial3.begin(115200);  // ESP32

  // Encoder
  pinMode(ENC_KANAN_A, INPUT_PULLUP);
  pinMode(ENC_KANAN_B, INPUT_PULLUP);
  pinMode(ENC_KIRI_A,  INPUT_PULLUP);
  pinMode(ENC_KIRI_B,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_KANAN_A), encoderKananA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_KIRI_A),  encoderKiriA,  RISING);

  // Motor Kiri
  pinMode(RPWM_KIRI, OUTPUT); pinMode(LPWM_KIRI, OUTPUT);
  pinMode(R_EN_KIRI, OUTPUT); pinMode(L_EN_KIRI, OUTPUT);
  digitalWrite(R_EN_KIRI, HIGH); digitalWrite(L_EN_KIRI, HIGH);
  analogWrite(RPWM_KIRI, 0);    analogWrite(LPWM_KIRI, 0);

  // Motor Kanan
  pinMode(RPWM_KANAN, OUTPUT); pinMode(LPWM_KANAN, OUTPUT);
  pinMode(R_EN_KANAN, OUTPUT); pinMode(L_EN_KANAN, OUTPUT);
  digitalWrite(R_EN_KANAN, HIGH); digitalWrite(L_EN_KANAN, HIGH);
  analogWrite(RPWM_KANAN, 0);     analogWrite(LPWM_KANAN, 0);

  // Kipas
  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(FAN_IN1_PIN, OUTPUT);
  pinMode(FAN_IN2_PIN, OUTPUT);
  digitalWrite(FAN_IN1_PIN, HIGH);
  digitalWrite(FAN_IN2_PIN, LOW);
  analogWrite(FAN_PWM_PIN, 0);

  Serial.println("[MEGA] Siap.");
  Serial2.println("[MEGA] Orange Pi terhubung.");
  
  // PERHATIAN: Pastikan menggunakan Voltage Divider / Logic Level Converter
  // sebelum menghubungkan TX Arduino Mega (5V) ke RX ESP32/OrangePi (3.3V) !!
  Serial3.println("[MEGA] ESP32 terhubung."); 
}

// ============================================
// RESET PID
// ============================================
void resetPID() {
  integralKanan  = integralKiri  = 0;
  lastErrorKanan = lastErrorKiri = 0;
}

// ============================================
// HITUNG LEVEL POLUTAN
// ============================================
int hitungLevel(float pm25, float pm10, float co, float voc) {
  int level = 1;
  if      (pm25 >= 125.5) level = max(level, 3);
  else if (pm25 >   35.4) level = max(level, 2);
  if      (pm10 >= 355)   level = max(level, 3);
  else if (pm10 >  154)   level = max(level, 2);
  if      (co   >= 50)    level = max(level, 3);
  else if (co   >  15)    level = max(level, 2);
  if      (voc  >= 100)   level = max(level, 3);
  else if (voc  >   20)   level = max(level, 2);
  return level;
}

// ============================================
// SET KIPAS
// ============================================
void setKipas(int speed) {
  fanSpeed = speed;
  if (speed == 0) {
    digitalWrite(FAN_IN1_PIN, LOW);
    digitalWrite(FAN_IN2_PIN, LOW);
  } else {
    digitalWrite(FAN_IN1_PIN, HIGH);
    digitalWrite(FAN_IN2_PIN, LOW);
  }
  analogWrite(FAN_PWM_PIN, speed);

  String status = "";
  if      (fanSpeed == 0)    status = "FAN:OFF";
  else if (fanSpeed <= 150)  status = "FAN:LOW";
  else if (fanSpeed <= 200)  status = "FAN:NORMAL";
  else                       status = "FAN:HIGH";
  if (modeAuto) status += ":AUTO";

  Serial3.println(status);
  Serial.println("[FAN] " + status);
}

// ============================================
// UPDATE MODE AUTO KIPAS
// ============================================
void updateAutoKipas() {
  if (!modeAuto) return;
  if (millis() - lastAutoUpdate < intervalAuto) return;
  lastAutoUpdate = millis();

  int level = hitungLevel(last_pm25, last_pm10, last_co, last_voc);
  if      (level == 3) setKipas(FAN_HIGH);
  else if (level == 2) setKipas(FAN_NORMAL);
  else                 setKipas(FAN_LOW);
}

// ============================================
// EKSEKUSI PERINTAH KIPAS
// ============================================
bool eksekusiFan(String cmd) {
  // Format baru: FAN:xxx
  if (cmd.startsWith("FAN:")) {
    String sub = cmd.substring(4);
    if      (sub == "HIGH")   { modeAuto = false; setKipas(FAN_HIGH);   }
    else if (sub == "NORMAL") { modeAuto = false; setKipas(FAN_NORMAL); }
    else if (sub == "LOW")    { modeAuto = false; setKipas(FAN_LOW);    }
    else if (sub == "OFF")    { modeAuto = false; setKipas(FAN_OFF);    }
    else if (sub.startsWith("SPEED:")) {
      modeAuto = false;
      setKipas(constrain(sub.substring(6).toInt(), 0, 255));
    }
    return true;
  }

  // Format dari ESP32: MANUAL:xxx
  if (cmd.startsWith("MANUAL:")) {
    String sub = cmd.substring(7);
    if      (sub == "HIGH")   { modeAuto = false; setKipas(FAN_HIGH);   }
    else if (sub == "NORMAL") { modeAuto = false; setKipas(FAN_NORMAL); }
    else if (sub == "LOW")    { modeAuto = false; setKipas(FAN_LOW);    }
    else if (sub == "OFF")    { modeAuto = false; setKipas(FAN_OFF);    }
    return true;
  }

  // Format OFF langsung
  if (cmd == "OFF") {
    modeAuto = false; setKipas(FAN_OFF); return true;
  }

  // Format AUTO dari ESP32 offline: AUTO:pm25:pm10:co:voc
  if (cmd.startsWith("AUTO:")) {
    modeAuto = true;
    String data = cmd.substring(5);
    float vals[4] = {-1, -1, -1, -1};
    int idx = 0, start = 0;
    for (int i = 0; i <= (int)data.length() && idx < 4; i++) {
      if (i == (int)data.length() || data.charAt(i) == ':') {
        vals[idx++] = data.substring(start, i).toFloat();
        start = i + 1;
      }
    }
    if (vals[0] >= 0) last_pm25 = vals[0];
    if (vals[1] >= 0) last_pm10 = vals[1];
    if (vals[2] >= 0) last_co   = vals[2];
    if (vals[3] >= 0) last_voc  = vals[3];
    Serial.print("[AUTO] PM25:"); Serial.print(last_pm25);
    Serial.print(" PM10:");       Serial.print(last_pm10);
    Serial.print(" CO:");         Serial.print(last_co);
    Serial.print(" VOC:");        Serial.println(last_voc);
    return true;
  }

  return false;
}

// ============================================
// EKSEKUSI PERINTAH MOTOR
// ============================================
void eksekusiPerintah(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (eksekusiFan(cmd)) return;

  if (cmd == "CMD,MAJU") {
    motorJalan     = true;
    modeGerak      = 0;
    targetRPMKanan = targetRPM;
    targetRPMKiri  = targetRPM;
    pwmBaseKanan   = pwmBase;
    pwmBaseKiri    = pwmBase;
    resetPID();
  }
  else if (cmd == "CMD,MUNDUR") {
    motorJalan     = true;
    modeGerak      = 1;
    targetRPMKanan = targetRPM;
    targetRPMKiri  = targetRPM;
    pwmBaseKanan   = pwmBase;
    pwmBaseKiri    = pwmBase;
    resetPID();
  }
  else if (cmd == "CMD,KANAN") {
    motorJalan     = true;
    modeGerak      = 2;
    targetRPMKanan = 0;
    targetRPMKiri  = targetRPM;
    pwmBaseKanan   = 0;
    pwmBaseKiri    = pwmBase;
    resetPID();
  }
  else if (cmd == "CMD,KIRI") {
    motorJalan     = true;
    modeGerak      = 3;
    targetRPMKanan = targetRPM;
    targetRPMKiri  = 0;
    pwmBaseKanan   = pwmBase;
    pwmBaseKiri    = 0;
    resetPID();
  }
  else if (cmd == "CMD,PUTAR_KANAN") {
    motorJalan   = true;
    modeGerak    = 4;
    pwmBaseKanan = pwmBase;
    pwmBaseKiri  = pwmBase;
    resetPID();
  }
  else if (cmd == "CMD,PUTAR_KIRI") {
    motorJalan   = true;
    modeGerak    = 5;
    pwmBaseKanan = pwmBase;
    pwmBaseKiri  = pwmBase;
    resetPID();
  }
  else if (cmd == "CMD,DIAM") {
    motorJalan = false;
    analogWrite(RPWM_KANAN, 0); analogWrite(LPWM_KANAN, 0);
    analogWrite(RPWM_KIRI,  0); analogWrite(LPWM_KIRI,  0);
    outputKanan = outputKiri = 0;
    resetPID();
  }
  else if (cmd.startsWith("SET,RPM,")) {
    float val = cmd.substring(8).toFloat();
    if (val > 0 && val <= 200) targetRPM = val;
  }
  // --- COMMAND DARI ROS 2 (NAV2) ---
  else if (cmd.startsWith("CMD,VEL,")) {
    // Format: CMD,VEL,<rpm_kiri>,<rpm_kanan>
    int commaIndex = cmd.indexOf(',', 8);
    if (commaIndex > 0) {
      float tKiri = cmd.substring(8, commaIndex).toFloat();
      float tKanan = cmd.substring(commaIndex + 1).toFloat();
      
      motorJalan = true;
      modeGerak = 6;
      targetRPMKiri = abs(tKiri);
      targetRPMKanan = abs(tKanan);
      dirKiriMaju = (tKiri >= 0);
      dirKananMaju = (tKanan >= 0);
      pwmBaseKanan = pwmBase;
      pwmBaseKiri  = pwmBase;
      resetPID();
    }
  }
}

// ============================================
// BACA SERIAL
// ============================================
void bacaOrangePi() {
  if (!Serial2.available()) return;
  String input = Serial2.readStringUntil('\n');
  if (input.length() == 0) return;
  sumberPerintah = 1;
  eksekusiPerintah(input);
}

void bacaESP32() {
  if (!Serial3.available()) return;
  String input = Serial3.readStringUntil('\n');
  if (input.length() == 0) return;
  sumberPerintah = 2;
  eksekusiPerintah(input);
}

// ============================================
// HITUNG PID
// ============================================
float hitungPID(float target, float rpm,
                float &integral, float &lastError, float &errOut, float base) {
  if (target <= 0.1) {
    integral = 0;
    return 0; // Berhenti jika target 0
  }
  errOut     = target - rpm;
  integral  += errOut;
  integral   = constrain(integral, -100, 100);
  float deriv = errOut - lastError;
  lastError   = errOut;
  float out   = base + (Kp * errOut) + (Ki * integral) + (Kd * deriv);
  return constrain(out, 0, 255);
}

// ============================================
// SET MOTOR
// ============================================
void setMotorKanan(int pwm, bool maju) {
  pwm = constrain(pwm, 0, 255);
  if (maju) { analogWrite(RPWM_KANAN, pwm); analogWrite(LPWM_KANAN, 0);   }
  else      { analogWrite(RPWM_KANAN, 0);   analogWrite(LPWM_KANAN, pwm); }
}

void setMotorKiri(int pwm, bool maju) {
  pwm = constrain(pwm, 0, 255);
  if (maju) { analogWrite(RPWM_KIRI, pwm); analogWrite(LPWM_KIRI, 0);   }
  else      { analogWrite(RPWM_KIRI, 0);   analogWrite(LPWM_KIRI, pwm); }
}

// ============================================
// GERAKKAN MOTOR
// ============================================
void gerakkanMotor() {
  switch (modeGerak) {
    case 0: case 1: case 2: case 3: {
      bool maju = (modeGerak != 1);
      outputKanan = hitungPID(targetRPMKanan, rpmKanan, integralKanan, lastErrorKanan, errorKanan, pwmBaseKanan);
      outputKiri  = hitungPID(targetRPMKiri,  rpmKiri,  integralKiri,  lastErrorKiri,  errorKiri,  pwmBaseKiri);
      setMotorKanan((int)outputKanan, maju);
      setMotorKiri ((int)outputKiri,  maju);
      break;
    }
    case 4:
      outputKanan = hitungPID(targetRPM, rpmKanan, integralKanan, lastErrorKanan, errorKanan, pwmBaseKanan);
      outputKiri  = hitungPID(targetRPM, rpmKiri,  integralKiri,  lastErrorKiri,  errorKiri,  pwmBaseKiri);
      setMotorKanan((int)outputKanan, false);
      setMotorKiri ((int)outputKiri,  true);
      break;
    case 5:
      outputKanan = hitungPID(targetRPM, rpmKanan, integralKanan, lastErrorKanan, errorKanan, pwmBaseKanan);
      outputKiri  = hitungPID(targetRPM, rpmKiri,  integralKiri,  lastErrorKiri,  errorKiri,  pwmBaseKiri);
      setMotorKanan((int)outputKanan, true);
      setMotorKiri ((int)outputKiri,  false);
      break;
    case 6: // ROS CMD_VEL Mode
      outputKanan = hitungPID(targetRPMKanan, rpmKanan, integralKanan, lastErrorKanan, errorKanan, pwmBaseKanan);
      outputKiri  = hitungPID(targetRPMKiri,  rpmKiri,  integralKiri,  lastErrorKiri,  errorKiri,  pwmBaseKiri);
      setMotorKanan((int)outputKanan, dirKananMaju);
      setMotorKiri ((int)outputKiri,  dirKiriMaju);
      break;
  }
}

// ============================================
// KIRIM ODOMETRI
// ============================================
void kirimOdometri() {
  // Mengirim data ke Orange Pi via Serial2
  // Format: ODOM,rpmKiri,rpmKanan
  Serial2.print("ODOM,");
  
  // Berikan tanda negatif jika motor bergerak mundur agar Orange Pi tau arahnya
  float odomKiri = (modeGerak == 1 || modeGerak == 5 || (modeGerak == 6 && !dirKiriMaju)) ? -rpmKiri : rpmKiri;
  float odomKanan = (modeGerak == 1 || modeGerak == 4 || (modeGerak == 6 && !dirKananMaju)) ? -rpmKanan : rpmKanan;
  
  Serial2.print(odomKiri, 1);
  Serial2.print(",");
  Serial2.println(odomKanan, 1);
}

// ============================================
// LOOP
// ============================================
void loop() {
  unsigned long now = millis();

  bacaOrangePi();
  bacaESP32();
  updateAutoKipas();

  // === PID ===
  if (now - lastPID >= intervalPID) {
    float dt = (now - lastPID) / 1000.0;
    lastPID  = now;

    long pK, pL;
    noInterrupts();
      pK = pulseKanan; pulseKanan = 0;
      pL = pulseKiri;  pulseKiri  = 0;
    interrupts();

    rpmKanan = (abs(pK) / PPR) / dt * 60.0;
    rpmKiri  = (abs(pL) / PPR) / dt * 60.0;

    if (motorJalan) gerakkanMotor();
  }

  // === ODOMETRI ===
  if (now - lastOdom >= intervalOdom) {
    lastOdom = now;
    kirimOdometri();
  }

  // === DEBUG ===
  if (now - lastPrint >= intervalPrint) {
    lastPrint = now;

    String mode;
    switch (modeGerak) {
      case 0: mode = "MAJU";   break;
      case 1: mode = "MUNDUR"; break;
      case 2: mode = "BLK_KN"; break;
      case 3: mode = "BLK_KI"; break;
      case 4: mode = "PUT_KN"; break;
      case 5: mode = "PUT_KI"; break;
      case 6: mode = "ROS_VEL"; break;
    }
    String src = sumberPerintah == 1 ? "OPI" :
                 sumberPerintah == 2 ? "ESP" : "---";

    String fanStr = fanSpeed == 0    ? "OFF"    :
                    fanSpeed <= 150  ? "LOW"    :
                    fanSpeed <= 200  ? "NORMAL" : "HIGH";
    if (modeAuto) fanStr += "(A)";

    Serial.print("Mode:"); Serial.print(motorJalan ? mode : (String)"DIAM");
    Serial.print(" Src:");    Serial.print(src);
    Serial.print(" RPM:");    Serial.print(targetRPM, 0);
    Serial.print(" RPM_K:");  Serial.print(rpmKanan, 1);
    Serial.print(" PWM_K:");  Serial.print(outputKanan, 1);
    Serial.print(" | RPM_L:"); Serial.print(rpmKiri, 1);
    Serial.print(" PWM_L:");  Serial.print(outputKiri, 1);
    Serial.print(" | FAN:");  Serial.println(fanStr);
  }
}
