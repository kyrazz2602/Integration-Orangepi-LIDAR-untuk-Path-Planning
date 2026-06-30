// ============================================
// AIRGUARD - ARDUINO MEGA 2560 FINAL
// Serial2 pin 16/17 → Orange Pi
// Serial3 pin 14/15 → ESP32
// IR Obstacle : pin 28, 29, 30 (deteksi lantai)
// HC-SR04     : pin 31/32, 33/34, 35/36
// WASD via Serial Monitor:
//   W = maju, S = mundur, A = kiri, D = kanan
//   X / spasi = diam
// ============================================

// === PIN ENCODER ===
#define ENC_KANAN_A 3
#define ENC_KANAN_B 2
#define ENC_KIRI_A 18
#define ENC_KIRI_B 19

// === PIN BTS7960 KIRI ===
#define RPWM_KIRI 8
#define LPWM_KIRI 9
#define R_EN_KIRI 24
#define L_EN_KIRI 25

// === PIN BTS7960 KANAN ===
#define RPWM_KANAN 4
#define LPWM_KANAN 5
#define R_EN_KANAN 22
#define L_EN_KANAN 23

// === PIN KIPAS ===
#define FAN_PWM_PIN 6
#define FAN_IN1_PIN 26
#define FAN_IN2_PIN 27

// === PIN IR SENSOR ===
#define IR_KIRI 28
#define IR_TENGAH 29
#define IR_KANAN 30

// === PIN HC-SR04 ===
#define TRIG_DEPAN 31
#define ECHO_DEPAN 32
#define TRIG_KIRI 33
#define ECHO_KIRI 34
#define TRIG_KANAN 35
#define ECHO_KANAN 36

// === THRESHOLD HC-SR04 ===
#define JARAK_STOP 20.0

// === KECEPATAN KIPAS ===
#define FAN_OFF 0
#define FAN_LOW 150
#define FAN_NORMAL 200
#define FAN_HIGH 255

// === MODE OPERASI ===
#define MODE_MANUAL 0
#define MODE_OTONOM 1
int modeOperasi = MODE_MANUAL;

// === PID PARAMETER ===
const float Kp = 0.15;
const float Ki = 0;
const float Kd = 0.1;
const float pwmBase = 120.0;

// === TARGET RPM ===
float targetRPM = 50.0;
float targetRPMKanan = 0.0;
float targetRPMKiri = 0.0;

// === PWM BASE PER RODA ===
float pwmBaseKanan = 80.0;
float pwmBaseKiri = 80.0;

// === ENCODER ===
volatile long pulseKanan = 0;
volatile long pulseKiri = 0;
const float PPR = 241.0 * 4.0;

// === ODOMETRI ===
float odomX = 0.0;
float odomY = 0.0;
float odomTheta = 0.0;
float jarakKanan = 0.0;
float jarakKiri = 0.0;
const float WHEEL_DIAMETER = 6.5;
const float WHEEL_BASE = 7.0;

// === PID VARIABLE - KANAN ===
float errorKanan = 0;
float lastErrorKanan = 0;
float integralKanan = 0;
float outputKanan = 0;
float rpmKanan = 0;

// === PID VARIABLE - KIRI ===
float errorKiri = 0;
float lastErrorKiri = 0;
float integralKiri = 0;
float outputKiri = 0;
float rpmKiri = 0;

// === KIPAS ===
int fanSpeed = 0;
bool modeAuto = false;
float last_pm25 = 0, last_pm10 = 0, last_co = 0, last_voc = 0;
unsigned long lastAutoUpdate = 0;
const unsigned long intervalAuto = 1000;

// === TIMING ===
unsigned long lastPID = 0;
unsigned long lastPrint = 0;
unsigned long lastOdom = 0;
unsigned long lastSensor = 0;
const int intervalPID = 100;
const int intervalPrint = 200;
const int intervalOdom = 100;
const int intervalSensor = 100;

// === STATUS ===
bool motorJalan = false;
int modeGerak = 0;
int sumberPerintah = 0;

// === DATA SENSOR HALANGAN ===
float jarakDepan = 999.0;
float jarakKiriS = 999.0;
float jarakKananS = 999.0;
bool irKiri = false;
bool irTengah = false;
bool irKananS = false;

// ============================================
// CEK BLOK ARAH
// ============================================
bool blockedDepan() { return (jarakDepan <= JARAK_STOP) || !irTengah; }
bool blockedKiri() { return (jarakKiriS <= JARAK_STOP) || !irKiri; }
bool blockedKanan() { return (jarakKananS <= JARAK_STOP) || !irKananS; }

// ============================================
// INTERRUPT ENCODER
// ============================================
void encoderKananA() {
  if (digitalRead(ENC_KANAN_A) == digitalRead(ENC_KANAN_B))
    pulseKanan--;
  else
    pulseKanan++;
}
void encoderKananB() {
  if (digitalRead(ENC_KANAN_A) == digitalRead(ENC_KANAN_B))
    pulseKanan++;
  else
    pulseKanan--;
}
void encoderKiriA() {
  if (digitalRead(ENC_KIRI_A) == digitalRead(ENC_KIRI_B))
    pulseKiri--;
  else
    pulseKiri++;
}
void encoderKiriB() {
  if (digitalRead(ENC_KIRI_A) == digitalRead(ENC_KIRI_B))
    pulseKiri++;
  else
    pulseKiri--;
}

// ============================================
// RESET PID
// ============================================
void resetPID() {
  integralKanan = integralKiri = 0;
  lastErrorKanan = lastErrorKiri = 0;
}

// ============================================
// STOP MOTOR
// ============================================
void stopMotor() {
  motorJalan = false;
  analogWrite(RPWM_KANAN, 0);
  analogWrite(LPWM_KANAN, 0);
  analogWrite(RPWM_KIRI, 0);
  analogWrite(LPWM_KIRI, 0);
  outputKanan = outputKiri = 0;
  resetPID();
}

// ============================================
// HITUNG LEVEL POLUTAN
// ============================================
int hitungLevel(float pm25, float pm10, float co, float voc) {
  int level = 1;
  if (pm25 >= 125.5)
    level = max(level, 3);
  else if (pm25 > 35.4)
    level = max(level, 2);
  if (pm10 >= 355)
    level = max(level, 3);
  else if (pm10 > 154)
    level = max(level, 2);
  if (co >= 50)
    level = max(level, 3);
  else if (co > 15)
    level = max(level, 2);
  if (voc >= 100)
    level = max(level, 3);
  else if (voc > 20)
    level = max(level, 2);
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
}

// ============================================
// UPDATE AUTO KIPAS
// ============================================
void updateAutoKipas() {
  if (!modeAuto)
    return;
  if (millis() - lastAutoUpdate < intervalAuto)
    return;
  lastAutoUpdate = millis();
  int level = hitungLevel(last_pm25, last_pm10, last_co, last_voc);
  if (level == 3)
    setKipas(FAN_HIGH);
  else if (level == 2)
    setKipas(FAN_NORMAL);
  else
    setKipas(FAN_LOW);
}

// ============================================
// EKSEKUSI PERINTAH KIPAS
// ============================================
bool eksekusiFan(String cmd) {
  if (cmd.startsWith("MANUAL:")) {
    String sub = cmd.substring(7);
    if (sub == "HIGH") {
      modeAuto = false;
      setKipas(FAN_HIGH);
    } else if (sub == "NORMAL") {
      modeAuto = false;
      setKipas(FAN_NORMAL);
    } else if (sub == "LOW") {
      modeAuto = false;
      setKipas(FAN_LOW);
    } else if (sub == "OFF") {
      modeAuto = false;
      setKipas(FAN_OFF);
    } else
      return false;
    return true;
  }
  if (cmd.startsWith("FAN:")) {
    String sub = cmd.substring(4);
    if (sub == "HIGH") {
      modeAuto = false;
      setKipas(FAN_HIGH);
    } else if (sub == "NORMAL") {
      modeAuto = false;
      setKipas(FAN_NORMAL);
    } else if (sub == "LOW") {
      modeAuto = false;
      setKipas(FAN_LOW);
    } else if (sub == "OFF") {
      modeAuto = false;
      setKipas(FAN_OFF);
    } else
      return false;
    return true;
  }
  if (cmd == "OFF") {
    modeAuto = false;
    setKipas(FAN_OFF);
    return true;
  }
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
    if (vals[0] >= 0)
      last_pm25 = vals[0];
    if (vals[1] >= 0)
      last_pm10 = vals[1];
    if (vals[2] >= 0)
      last_co = vals[2];
    if (vals[3] >= 0)
      last_voc = vals[3];
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

  if (eksekusiFan(cmd))
    return;

  if (cmd == "MODE,OTONOM") {
    modeOperasi = MODE_OTONOM;
    Serial2.println("ACK:MODE_OTONOM");
    return;
  }
  if (cmd == "MODE,MANUAL") {
    modeOperasi = MODE_MANUAL;
    Serial2.println("ACK:MODE_MANUAL");
    return;
  }

  if (cmd == "CMD,MAJU") {
    if (blockedDepan()) {
      Serial2.println("EVT:BLOCKED_DEPAN");
      return;
    }
    motorJalan = true;
    modeGerak = 0;
    targetRPMKanan = targetRPM;
    targetRPMKiri = targetRPM;
    pwmBaseKanan = pwmBase;
    pwmBaseKiri = pwmBase;
    resetPID();
    Serial2.println("ACK:MAJU");
  } else if (cmd == "CMD,MUNDUR") {
    motorJalan = true;
    modeGerak = 1;
    targetRPMKanan = targetRPM;
    targetRPMKiri = targetRPM;
    pwmBaseKanan = pwmBase;
    pwmBaseKiri = pwmBase;
    resetPID();
    Serial2.println("ACK:MUNDUR");
  } else if (cmd == "CMD,KANAN") {
    if (blockedKanan()) {
      Serial2.println("EVT:BLOCKED_KANAN");
      return;
    }
    if (motorJalan && modeGerak == 2) {
      Serial2.println("ACK:KANAN");
      return;
    }
    motorJalan = true;
    modeGerak = 2;
    targetRPMKanan = targetRPM * 0.5;
    targetRPMKiri = targetRPM;
    pwmBaseKanan = pwmBase * 0.5;
    pwmBaseKiri = pwmBase;
    resetPID();
    Serial2.println("ACK:KANAN");
  } else if (cmd == "CMD,KIRI") {
    if (blockedKiri()) {
      Serial2.println("EVT:BLOCKED_KIRI");
      return;
    }
    if (motorJalan && modeGerak == 3) {
      Serial2.println("ACK:KIRI");
      return;
    }
    motorJalan = true;
    modeGerak = 3;
    targetRPMKanan = targetRPM;
    targetRPMKiri = targetRPM * 0.5;
    pwmBaseKanan = pwmBase;
    pwmBaseKiri = pwmBase * 0.5;
    resetPID();
    Serial2.println("ACK:KIRI");
  } else if (cmd == "CMD,DIAM") {
    stopMotor();
    Serial2.println("ACK:DIAM");
  } else if (cmd.startsWith("SET,RPM,")) {
    float val = cmd.substring(8).toFloat();
    if (val > 0 && val <= 200) {
      targetRPM = val;
      Serial2.print("ACK:RPM:");
      Serial2.println(targetRPM, 0);
    }
  } else if (cmd == "RESET,ODOM") {
    odomX = odomY = odomTheta = jarakKanan = jarakKiri = 0;
    Serial2.println("ACK:RESET_ODOM");
  }
}

// ============================================
// BACA SERIAL MONITOR (+ WASD)
// ============================================
void bacaSerialMonitor() {
  if (!Serial.available())
    return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0)
    return;
  sumberPerintah = 0;

  // === HANDLER WASD (single char) ===
  if (input.length() == 1) {
    char c = tolower(input.charAt(0));
    String mapped = "";
    if (c == 'w')
      mapped = "CMD,MAJU";
    else if (c == 's')
      mapped = "CMD,MUNDUR";
    else if (c == 'a')
      mapped = "CMD,KIRI";
    else if (c == 'd')
      mapped = "CMD,KANAN";
    else if (c == 'x' || c == ' ')
      mapped = "CMD,DIAM";

    if (mapped.length() > 0) {
      Serial.println("[WASD] " + input + " -> " + mapped);
      eksekusiPerintah(mapped);
      return;
    }
  }

  // === COMMAND BIASA ===
  Serial.println("[TEST] " + input);
  eksekusiPerintah(input);
}

// ============================================
// BACA ORANGE PI
// ============================================
void bacaOrangePi() {
  if (!Serial2.available())
    return;
  String input = Serial2.readStringUntil('\n');
  input.trim();
  if (input.length() == 0)
    return;
  sumberPerintah = 1;
  Serial.println("[OPI] " + input);
  eksekusiPerintah(input);
}

// ============================================
// BACA ESP32 + FORWARD WIFI KE ORANGE PI
// ============================================
void bacaESP32() {
  // ESP32 communication is handled directly by Orange Pi now.
}

// ============================================
// BACA HC-SR04 (dengan koreksi kalibrasi)
// ============================================
float bacaJarak(int trigPin, int echoPin, float koefRegresi) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 30000);
  if (dur == 0)
    return 999.0;
  float raw = dur * 0.0343 / 2.0;
  return raw / koefRegresi;
}

// ============================================
// BACA SEMUA SENSOR HALANGAN
// ============================================
void bacaSensorHalangan() {
  jarakDepan = bacaJarak(TRIG_DEPAN, ECHO_DEPAN, 0.9915);
  jarakKiriS = bacaJarak(TRIG_KIRI, ECHO_KIRI, 1.0377);
  jarakKananS = bacaJarak(TRIG_KANAN, ECHO_KANAN, 0.9736);

  irKiri = !digitalRead(IR_KIRI);
  irTengah = !digitalRead(IR_TENGAH);
  irKananS = !digitalRead(IR_KANAN);

  Serial2.print("SENSOR,");
  Serial2.print(jarakDepan, 1);
  Serial2.print(",");
  Serial2.print(jarakKiriS, 1);
  Serial2.print(",");
  Serial2.print(jarakKananS, 1);
  Serial2.print(",");
  Serial2.print(irKiri ? 1 : 0);
  Serial2.print(",");
  Serial2.print(irTengah ? 1 : 0);
  Serial2.print(",");
  Serial2.println(irKananS ? 1 : 0);

  if (!motorJalan)
    return;

  bool adaHalangan = false;
  if (modeGerak == 0 && blockedDepan())
    adaHalangan = true;
  if (modeGerak == 2 && blockedKanan())
    adaHalangan = true;
  if (modeGerak == 3 && blockedKiri())
    adaHalangan = true;

  if (!adaHalangan)
    return;

  stopMotor();
  if (modeOperasi == MODE_MANUAL)
    Serial2.println("EVT:OBSTACLE_STOP");
  else if (modeOperasi == MODE_OTONOM)
    Serial2.println("EVT:OBSTACLE_OTONOM");
}

// ============================================
// PID
// ============================================
float hitungPID(float target, float rpm, float &integral, float &lastError,
                float &errOut, float base) {
  errOut = target - rpm;
  integral += errOut;
  integral = constrain(integral, -100, 100);
  float deriv = errOut - lastError;
  lastError = errOut;
  float out = base + (Kp * errOut) + (Ki * integral) + (Kd * deriv);
  return constrain(out, 0, 255);
}

// ============================================
// SET MOTOR
// ============================================
void setMotorKanan(int pwm, bool maju) {
  pwm = constrain(pwm, 0, 255);
  if (maju) {
    analogWrite(RPWM_KANAN, pwm);
    analogWrite(LPWM_KANAN, 0);
  } else {
    analogWrite(RPWM_KANAN, 0);
    analogWrite(LPWM_KANAN, pwm);
  }
}

void setMotorKiri(int pwm, bool maju) {
  pwm = constrain(pwm, 0, 255);
  if (maju) {
    analogWrite(RPWM_KIRI, pwm);
    analogWrite(LPWM_KIRI, 0);
  } else {
    analogWrite(RPWM_KIRI, 0);
    analogWrite(LPWM_KIRI, pwm);
  }
}

// ============================================
// GERAKKAN MOTOR
// ============================================
void gerakkanMotor() {
  switch (modeGerak) {
  case 0:
    outputKanan = hitungPID(targetRPMKanan, rpmKanan, integralKanan,
                            lastErrorKanan, errorKanan, pwmBaseKanan);
    outputKiri = hitungPID(targetRPMKiri, rpmKiri, integralKiri, lastErrorKiri,
                           errorKiri, pwmBaseKiri);
    setMotorKanan((int)outputKanan, true);
    setMotorKiri((int)outputKiri, true);
    break;
  case 1:
    outputKanan = hitungPID(targetRPMKanan, rpmKanan, integralKanan,
                            lastErrorKanan, errorKanan, pwmBaseKanan);
    outputKiri = hitungPID(targetRPMKiri, rpmKiri, integralKiri, lastErrorKiri,
                           errorKiri, pwmBaseKiri);
    setMotorKanan((int)outputKanan, false);
    setMotorKiri((int)outputKiri, false);
    break;
  case 2:
    outputKiri = hitungPID(targetRPMKiri, rpmKiri, integralKiri, lastErrorKiri,
                           errorKiri, pwmBaseKiri);
    setMotorKanan(0, true);
    setMotorKiri((int)outputKiri, true);
    break;
  case 3:
    outputKanan = hitungPID(targetRPMKanan, rpmKanan, integralKanan,
                            lastErrorKanan, errorKanan, pwmBaseKanan);
    setMotorKanan((int)outputKanan, true);
    setMotorKiri(0, true);
    break;
  }
}

// ============================================
// UPDATE ODOMETRI
// ============================================
void updateOdometri(long pK, long pL) {
  float dK = (abs(pK) / PPR) * (PI * WHEEL_DIAMETER);
  float dL = (abs(pL) / PPR) * (PI * WHEEL_DIAMETER);

  if (modeGerak == 1) {
    dK = -dK;
    dL = -dL;
  }
  if (modeGerak == 2) {
    dK = 0;
  }
  if (modeGerak == 3) {
    dL = 0;
  }

  jarakKanan += dK;
  jarakKiri += dL;

  float dCenter = (dK + dL) / 2.0;
  float dTheta = (dK - dL) / WHEEL_BASE;

  odomTheta += dTheta;
  odomX += dCenter * cos(odomTheta);
  odomY += dCenter * sin(odomTheta);
}

// ============================================
// KIRIM ODOMETRI KE ORANGE PI
// ============================================
void kirimOdometri() {
  Serial2.print("ODOM,");
  Serial2.print(odomX, 2);
  Serial2.print(",");
  Serial2.print(odomY, 2);
  Serial2.print(",");
  Serial2.print(odomTheta, 4);
  Serial2.print(",");
  Serial2.print(rpmKanan, 1);
  Serial2.print(",");
  Serial2.println(rpmKiri, 1);
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  pinMode(ENC_KANAN_A, INPUT_PULLUP);
  pinMode(ENC_KANAN_B, INPUT_PULLUP);
  pinMode(ENC_KIRI_A, INPUT_PULLUP);
  pinMode(ENC_KIRI_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_KANAN_A), encoderKananA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_KANAN_B), encoderKananB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_KIRI_A), encoderKiriA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_KIRI_B), encoderKiriB, CHANGE);

  pinMode(RPWM_KIRI, OUTPUT);
  pinMode(LPWM_KIRI, OUTPUT);
  pinMode(R_EN_KIRI, OUTPUT);
  pinMode(L_EN_KIRI, OUTPUT);
  digitalWrite(R_EN_KIRI, HIGH);
  digitalWrite(L_EN_KIRI, HIGH);
  analogWrite(RPWM_KIRI, 0);
  analogWrite(LPWM_KIRI, 0);

  pinMode(RPWM_KANAN, OUTPUT);
  pinMode(LPWM_KANAN, OUTPUT);
  pinMode(R_EN_KANAN, OUTPUT);
  pinMode(L_EN_KANAN, OUTPUT);
  digitalWrite(R_EN_KANAN, HIGH);
  digitalWrite(L_EN_KANAN, HIGH);
  analogWrite(RPWM_KANAN, 0);
  analogWrite(LPWM_KANAN, 0);

  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(FAN_IN1_PIN, OUTPUT);
  pinMode(FAN_IN2_PIN, OUTPUT);
  digitalWrite(FAN_IN1_PIN, HIGH);
  digitalWrite(FAN_IN2_PIN, LOW);
  analogWrite(FAN_PWM_PIN, 0);

  pinMode(IR_KIRI, INPUT);
  pinMode(IR_TENGAH, INPUT);
  pinMode(IR_KANAN, INPUT);

  pinMode(TRIG_DEPAN, OUTPUT);
  pinMode(ECHO_DEPAN, INPUT);
  pinMode(TRIG_KIRI, OUTPUT);
  pinMode(ECHO_KIRI, INPUT);
  pinMode(TRIG_KANAN, OUTPUT);
  pinMode(ECHO_KANAN, INPUT);

  Serial.println("=== AIRGUARD SIAP ===");
  Serial.println("WASD: W=maju S=mundur A=kiri D=kanan X=diam");
}

// ============================================
// LOOP
// ============================================
void loop() {
  unsigned long now = millis();

  bacaSerialMonitor();
  bacaOrangePi();
  updateAutoKipas();

  if (now - lastSensor >= intervalSensor) {
    lastSensor = now;
    bacaSensorHalangan();
  }

  if (now - lastPrint >= intervalPrint) {
    lastPrint = now;

    Serial.print("HCSR Depan:");
    Serial.print(jarakDepan, 1);
    Serial.print("cm");
    Serial.print(blockedDepan() ? "(BLOCK)" : "(OK)");
    Serial.print(" | Kiri:");
    Serial.print(jarakKiriS, 1);
    Serial.print("cm");
    Serial.print(blockedKiri() ? "(BLOCK)" : "(OK)");
    Serial.print(" | Kanan:");
    Serial.print(jarakKananS, 1);
    Serial.print("cm");
    Serial.println(blockedKanan() ? "(BLOCK)" : "(OK)");

    Serial.print("IR Kiri:");
    Serial.print(irKiri ? "LANTAI" : "JURANG");
    Serial.print(" | Tengah:");
    Serial.print(irTengah ? "LANTAI" : "JURANG");
    Serial.print(" | Kanan:");
    Serial.println(irKananS ? "LANTAI" : "JURANG");
    Serial.println("----------------------------");

    Serial.print("RPM Kanan:");
    Serial.print(rpmKanan, 1);
    Serial.print(" | RPM Kiri:");
    Serial.println(rpmKiri, 1);
  }

  if (now - lastPID >= intervalPID) {
    float dt = (now - lastPID) / 1000.0;
    lastPID = now;

    long pK, pL;
    noInterrupts();
    pK = pulseKanan;
    pulseKanan = 0;
    pL = pulseKiri;
    pulseKiri = 0;
    interrupts();

    rpmKanan = (abs(pK) / PPR) / dt * 60.0;
    rpmKiri = (abs(pL) / PPR) / dt * 60.0;

    if (motorJalan) {
      updateOdometri(pK, pL);
      gerakkanMotor();
    }
  }

  if (now - lastOdom >= intervalOdom) {
    lastOdom = now;
    kirimOdometri();
  }
}