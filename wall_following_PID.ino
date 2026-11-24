/*
 * ============================================================
 *  WALL FOLLOWING DDMR - Arduino Mega + TB6612FNG + 3x HC-SR04
 * ============================================================
 *  Robot menyusuri dinding (default: dinding KANAN) dengan kontrol PID.
 *
 *  Koneksi:
 *  Driver TB6612FNG
 *    Motor Kiri  : IN1 = 8, IN2 = 7,  PWM = 6
 *    Motor Kanan : IN1 = 9, IN2 = 10, PWM = 11
 *    STBY        : hubungkan ke 5V (atau isi PIN_STBY di bawah)
 *
 *  Encoder JGB37
 *    Motor Kiri  : A = 2, B = 4
 *    Motor Kanan : A = 3, B = 5
 *
 *  HC-SR04
 *    Kanan  : echo = 34, trig = 36
 *    Tengah : echo = 38, trig = 40
 *    Kiri   : echo = 42, trig = 44
 *
 *  Serial Monitor: 115200 baud
 * ============================================================
 */

// ================== PILIH SISI DINDING ==================
#define KANAN  1
#define KIRI  -1
const int SISI_DINDING = KANAN;   // ganti ke KIRI untuk mengikuti dinding kiri

// ================== PIN ==================
// Motor kiri
const int L_IN1 = 8, L_IN2 = 7, L_PWM = 6;
// Motor kanan
const int R_IN1 = 9, R_IN2 = 10, R_PWM = 11;
// STBY TB6612: isi nomor pin jika STBY disambung ke pin Arduino,
// biarkan -1 jika STBY langsung disambung ke 5V
const int PIN_STBY = -1;

// Encoder
const int ENC_L_A = 2, ENC_L_B = 4;
const int ENC_R_A = 3, ENC_R_B = 5;

// Ultrasonik
const int TRIG_KANAN  = 36, ECHO_KANAN  = 34;
const int TRIG_TENGAH = 40, ECHO_TENGAH = 38;
const int TRIG_KIRI   = 44, ECHO_KIRI   = 42;

// ================== PARAMETER ==================
const float SETPOINT     = 10.0;  // jarak target ke dinding samping (cm)
const float BATAS_DEPAN  = 20.0;  // dinding depan < ini -> belok menjauhi dinding (sudut dalam)
const float BATAS_HILANG = 30.0;  // dinding samping > ini -> dinding hilang (sudut luar)

const int V_DASAR = 120;          // kecepatan dasar PWM (0-255), mulai pelan dulu
const int V_MAX   = 200;
const int V_PUTAR = 110;          // kecepatan saat putar di tempat

// Konstanta PID (tuning: Kp dulu -> Kd -> Ki terakhir)
float Kp = 5.0;
float Ki = 0.0;
float Kd = 2.5;

// Jika arah putar motor terbalik (robot mundur saat seharusnya maju),
// ubah ke true untuk motor yang terbalik
const bool BALIK_KIRI  = true;
const bool BALIK_KANAN = true;

// Faktor koreksi jika salah satu motor lebih cepat (1.0 = normal)
const float TRIM_KIRI  = 1.0;
const float TRIM_KANAN = 1.0;

const unsigned long SAMPLING_MS = 60;   // periode kontrol (ms)

// ================== VARIABEL ==================
volatile long tickKiri  = 0;
volatile long tickKanan = 0;

float errorSebelum = 0, integral = 0;
unsigned long waktuSebelum = 0;

// nilai terakhir yang valid (dipakai jika sensor sesekali ERR)
float lastDepan = 100, lastSamping = SETPOINT;

enum State { IKUTI_DINDING, SUDUT_DALAM, SUDUT_LUAR };
State state = IKUTI_DINDING;

// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_PWM, OUTPUT);
  if (PIN_STBY >= 0) {
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH);   // aktifkan driver
  }

  pinMode(TRIG_KANAN, OUTPUT);  pinMode(ECHO_KANAN, INPUT);
  pinMode(TRIG_TENGAH, OUTPUT); pinMode(ECHO_TENGAH, INPUT);
  pinMode(TRIG_KIRI, OUTPUT);   pinMode(ECHO_KIRI, INPUT);

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrKiri,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrKanan, RISING);

  berhenti();
  Serial.println("Wall following siap. Mulai dalam 3 detik...");
  delay(3000);
  waktuSebelum = millis();
}

// ============================================================
void loop() {
  unsigned long sekarang = millis();
  if (sekarang - waktuSebelum < SAMPLING_MS) return;
  float dt = (sekarang - waktuSebelum) / 1000.0;
  waktuSebelum = sekarang;

  // ---------- Baca sensor ----------
  float depan = bacaJarak(TRIG_TENGAH, ECHO_TENGAH);
  delay(10);
  float samping = (SISI_DINDING == KANAN)
                  ? bacaJarak(TRIG_KANAN, ECHO_KANAN)
                  : bacaJarak(TRIG_KIRI, ECHO_KIRI);

  // jika ERR (-1), pakai nilai valid terakhir
  if (depan   > 0) lastDepan   = depan;   else depan   = lastDepan;
  if (samping > 0) lastSamping = samping; else samping = lastSamping;

  // ---------- Tentukan state ----------
  if (depan < BATAS_DEPAN)         state = SUDUT_DALAM;
  else if (samping > BATAS_HILANG) state = SUDUT_LUAR;
  else                             state = IKUTI_DINDING;

  // ---------- Aksi ----------
  int s = SISI_DINDING;   // +1 dinding kanan, -1 dinding kiri
  int vKiri = 0, vKanan = 0;

  switch (state) {
    case IKUTI_DINDING: {
      // error > 0 : terlalu dekat dinding  -> menjauh
      // error < 0 : terlalu jauh dari dinding -> mendekat
      float error = SETPOINT - samping;
      integral += error * dt;
      integral = constrain(integral, -40, 40);          // anti windup
      float turunan = (error - errorSebelum) / dt;
      errorSebelum = error;

      float koreksi = Kp * error + Ki * integral + Kd * turunan;

      vKiri  = V_DASAR - s * koreksi;
      vKanan = V_DASAR + s * koreksi;
      break;
    }

    case SUDUT_DALAM:
      // ada dinding di depan -> putar di tempat menjauhi dinding samping
      vKiri  = -s * V_PUTAR;
      vKanan =  s * V_PUTAR;
      resetPID();
      break;

    case SUDUT_LUAR:
      // dinding samping hilang -> belok melengkung ke arah dinding
      vKiri  = V_DASAR + s * 40;
      vKanan = V_DASAR - s * 60;
      resetPID();
      break;
  }

  setMotor(vKiri, vKanan);

  // ---------- Debug ----------
  noInterrupts();
  long tL = tickKiri, tR = tickKanan;
  interrupts();

  Serial.print("Depan:");    Serial.print(depan, 1);
  Serial.print(" Samping:"); Serial.print(samping, 1);
  Serial.print(" State:");   Serial.print(namaState(state));
  Serial.print(" PWM_L:");   Serial.print(vKiri);
  Serial.print(" PWM_R:");   Serial.print(vKanan);
  Serial.print(" EncL:");    Serial.print(tL);
  Serial.print(" EncR:");    Serial.println(tR);
}

// ============================================================
//  SENSOR ULTRASONIK
// ============================================================
// Mengembalikan jarak (cm), atau -1 jika tidak ada pantulan
float bacaJarak(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long durasi = pulseIn(echo, HIGH, 25000);   // timeout ~4 m
  if (durasi == 0) return -1;
  return durasi * 0.0343 / 2.0;
}

// ============================================================
//  MOTOR (TB6612FNG)
// ============================================================
// kecepatan bertanda: positif = maju, negatif = mundur
void setMotor(int vKiri, int vKanan) {
  vKiri  = constrain(vKiri  * TRIM_KIRI,  -V_MAX, V_MAX);
  vKanan = constrain(vKanan * TRIM_KANAN, -V_MAX, V_MAX);
  satuMotor(L_IN1, L_IN2, L_PWM, vKiri,  BALIK_KIRI);
  satuMotor(R_IN1, R_IN2, R_PWM, vKanan, BALIK_KANAN);
}

void satuMotor(int in1, int in2, int pwm, int v, bool balik) {
  if (balik) v = -v;
  if (v > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (v < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);
  }
  analogWrite(pwm, abs(v));
}

void berhenti() {
  setMotor(0, 0);
}

void resetPID() {
  integral = 0;
  errorSebelum = 0;
}

// ============================================================
//  ENCODER (untuk monitoring; arah dari channel B)
// ============================================================
void isrKiri() {
  if (digitalRead(ENC_L_B)) tickKiri--; else tickKiri++;
}

void isrKanan() {
  if (digitalRead(ENC_R_B)) tickKanan++; else tickKanan--;
}

const char* namaState(State st) {
  switch (st) {
    case IKUTI_DINDING: return "IKUTI";
    case SUDUT_DALAM:   return "SUDUT_DALAM";
    case SUDUT_LUAR:    return "SUDUT_LUAR";
  }
  return "?";
}
