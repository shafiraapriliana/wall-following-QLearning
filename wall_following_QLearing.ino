/*
 * ============================================================
 *  WALL FOLLOWING DDMR (Q-LEARNING + EEPROM) - Arduino Mega
 * ============================================================
 *  Perintah Serial Monitor (115200 baud):
 *    'p' -> Cetak isi Q-Table saat ini
 *    's' -> Simpan Q-Table & epsilon ke EEPROM secara manual
 *    'r' -> Reset Q-Table & hapus data EEPROM (mulai belajar dari nol)
 * ============================================================
 */

#include <EEPROM.h>

// ================== MODE OPERASI ==================
const bool MODE_TRAINING = true;   // true = belajar, false = jalan pakai hasil EEPROM

// ================== PILIH SISI DINDING ==================
#define KANAN  1
#define KIRI  -1
const int SISI_DINDING = KANAN;

// ================== PIN (Sesuai Hardware Anda) ==================
const int L_IN1 = 8, L_IN2 = 7, L_PWM = 6;
const int R_IN1 = 9, R_IN2 = 10, R_PWM = 11;
const int PIN_STBY = -1;

const int ENC_L_A = 2, ENC_L_B = 4;
const int ENC_R_A = 3, ENC_R_B = 5;

const int TRIG_KANAN  = 36, ECHO_KANAN  = 34;
const int TRIG_TENGAH = 40, ECHO_TENGAH = 38;
const int TRIG_KIRI   = 44, ECHO_KIRI   = 42;

// ================== PARAMETER ROBOT ==================
const int V_DASAR = 85;
const int V_MAX   = 150;
const int V_PUTAR = 85;

const bool BALIK_KIRI  = true;
const bool BALIK_KANAN = true;

const float TRIM_KIRI  = 1.0;
const float TRIM_KANAN = 1.0;

const unsigned long SAMPLING_MS = 60;

// ================== PARAMETER Q-LEARNING ==================
const int JUMLAH_STATE = 8;
const int JUMLAH_AKSI  = 5;

const float ALPHA = 0.2;
const float GAMMA = 0.9;
float epsilon     = 0.8;
const float EPSILON_MIN   = 0.05;
const float EPSILON_DECAY = 0.995;

float Q[JUMLAH_STATE][JUMLAH_AKSI] = {0};

// ================== KONFIGURASI EEPROM ==================
const int  ADDR_MAGIC   = 0;
const byte MAGIC_KEY    = 0x5A; // Penanda bahwa EEPROM sudah berisi data valid
const int  ADDR_EPSILON = 1;
const int  ADDR_QTABLE  = ADDR_EPSILON + sizeof(float);

const unsigned long INTERVAL_SIMPAN_MS = 30000; // Auto-save tiap 30 detik saat training
unsigned long waktuSimpanTerakhir = 0;

// ================== VARIABEL ==================
volatile long tickKiri  = 0;
volatile long tickKanan = 0;

unsigned long waktuSebelum = 0;
float lastDepan = 100.0, lastSamping = 20.0;

int stateLama = -1;
int aksiLama  = -1;

// ============================================================
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0));

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_PWM, OUTPUT);
  if (PIN_STBY >= 0) {
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH);
  }

  pinMode(TRIG_KANAN, OUTPUT);  pinMode(ECHO_KANAN, INPUT);
  pinMode(TRIG_TENGAH, OUTPUT); pinMode(ECHO_TENGAH, INPUT);
  pinMode(TRIG_KIRI, OUTPUT);   pinMode(ECHO_KIRI, INPUT);

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrKiri,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrKanan, RISING);

  berhenti();
  muatEEPROM(); // Otomatis ambil hasil belajar sebelumnya jika ada

  Serial.println("Q-Learning DDMR siap. Ketik 'p' (print), 's' (save), 'r' (reset).");
  Serial.println("Mulai dalam 3 detik...");
  delay(3000);
  waktuSebelum = millis();
  waktuSimpanTerakhir = millis();
}

// ============================================================
void loop() {
  cekPerintahSerial();

  unsigned long sekarang = millis();
  if (sekarang - waktuSebelum < SAMPLING_MS) return;
  waktuSebelum = sekarang;

  // ---------- 1. Baca Sensor ----------
  float depan = bacaJarak(TRIG_TENGAH, ECHO_TENGAH);
  delay(10);
  float samping = (SISI_DINDING == KANAN)
                  ? bacaJarak(TRIG_KANAN, ECHO_KANAN)
                  : bacaJarak(TRIG_KIRI, ECHO_KIRI);

  if (depan   > 0) lastDepan   = depan;   else depan   = lastDepan;
  if (samping > 0) lastSamping = samping; else samping = lastSamping;

  // ---------- 2. Tentukan State & Reward ----------
  int stateBaru = getState(depan, samping);
  float reward  = getReward(depan, samping);

  // ---------- 3. Update Q-Table ----------
  if (MODE_TRAINING && stateLama != -1 && aksiLama != -1) {
    float maxQBerikutnya = getMaxQ(stateBaru);
    Q[stateLama][aksiLama] += ALPHA * (reward + GAMMA * maxQBerikutnya - Q[stateLama][aksiLama]);

    if (epsilon > EPSILON_MIN) {
      epsilon *= EPSILON_DECAY;
    }

    // Auto-save berkala ke EEPROM
    if (sekarang - waktuSimpanTerakhir >= INTERVAL_SIMPAN_MS) {
      simpanEEPROM();
      waktuSimpanTerakhir = sekarang;
    }
  }

  // ---------- 4. Pilih & Eksekusi Aksi ----------
  int aksiBaru = pilihAksi(stateBaru);
  int vKiri = 0, vKanan = 0;
  eksekusiAksi(aksiBaru, vKiri, vKanan);
  // ---------- Pengaman Darurat Depan ----------
  if (depan < 22.0) {
    // Paksa putar di tempat menjauhi dinding, abaikan aksi acak Q-Learning
    vKiri  = -SISI_DINDING * V_PUTAR;
    vKanan =  SISI_DINDING * V_PUTAR;
  }

  setMotor(vKiri, vKanan);
  setMotor(vKiri, vKanan);

  stateLama = stateBaru;
  aksiLama  = aksiBaru;

  // ---------- 5. Debug Serial ----------
  noInterrupts();
  long tL = tickKiri, tR = tickKanan;
  interrupts();

  Serial.print("D:");   Serial.print(depan, 1);
  Serial.print(" S:");  Serial.print(samping, 1);
  Serial.print(" St:"); Serial.print(stateBaru);
  Serial.print(" Act:");Serial.print(aksiBaru);
  Serial.print(" Rw:"); Serial.print(reward, 0);
  Serial.print(" Eps:");Serial.print(epsilon, 2);
  Serial.print(" L:");  Serial.print(vKiri);
  Serial.print(" R:");  Serial.print(vKanan);
  Serial.print(" Enc:");Serial.print(tL); Serial.print("/"); Serial.println(tR);
}

// ============================================================
//  FUNGSI EEPROM & SERIAL COMMAND
// ============================================================
void simpanEEPROM() {
  EEPROM.update(ADDR_MAGIC, MAGIC_KEY);
  EEPROM.put(ADDR_EPSILON, epsilon);
  EEPROM.put(ADDR_QTABLE, Q);
  Serial.println(">> [EEPROM] Q-Table & epsilon berhasil disimpan.");
}

void muatEEPROM() {
  if (EEPROM.read(ADDR_MAGIC) == MAGIC_KEY) {
    EEPROM.get(ADDR_EPSILON, epsilon);
    EEPROM.get(ADDR_QTABLE, Q);
    Serial.print(">> [EEPROM] Data dimuat. Epsilon terakhir: ");
    Serial.println(epsilon, 3);
  } else {
    Serial.println(">> [EEPROM] Belum ada data tersimpan. Mulai dari Q-Table kosong.");
  }
}

void resetEEPROM() {
  for (int s = 0; s < JUMLAH_STATE; s++) {
    for (int a = 0; a < JUMLAH_AKSI; a++) {
      Q[s][a] = 0.0;
    }
  }
  epsilon = 0.8;
  EEPROM.update(ADDR_MAGIC, 0x00);
  Serial.println(">> [EEPROM] Q-Table di-reset ke 0.");
}

void cetakQTable() {
  Serial.println("========== Q-TABLE ==========");
  for (int s = 0; s < JUMLAH_STATE; s++) {
    Serial.print("S"); Serial.print(s); Serial.print(": ");
    for (int a = 0; a < JUMLAH_AKSI; a++) {
      Serial.print(Q[s][a], 2);
      if (a < JUMLAH_AKSI - 1) Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println("=============================");
}

void cekPerintahSerial() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  if (cmd == 's' || cmd == 'S') simpanEEPROM();
  else if (cmd == 'p' || cmd == 'P') cetakQTable();
  else if (cmd == 'r' || cmd == 'R') resetEEPROM();
}

// ============================================================
//  FUNGSI Q-LEARNING
// ============================================================
int getState(float depan, float samping) {
  int sDepan = (depan < 15.0) ? 0 : 1;     // 0: Dekat (<15 cm), 1: Aman

  int sSamping;
  if      (samping < 12.0)  sSamping = 0;  // Terlalu Dekat
  else if (samping <= 18.0) sSamping = 1;  // Ideal (Target ~15 cm)
  else if (samping <= 23.0) sSamping = 2;  // Agak Jauh
  else                      sSamping = 3;  // Hilang (Sudut Luar)

  return (sDepan * 4) + sSamping;
}

float getReward(float depan, float samping) {
  if (depan < 15.0) return -50.0;                      // Bahaya tabrak depan
  if (samping > 23.0) return -20.0;                    // Kehilangan dinding
  if (samping >= 12.0 && samping <= 18.0) return 10.0; // Zona ideal (~15 cm)
  return -2.0;
}

float getMaxQ(int state) {
  float maxVal = Q[state][0];
  for (int a = 1; a < JUMLAH_AKSI; a++) {
    if (Q[state][a] > maxVal) maxVal = Q[state][a];
  }
  return maxVal;
}

int pilihAksi(int state) {
  if (MODE_TRAINING && ((float)random(0, 1000) / 1000.0 < epsilon)) {
    return random(0, JUMLAH_AKSI);
  }
  int aksiTerbaik = 0;
  float maxVal = Q[state][0];
  for (int a = 1; a < JUMLAH_AKSI; a++) {
    if (Q[state][a] > maxVal) {
      maxVal = Q[state][a];
      aksiTerbaik = a;
    }
  }
  return aksiTerbaik;
}

void eksekusiAksi(int aksi, int &vKiri, int &vKanan) {
  int s = SISI_DINDING;
  switch (aksi) {
    case 0: // Maju lurus
      vKiri = V_DASAR;          vKanan = V_DASAR;          break;
    case 1: // Serong menjauhi dinding
      vKiri = V_DASAR - s * 25; vKanan = V_DASAR + s * 25; break;
    case 2: // Serong mendekati dinding
      vKiri = V_DASAR + s * 25; vKanan = V_DASAR - s * 25; break;
    case 3: // Putar di tempat menjauhi dinding
      vKiri = -s * V_PUTAR;     vKanan =  s * V_PUTAR;     break;
    case 4: // Belok melengkung ke arah dinding
      vKiri = V_DASAR + s * 25; vKanan = V_DASAR - s * 40; break;
  }
}

// ============================================================
//  SENSOR, MOTOR & ENCODER
// ============================================================
float bacaJarak(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long durasi = pulseIn(echo, HIGH, 25000);
  if (durasi == 0) return -1;
  return durasi * 0.0343 / 2.0;
}

void setMotor(int vKiri, int vKanan) {
  vKiri  = constrain(vKiri  * TRIM_KIRI,  -V_MAX, V_MAX);
  vKanan = constrain(vKanan * TRIM_KANAN, -V_MAX, V_MAX);
  satuMotor(L_IN1, L_IN2, L_PWM, vKiri,  BALIK_KIRI);
  satuMotor(R_IN1, R_IN2, R_PWM, vKanan, BALIK_KANAN);
}

void satuMotor(int in1, int in2, int pwm, int v, bool balik) {
  if (balik) v = -v;
  if (v > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (v < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else            { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
  analogWrite(pwm, abs(v));
}

void berhenti() { setMotor(0, 0); }

void isrKiri()  { if (digitalRead(ENC_L_B)) tickKiri--;  else tickKiri++;  }
void isrKanan() { if (digitalRead(ENC_R_B)) tickKanan++; else tickKanan--; }
