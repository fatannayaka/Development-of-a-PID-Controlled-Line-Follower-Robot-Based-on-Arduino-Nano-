#include <EEPROM.h>

// ================== MULTIPLEXER SENSOR ==================
#define S1 2
#define S2 3
#define S3 4
#define SENSOR_PIN A0

boolean A[8] = {0,1,0,1,0,1,0,1};
boolean B[8] = {0,0,1,1,0,0,1,1};
boolean C[8] = {0,0,0,0,1,1,1,1};

// ================== PIN ==================
#define CAL_BUTTON 7  // Tombol kalibrasi
#define pin_pwm_motor_L 6
#define pin_dir_motor_L 9
#define pin_pwm_motor_R 5
#define pin_dir_motor_R 10

// ================== VARIABEL ==================
int threshold = 500;
float Kp = 15, Ki = 0, Kd = 8;
float lastError = 0, sumError = 0;

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);

  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(CAL_BUTTON, INPUT_PULLUP);

  pinMode(pin_pwm_motor_L, OUTPUT);
  pinMode(pin_dir_motor_L, OUTPUT);
  pinMode(pin_pwm_motor_R, OUTPUT);
  pinMode(pin_dir_motor_R, OUTPUT);

  // Baca threshold dari EEPROM
  EEPROM.get(0, threshold);
  if (threshold < 100 || threshold > 900) threshold = 500;
  Serial.print("Threshold Awal: "); Serial.println(threshold);

  // Mode kalibrasi jika tombol ditekan saat nyala
  if (digitalRead(CAL_BUTTON) == LOW) {
    kalibrasiSensor();
  }

  Serial.println("Mulai Line Follower...");
  delay(1000);
}

// ================== LOOP ==================
void loop() {
  byte s = read_sensor();

  if (s == 0) {  // garis hilang
    setMotor(0, 0);
    Serial.println("Tidak ada garis!");
    return;
  }

  int error = getError(s);

  float P = error * Kp;
  sumError += error;
  float I = sumError * Ki;
  float D = (error - lastError) * Kd;
  lastError = error;

  float PID = P + I + D;

  int baseSpeed = 120;
  int leftMotor = baseSpeed - PID;
  int rightMotor = baseSpeed + PID;

  setMotor(leftMotor, rightMotor);
}

// ================== MOTOR CONTROL ==================
void setMotor(int L, int R) {
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  digitalWrite(pin_dir_motor_L, (L >= 0) ? HIGH : LOW);
  analogWrite(pin_pwm_motor_L, abs(L));

  digitalWrite(pin_dir_motor_R, (R >= 0) ? HIGH : LOW);
  analogWrite(pin_pwm_motor_R, abs(R));
}

// ================== SENSOR READER ==================
byte read_sensor() {
  byte out = 0;
  for (int i = 0; i < 8; i++) {
    digitalWrite(S1, A[i]);
    digitalWrite(S2, B[i]);
    digitalWrite(S3, C[i]);
    delayMicroseconds(200);
    int value = analogRead(SENSOR_PIN);
    if (value > threshold) out |= (1 << (7 - i));
  }
  return out;
}

// ================== ERROR MAPPING ==================
int getError(byte s) {
  switch (s) {
    case 0b00011000: return 0;
    case 0b00011100: return -1;
    case 0b00111000: return 1;
    case 0b00001100: return -2;
    case 0b00110000: return 2;
    case 0b00001110: return -3;
    case 0b01110000: return 3;
    case 0b00000110: return -4;
    case 0b01100000: return 4;
    case 0b00000010: return -5;
    case 0b01000000: return 5;
    case 0b00000001: return -6;
    case 0b10000000: return 6;
    default: return 0;
  }
}

// ================== KALIBRASI SENSOR ==================
void kalibrasiSensor() {
  int minVal = 1023, maxVal = 0;

  Serial.println("=== MODE KALIBRASI SENSOR ===");
  Serial.println("Geser robot di atas garis hitam & putih...");

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {  // kalibrasi 5 detik
    for (int i = 0; i < 8; i++) {
      digitalWrite(S1, A[i]);
      digitalWrite(S2, B[i]);
      digitalWrite(S3, C[i]);
      delayMicroseconds(200);
      int value = analogRead(SENSOR_PIN);
      if (value < minVal) minVal = value;
      if (value > maxVal) maxVal = value;
    }
  }

  threshold = (minVal + maxVal) / 2;
  EEPROM.put(0, threshold);

  Serial.print("Kalibrasi selesai! Threshold disimpan: ");
  Serial.println(threshold);
  delay(1000);
}