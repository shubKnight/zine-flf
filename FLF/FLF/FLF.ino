#include <Wire.h>
#include <U8g2lib.h>
#include <EEPROM.h>

// OLED Display
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// EEPROM Addresses
#define EEPROM_ADDRESS_CAL_MIN 0
#define EEPROM_ADDRESS_CAL_MAX (EEPROM_ADDRESS_CAL_MIN + sizeof(double) * sensor_count)
#define EEPROM_KP 100
#define EEPROM_KI 104
#define EEPROM_KD 108

#define PID_MAX 40
#define PID_MIN -40

// Motor Pins
#define pwmA 11
#define inA1 3
#define inA2 4
#define inB1 6
#define inB2 7
#define pwmB 9
#define STBY 5

// Multiplexer Pins
#define muxSig A0
#define muxS0 8
#define muxS1 10
#define muxS2 12

// Buttons
#define btnSelect A1
#define btnUp A2
#define btnDown A3

// Constants
const int sensor_count = 8;
float sensor_reading[sensor_count];
float cal_min[sensor_count];
float cal_max[sensor_count];
float weights[sensor_count / 2] = {40.0, 30.0, 20.0, 10.0};
float Kp = 1.5, Ki = 0.0, Kd = 8.0;   // default values
float PID = 0, integral_error = 0, previous_error = 0;

// Menu
String modes[] = {"Line Follow", "Calibrate", "PID Tuning", "Threshold Change", "Motor Test", "Sensor Reading"};
const int totalModes = sizeof(modes) / sizeof(modes[0]);
int modeIndex = 0;
bool running = false;

// -------- MUX READING ----------
int muxRead(int channel) {
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  delayMicroseconds(5);
  return analogRead(muxSig);
}

// -------- SENSOR READ ----------
// void sensor_read() {
//   for (int i = 0; i < sensor_count; i++) {
//     int raw = muxRead(i);
//     raw = constrain(raw, (long)cal_min[i], (long)cal_max[i]);
//     sensor_reading[i] = map(raw, cal_min[i], cal_max[i], 0, 1000);
//   }
// }
void sensor_read() {
  for (int i = 0; i < sensor_count; i++) {
    int raw = muxRead(i);
    raw = constrain(raw, (long)cal_min[i], (long)cal_max[i]);
    // sensor_reading[i] = map(raw, cal_min[i], cal_max[i], 0, 1000);
    sensor_reading[i] = 1000 - map(raw, cal_min[i], cal_max[i], 0, 1000);

    // Print in "min/present/max" format
    Serial.print("S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(cal_min[i], 0);   // min
    Serial.print("/");
    Serial.print(sensor_reading[i], 0); // present (mapped)
    Serial.print("/");
    Serial.print(cal_max[i], 0);   // max
    Serial.print("  ");
  }
  Serial.println();
}



// -------- CALIBRATION ----------
void calibrate() {
  u8g2.clearBuffer();
  u8g2.setCursor(0, 12);
  u8g2.print("Calibrating...");
  u8g2.sendBuffer();

  for (int i = 0; i < sensor_count; i++) {
    cal_min[i] = 1023.0;
    cal_max[i] = 0.0;
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 7000) { // 7 sec
    for (int i = 0; i < sensor_count; i++) {
      int raw = muxRead(i);
      if (raw < cal_min[i]) cal_min[i] = raw;
      if (raw > cal_max[i]) cal_max[i] = raw;
    }
    delay(10);
  }

  EEPROM.put(EEPROM_ADDRESS_CAL_MIN, cal_min);
  EEPROM.put(EEPROM_ADDRESS_CAL_MAX, cal_max);

  u8g2.clearBuffer();
  u8g2.setCursor(0, 12);
  u8g2.print("Calibration Done");
  u8g2.sendBuffer();
  delay(1000);
}

// -------- PID FOLLOWER ----------
// void pid_follower() {
//   float error = 0;
//   sensor_read();
//   for (int i = 0; i < sensor_count / 2; i++) {
//     error += (sensor_reading[i] - sensor_reading[7 - i]) * weights[i];
//   }
//   integral_error += error;
//   float derivative = error - previous_error;
//   PID = (Kp * error) + (Ki * integral_error) + (Kd * derivative);
//   previous_error = error;
// }
void pid_follower() {
  float error = 0;
  sensor_read();

  for (int i = 0; i < sensor_count / 2; i++) {
    error += (sensor_reading[i] - sensor_reading[7 - i]) * weights[i];
  }

  integral_error += error;
  float derivative = error - previous_error;
  PID = (Kp * error) + (Ki * integral_error) + (Kd * derivative);

  // ---------- Limit PID output ----------
  PID = constrain(PID, PID_MIN, PID_MAX);

  previous_error = error;
}

void line_follow() {
  pid_follower();
  int baseSpeed = 150;
  int leftSpeed = baseSpeed + PID;
  int rightSpeed = baseSpeed - PID;
  motor_speed(leftSpeed, rightSpeed);
}

// -------- MOTOR CONTROL ----------
void motor_speed(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(inA1, left > 0);
  digitalWrite(inA2, left <= 0);
  analogWrite(pwmA, abs(left));

  digitalWrite(inB1, right > 0);
  digitalWrite(inB2, right <= 0);
  analogWrite(pwmB, abs(right));
}
// ---------- PID TUNING MODE ----------
// ---------- PID TUNING MODE ----------
void pidTuningMode() {
  while (true) {
    // Check if back button pressed → return to menu
    if (digitalRead(btnDown) == LOW) {
      delay(200);
      return;
    }

    // Process incoming serial data (Bluetooth)
    if (Serial.available()) {
      char incoming = Serial.read();
      if (incoming == 'p' || incoming == 'i' || incoming == 'd') {
        String constant = "";
        unsigned long startTime = millis();
        while (millis() - startTime < 100) {
          if (Serial.available()) {
            char data = Serial.read();
            if (data == '\n') break;
            constant += data;
          }
        }
        float value = constant.toFloat();

        if (incoming == 'p') {
          Kp = value;
          EEPROM.put(EEPROM_KP, Kp);
        } 
        else if (incoming == 'i') {
          Ki = value;
          EEPROM.put(EEPROM_KI, Ki);
        } 
        else if (incoming == 'd') {
          Kd = value;
          EEPROM.put(EEPROM_KD, Kd);
        }

        Serial.print("PID Update: ");
        Serial.print(incoming);
        Serial.print(" = ");
        Serial.println(value);
      }
    }

    // ---------- U8g2 Display Update ----------
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_6x10_tr); // small readable font
      u8g2.setCursor(0, 12);
      u8g2.print("PID Tuning Mode");

      u8g2.setCursor(0, 26);
      u8g2.print("Kp: "); u8g2.print(Kp, 2);

      u8g2.setCursor(0, 38);
      u8g2.print("Ki: "); u8g2.print(Ki, 2);

      u8g2.setCursor(0, 50);
      u8g2.print("Kd: "); u8g2.print(Kd, 2);

      u8g2.setCursor(0, 62);
      u8g2.print("Down=Back");
    } while (u8g2.nextPage());

    delay(150); // small refresh delay
  }
}



// -------- OLED DISPLAY ----------
void show_menu() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 12);
    u8g2.print("Menu:");
    for (int i = 0; i < totalModes; i++) {
      u8g2.setCursor(0, 24 + (i * 10));
      if (i == modeIndex) u8g2.print(">");
      else u8g2.print(" ");
      u8g2.print(modes[i]);
    }
  } while ( u8g2.nextPage() );
}

// -------- MODE HANDLERS ----------
void run_mode(int idx) {
  running = true;

  if (idx == 0) { // Line Follow
    while (running) {
      line_follow();
      if (!digitalRead(btnDown)) running = false;
    }
  }

  else if (idx == 1) { // Calibrate
    calibrate();
    running = false;
  }

  else if (idx == 2) { // PID tuning
    pidTuningMode();
  }

  else if (idx == 3) { // Threshold change
    u8g2.clearBuffer();
    u8g2.setCursor(0, 12);
    u8g2.print("Threshold Change");
    u8g2.sendBuffer();
    delay(2000);
    running = false;
  }

  else if (idx == 4) { // Motor Test
    u8g2.clearBuffer();
    u8g2.setCursor(0, 12);
    u8g2.print("Motor Test");
    u8g2.sendBuffer();

    motor_speed(150, 0); // left
    delay(4000);
    motor_speed(0, 150); // right
    delay(4000);
    motor_speed(0, 0);

    running = false;
  }

  else if (idx == 5) { // Sensor Reading
    while (running) {
      sensor_read();
      u8g2.firstPage();
      do {
        u8g2.setCursor(0, 12);
        u8g2.print("Sensors:");
        for (int i = 0; i < sensor_count; i++) {
          u8g2.setCursor(0, 24 + (i * 8));
          u8g2.print(i); u8g2.print(":"); u8g2.print(sensor_reading[i]);
        }
      } while (u8g2.nextPage());

      if (!digitalRead(btnDown)) running = false;
      delay(100);
    }
  }
}

// -------- SETUP ----------
void setup() {
  Serial.begin(9600);

  pinMode(btnSelect, INPUT_PULLUP);
  pinMode(btnUp, INPUT_PULLUP);
  pinMode(btnDown, INPUT_PULLUP);

  u8g2.begin();

  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  for (int i = 0; i < sensor_count; i++) {
    cal_min[i] = 0;
    cal_max[i] = 1023;
  }
  EEPROM.get(EEPROM_KP, Kp);
  EEPROM.get(EEPROM_KI, Ki);
  EEPROM.get(EEPROM_KD, Kd);
  EEPROM.get(EEPROM_ADDRESS_CAL_MIN, cal_min );
  EEPROM.get(EEPROM_ADDRESS_CAL_MAX, cal_max);

  show_menu();
}

// -------- LOOP ----------
void loop() {
  if (!digitalRead(btnUp)) {
    modeIndex = (modeIndex + 1) % totalModes;
    show_menu();
    delay(300);
  }
  if (!digitalRead(btnDown)) {
    modeIndex = (modeIndex - 1 + totalModes) % totalModes;
    show_menu();
    delay(300);
  }
  if (!digitalRead(btnSelect)) {
    run_mode(modeIndex);
    show_menu();
    delay(300);
  }
}
