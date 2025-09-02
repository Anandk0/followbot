#include <ArduinoJson.h>

// Motor pins
#define MOTOR_L1 5    // D1
#define MOTOR_L2 4    // D2
#define MOTOR_R1 0    // D3
#define MOTOR_R2 2    // D4
#define MOTOR_L_PWM 14 // D5
#define MOTOR_R_PWM 12 // D6

// Ultrasonic pins
#define TRIG_PIN 13   // D7
#define ECHO_PIN 15   // D8

unsigned long lastTelemetry = 0;

void setup() {
  Serial.begin(115200);
  
  // Motor pins
  pinMode(MOTOR_L1, OUTPUT);
  pinMode(MOTOR_L2, OUTPUT);
  pinMode(MOTOR_R1, OUTPUT);
  pinMode(MOTOR_R2, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  
  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  stopMotors();
}

void loop() {
  handleSerial();
  
  // Send telemetry every 100ms
  if (millis() - lastTelemetry > 100) {
    sendTelemetry();
    lastTelemetry = millis();
  }
}

void handleSerial() {
  if (Serial.available()) {
    String json = Serial.readStringUntil('\n');
    DynamicJsonDocument doc(256);
    
    if (deserializeJson(doc, json) == DeserializationError::Ok) {
      String cmd = doc["cmd"];
      
      if (cmd == "set_speed") {
        int left = doc["left"];
        int right = doc["right"];
        setMotorSpeeds(left, right);
      }
      else if (cmd == "stop") {
        stopMotors();
      }
    }
  }
}

void setMotorSpeeds(int left, int right) {
  // Left motor
  if (left > 0) {
    digitalWrite(MOTOR_L1, HIGH);
    digitalWrite(MOTOR_L2, LOW);
  } else if (left < 0) {
    digitalWrite(MOTOR_L1, LOW);
    digitalWrite(MOTOR_L2, HIGH);
  } else {
    digitalWrite(MOTOR_L1, LOW);
    digitalWrite(MOTOR_L2, LOW);
  }
  analogWrite(MOTOR_L_PWM, map(abs(left), 0, 100, 0, 1023));
  
  // Right motor
  if (right > 0) {
    digitalWrite(MOTOR_R1, HIGH);
    digitalWrite(MOTOR_R2, LOW);
  } else if (right < 0) {
    digitalWrite(MOTOR_R1, LOW);
    digitalWrite(MOTOR_R2, HIGH);
  } else {
    digitalWrite(MOTOR_R1, LOW);
    digitalWrite(MOTOR_R2, LOW);
  }
  analogWrite(MOTOR_R_PWM, map(abs(right), 0, 100, 0, 1023));
}

void stopMotors() {
  digitalWrite(MOTOR_L1, LOW);
  digitalWrite(MOTOR_L2, LOW);
  digitalWrite(MOTOR_R1, LOW);
  digitalWrite(MOTOR_R2, LOW);
  analogWrite(MOTOR_L_PWM, 0);
  analogWrite(MOTOR_R_PWM, 0);
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999.0; // No echo
  
  return (duration * 0.034) / 2.0; // Convert to cm
}

void sendTelemetry() {
  DynamicJsonDocument doc(128);
  doc["telemetry"]["distance_cm"] = getDistance();
  doc["telemetry"]["bat_v"] = 3.3; // Placeholder
  
  serializeJson(doc, Serial);
  Serial.println();
}