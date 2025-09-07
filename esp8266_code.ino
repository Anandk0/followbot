#include <ArduinoJson.h>

// Motor pins (L298N driver)
#define MOTOR_L1 5    // D1 - Left motor direction 1
#define MOTOR_L2 4    // D2 - Left motor direction 2
#define MOTOR_R1 0    // D3 - Right motor direction 1
#define MOTOR_R2 2    // D4 - Right motor direction 2
#define MOTOR_L_PWM 14 // D5 - Left motor PWM (ENA)
#define MOTOR_R_PWM 12 // D6 - Right motor PWM (ENB)

// Ultrasonic sensor pins (HC-SR04)
#define TRIG_PIN 13   // D7
#define ECHO_PIN 15   // D8

// Status LED
#define LED_PIN 2     // Built-in LED

unsigned long lastTelemetry = 0;
unsigned long lastCommand = 0;
bool motorsActive = false;

void setup() {
  Serial.begin(115200);
  
  // Motor pins setup
  pinMode(MOTOR_L1, OUTPUT);
  pinMode(MOTOR_L2, OUTPUT);
  pinMode(MOTOR_R1, OUTPUT);
  pinMode(MOTOR_R2, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  
  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Status LED
  pinMode(LED_PIN, OUTPUT);
  
  stopMotors();
  
  // Startup indication
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
  }
  
  Serial.println("{\"status\":\"ESP8266 Motor Controller Ready\"}");
}

void loop() {
  handleSerial();
  
  // Send telemetry every 100ms
  if (millis() - lastTelemetry > 100) {
    sendTelemetry();
    lastTelemetry = millis();
  }
  
  // Safety timeout - stop motors if no command received for 1 second
  if (motorsActive && (millis() - lastCommand > 1000)) {
    stopMotors();
    motorsActive = false;
    Serial.println("{\"warning\":\"Motor timeout - stopped\"}");
  }
  
  // Blink LED when active
  if (motorsActive) {
    digitalWrite(LED_PIN, (millis() / 250) % 2);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}

void handleSerial() {
  if (Serial.available()) {
    String json = Serial.readStringUntil('\n');
    json.trim();
    
    if (json.length() == 0) return;
    
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, json);
    
    if (error) {
      Serial.print("{\"error\":\"JSON parse failed: ");
      Serial.print(error.c_str());
      Serial.println("\"}");
      return;
    }
    
    lastCommand = millis();
    String cmd = doc["cmd"];
    
    if (cmd == "set_speed") {
      int left = doc["left"];
      int right = doc["right"];
      setMotorSpeeds(left, right);
      motorsActive = (left != 0 || right != 0);
    }
    else if (cmd == "stop") {
      stopMotors();
      motorsActive = false;
    }
    else if (cmd == "ping") {
      Serial.println("{\"pong\":\"ESP8266 alive\"}");
    }
    else {
      Serial.print("{\"error\":\"Unknown command: ");
      Serial.print(cmd);
      Serial.println("\"}");
    }
  }
}

void setMotorSpeeds(int left, int right) {
  // Clamp speeds to valid range
  left = constrain(left, -100, 100);
  right = constrain(right, -100, 100);
  
  // Left motor control
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
  
  // Right motor control
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
  
  // Set PWM speeds (ESP8266 PWM range is 0-1023)
  int leftPWM = map(abs(left), 0, 100, 0, 1023);
  int rightPWM = map(abs(right), 0, 100, 0, 1023);
  
  analogWrite(MOTOR_L_PWM, leftPWM);
  analogWrite(MOTOR_R_PWM, rightPWM);
  
  // Debug output
  if (left != 0 || right != 0) {
    Serial.print("{\"motor_debug\":{\"left\":");
    Serial.print(left);
    Serial.print(",\"right\":");
    Serial.print(right);
    Serial.print(",\"pwm_left\":");
    Serial.print(leftPWM);
    Serial.print(",\"pwm_right\":");
    Serial.print(rightPWM);
    Serial.println("}}");
  }
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
  DynamicJsonDocument doc(256);
  
  doc["telemetry"]["distance_cm"] = getDistance();
  doc["telemetry"]["bat_v"] = 3.3; // Placeholder - add real battery monitoring
  doc["telemetry"]["uptime_ms"] = millis();
  doc["telemetry"]["motors_active"] = motorsActive;
  doc["telemetry"]["free_heap"] = ESP.getFreeHeap();
  
  serializeJson(doc, Serial);
  Serial.println();
}