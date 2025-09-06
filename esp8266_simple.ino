// ESP8266 Motor Control for FollowBot
// Motor Driver L298N pins
#define IN1 D1  // GPIO5 - Left motor direction 1
#define IN2 D2  // GPIO4 - Left motor direction 2  
#define IN3 D3  // GPIO0 - Right motor direction 1
#define IN4 D4  // GPIO2 - Right motor direction 2
#define ENA D5  // GPIO14 - Left motor speed (PWM)
#define ENB D6  // GPIO12 - Right motor speed (PWM)

// Ultrasonic sensor HC-SR04
#define TRIG D7 // GPIO13
#define ECHO D8 // GPIO15

void setup() {
  Serial.begin(115200);
  
  // Motor pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Ultrasonic pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  // Stop motors initially
  stopMotors();
  
  Serial.println("ESP8266 FollowBot Ready");
}

void loop() {
  // Handle serial commands from Pi
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
  
  // Send distance data every 100ms
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 100) {
    sendDistance();
    lastSend = millis();
  }
}

void processCommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("MOVE:")) {
    // Format: MOVE:left_speed,right_speed (-100 to 100)
    int comma = cmd.indexOf(',');
    if (comma > 0) {
      int leftSpeed = cmd.substring(5, comma).toInt();
      int rightSpeed = cmd.substring(comma + 1).toInt();
      setMotorSpeeds(leftSpeed, rightSpeed);
    }
  }
  else if (cmd == "STOP") {
    stopMotors();
  }
}

void setMotorSpeeds(int left, int right) {
  // Constrain speeds to -100 to 100
  left = constrain(left, -100, 100);
  right = constrain(right, -100, 100);
  
  // Left motor control
  if (left > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (left < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, map(abs(left), 0, 100, 0, 1023));
  
  // Right motor control
  if (right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (right < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, map(abs(right), 0, 100, 0, 1023));
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

float getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0) return 999.0;
  
  return (duration * 0.034) / 2.0;
}

void sendDistance() {
  float dist = getDistance();
  Serial.print("DIST:");
  Serial.println(dist);
}