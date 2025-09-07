#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

// Fixed IP configuration
IPAddress local_IP(10, 163, 221, 143);  // ESP8266 IP in same network as desktop
IPAddress gateway(10, 163, 221, 1);     // Usually your router IP
IPAddress subnet(255, 255, 255, 0);     // Standard subnet mask

// Motor pins
const int IN1 = D1, IN2 = D2, IN3 = D3, IN4 = D4;
const int ENA = D5, ENB = D6;

// Sensor pins
const int TRIG = D7, ECHO = D8;

ESP8266WebServer server(80);
unsigned long lastCommand = 0;
const unsigned long MOTOR_TIMEOUT = 1000;

void setup() {
  Serial.begin(115200);
  
  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  
  // Sensor pins
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  
  stopMotors();
  
  // Configure static IP
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure static IP");
  }
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! Fixed IP: ");
  Serial.println(WiFi.localIP());
  
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/motor", HTTP_POST, handleMotor);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  
  // Motor timeout safety
  if (millis() - lastCommand > MOTOR_TIMEOUT) {
    stopMotors();
  }
}

void handleRoot() {
  server.send(200, "text/plain", "ESP8266 FollowBot WiFi Ready");
}

void handleMotor() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    Serial.println("Received: " + body);
    DynamicJsonDocument doc(1024);
    
    if (deserializeJson(doc, body) == DeserializationError::Ok) {
      String cmd = doc["cmd"];
      
      if (cmd == "set_speed") {
        int left = doc["left"];
        int right = doc["right"];
        setMotorSpeeds(left, right);
        server.send(200, "application/json", "{\"status\":\"ok\"}");
      }
      else if (cmd == "stop") {
        stopMotors();
        server.send(200, "application/json", "{\"status\":\"stopped\"}");
      }
      else {
        server.send(400, "application/json", "{\"error\":\"unknown command\"}");
      }
      
      lastCommand = millis();
    } else {
      server.send(400, "application/json", "{\"error\":\"invalid json\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"no data\"}");
  }
}

void handleStatus() {
  float distance = getDistance();
  
  DynamicJsonDocument doc(512);
  doc["distance_cm"] = distance;
  doc["uptime_ms"] = millis();
  doc["free_heap"] = ESP.getFreeHeap();
  doc["ip"] = WiFi.localIP().toString();
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void setMotorSpeeds(int left, int right) {
  Serial.print("Motor speeds: L="); Serial.print(left); Serial.print(", R="); Serial.println(right);
  
  // Apply minimum speed threshold (motors need at least 25% to move)
  int leftPWM = 0, rightPWM = 0;
  
  if (abs(left) > 0) {
    leftPWM = map(abs(left), 0, 100, 300, 1023);  // Min 300 (~30%) for movement
  }
  if (abs(right) > 0) {
    rightPWM = map(abs(right), 0, 100, 300, 1023);  // Min 300 (~30%) for movement
  }
  
  // Left motor (reversed)
  if (left > 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, leftPWM);
  } else if (left < 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, leftPWM);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
  
  // Right motor (reversed)
  if (right > 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, rightPWM);
  } else if (right < 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, rightPWM);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

float getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0) return 999.0;
  
  return duration * 0.034 / 2;
}