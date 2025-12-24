#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// =================================================================================
// ---------------------- WIFI SETTINGS (MUST MATCH CAR) ---------------------------
// =================================================================================
const char* ssid = "maani zamin";        // Enter Home Wi-Fi Name
const char* password = "password"; // Enter Home Wi-Fi Password

// IMPORTANT: Check Car's Serial Monitor to find this number!
const char* carIP = "http://10.132.19.234";   // Example: http://192.168.1.15
// =================================================================================

#define VIB_MOTOR_PIN 2  // Vibration Motor Pin

Adafruit_MPU6050 mpu;
float threshold = 4.0; // Sensitivity
String lastCommand = ""; 
unsigned long lastRequestTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(VIB_MOTOR_PIN, OUTPUT);
  digitalWrite(VIB_MOTOR_PIN, LOW);

  // 1. Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU NOT FOUND! Check Wiring.");
    while (1) { 
      digitalWrite(VIB_MOTOR_PIN, HIGH); delay(100); 
      digitalWrite(VIB_MOTOR_PIN, LOW); delay(100); 
    }
  }
  Serial.println("MPU Ready.");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // 2. Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  
  // Connection Buzz
  digitalWrite(VIB_MOTOR_PIN, HIGH); delay(300); digitalWrite(VIB_MOTOR_PIN, LOW);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  String currentCommand = "stop"; 

  // ---------------- AXIS MAPPING ----------------
  // X-Axis = Forward/Backward
  // Y-Axis = Left/Right
  
  if (a.acceleration.x < -threshold) {
    currentCommand = "forward"; 
  } 
  else if (a.acceleration.x > threshold) {
    currentCommand = "backward"; 
  }
  else if (a.acceleration.y < -threshold) {
    currentCommand = "left";    
  }
  else if (a.acceleration.y > threshold) {
    currentCommand = "right";   
  }
  else {
    currentCommand = "stop";
  }

  // ---------------- SEND LOGIC ----------------
  // Send if command changed OR if moving (to get constant DANGER updates)
  if (currentCommand != lastCommand || (currentCommand != "stop" && millis() - lastRequestTime > 150)) {
    sendRequest(currentCommand);
    lastCommand = currentCommand;
    lastRequestTime = millis();
  }

  delay(50); 
}

void sendRequest(String cmd) {
  if(WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(carIP) + "/" + cmd;
    
    http.begin(url);
    int httpCode = http.GET(); 
    
    if(httpCode > 0) {
      String payload = http.getString();
      payload.trim(); 

      // Haptic Feedback Logic
      if (payload == "DANGER") {
        digitalWrite(VIB_MOTOR_PIN, HIGH); // Vibrate on Danger
      } else {
        digitalWrite(VIB_MOTOR_PIN, LOW);
      }
    } 
    else {
      digitalWrite(VIB_MOTOR_PIN, LOW);
    }
    http.end();
  }
}