#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =================================================================================
// ---------------------- WIFI SETTINGS ---------------------------
// =================================================================================
const char* ssid = "maani zamin";        
const char* password = "password"; 
// =================================================================================

// =================================================================================
// ---------------------- ⚙️ MOTOR CALIBRATION SETTINGS ⚙️ -----------------------
// =================================================================================

// 1. DIRECTION FIX: 
// Since "Forward" made the car turn, one side is backwards.
// We invert the Left Motor to sync it with the Right Motor.
#define INVERT_RIGHT_MOTOR  false 
#define INVERT_LEFT_MOTOR   true   // <--- CHANGED TO TRUE

// 2. STEERING FIX:
// Since we fixed the motor direction above, we disable the manual swap 
// so Left Button = Left Turn.
#define SWAP_MANUAL_STEERING false // <--- CHANGED TO FALSE

// =================================================================================

WebServer server(80);
Servo myServo;

// ---------------- PIN CONFIG ----------------
// Right Motors (IN1/IN2)
#define IN1 4
#define IN2 27 
// Left Motors (IN3/IN4)
#define IN3 18
#define IN4 19

#define HEADLIGHT 22
#define HIGHBEAM 23
#define BACKLIGHT 13
#define BUZZER 14

#define FIXED_LIDAR_PIN 32
#define SCAN_LIDAR_PIN 35
#define SERVO_PIN 15

// ---------------- VARIABLES ----------------
const int dangerThreshold = 2000;    
const int clearThreshold = 1000;     
const int turnTime = 385;            
const int clearanceTime = 400;       

bool autopilot = false;
bool collisionBeepMode = false;
bool safetyOverride = false;

int evadeDirection = 0; 
unsigned long stateTimer = 0;
unsigned long manualOverrideTime = 0;
const unsigned long manualOverrideTimeout = 2000;

enum AutoState { CRUISE, SCANNING, EVADING, TRACKING, RECOVERING };
AutoState currentState = CRUISE;

// ---------------- HELPERS ----------------
int readFixed() { return analogRead(FIXED_LIDAR_PIN); }
int readScanner() { return analogRead(SCAN_LIDAR_PIN); }

String checkSurroundings() {
  if (readFixed() > dangerThreshold || readScanner() > dangerThreshold) return "DANGER";
  return "SAFE";
}

void performSafetyStop() {
  stopCar();
  digitalWrite(BACKLIGHT, HIGH); 
  if (collisionBeepMode) { digitalWrite(BUZZER, HIGH); delay(200); digitalWrite(BUZZER, LOW); }
}

// ---------------- MOTOR PRIMITIVES (Low Level) ----------------
void setRightMotor(int dir) { // 1=Fwd, -1=Back, 0=Stop
  if(INVERT_RIGHT_MOTOR) dir *= -1;
  if(dir == 1) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if(dir == -1) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
}

void setLeftMotor(int dir) { // 1=Fwd, -1=Back, 0=Stop
  if(INVERT_LEFT_MOTOR) dir *= -1; // Logic flip applied here
  
  if(dir == 1) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if(dir == -1) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
}

// ---------------- MOVEMENT FUNCTIONS ----------------
void stopCar() { 
  setRightMotor(0); setLeftMotor(0);
  digitalWrite(BACKLIGHT, LOW); digitalWrite(BUZZER, LOW);
}

void moveForward() { 
  setRightMotor(1); setLeftMotor(1);
  digitalWrite(BACKLIGHT, LOW);
}

void moveBackward() { 
  setRightMotor(-1); setLeftMotor(-1);
  digitalWrite(BACKLIGHT, HIGH); 
}

void turnLeft() { 
  // Standard Tank Turn Left: Right Fwd, Left Back
  setRightMotor(1); setLeftMotor(-1);
}

void turnRight() { 
  // Standard Tank Turn Right: Right Back, Left Fwd
  setRightMotor(-1); setLeftMotor(1);
}

// ---------------- MANUAL CONTROL ----------------
void manualStop() { stopCar(); myServo.write(90); manualOverrideTime = millis(); }

void manualForward() {
  if (!safetyOverride && checkSurroundings() == "DANGER") { performSafetyStop(); return; }
  moveForward(); myServo.write(90); manualOverrideTime = millis();
}

void manualVoiceForward() {
  if (!safetyOverride && checkSurroundings() == "DANGER") { performSafetyStop(); return; }
  moveForward(); myServo.write(90); manualOverrideTime = millis();
}

void manualLeft() {
  if (SWAP_MANUAL_STEERING) turnRight(); else turnLeft();
  myServo.write(170); manualOverrideTime = millis();
}

void manualRight() {
  if (SWAP_MANUAL_STEERING) turnLeft(); else turnRight();
  myServo.write(10); manualOverrideTime = millis();
}

void manualBackward() { moveBackward(); myServo.write(90); manualOverrideTime = millis(); }

// ---------------- AUTOPILOT LOGIC ----------------
void autoPilotLogic() {
  if (!autopilot) return;
  if (millis() - manualOverrideTime < manualOverrideTimeout) return;

  switch (currentState) {
    case CRUISE:
      myServo.write(90); 
      if (readFixed() > dangerThreshold) {
        stopCar();
        if(collisionBeepMode) { digitalWrite(BUZZER, HIGH); delay(100); digitalWrite(BUZZER, LOW); }
        currentState = SCANNING;
        stateTimer = millis();
      } else {
        moveForward();
      }
      break;

    case SCANNING: {
      if (millis() - stateTimer < 500) { myServo.write(170); return; } 
      int distLeft = readScanner();
      
      if (millis() - stateTimer < 1000) { myServo.write(10); return; }
      int distRight = readScanner();
      
      if (distLeft < distRight) {
        // Left is safer -> Turn Left
        evadeDirection = 1; 
        turnLeft(); 
        myServo.write(10); // <--- CHANGED: Look RIGHT to track obstacle
      } else {
        // Right is safer -> Turn Right
        evadeDirection = 2; 
        turnRight(); 
        myServo.write(170); // <--- CHANGED: Look LEFT to track obstacle
      }
      currentState = EVADING;
      stateTimer = millis();
      break;
    }

    case EVADING:
      if (millis() - stateTimer > turnTime) { moveForward(); currentState = TRACKING; }
      break;

    case TRACKING:
      if (readScanner() < clearThreshold) { 
        delay(clearanceTime); stopCar(); currentState = RECOVERING; stateTimer = millis();
        if (evadeDirection == 1) turnRight(); else turnLeft();
      }
      break;

    case RECOVERING:
      if (millis() - stateTimer > turnTime) { stopCar(); currentState = CRUISE; }
      break;
  }
}

// ---------------- WEBPAGE UI ----------------
String webpage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>Blind RC</title>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<style>
  body { background: #ffe6eb; color: #d63384; font-family: 'Comic Sans MS', cursive, sans-serif; text-align: center; margin: 0; overflow-x: hidden; user-select: none; }
  h1 { text-shadow: 2px 2px #ffb7c5; margin-top: 10px; }
  .btn { width: 80px; height: 50px; margin: 5px; font-size: 16px; border-radius: 15px; border: none; font-weight: bold; cursor: pointer; box-shadow: 0 4px #ffb7c5; transition: 0.1s; touch-action: manipulation; }
  .btn:active { box-shadow: 0 0 #ffb7c5; transform: translateY(4px); }
  .move { background: #ff85a2; color: white; }
  .stop { background: #ff0055; color: white; width: 80px; border-radius: 50%; height: 80px; }
  .aux { background: #ffd1dc; color: #d63384; font-size: 12px; width: 70px; height: 40px;}
  .auto { background: #a2d2ff; color: white; width: 200px; margin-top:15px;}
  .mic { background: #ffc8dd; color: #d63384; width: 90%; height: 70px; font-size: 22px; margin: 10px auto; display:block; border-radius: 20px; box-shadow: 0 0 15px #ffb7c5; animation: pulse 2s infinite; }
  .toggle-on { background: #bde0fe; border: 2px solid #a2d2ff; }
  .toggle-off { background: #ccc; }
  @keyframes pulse { 0% {transform: scale(1);} 50% {transform: scale(1.02);} 100% {transform: scale(1);} }
  .petal { position: absolute; background: #ffb7c5; width: 10px; height: 10px; border-radius: 10px 0; animation: fall 10s linear infinite; z-index: -1; }
  @keyframes fall { 0% { top: -10%; transform: rotate(0deg); } 100% { top: 110%; transform: rotate(360deg); } }
</style>
</head>
<body>
<script>
  for(let i=0; i<15; i++){ let p = document.createElement('div'); p.className = 'petal'; p.style.left = Math.random()*100 + '%'; p.style.animationDuration = (Math.random()*5 + 5) + 's'; p.style.animationDelay = Math.random()*5 + 's'; document.body.appendChild(p); }
</script>
<h1>🌸 Blind Assist Dog 🌸</h1>
<div id="transcript" style="color: #ff0055; height:25px; font-size:14px;">Tap Mic to Speak...</div>
<button class='btn mic' onclick="startDictation()">🎙️ VOICE COMMAND</button>
<div style="display:flex; justify-content:center; align-items:center; flex-direction:column;">
  <button class='btn move' ontouchstart="go('forward'); event.preventDefault();" ontouchend="go('stop');" onmousedown="go('forward');" onmouseup="go('stop');" onmouseleave="go('stop');">▲</button>
  <div>
    <button class='btn move' ontouchstart="go('left'); event.preventDefault();" ontouchend="go('stop');" onmousedown="go('left');" onmouseup="go('stop');" onmouseleave="go('stop');">◄</button>
    <button class='btn stop' onclick="go('stop')">STOP</button>
    <button class='btn move' ontouchstart="go('right'); event.preventDefault();" ontouchend="go('stop');" onmousedown="go('right');" onmouseup="go('stop');" onmouseleave="go('stop');">►</button>
  </div>
  <button class='btn move' ontouchstart="go('backward'); event.preventDefault();" ontouchend="go('stop');" onmousedown="go('backward');" onmouseup="go('stop');" onmouseleave="go('stop');">▼</button>
</div>
<div style="margin-top:15px; background: rgba(255,255,255,0.5); padding:10px; border-radius:15px;">
  <button class='btn aux' onclick="go('headlight/on')">Head ON</button> <button class='btn aux' onclick="go('headlight/off')">Head OFF</button>
  <button class='btn aux' onclick="go('highbeam/on')">High ON</button> <button class='btn aux' onclick="go('highbeam/off')">High OFF</button><br>
  <button class='btn aux' onclick="go('backlight/on')">Back ON</button> <button class='btn aux' onclick="go('backlight/off')">Back OFF</button>
  <button class='btn aux' style="background:#ffcc00; color:black;" ontouchstart="go('horn/on'); event.preventDefault();" ontouchend="go('horn/off');" onmousedown="go('horn/on');" onmouseup="go('horn/off');" onmouseleave="go('horn/off');">📢 HORN</button>
</div>
<div style="margin-top:15px;">
  <button id="safetyBtn" class='btn aux toggle-on' style="width:150px;" onclick="toggleSafety()">Safety: ON</button><br>
  <button id="beepBtn" class='btn aux toggle-off' style="width:150px;" onclick="toggleBeep()">Collision Beep: OFF</button><br>
  <button class='btn auto' onclick="go('autopilot/toggle')">🌸 AUTOPILOT 🌸</button>
</div>
<script>
function go(cmd){ fetch('/'+cmd).then(r=>r.text()).then(t=>console.log(t)); }
function toggleBeep() { fetch('/toggle/beep').then(r=>r.text()).then(s => { let b = document.getElementById('beepBtn'); b.innerText = "Collision Beep: " + s; b.className = s=="ON" ? "btn aux toggle-on" : "btn aux toggle-off"; }); }
function toggleSafety() { fetch('/toggle/safety').then(r=>r.text()).then(s => { let b = document.getElementById('safetyBtn'); b.innerText = "Safety: " + s; b.className = s=="ON" ? "btn aux toggle-on" : "btn aux toggle-off"; }); }
function startDictation() {
  if (window.hasOwnProperty('webkitSpeechRecognition')) {
    var recognition = new webkitSpeechRecognition();
    recognition.continuous = false; recognition.interimResults = false; recognition.lang = "en-US";
    try { recognition.start(); document.getElementById('transcript').innerText = "Listening..."; } catch(e) { alert("Mic Error: " + e.message); }
    recognition.onresult = function(e) {
      var text = e.results[0][0].transcript.toLowerCase();
      document.getElementById('transcript').innerText = '"' + text + '"';
      if (text.includes("forward") || text.includes("go")) go('voice_forward');
      else if (text.includes("back")) go('backward');
      else if (text.includes("left")) { go('left'); setTimeout(() => go('stop'), 1000); }
      else if (text.includes("right")) { go('right'); setTimeout(() => go('stop'), 1000); }
      else if (text.includes("stop")) go('stop');
      else if (text.includes("head") || text.includes("light")) go('headlight/on');
      else if (text.includes("high") || text.includes("beam")) go('highbeam/on');
      else if (text.includes("off")) { go('headlight/off'); go('highbeam/off'); }
      else if (text.includes("horn")) { go('horn/on'); setTimeout(() => go('horn/off'), 500); }
      else if (text.includes("auto")) go('autopilot/toggle');
    };
    recognition.onerror = function(e) { document.getElementById('transcript').innerText = "Error: " + e.error; };
  } else { alert("Use Chrome!"); }
}
</script>
</body></html>
)rawliteral";

// ---------------- SETUP ----------------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);
  myServo.attach(SERVO_PIN);
  myServo.write(90); 

  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);

  pinMode(HEADLIGHT, OUTPUT); pinMode(HIGHBEAM, OUTPUT); pinMode(BACKLIGHT, OUTPUT); pinMode(BUZZER, OUTPUT);
  pinMode(FIXED_LIDAR_PIN, INPUT);
  pinMode(SCAN_LIDAR_PIN, INPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  
  server.on("/", [](){ server.send(200, "text/html", webpage); });
  server.on("/forward", [](){ manualForward(); server.send(200,"text/plain", checkSurroundings()); });
  server.on("/voice_forward", [](){ manualVoiceForward(); server.send(200,"text/plain", checkSurroundings()); });
  server.on("/left", [](){ manualLeft(); server.send(200,"text/plain", checkSurroundings()); });
  server.on("/right", [](){ manualRight(); server.send(200,"text/plain", checkSurroundings()); });
  server.on("/stop", [](){ manualStop(); server.send(200,"text/plain", checkSurroundings()); });
  server.on("/backward", [](){ manualBackward(); server.send(200,"text/plain", "SAFE"); });
  server.on("/autopilot/toggle", [](){ autopilot = !autopilot; currentState = CRUISE; manualStop(); server.send(200,"text/plain", autopilot?"ON":"OFF"); });
  server.on("/toggle/beep", [](){ collisionBeepMode = !collisionBeepMode; server.send(200, "text/plain", collisionBeepMode ? "ON" : "OFF"); });
  server.on("/toggle/safety", [](){ safetyOverride = !safetyOverride; server.send(200, "text/plain", !safetyOverride ? "ON" : "OFF"); });
  server.on("/headlight/on", [](){ digitalWrite(HEADLIGHT,HIGH); server.send(200,"OK"); });
  server.on("/headlight/off", [](){ digitalWrite(HEADLIGHT,LOW); server.send(200,"OK"); });
  server.on("/highbeam/on", [](){ digitalWrite(HIGHBEAM,HIGH); server.send(200,"OK"); });
  server.on("/highbeam/off", [](){ digitalWrite(HIGHBEAM,LOW); server.send(200,"OK"); });
  server.on("/backlight/on", [](){ digitalWrite(BACKLIGHT,HIGH); server.send(200,"OK"); });
  server.on("/backlight/off", [](){ digitalWrite(BACKLIGHT,LOW); server.send(200,"OK"); });
  server.on("/horn/on", [](){ digitalWrite(BUZZER,HIGH); server.send(200,"OK"); });
  server.on("/horn/off", [](){ digitalWrite(BUZZER,LOW); server.send(200,"OK"); });

  server.begin();
  Serial.println("System Ready.");
}

void loop() { server.handleClient(); autoPilotLogic(); }