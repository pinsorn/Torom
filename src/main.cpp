#include <Arduino.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>

// ====================================================
//                 CONSTANTS & CONFIG
// ====================================================

// --- Position & Calibration Settings ---
// NOTE: Please adjust these values according to your actual mechanism
#define STEP_PER_100_MicroLitre   305  // Steps required for 100 uL
#define TIP_DISPENSE_POSITION     1400 // Position to dispense tip
#define BLOWOUT_POSITION          1220   // Position for blowout (near home)
#define ASPIRATE_DEFAULT_POSITION 915  // Default ready position
#define MAX_POSITION              1450 // Maximum allowed position

// --- Pin Configuration ---
#define RXD1_PIN 3 
#define TXD1_PIN 4 
#define EN_PIN   5 
#define STEP_PIN 8 
#define DIR_PIN  9 
#define LIMIT_PIN 1 // Single Limit Pin

// --- Stepper Driver Configuration ---
#define R_SENSE 0.11f
#define SERIAL_ADDRESS 0
#define MOTOR_CURRENT_RMS 1300
#define MICROSTEPS 0 // Full step

// --- Calculation ---
#define STEPS_PER_REVOLUTION (200 * (MICROSTEPS > 0 ? MICROSTEPS : 1))
#define MAX_SPEED_DEFAULT STEPS_PER_REVOLUTION * 3
#define MAX_ACCEL_DEFAULT STEPS_PER_REVOLUTION * 1
float LIMIT_COMPENSATION_RATIO = 1.0f; 

// --- JSON Status Codes ---
enum StatusCode {
  CODE_INFO = 100,
  CODE_CMD_LIST = 101,
  CODE_SUCCESS = 200,
  CODE_TARGET_REACHED = 201,
  CODE_WARNING = 300,
  CODE_LIMIT_TRIGGERED = 301,
  CODE_ERROR = 400,
  CODE_INVALID_STATE = 401,
  CODE_POS_EXCEEDED = 402
};

enum ReportType {
  INFO_TYPE,
  WARNING_TYPE,
  ERROR_TYPE,
  SUCCESS_TYPE
};

// ====================================================
//                 GLOBALS
// ====================================================

String serialBuffer = ""; 
boolean commandReady = false;
Stream *cmdSerial = &Serial;

// Driver & Serial Objects
TMC2209Stepper driver(&Serial1, R_SENSE, SERIAL_ADDRESS);
SoftwareSerial swSerial(6, 7); // RX, TX

// ====================================================
//                 HELPER FUNCTIONS
// ====================================================

void displayJSON(ReportType type, String message, int code, String motorName = "") {
  String typeStr;
  switch(type) {
    case INFO_TYPE: typeStr = "INFO"; break;
    case WARNING_TYPE: typeStr = "WARNING"; break;
    case ERROR_TYPE: typeStr = "ERROR"; break;
    case SUCCESS_TYPE: typeStr = "SUCCESS"; break;
  }
  
  String output = "{\"type\":\"" + typeStr + "\"";
  output += ",\"code\":" + String(code);
  output += ",\"message\":\"" + message + "\"";
  if (motorName.length() > 0) {
    output += ",\"motor\":\"" + motorName + "\"";
  }
  output += "}";
  
  Serial.println(output);
  swSerial.println(output);
}

// ====================================================
//                 STEPPER CLASS
// ====================================================

class StepperMotor {
private:
  AccelStepper stepper;
  uint8_t enPin;
  uint8_t limitPin;
  long stepsPerRev;
  bool movementComplete;
  bool limitEnabled;
  bool lastLimitState;
  String motorName;
  
  // For sequence moves (e.g. Blowout -> Default)
  long pendingTarget = -999999; 
  bool hasPendingTarget = false;

public:
  struct Config {
    uint8_t enPin;
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t limitPin;
    String name;
  };

  StepperMotor(const Config &cfg)
      : stepper(AccelStepper::DRIVER, cfg.stepPin, cfg.dirPin),
        enPin(cfg.enPin),
        limitPin(cfg.limitPin),
        movementComplete(true),
        limitEnabled(cfg.limitPin != 0),
        lastLimitState(HIGH),
        motorName(cfg.name) {
    
    pinMode(cfg.enPin, OUTPUT);
    digitalWrite(cfg.enPin, LOW); // Enable by default

    if (cfg.limitPin) {
      pinMode(cfg.limitPin, INPUT_PULLUP);
      displayJSON(INFO_TYPE, "Limit switch enabled on pin " + String(cfg.limitPin), CODE_INFO, motorName);
    }

    stepsPerRev = STEPS_PER_REVOLUTION;
    stepper.setMaxSpeed(MAX_SPEED_DEFAULT);
    stepper.setAcceleration(MAX_ACCEL_DEFAULT);
  }

  void moveTo(long target) {
    // Check Max Position
    if (target > MAX_POSITION) {
      displayJSON(ERROR_TYPE, "Target exceeds MAX_POSITION. Moving to Default.", CODE_POS_EXCEEDED, motorName);
      target = ASPIRATE_DEFAULT_POSITION;
      // Clear any pending sequence to prevent weird loops
      hasPendingTarget = false;
    }

    stepper.moveTo(target);
    movementComplete = false;
  }

  void move(long steps) {
    long target = stepper.currentPosition() + steps;
    moveTo(target);
  }

  // Set a target to go to AFTER the current movement finishes
  void setPendingTarget(long target) {
    pendingTarget = target;
    hasPendingTarget = true;
  }

  void update() {
    // 1. Check Limit Switch
    if (limitEnabled) {
      bool limitState = digitalRead(limitPin);
      
      // Assuming INPUT_PULLUP: HIGH means pressed (Inverted Logic)
      if (limitState == HIGH && lastLimitState == LOW) {
        // If moving towards limit (negative direction usually)
        if (stepper.speed() < 0) {
          emergencyStop();
          displayJSON(WARNING_TYPE, "LIMIT SWITCH TRIGGERED - STEPPING BACK", CODE_LIMIT_TRIGGERED, motorName);
          
          // Bounce back
          stepper.move(STEPS_PER_REVOLUTION * LIMIT_COMPENSATION_RATIO); 
          // Note: This move is relative, so it sets a new target. 
          // We should clear pending targets to be safe.
          hasPendingTarget = false;
        }
      }
      lastLimitState = limitState;
    }
    
    // 2. Run Stepper
    if (stepper.distanceToGo() != 0) {
      stepper.run();
    } else if (!movementComplete) {
      // Movement just finished
      movementComplete = true;
      displayJSON(SUCCESS_TYPE, "Target reached: " + String(stepper.currentPosition()), CODE_TARGET_REACHED, motorName);
      
      // Check if there is a pending sequence (e.g. return to default)
      if (hasPendingTarget) {
        displayJSON(INFO_TYPE, "Executing pending move to " + String(pendingTarget), CODE_INFO, motorName);
        moveTo(pendingTarget);
        hasPendingTarget = false;
      }
    }
  }

  // Blocking Calibration Routine
  void calibrate() {
    displayJSON(INFO_TYPE, "Starting Calibration...", CODE_INFO, motorName);
    
    if (!limitEnabled) {
      displayJSON(ERROR_TYPE, "Cannot calibrate: No limit switch", CODE_ERROR, motorName);
      return;
    }

    float oldMaxSpeed = stepper.maxSpeed(); // Save current max speed

    // 1. Move Negative until Limit Hit (Step by Step)
    // Using runSpeedToPosition for controlled constant speed stepping
    while (digitalRead(limitPin) == LOW) { // While NOT pressed (LOW)
      stepper.move(-1); // Move 1 step
      stepper.setSpeed(-MAX_SPEED_DEFAULT / 2); // Set speed
      while (stepper.distanceToGo() != 0) {
        stepper.runSpeedToPosition();
      }
    }
    
    delay(200);

    // 2. Bounce back until Limit Released (Step by Step)
    while (digitalRead(limitPin) == HIGH) { // While Pressed (HIGH)
      stepper.move(1); // Move 1 step
      stepper.setSpeed(MAX_SPEED_DEFAULT / 5); // Very slow
      while (stepper.distanceToGo() != 0) {
        stepper.runSpeedToPosition();
      }
    }
    
    // 3. Set Home
    stepper.setCurrentPosition(0);
    stepper.moveTo(0); // Clear target
    movementComplete = true;
    hasPendingTarget = false;
    
    stepper.setMaxSpeed(oldMaxSpeed); // Restore previous max speed

    // 4. Re-initialize TMC Driver (Fix for potential disconnect/reset)
    driver.rms_current(MOTOR_CURRENT_RMS);
    driver.microsteps(MICROSTEPS);
    driver.toff(5);
    driver.pdn_disable(true);
    
    displayJSON(SUCCESS_TYPE, "Calibration Complete. Home Set.", CODE_SUCCESS, motorName);
  }

  bool isRunning() { return stepper.distanceToGo() != 0; }
  long getCurrentPosition() { return stepper.currentPosition(); }
  
  void stop() {
    stepper.stop();
    hasPendingTarget = false;
    displayJSON(INFO_TYPE, "Motor stopped", CODE_INFO, motorName);
  }

  void emergencyStop() {
    stepper.setCurrentPosition(stepper.currentPosition());
    hasPendingTarget = false;
  }

  void setHome() {
    stepper.setCurrentPosition(0);
    displayJSON(INFO_TYPE, "Home position set manually", CODE_INFO, motorName);
  }

  void enable() {
    digitalWrite(enPin, LOW);
    displayJSON(INFO_TYPE, "Motor enabled", CODE_INFO, motorName);
  }

  void disable() {
    digitalWrite(enPin, HIGH);
    displayJSON(INFO_TYPE, "Motor disabled", CODE_INFO, motorName);
  }

  void setSpeed(float speed) { 
    stepper.setMaxSpeed(speed * stepsPerRev); 
    displayJSON(INFO_TYPE, "Max speed set to: " + String(speed) + " rev/s", CODE_INFO, motorName);
  }
  
  void setAcceleration(float accel) { 
    stepper.setAcceleration(accel * stepsPerRev); 
    displayJSON(INFO_TYPE, "Acceleration set to: " + String(accel) + " rev/s^2", CODE_INFO, motorName);
  }

  void displayPosition() {
    String output = "{\"type\":\"INFO\",\"code\":" + String(CODE_INFO) + ",\"motor\":\"" + motorName + "\"";
    output += ",\"position\":" + String(stepper.currentPosition());
    output += ",\"revolutions\":" + String((float)stepper.currentPosition() / stepsPerRev, 2);
    output += ",\"status\":\"" + String(isRunning() ? "MOVING" : "IDLE") + "\"";
    if (limitEnabled) {
       output += ",\"limitPressed\":" + String(digitalRead(limitPin) == HIGH ? "true" : "false");
    }
    output += "}";
    Serial.println(output);
    swSerial.println(output);
  }
  
  String getName() { return motorName; }
};

// ====================================================
//                 INSTANTIATION
// ====================================================

StepperMotor::Config M1_Config = {
  EN_PIN,
  STEP_PIN, 
  DIR_PIN,
  LIMIT_PIN,
  "Motor1"
};

StepperMotor motorX(M1_Config);

// ====================================================
//                 COMMAND PROCESSING
// ====================================================

void printHelp() {
  String help = "{\"type\":\"INFO\",\"code\":" + String(CODE_CMD_LIST) + ",\"name\":\"P200 Syringe Pipette Module C3\",\"commands\":[";
  help += "{\"cmd\":\"c\",\"inputType\":\"none\",\"desc\":\"Calibrate (Auto Home)\",\"isQueue\":true,\"success\":" + String(CODE_SUCCESS) + ",\"errors\":[" + String(CODE_ERROR) + "]},";
  help += "{\"cmd\":\"u<val>\",\"inputType\":\"float\",\"desc\":\"Aspirate x uL (e.g. u100)\",\"isQueue\":true,\"success\":" + String(CODE_TARGET_REACHED) + ",\"errors\":[" + String(CODE_ERROR) + "," + String(CODE_INVALID_STATE) + "," + String(CODE_POS_EXCEEDED) + "]},";
  help += "{\"cmd\":\"b\",\"inputType\":\"none\",\"desc\":\"Blowout sequence\",\"isQueue\":true,\"success\":" + String(CODE_TARGET_REACHED) + ",\"errors\":[" + String(CODE_ERROR) + "]},";
  help += "{\"cmd\":\"k\",\"inputType\":\"none\",\"desc\":\"Tip Dispense sequence\",\"isQueue\":true,\"success\":" + String(CODE_TARGET_REACHED) + ",\"errors\":[" + String(CODE_ERROR) + "]},";
  help += "{\"cmd\":\"u\",\"inputType\":\"none\",\"desc\":\"Go to Default Position\",\"isQueue\":true,\"success\":" + String(CODE_TARGET_REACHED) + ",\"errors\":[" + String(CODE_ERROR) + "]},";
  help += "{\"cmd\":\"a<val>\",\"inputType\":\"float\",\"desc\":\"Set Acceleration (rev/s^2)\",\"isQueue\":false,\"success\":" + String(CODE_INFO) + ",\"errors\":[" + String(CODE_ERROR) + "]},";
  help += "{\"cmd\":\"x<val>\",\"inputType\":\"float\",\"desc\":\"Set Max Speed (rev/s)\",\"isQueue\":false,\"success\":" + String(CODE_INFO) + ",\"errors\":[" + String(CODE_ERROR) + "]},";
  help += "{\"cmd\":\"i<val>\",\"inputType\":\"float\",\"desc\":\"Set Limit Compensation Ratio\",\"isQueue\":false,\"success\":" + String(CODE_INFO) + ",\"errors\":[" + String(CODE_ERROR) + "]},";
  help += "{\"cmd\":\"s\",\"inputType\":\"none\",\"desc\":\"Stop\",\"isQueue\":false,\"success\":" + String(CODE_INFO) + ",\"errors\":[]},";
  help += "{\"cmd\":\"e\",\"inputType\":\"none\",\"desc\":\"Emergency Stop\",\"isQueue\":false,\"success\":" + String(CODE_INFO) + ",\"errors\":[]},";
  help += "{\"cmd\":\"p\",\"inputType\":\"none\",\"desc\":\"Get Position\",\"isQueue\":false,\"success\":" + String(CODE_INFO) + ",\"errors\":[]},";
  help += "{\"cmd\":\"d\",\"inputType\":\"none\",\"desc\":\"Display Detailed Status\",\"isQueue\":false,\"success\":" + String(CODE_INFO) + ",\"errors\":[]},";
  help += "{\"cmd\":\"?\",\"inputType\":\"none\",\"desc\":\"Print Help\",\"isQueue\":false,\"success\":" + String(CODE_SUCCESS) + ",\"errors\":[]}";
  help += "]}";
  Serial.println(help);
  swSerial.println(help);
  
  //displayJSON(SUCCESS_TYPE, "Help sent", CODE_SUCCESS);
}

void processCommand(String input, boolean motorStatus) {
  input.trim();
  String command = input;
  
  // Handle prefix "1:" if present (legacy support)
  if (input.startsWith("1:")) {
    command = input.substring(2);
  }

  // --- New Commands ---

  if (command.equalsIgnoreCase("?")) {
    printHelp();
    return;
  }
  else if (command.equalsIgnoreCase("c")) {
    if (motorStatus) {
      displayJSON(ERROR_TYPE, "Cannot calibrate while running", CODE_ERROR);
      return;
    }
    motorX.calibrate();
  }
  else if (command.equalsIgnoreCase("u")) {
    if (motorStatus) {
      displayJSON(ERROR_TYPE, "Busy", CODE_ERROR);
      return;
    }
    motorX.moveTo(ASPIRATE_DEFAULT_POSITION);
  }
  else if (command.startsWith("u")) {
    if (motorStatus) {
      displayJSON(ERROR_TYPE, "Busy", CODE_ERROR);
      return;
    }
    
    // Check if at Aspirate Default Position (allow small error margin)
    if (abs(motorX.getCurrentPosition() - ASPIRATE_DEFAULT_POSITION) > 50) {
      displayJSON(ERROR_TYPE, "Must be at Aspirate Default Position to Aspirate", CODE_INVALID_STATE);
      return;
    }

    String valStr = command.substring(1);
    float uL = valStr.toFloat();
    long steps = (long)((uL / 100.0f) * STEP_PER_100_MicroLitre);
    
    // Aspirate usually means pulling plunger UP/BACK (Positive steps?) 
    // or DOWN? Assuming Positive increases volume inside tip.
    // Target = Current + Steps
    motorX.move(-steps); 
    displayJSON(INFO_TYPE, "Aspirating " + String(uL) + " uL (" + String(steps) + " steps)", CODE_INFO);
  }
  else if (command.equalsIgnoreCase("b")) {
    if (motorStatus) {
      displayJSON(ERROR_TYPE, "Busy", CODE_ERROR);
      return;
    }
    displayJSON(INFO_TYPE, "Starting Blowout Sequence", CODE_INFO);
    motorX.moveTo(BLOWOUT_POSITION);
    motorX.setPendingTarget(ASPIRATE_DEFAULT_POSITION);
  }
  else if (command.equalsIgnoreCase("k")) {
    if (motorStatus) {
      displayJSON(ERROR_TYPE, "Busy", CODE_ERROR);
      return;
    }
    displayJSON(INFO_TYPE, "Starting Tip Dispense Sequence", CODE_INFO);
    motorX.moveTo(TIP_DISPENSE_POSITION);
    motorX.setPendingTarget(ASPIRATE_DEFAULT_POSITION);
  }

  else if (command.startsWith("a")) {
    if (motorStatus) {
      displayJSON(ERROR_TYPE, "Cannot change acceleration while running", CODE_ERROR);
      return;
    }
    float accel = command.substring(1).toFloat();
    motorX.setAcceleration(accel);
  }
  else if (command.startsWith("x")) {
    if (motorStatus) {
      displayJSON(ERROR_TYPE, "Cannot change speed while running", CODE_ERROR);
      return;
    }
    float speed = command.substring(1).toFloat();
    motorX.setSpeed(speed);
  }
  else if (command.startsWith("i")) {
    if (motorStatus) {
      displayJSON(ERROR_TYPE, "Cannot change limit compensation while running", CODE_ERROR);
      return;
    }
    LIMIT_COMPENSATION_RATIO = command.substring(1).toFloat();
    displayJSON(INFO_TYPE, "Limit Compensation Ratio set to: " + String(LIMIT_COMPENSATION_RATIO), CODE_INFO);
  }
  
  // --- Legacy/Basic Commands ---
  
  else if (command.equalsIgnoreCase("s")) {
    motorX.stop();
  }
  else if (command.equalsIgnoreCase("e")) {
    motorX.emergencyStop();
  }
  else if (command.equalsIgnoreCase("h")) {
    motorX.setHome();
  }
  else if (command.equalsIgnoreCase("p")) {
    motorX.displayPosition();
  }
  else if (command.equalsIgnoreCase("d")) {
    motorX.displayPosition();
  }
  else if (command.equalsIgnoreCase("on")) {
    motorX.enable();
  }
  else if (command.equalsIgnoreCase("off")) {
    motorX.disable();
  }
  // Direct Move: +100, -100, 1000
  else if (command.startsWith("+")) {
    if(motorStatus) { displayJSON(ERROR_TYPE, "Busy", CODE_ERROR); return; }
    motorX.move(command.substring(1).toInt());
  }
  else if (command.startsWith("-")) {
    if(motorStatus) { displayJSON(ERROR_TYPE, "Busy", CODE_ERROR); return; }
    motorX.move(command.toInt()); // toInt handles negative sign
  }
  else if (command.toInt() != 0 || command == "0") { // Numeric absolute move
    if(motorStatus) { displayJSON(ERROR_TYPE, "Busy", CODE_ERROR); return; }
    motorX.moveTo(command.toInt());
  }
  else {
    displayJSON(ERROR_TYPE, "Unknown Command", CODE_ERROR);
  }
}

// ====================================================
//                 SETUP & LOOP
// ====================================================

void setup() {
  // 1. Init Serials
  Serial.begin(115200); 
  swSerial.begin(9600);
  Serial1.begin(115200, SERIAL_8N1, RXD1_PIN, TXD1_PIN);

  // 2. Startup Delay
  delay(1000); 

  // 3. Init Driver
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); 
  
  driver.begin();
  driver.rms_current(MOTOR_CURRENT_RMS);
  driver.microsteps(MICROSTEPS);
  driver.toff(5);
  driver.pdn_disable(true);

  Serial.println("=== Single Motor Controller Started ===");
  
  // 4. Send Available Commands JSON
  printHelp();
}

void loop() {
  // 1. Update Motor
  motorX.update();
  boolean isRunning = motorX.isRunning();

  // 2. Read Software Serial
  if (swSerial.available()) {
    String receivedMessage = swSerial.readStringUntil('\n');
    if (receivedMessage.length() > 0) {
      Serial.print("CMD Received: ");
      Serial.println(receivedMessage);
      serialBuffer = receivedMessage;
      commandReady = true;
    }
  }
  
  // 3. Read USB Serial (Optional, for debugging)
  if (Serial.available()) {
    String receivedMessage = Serial.readStringUntil('\n');
    if (receivedMessage.length() > 0) {
      serialBuffer = receivedMessage;
      commandReady = true;
    }
  }

  // 4. Process Command
  if (commandReady) {
    processCommand(serialBuffer, isRunning);
    serialBuffer = "";
    commandReady = false;
  }
}

