#include <Encoder.h>
#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <initializer_list>

// Dynamixel Setup
const uint8_t DXL_DIR_PIN = 2;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
const uint8_t DXL_ID_M1 = 1;  // Motor 1
const uint8_t DXL_ID_M2 = 2;  // Motor 2
const uint8_t DXL_ID_M3 = 3;  // Motor 3
const int32_t steps = 4096;
const float degreesPerStep = 360.0 / steps;  // Degrees per step

// Limit Switch Setup
const uint8_t LS_M1 = 3;
const uint8_t LS_M2 = 4;
const uint8_t LS_M3 = 5;
bool firstRun = true;  // flag for the homing system

// Global variables for maximum positions
int32_t maxPosM1, maxPosM2, maxPosM3;

bool limitSwitchPressed(uint8_t pin) {
  return digitalRead(pin) == LOW;  // Assumes LOW when pressed
}

void motorHoming(int motorID, int LS_ID) {
  bool switchPressed = limitSwitchPressed(LS_ID);
  while (!switchPressed) {
    dxl.setGoalVelocity(motorID, 200);
    switchPressed = limitSwitchPressed(LS_ID);
  }
  while (switchPressed) {
    dxl.setGoalVelocity(motorID, -10);
    switchPressed = limitSwitchPressed(LS_ID);
  }
  dxl.setGoalVelocity(motorID, 0);
}

void homingSequence() {
  motorHoming(DXL_ID_M1, LS_M1);
  motorHoming(DXL_ID_M2, LS_M2);
  motorHoming(DXL_ID_M3, LS_M3);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // Dynamixel Initialization
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  // Connection Tests with Motors
  for (uint8_t id : { DXL_ID_M1, DXL_ID_M2, DXL_ID_M3 }) {
    if (dxl.ping(id)) {
      Serial.println("Motor " + String(id) + " is connected.");
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_VELOCITY);
      dxl.torqueOn(id);
    } else {
      Serial.print("Failed to connect to Motor " + String(id));
      while (1)
        ;
    }
  }
  // Limit Switch Initialization
  pinMode(LS_M1, INPUT_PULLUP);
  pinMode(LS_M2, INPUT_PULLUP);
  pinMode(LS_M3, INPUT_PULLUP);
}

void loop() {
  if (firstRun) {
    homingSequence();
    firstRun = false;
    // Get the position values for each motor after homing
    maxPosM1 = dxl.getPresentPosition(DXL_ID_M1);
    maxPosM2 = dxl.getPresentPosition(DXL_ID_M2);
    maxPosM3 = dxl.getPresentPosition(DXL_ID_M3);
  }

  int32_t currentPosM1 = dxl.getPresentPosition(DXL_ID_M1);
  int32_t currentPosM2 = dxl.getPresentPosition(DXL_ID_M2);
  int32_t currentPosM3 = dxl.getPresentPosition(DXL_ID_M3);

  // Check if current position exceeds max position and handle accordingly
  if (currentPosM1 > maxPosM1) {
    dxl.setGoalVelocity(DXL_ID_M1, 0);
    Serial.println("Motor 1 exceeded max position, stopping motor.");
  }
  if (currentPosM2 > maxPosM2) {
    dxl.setGoalVelocity(DXL_ID_M2, 0);
    Serial.println("Motor 2 exceeded max position, stopping motor.");
  }
  if (currentPosM3 > maxPosM3) {
    dxl.setGoalVelocity(DXL_ID_M3, 0);
    Serial.println("Motor 3 exceeded max position, stopping motor.");
  }

  delay(100);  // Adjust delay as needed
}
