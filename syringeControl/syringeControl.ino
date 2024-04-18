#include <Encoder.h>
#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <initializer_list>

// Dynamixel Setup
const uint8_t DXL_DIR_PIN = 2;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
const uint8_t DXL_ID_M1 = 1;                   // Motor 1
const uint8_t DXL_ID_M2 = 2;                   // Motor 2
const uint8_t DXL_ID_M3 = 3;                   // Motor 3
const int32_t steps = 4096;
const float degreesPerStep = 360.0 / steps;  // Degrees per step
// Limit Switch Setup
const uint8_t LS_M1 = 3;
const uint8_t LS_M2 = 4;
const uint8_t LS_M3 = 5;
bool firstRun = true;
bool limitSwitchPressed(uint8_t pin) {
  return digitalRead(pin) == LOW;  // Assumes LOW when pressed
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Dynamixel Initialization
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  // Connection Tests with Motors
  for (uint8_t id : { DXL_ID_M1, DXL_ID_M2, DXL_ID_M3 }) {
    if (dxl.ping(id)) {
      Serial.print("Motor ");
      Serial.print(id);
      Serial.println(" is connected.");
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_VELOCITY);
      dxl.torqueOn(id);
    } else {
      Serial.print("Failed to connect to Motor ");
      Serial.print(id);
      while (1);
    }
  }
  // Limit Switch Initialization
  pinMode(LS_M1, INPUT_PULLUP);
  pinMode(LS_M2, INPUT_PULLUP);
  pinMode(LS_M3, INPUT_PULLUP);
}

void homingSequence1() {
  bool m1Pressed = limitSwitchPressed(LS_M1);
  while (!m1Pressed) {
    dxl.setGoalVelocity(DXL_ID_M1, 200);
    m1Pressed = limitSwitchPressed(LS_M1);
    Serial.print("loop1");
  }
  while (m1Pressed) {
    dxl.setGoalVelocity(DXL_ID_M1, -10);
    m1Pressed = limitSwitchPressed(LS_M1);
  }
  dxl.setGoalVelocity(DXL_ID_M1, 0);
}

void homingSequence2() {
  bool m2Pressed = limitSwitchPressed(LS_M2);
  while (!m2Pressed) {
    dxl.setGoalVelocity(DXL_ID_M2, 200);
    m2Pressed = limitSwitchPressed(LS_M2);
  }
  while (m2Pressed) {
    dxl.setGoalVelocity(DXL_ID_M2, -10);
    m2Pressed = limitSwitchPressed(LS_M2);
  }
  dxl.setGoalVelocity(DXL_ID_M2, 0);
}



// void homeAll() {
//   homingSequence1();
//   homingSequence2();
//   homingSequence3();
// }

void loop() {
  // Check the state of each limit switch
  bool m1Pressed = limitSwitchPressed(LS_M1);
  bool m2Pressed = limitSwitchPressed(LS_M2);
  bool m3Pressed = limitSwitchPressed(LS_M3);


  if (firstRun) {
    homingSequence1();
    homingSequence2();
    firstRun = false;
  }

  // Get the position values for each motor
  int32_t rawPos1 = dxl.getPresentPosition(DXL_ID_M1);
  // Get the velocity values for each motor
  int32_t rawVel1 = dxl.getPresentVelocity(DXL_ID_M1);

  // Get the position values for each motor
  int32_t rawPos2 = dxl.getPresentPosition(DXL_ID_M2);
  // Get the velocity values for each motor
  int32_t rawVel2 = dxl.getPresentVelocity(DXL_ID_M2);

  // Get the position values for each motor
  int32_t rawPos3 = dxl.getPresentPosition(DXL_ID_M3);
  // Get the velocity values for each motor
  int32_t rawVel3 = dxl.getPresentVelocity(DXL_ID_M3);


  // // BNO055 Orientation
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Output all readings to Serial Monitor in CSV format
  // Positions M1, M2, M3
  // Serial.print(rawPos1);
  // Serial.print(",");
  // Serial.print(rawPos2);
  // Serial.print(",");
  // Serial.print(rawPos3);
  // Serial.print(",");
  // // Velocities M1, M2, M3
  // Serial.print(rawVel1);
  // Serial.print(",");
  // Serial.print(rawVel2);
  // Serial.print(",");
  // Serial.print(rawVel3);
  // Serial.print(",");
  // // Orientations X,Y,Z
  // Serial.print(euler.x());
  // Serial.print(",");
  // Serial.print(euler.y());
  // Serial.print(",");
  // Serial.print(euler.z());
  // Serial.print("\n");

  delay(100);  // Adjust delay as needed
}
