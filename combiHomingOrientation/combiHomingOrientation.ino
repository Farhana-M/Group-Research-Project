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
// Orientation Setup
const float r = 6.6;                // length in cm
const float theta_x_desired = 5.0;  //bending aboout x axis (pitch)
const float theta_y_desired = 5.0;  //bending about y axis (roll)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// PID Gains
float Kp = 5.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.0;  // Derivative gain
// PID delta L
float lastDeltaL[3] = { 0, 0, 0 };      // Last delta L for PID feedback
float integralDeltaL[3] = { 0, 0, 0 };  // Integral of delta L for PID
// Pseudo-inverse Jacobian
float pseudo_inv_jacobian[3][2] = {
  { 2 / (3.0 * r), 0 },
  { -1 / (3.0 * r), -sqrt(3) / (3.0 * r) },
  { -1 / (3.0 * r), sqrt(3) / (3.0 * r) }
};
unsigned long lastTime;
// Global variables for maximum and minimum positions
int32_t maxPosM1, maxPosM2, maxPosM3;
int32_t minPosM1, minPosM2, minPosM3;

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

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  // Homing all motors before starting the main loop
  homingSequence();

  // Establish motor boundaries post-homing
  maxPosM1 = dxl.getPresentPosition(DXL_ID_M1);
  minPosM1 = maxPosM1 - leadLengthM1;
  maxPosM2 = dxl.getPresentPosition(DXL_ID_M2);
  minPosM2 = maxPosM2 - leadLengthM2;
  maxPosM3 = dxl.getPresentPosition(DXL_ID_M3);
  minPosM3 = maxPosM3 - leadLengthM3;

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
    while (1)
      ;
  }
  bno.setExtCrystalUse(true);

  lastTime = millis();

  pinMode(LS_M1, INPUT_PULLUP);
  pinMode(LS_M2, INPUT_PULLUP);
  pinMode(LS_M3, INPUT_PULLUP);
}

void loop() {
  // Main control loop
  while (true) {
    int32_t currentPosM1 = dxl.getPresentPosition(DXL_ID_M1);
    int32_t currentPosM2 = dxl.getPresentPosition(DXL_ID_M2);
    int32_t currentPosM3 = dxl.getPresentPosition(DXL_ID_M3);

    // Check if current position exceeds maximum or minimum position and handle accordingly
    if (currentPosM1 > maxPosM1 || currentPosM1 < minPosM1 || currentPosM2 > maxPosM2 || currentPosM2 < minPosM2 || currentPosM3 > maxPosM3 || currentPosM3 < minPosM3) {
      dxl.setGoalVelocity(DXL_ID_M1, 0);
      dxl.setGoalVelocity(DXL_ID_M2, 0);
      dxl.setGoalVelocity(DXL_ID_M3, 0);
      Serial.println("Motor outside safe boundaries, stopping motor.");
      continue;  // Optionally, add further handling here
    }

    unsigned long now = millis();
    float timeChange = (float)(now - lastTime) / 1000.0;  // Time in seconds

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float theta_x_measured = euler.x();
    float theta_y_measured = euler.y();
    //Serial.println(theta_x_measured);
    //Serial.println(theta_y_measured);

    // Calculate the errors
    float theta_errors[2][1] = { { theta_x_desired - theta_x_measured }, { theta_y_desired - theta_y_measured } };
    //Serial.print("Theta X Error: ");
    //Serial.println(theta_errors[0][0]);
    //Serial.print("Theta Y Error: ");
    //Serial.println(theta_errors[1][0]);
    // Compute initial delta L based on orientation errors
    float currentDeltaL[3] = { 0 };
    for (int i = 0; i < 3; i++) {
      for (int k = 0; k < 2; k++) {
        currentDeltaL[i] += pseudo_inv_jacobian[i][k] * theta_errors[k][0];
        Serial.println(currentDeltaL[i]);
      }
    }

    // Apply PID control to the change in delta L
    float deltaV[3] = { 0 };  // This will store the PID adjusted delta L
    for (int i = 0; i < 3; i++) {
      float error = currentDeltaL[i] - lastDeltaL[i];
      integralDeltaL[i] += error * timeChange;
      float derivative = (error - (lastDeltaL[i] - currentDeltaL[i])) / timeChange;
      deltaV[i] = Kp * error + Ki * integralDeltaL[i] + Kd * derivative;
      //Serial.println(deltaV[0]);
      lastDeltaL[i] = currentDeltaL[i];  // Update last delta V for next iteration
    }

    for (int i = 0; i < 3; i++) {
      int32_t currentPos = dxl.getPresentPosition(i + 1, UNIT_DEGREE);  // Assuming DXL_IDs are 1, 2, 3
      //Serial.println(currentPos);
      int32_t newPos = currentPos + static_cast<int32_t>(deltaV[i]);
      //Serial.println(newPos);
      //int32_t newPos = currentPos + static_cast<int32_t>(deltaV[i] * degreesPerStep); // Convert deltaV to steps and adjust
      dxl.setGoalPosition(i + 1, newPos);
      //Serial.println(newPos);
    }

    //Serial.println("Delta V: ");
    //for (int i = 0; i < 3; i++) {
    //  Serial.println(deltaV[i], 6);
    //}

    lastTime = now;
    //delay(100); // Adjust based on your control loop requirements
  }
}
