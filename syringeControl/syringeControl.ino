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
const float res = 4096.0;  // Resolution of the motor
const float degreesPerStep = 360.0 / res;  // Degrees per step
// Limit Switch Setup
const uint8_t LS_M1 = 3;
const uint8_t LS_M2 = 4;
const uint8_t LS_M3 = 5;
// BNO055 setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);


void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to connect

  // Dynamixel Initialisation
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  
  // Connection Tests with Motors
  for (uint8_t id: {DXL_ID_M1, DXL_ID_M2, DXL_ID_M3}) {
    if (dxl.ping(id)) {
      Serial.print("Motor ");
      Serial.print(id);
      Serial.println(" is connected.");
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_EXTENDED_POSITION);
      dxl.torqueOn(id);
    } else {
      Serial.print("Failed to connect to Motor ")
      Serial.print(id);
      while(1);
    }
  }

  // Limit Switch Initialisation
  pinMode(LS_M1, INPUT_PULLUP);
  pinMode(LS_M2, INPUT_PULLUP);
  pinMode(LS_M3, INPUT_PULLUP);

  // BNO055 IMU Initialisation
  if (!bno.begin()){
    Serial.println("No BNO055 detected.");
    while (1); // Stop if connection fails
  }
  delay(1000); // Delay loop to complete setup
}

void loop() {
  
  // Dynamixel Position and Velocities
  // Edit to change desired position of M1
  const float desPosM1 = 1000;
  // Move motor to desired position
  dxl.setGoalPosition(DXL_ID_M1, desPosM1);
  // Get the position values for each motor
  int32_t rawPos1 = dxl.getPresentPosition(DXL_ID_M1);
  // Get the velocity values for each motor
  int32_t rawVel1 = dxl.getPresentVelocity(DXL_ID_M1);
  
  // Edit to change desired position of M2
  const float desPosM2 = 1000;
  // Move motor to desired position
  dxl.setGoalPosition(DXL_ID_M2, desPosM2);
  // Get the position values for each motor
  int32_t rawPos2 = dxl.getPresentPosition(DXL_ID_M2);
  // Get the velocity values for each motor
  int32_t rawVel2 = dxl.getPresentVelocity(DXL_ID_M2);

  // Edit to change desired position of M3
  const float desPosM3 = 1000;
  // Move motor to desired position
  dxl.setGoalPosition(DXL_ID_M3, desPosM3);
  // Get the position values for each motor
  int32_t rawPos3 = dxl.getPresentPosition(DXL_ID_M3);
  // Get the velocity values for each motor
  int32_t rawVel3 = dxl.getPresentVelocity(DXL_ID_M3);

  // BNO055 Orientation
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Output all readings to Serial Monitor in CSV format
  // Positions M1, M2, M3
  Serial.print(rawPos1); 
  Serial.print(",");
  Serial.print(rawPos2);
  Serial.print(",");
  Serial.print(rawPos3); 
  Serial.print(",");
  // Velocities M1, M2, M3
  Serial.print(rawVel1);
  Serial.print(",")
  Serial.print(rawVel2);
  Serial.print(",")
  Serial.print(rawVel3);
  Serial.print(",")
  // Orientations X,Y,Z
  Serial.print(euler.x());
  Serial.print(",");
  Serial.print(euler.y());
  Serial.print(",");
  Serial.print(euler.z());
  Serial.print(";");

  delay(500);

}