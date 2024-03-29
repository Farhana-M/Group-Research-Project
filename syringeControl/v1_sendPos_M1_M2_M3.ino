#include <Encoder.h>
#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Dynamixel Setup
const uint8_t DXL_DIR_PIN = 2;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
const uint8_t DXL_ID_M1 = 1;  // Motor 1
const uint8_t DXL_ID_M2 = 2;  // Motor 2
const uint8_t DXL_ID_M3 = 3;  // Motor 3
const float res = 4096.0;  // Resolution of the motor
const float degreesPerStep = 360.0 / res;  // Degrees per step


// BNO055 setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to connect

  // Dynamixel Initialisation
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  
  // Test connection with M1
  if (dxl.ping(DXL_ID_M1)) {
    Serial.println("M1 is connected!");
  } else {
    Serial.println("Failed to connect to M1.");
    while (1); // Stop if connection fails
  }
  dxl.torqueOff(DXL_ID_M1);
  dxl.setOperatingMode(DXL_ID_M1, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_M1);
  
  // Test connection with M2
  if (dxl.ping(DXL_ID_M2)) {
    Serial.println("M2 is connected!");
  } else {
    Serial.println("Failed to connect to M2.");
    while (1); // Stop if connection fails
  }
  dxl.torqueOff(DXL_ID_M2);
  dxl.setOperatingMode(DXL_ID_M2, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_M2);
  
  // Test connection with M3
  if (dxl.ping(DXL_ID_M3)) {
    Serial.println("M3 is connected!");
  } else {
    Serial.println("Failed to connect to M3.");
    while (1); // Stop if connection fails
  }
  dxl.torqueOff(DXL_ID_M3);
  dxl.setOperatingMode(DXL_ID_M3, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_M3);
  

  //BNO055 IMU Initialisation
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
  dxl.setGoalPosition(DXL_ID_M2, desPosM2, UNIT_RAW);
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

  // Output all readings to Serial Monitor
  // M1 Outputs
  Serial.print("M1 || Position : ");
  Serial.print(rawPos1);
  Serial.print(" | Velocity: ");
  Serial.println(rawVel1);
  delay(500);  // Delay 0.5 second
  
  // M2 Outputs
  Serial.print("M2 || Position : ");
  Serial.print(rawPos2);
  Serial.print(" | Velocity: ");
  Serial.println(rawVel2);
  delay(500);  // Delay 0.5 second

  // M3 Outputs
  Serial.print("M3 || Position : ");
  Serial.print(rawPos3);
  Serial.print(" | Velocity: ");
  Serial.println(rawVel3);
  delay(500);  // Delay 0.5 second

  // BNO055 Outputs
  Serial.print("IMU || Yaw(X): ");
  Serial.print(euler.x());
  Serial.print(" | Pitch(Y): ");
  Serial.print(euler.y());
  Serial.print(" | Roll(Z): ");
  Serial.println(euler.z());
  delay(500);  // Delay 0.5 second

}