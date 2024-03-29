#include <Encoder.h>
#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Dynamixel Setup
const uint8_t DXL_DIR_PIN = 2;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
const uint8_t M1 = 1;  // Motor 1
const uint8_t M2 = 2;  // Motor 2
const uint8_t M3 = 3;  // Motor 3
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
  if (dxl.ping(M1)) {
    Serial.println("M1 is connected!");
  } else {
    Serial.println("Failed to connect to M1.");
    while (1); // Stop if connection fails
  }
  if (dxl.ping(M2)) {
    Serial.println("M2 is connected!");
  } else {
    Serial.println("Failed to connect to M2.");
    while (1); // Stop if connection fails
  }
  if (dxl.ping(M3)) {
    Serial.println("M3 is connected!");
  } else {
    Serial.println("Failed to connect to M3.");
    while (1); // Stop if connection fails
  }
  dxl.torqueOff(M3);
  dxl.setOperatingMode(M1, OP_EXTENDED_POSITION);
  dxl.torqueOn(M3);

  //dxl.writeControlTableItem(PROFILE_VELOCITY, M1, 30);
  
  // Move motor to initial position of 0
  // dxl.setGoalPosition(M1, 0);
  // delay(1000); // Wait for the motor to reach the initial position
  

  //BNO055 Initialisation
  if (!bno.begin()){
    Serial.println("No BNO055 detected.");
    while (1); // Stop if connection fails
  }
  delay(1000); // Delay loop to complete setup
}

void loop() {
  // Dynamixel Position and Velocities
  // Edit to change desired position
  const float desPosM1 = -4096;
  // Move motor to desired position
  dxl.setGoalPosition(M1, desPosM1);
  // Get the position values for each motor
  int32_t rawPos1 = dxl.getPresentPosition(M1);
  // Get the velocity values for each motor
  int32_t rawVel1 = dxl.getPresentVelocity(M1);

  // BNO055 Orientation
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Output all readings to Serial Monitor
  // M1 Outputs
  Serial.print("M1 || Position : ");
  Serial.print(rawPos1);
  Serial.print(" | Velocity: ");
  Serial.println(rawVel1);
  // BNO055 Outputs
  Serial.print("IMU || Yaw(X): ");
  Serial.print(euler.x());
  Serial.print(" | Pitch(Y): ");
  Serial.print(euler.y());
  Serial.print(" | Roll(Z): ");
  Serial.println(euler.z());

  delay(100);  // Delay 1 second
}
