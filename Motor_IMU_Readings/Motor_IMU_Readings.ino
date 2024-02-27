#include <Encoder.h>
#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Dynamixel Setup
const uint8_t DXL_DIR_PIN = 2;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
const uint8_t M1 = 1;  // Motor 1
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
  dxl.torqueOff(M1);
  dxl.setOperatingMode(M1, OP_EXTENDED_POSITION);
  dxl.torqueOn(M1);
  

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
  const float desPosM1 = -5000;
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
