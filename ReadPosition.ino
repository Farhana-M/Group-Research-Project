#include <Dynamixel2Arduino.h>

const uint8_t DXL_DIR_PIN = 2;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);

const uint8_t Motor_1 = 1;  // Motor ID
const float res = 4096.0;  // Resolution of the motor
const float degreesPerStep = 360.0 / res;  // Degrees per step

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to connect
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  // Test connection with Motor
  if (dxl.ping(Motor_1)) {
    Serial.println("Motor 1 is connected!");
  } else {
    Serial.println("Failed to connect to Motor 1.");
    return; 
  }
}

void loop() {
  int32_t rawPos = dxl.getPresentPosition(Motor_1);  // Get raw motor position
  float degPos = rawPos * degreesPerStep; // Convert into degrees
  Serial.println(positionInDegrees);  // Print position in degrees
  delay(10);  // Delay 1 second
}
