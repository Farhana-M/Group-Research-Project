#include <Dynamixel2Arduino.h>

const uint8_t DXL_DIR_PIN = 2;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);

const uint8_t Motor_1 = 1;  // Motor ID
const uint8_t Motor_2 = 2;  // Motor ID
const uint8_t Motor_3 = 3;  // Motor ID
const float res = 4096.0;  // Resolution of the motor
const float degreesPerStep = 360.0 / res;  // Degrees per step

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to connect
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  
  // Test connection with Motor 1
  if (dxl.ping(Motor_1)) {
    Serial.println("Motor 1 is connected!");
  } else {
    Serial.println("Failed to connect to Motor 1.");
    return; 
  }
  // Test connection with Motor 2
  if (dxl.ping(Motor_2)) {
    Serial.println("Motor 2 is connected!");
  } else {
    Serial.println("Failed to connect to Motor 2.");
    return; 
  }
  // Test connection with Motor 3
  if (dxl.ping(Motor_3)) {
    Serial.println("Motor 3 is connected!");
  } else {
    Serial.println("Failed to connect to Motor 3.");
    return; 
  }
}

void loop() {
  int32_t rawPos1 = dxl.getPresentPosition(Motor_1);
  int32_t rawPos2 = dxl.getPresentPosition(Motor_2);
  int32_t rawPos3 = dxl.getPresentPosition(Motor_3);

  float degPos1 = rawPos1 * degreesPerStep;
  float degPos2 = rawPos2 * degreesPerStep;
  float degPos3 = rawPos3 * degreesPerStep;
  Serial.print(degPos1);
  Serial.print(" || ");
  Serial.print(degPos2);
  Serial.print(" || ");
  Serial.println(degPos3);
  delay(10);  // Delay 1 second
}
