#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

const float r = 6.6;
const float theta_x_desired = 0.0;
const float theta_y_desired = 20.0;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

float pseudo_inv_jacobian[3][2] = {
    {2/(3.0*r), 0},
    {-1/(3.0*r), -sqrt(3)/(3.0*r)},
    {-1/(3.0*r), sqrt(3)/(3.0*r)}
};

float theta_errors[2][1];

void setup() {
  Serial.begin(9600);
  if (!bno.begin()) {
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  // Get the orientation data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  float theta_x_measured = euler.x();
  float theta_y_measured = euler.y();

  // Calculate the errors
  theta_errors[0][0] = theta_x_desired - theta_x_measured;
  theta_errors[1][0] = theta_y_desired - theta_y_measured;

  // Placeholder for the result of the matrix multiplication
  float delta_V_dot[3][1] = {0};

  // Perform matrix multiplication
  for(int i = 0; i < 3; i++) { // Row number of output
    for(int j = 0; j < 1; j++) { // Column number of output
      delta_V_dot[i][j] = 0; // Initialize the element
      for(int k = 0; k < 2; k++) { // Elements in row/column
        delta_V_dot[i][j] += pseudo_inv_jacobian[i][k] * theta_errors[k][j];
      }
    }
  }

  // Print the result
  Serial.print("Delta V_dot: ");
  for(int i = 0; i < 3; i++) {
    Serial.println(delta_V_dot[i][0], 6);
  }

  // Delay between readings
  delay(1000);
}
