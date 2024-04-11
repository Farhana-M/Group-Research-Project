//**************************************************************************
// RTOS for needle tracking using soft robotics
// by Estya Nadya Meitavany
// 
// Acknowledgement: 
// FreeRtos on Samd21 by Scott Briscoe
//
//**************************************************************************

#include <FreeRTOS_SAMD21.h>
#include <Dynamixel2Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Variables for serial communication
String pos = ""; 
char msgread;

// Variables for Dynamixel
const uint8_t DXL_DIR_PIN = 2;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
const uint8_t M1 = 1;  // Motor 1
const uint8_t M2 = 2;  // Motor 2
const uint8_t M3 = 3;  // Motor 3
// Set operating mode of each motor to control position based on current
using namespace ControlTableItem;

// Variables for BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire); 

// Variables for microcontroller
// #define  ERROR_LED_PIN  13 //Led Pin: Typical Arduino Board
#define  ERROR_LED_PIN  2 //Led Pin: samd21 xplained board
#define ERROR_LED_LIGHTUP_STATE  HIGH // the state that makes the led light up on your board, either low or high
// Select the serial port the project should use and communicate over
// Some boards use SerialUSB, some use Serial
// #define SERIAL          SerialUSB //Sparkfun Samd21 Boards
#define SERIAL          Serial //Adafruit, other Samd21 Boards

// Variables for motor control
float x = 0.0;
float y = 0.0;

//Variables for RTOS
TaskHandle_t Handle_readPosM1Task;
TaskHandle_t Handle_readPosM2Task;
TaskHandle_t Handle_readPosM3Task;
TaskHandle_t Handle_readDemandedPosTask;
TaskHandle_t Handle_readIMUTask;
TaskHandle_t Handle_motorControlTask;
TaskHandle_t Handle_aTask;
TaskHandle_t Handle_bTask;
TaskHandle_t Handle_monitorTask;

// RTOS delay 
void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}

// RTOS tasks
static void readPosM1(void *pvParameters)
{
  while(1)
  {
    // Print the time when this task was started
    // TickType_t startTime = xTaskGetTickCount(); // Record the start time
    // Serial.print("Task readPosM1 started at tick count: ");
    // Serial.println(startTime);

    int32_t rawPos1 = dxl.getPresentPosition(M1);
    Serial.println("M1 Pos: ");
    Serial.println(rawPos1);
    myDelayMs(3000);
  }
}

static void readPosM2(void *pvParameters)
{
  while(1)
  {
    myDelayMs(1000);
    int32_t rawPos2 = dxl.getPresentPosition(M2);
    Serial.println("M2 Pos: ");
    Serial.println(rawPos2);
    myDelayMs(3000);
  }
}

static void readPosM3(void *pvParameters)
{
  while(1)
  {
    myDelayMs(2000);
    int32_t rawPos3 = dxl.getPresentPosition(M3);
    Serial.println("M3 Pos: ");
    Serial.println(rawPos3);
    myDelayMs(3000);
  }
}

// static void readDemandedPos(void *pvParameters)
// // Read the demanded needle position
// {
//   while(1)
//   {
//     if SERIAL.available()>0
//     {
//       msgRead = Serial.read();
//       demandedPos = demandedPos + msgRead;
//       int commaPos = pos.indexOf(",");
//       int semicolonPos = pos.indexOf(";");
//       x = demandedPos.substring(1, commaPos).toFloat();
//       y = demandedPos.substring(commaPos+1, semicolonPos);
//       SERIAL.println("X:")
//       SERIAL.println(x)
//       SERIAL.println("Y:")
//       SERIAL.println(y)


//     }
//     myDelayMs(3000);
//   }
// }

static void readIMU(void *pvParameters)
{
  while(1)
  {
    // BNO055 Orientation
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    // BNO055 Outputs
    Serial.println("Yaw(X): ");
    // Serial.print(euler.x());
    myDelayMs(1000);

    Serial.println("Pitch(Y): ");
    // Serial.print(euler.y());
    myDelayMs(1000);

    Serial.println("Roll(Z): ");
    // Serial.println(euler.z());  
    myDelayMs(1000);
  }
}

static void motorControl(void *pvParameters)
{
  while(1)
  {
    Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
    
    // read current position of dynamixel motors
    float currentPosM1 = dxl.getPresentPosition(M1, UNIT_DEGREE);
    float currentPosM2 = dxl.getPresentPosition(M2, UNIT_DEGREE);
    float currentPosM3 = dxl.getPresentPosition(M3, UNIT_DEGREE);

    // process demanded position in task-space into motor-space

    // send demanded position in motor-space
    dxl.setGoalPosition(M1, 0.0, UNIT_DEGREE);
    dxl.setGoalPosition(M2, 0.0, UNIT_DEGREE);
    dxl.setGoalPosition(M3, 0.0, UNIT_DEGREE);
  }
}

static void threadA( void *pvParameters ) // thread example from FreeRTOS 
{
  
  SERIAL.println("Thread A: Started");
  while(1)
  {
    myDelayMs(1000);
  }
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  SERIAL.println("Thread A: Deleting");
  vTaskDelete( NULL );
}

static void threadB( void *pvParameters ) // thread example from FreeRTOS 
{
  SERIAL.println("Thread B: Started");

  while(1)
  {
    // SERIAL.println("B");
    SERIAL.flush();
    myDelayMs(2000);
  }

}

//*****************************************************************
// Task will periodically print out useful information about the tasks running
// Is a useful tool to help figure out stack sizes being used
// Run time stats are generated from all task timing collected since startup
// No easy way yet to clear the run time stats yet
//*****************************************************************
static char ptrTaskList[400]; //temporary string buffer for task stats

void taskMonitor(void *pvParameters)
{
    int x;
    int measurement;
    
    SERIAL.println("Task Monitor: Started");

    // run this task afew times before exiting forever
    while(1)
    {
    	myDelayMs(10000); // print every 10 seconds

    	SERIAL.flush();
		SERIAL.println("");			 
    	SERIAL.println("****************************************************");
    	SERIAL.print("Free Heap: ");
    	SERIAL.print(xPortGetFreeHeapSize());
    	SERIAL.println(" bytes");

    	SERIAL.print("Min Heap: ");
    	SERIAL.print(xPortGetMinimumEverFreeHeapSize());
    	SERIAL.println(" bytes");
    	SERIAL.flush();

    	SERIAL.println("****************************************************");
    	SERIAL.println("Task            ABS             %Util");
    	SERIAL.println("****************************************************");

    	vTaskGetRunTimeStats(ptrTaskList); //save stats to char array
    	SERIAL.println(ptrTaskList); //prints out already formatted stats
    	SERIAL.flush();

		SERIAL.println("****************************************************");
		SERIAL.println("Task            State   Prio    Stack   Num     Core" );
		SERIAL.println("****************************************************");

		vTaskList(ptrTaskList); //save stats to char array
		SERIAL.println(ptrTaskList); //prints out already formatted stats
		SERIAL.flush();

		SERIAL.println("****************************************************");
		SERIAL.println("[Stacks Free Bytes Remaining] ");

		measurement = uxTaskGetStackHighWaterMark( Handle_readPosM1Task );
		SERIAL.print("Read Pos M1: ");
		SERIAL.println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_readPosM2Task );
		SERIAL.print("Read Pos M2: ");
		SERIAL.println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_readPosM3Task );
		SERIAL.print("Read Pos M3: ");
		SERIAL.println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_motorControlTask );
		SERIAL.print("Motor Control: ");
		SERIAL.println(measurement);

    measurement = uxTaskGetStackHighWaterMark( Handle_readIMUTask );
		SERIAL.print("IMU: ");
		SERIAL.println(measurement);

		measurement = uxTaskGetStackHighWaterMark( Handle_aTask );
		SERIAL.print("Thread A: ");
		SERIAL.println(measurement);

		measurement = uxTaskGetStackHighWaterMark( Handle_bTask );
		SERIAL.print("Thread B: ");
		SERIAL.println(measurement);

		measurement = uxTaskGetStackHighWaterMark( Handle_monitorTask );
		SERIAL.print("Monitor Stack: ");
		SERIAL.println(measurement);

		SERIAL.println("****************************************************");
		SERIAL.flush();

    }

    // delete ourselves.
    // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
    SERIAL.println("Task Monitor: Deleting");
    vTaskDelete( NULL );

}


//*****************************************************************

void setup() 
{

  SERIAL.begin(115200);

  delay(1000); // prevents usb driver crash on startup, do not omit this
  while (!SERIAL) ;  // Wait for serial terminal to open port before starting program

  //BNO055 Initialisation
  if (!bno.begin()){
    Serial.println("No BNO055 detected.");
    while (1); // Stop if connection fails
  }
  delay(1000); // Delay loop to complete setup

  SERIAL.println("");
  SERIAL.println("******************************");
  SERIAL.println("        Program start         ");
  SERIAL.println("******************************");
  SERIAL.flush();

  // Set the led the rtos will blink when we have a fatal rtos error
  // RTOS also Needs to know if high/low is the state that turns on the led.
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object. 
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack! 
  //               Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);

  // sets the serial port to print errors to when the rtos crashes
  // if this is not set, serial information is not printed by default
  vSetErrorSerial(&SERIAL);

  // Create the threads that will be managed by the rtos
  // Sets the stack size and priority of each task
  // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks
  
  // xTaskCreate(TaskFunction_t pxTaskCode, const char *const pcName, const uint16_t usStackDepth, 
  // void *const pvParameters, UBaseType_t uxPriority, TaskHandle_t *const pxCreatedTask) -> BaseType_t
 
  // xTaskCreate(readPosM1,    "Read Pos M1",    256, NULL, tskIDLE_PRIORITY + 3, &Handle_readPosM1Task);
  // xTaskCreate(readPosM2,    "Read Pos M2",    256, NULL, tskIDLE_PRIORITY + 3, &Handle_readPosM2Task);
  // xTaskCreate(readPosM3,    "Read Pos M3",    256, NULL, tskIDLE_PRIORITY + 3, &Handle_readPosM3Task);
  // xTaskCreate(threadB,     "Task B",        256, NULL, tskIDLE_PRIORITY + 2, &Handle_bTask);
  // xTaskCreate(motorControl,     "Motor Control",      256, NULL, tskIDLE_PRIORITY + 3, &Handle_motorControlTask);
  xTaskCreate(readIMU,          "Read IMU",           256, NULL, tskIDLE_PRIORITY + 1, &Handle_motorControlTask);
  // xTaskCreate(readDemandedPos,  "Read demanded pos",  256, NULL, tskIDLE_PRIORITY + 1, &Handle_readDemandedPosTask);
  xTaskCreate(taskMonitor,      "Task Monitor",       256, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask);

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  // error scheduler failed to start
  // should never get here
  while(1)
  {
	  SERIAL.println("Scheduler Failed! \n");
	  SERIAL.flush();
	  delay(1000);
  }


  dxl.torqueOff(M1);
  dxl.setOperatingMode(M1, OP_POSITION);
  dxl.torqueOn(M1);
}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop() 
{
    Serial.begin(115200);
    while (!Serial);  // Wait for serial port to connect

    // Dynamixel Initialisation
    dxl.begin(57600);
    dxl.setPortProtocolVersion(2.0);

    // Optional commands, can comment/uncomment below
    // SERIAL.print("."); //print out dots in terminal, we only do this when the RTOS is in the idle state
    SERIAL.flush();
    delay(100); //delay is interrupt friendly, unlike vNopDelayMS
}


//*****************************************************************

