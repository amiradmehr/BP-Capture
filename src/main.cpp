// #include <Arduino.h>

// #include <SafeString.h>
// #include <loopTimer.h>
// #include <BufferedOutput.h>
// #include <millisDelay.h>
// #include <SafeStringReader.h>

// #include <AccelStepper.h>
// #include <Adafruit_MCP9808.h>
// #include <Adafruit_NAU7802.h>

// #define BAUD_RATE 115200

// // Define motor interface type and pins
// #define MOTOR_INTERFACE_TYPE 4 // 4-wire motor
// #define AIN1_PIN 4
// #define AIN2_PIN 5
// #define BIN1_PIN 7
// #define BIN2_PIN 6
// #define MAX_SPEED 1000         // Maximum speed in steps per second
// #define ACCELERATION 200       // Acceleration in steps per second squared
// #define READBUFFERTIMEOUT 1000 // 1000ms == 1 sec non-blocking timeout

// // Define NAU7802 settings
// #define LDO_VOLTAGE NAU7802_3V3     // LDO voltage setting
// #define GAIN NAU7802_GAIN_128       // Gain setting
// #define ADC_RATE NAU7802_RATE_10SPS // ADC data rate
// #define ADC_READ_INTERVAL 1000      // ADC read interval in milliseconds

// Adafruit_MCP9808 mcp = Adafruit_MCP9808();
// AccelStepper stepper(MOTOR_INTERFACE_TYPE, AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN);
// Adafruit_NAU7802 nau;

// createSafeStringReader(sfReader, 15, " ,\r\n"); // Create a SafeString reader with delimiters: space, comma, CR, LF
// createBufferedOutput(bufferedOut, 80, DROP_UNTIL_EMPTY);

// int led_pin = 13;
// bool ledOn = false; // Track LED state

// millisDelay ledDelay;
// millisDelay printDelay;
// millisDelay mcpDelay;
// millisDelay adcDelay;

// float tempC = 0.0;
// float simulatedTempReading = 0.0; // Value from user input (0.0 to 100.0)
// bool startreading = false;
// bool closeDampler = true; // true means damper closed

// // Global variable to hold the target position (in steps)
// long targetPosition = 0;

// // Global variable to hold the motor's constant speed (steps per second)
// // Default is 500 steps per second.
// int constantSpeed = 500;

// // ADC variables
// int32_t adcReading = 0; // Variable to store ADC reading
// bool startadc = false;  // Flag to indicate if ADC reading has started

// void setup()
// {
//   Serial.begin(BAUD_RATE);
//   // Wait for serial port connection
//   for (int i = 0; i < 10; i++)
//   {
//     Serial.print(i);
//     delay(200);
//   }
//   Serial.println();

//   bufferedOut.connect(Serial);
//   sfReader.connect(bufferedOut);
//   sfReader.echoOn();
//   sfReader.setTimeout(READBUFFERTIMEOUT);

//   pinMode(led_pin, OUTPUT);

//   // Setup the MCP9808 temperature sensor
//   if (!mcp.begin())
//   {
//     Serial.println("Could not find MCP9808 sensor, check wiring!");
//     while (1)
//       ;
//   }
//   Serial.println("MCP9808 sensor found!");
//   mcp.setResolution(1); // Set resolution to 0.25°C

//   // Setup the NAU7802 ADC
//   if (!nau.begin())
//   {
//     Serial.println("Failed to find NAU7802");
//     while (1)
//       ;
//   }
//   Serial.println("Found NAU7802");
//   nau.setLDO(LDO_VOLTAGE);
//   nau.setGain(GAIN);
//   nau.setRate(ADC_RATE);

//   // Take 10 readings to flush out readings
//   for (uint8_t i = 0; i < 10; i++)
//   {
//     while (!nau.available())
//       delay(1);
//     nau.read();
//   }

//   // Start delays for LED, printing, and ADC readings
//   ledDelay.start(1000);              // Toggle LED every 1000ms
//   printDelay.start(2000);            // Print status every 2000ms
//   adcDelay.start(ADC_READ_INTERVAL); // Read ADC every 1000ms

//   Serial.println(F("Enter simulated temperature (0 to 100), 'run', 'close', or 's<value>' to change speed."));

//   // Setup the stepper motor
//   stepper.setMaxSpeed(MAX_SPEED);
//   stepper.setAcceleration(ACCELERATION);
// }

// void blinkled(bool stop)
// {
//   if (ledDelay.justFinished())
//   {
//     ledDelay.repeat();
//     if (stop)
//     {
//       digitalWrite(led_pin, LOW);
//       ledOn = false;
//       return;
//     }
//     digitalWrite(led_pin, ledOn ? LOW : HIGH);
//     ledOn = !ledOn;
//   }
// }

// void printtemp()
// {
//   if (printDelay.justFinished())
//   {
//     printDelay.repeat();
//     bufferedOut.print(F("Simulated Temperature: "));
//     bufferedOut.println(simulatedTempReading, 2);
//     bufferedOut.print(F("Current Position: "));
//     bufferedOut.println(stepper.currentPosition());
//     bufferedOut.print(F("Motor Constant Speed: "));
//     bufferedOut.println(constantSpeed);
//     if (closeDampler)
//     {
//       bufferedOut.println(F(" Damper Closed"));
//     }
//     else
//     {
//       bufferedOut.println(F(" Damper Running"));
//     }
//   }
// }

// void processUserInput()
// {
//   if (sfReader.read())
//   {                         // Non-blocking read
//     sfReader.toLowerCase(); // Make input case-insensitive

//     // New branch: if the input starts with "s", process as a speed command.
//     if (sfReader.startsWith("s"))
//     {
//       // Allocate a buffer for the speed value (exclude the 's')
//       char speedBuf[15] = "";
//       SafeString speedValue(15, speedBuf, "");
//       sfReader.substring(speedValue, 1); // Get substring after the "s"
//       int newSpeed = constantSpeed;
//       if (speedValue.toInt(newSpeed))
//       {
//         if (newSpeed > 0 && newSpeed <= MAX_SPEED)
//         {
//           constantSpeed = newSpeed;
//           bufferedOut.print(F("Motor constant speed set to: "));
//           bufferedOut.println(constantSpeed);
//         }
//         else
//         {
//           bufferedOut.print(F("Invalid speed. Must be between 1 and "));
//           bufferedOut.println(MAX_SPEED);
//         }
//       }
//       else
//       {
//         bufferedOut.println(F("Invalid speed format. Use 's<value>'."));
//       }
//       return; // Exit the function after processing the speed command.
//     }

//     // Process damper commands.
//     if (sfReader == "close")
//     {
//       closeDampler = true;
//     }
//     else if (sfReader == "run")
//     {
//       closeDampler = false;
//     }
//     else
//     { // Try to interpret the input as a temperature value
//       float newSimulatedTempReading = simulatedTempReading;
//       if (!sfReader.toFloat(newSimulatedTempReading))
//       {
//         bufferedOut.print(F(" -- Invalid temperature or command."));
//       }
//       else
//       {
//         if ((newSimulatedTempReading < 0.0) || (newSimulatedTempReading > 100.0))
//         {
//           bufferedOut.print(F(" -- Temperature must be between 0.0 and 100.0 "));
//           bufferedOut.println();
//         }
//         else
//         {
//           simulatedTempReading = newSimulatedTempReading;
//         }
//       }
//     }
//   }
// }

// void printadcreading()
// {
//   if (adcDelay.justFinished())
//   {
//     adcDelay.repeat();
//     if (startadc)
//     {
//       bufferedOut.print(F("ADC Reading: "));
//       bufferedOut.println(adcReading);
//     }
//     else
//     {
//       bufferedOut.println(F("ADC Reading not started."));
//     }
//   }
// }

// int readadc() // Read ADC value
// {
//   adcReading = nau.read();
//   return 0; // Return 0 to indicate success
// }

// int readTemp()
// {
//   if (!startreading)
//   {
//     mcpDelay.start(1000);
//     startreading = true;
//   }
//   if (mcpDelay.justFinished())
//   {
//     startreading = false;
//     mcpDelay.repeat();
//     tempC = mcp.readTempC();
//     return 0; // new reading
//   }
//   return -1; // no new reading yet
// }

// // Update the target position based on the damper state and the user input temperature.
// // If the damper is closed, the target is 0; otherwise, it's simulatedTempReading * 50.
// void setDamperPosition()
// {
//   if (closeDampler)
//   {
//     targetPosition = 0;
//   }
//   else
//   {
//     targetPosition = simulatedTempReading * 50;
//   }
// }

// // Run the stepper at a constant speed (using the variable 'constantSpeed') toward the target position.
// // Once the motor is within 1 step of the target, the motor stops.
// void runstepper()
// {
//   long currentPos = stepper.currentPosition();
//   long error = targetPosition - currentPos;

//   if (abs(error) <= 1)
//   {
//     // Target reached; stop the motor.
//     stepper.setSpeed(0);
//   }
//   else
//   {
//     // Set constant speed with proper direction.
//     if (error > 0)
//     {
//       stepper.setSpeed(constantSpeed);
//     }
//     else
//     {
//       stepper.setSpeed(-constantSpeed);
//     }
//     stepper.runSpeed();
//   }
// }

// void loop()
// {
//   bufferedOut.nextByteOut();    // Flush buffered output
//   loopTimer.check(bufferedOut); // Check the loop timer
//   processUserInput();
//   blinkled(closeDampler);
//   printtemp();
//   readTemp();

//   // Update target position based on user input and damper state.
//   setDamperPosition();
//   // Run the motor at constant speed until the target is reached.
//   runstepper();
// }

// #include <Arduino.h>
// #include <SafeString.h>
// #include <loopTimer.h>
// #include <BufferedOutput.h>
// #include <millisDelay.h>
// #include <SafeStringReader.h>

// #include <AccelStepper.h>
// #include <Adafruit_NAU7802.h>
// #include <Adafruit_MCP9808.h>

// // --------------- Configuration ---------------
// #define BAUD_RATE 115200
// #define READBUFFERTIMEOUT 10 // 1000ms == 1 sec non-blocking timeout

// // Define motor interface type and pins
// #define MOTOR_INTERFACE_TYPE 4 // 4-wire motor
// #define AIN1_PIN 4
// #define AIN2_PIN 5
// #define BIN1_PIN 7
// #define BIN2_PIN 6
// #define MAX_SPEED 1000   // Maximum allowed speed (for safety)
// #define ACCELERATION 200 // (Not used here, but available if needed)
// int constantSpeed = 500; // Used for the half-second commands

// // ADC read interval (in ms)
// #define ADC_READ_INTERVAL 100

// // --------------- Global Objects and Variables ---------------

// // Create objects for buffered output and safe string reading.
// createSafeStringReader(sfReader, 15, " ,\r\n"); // Delimiters: space, comma, CR, LF
// createBufferedOutput(bufferedOut, 80, DROP_UNTIL_EMPTY);

// // Create the stepper motor object
// AccelStepper stepper(MOTOR_INTERFACE_TYPE, AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN);

// // Create the ADC object
// Adafruit_NAU7802 nau;

// // Timer for ADC reading
// millisDelay adcDelay;

// // Enumeration and variables to manage the motor command actions.
// enum MotorAction
// {
//   NONE,
//   FORWARD_HALF,
//   BACKWARD_HALF,
//   FORWARD_60
// };
// MotorAction currentAction = NONE;
// unsigned long actionStartTime = 0; // Stores when the current command started

// // --------------- Function Prototypes ---------------
// void processUserInput();
// void updateMotorAction();
// void printAdcReading();

// // --------------- Setup ---------------
// void setup()
// {
//   Serial.begin(BAUD_RATE);
//   // Wait a little for the Serial port to be ready
//   for (int i = 0; i < 10; i++)
//   {
//     Serial.print(i);
//     delay(200);
//   }
//   Serial.println();

//   bufferedOut.connect(Serial);
//   sfReader.connect(bufferedOut);
//   // sfReader.echoOn();
//   sfReader.setTimeout(READBUFFERTIMEOUT);

//   // Setup the NAU7802 ADC
//   if (!nau.begin())
//   {
//     Serial.println("Failed to find NAU7802");
//     while (1)
//       ; // halt if ADC not found
//   }
//   Serial.println("Found NAU7802");
//   nau.setLDO(NAU7802_3V3);
//   nau.setGain(NAU7802_GAIN_128);
//   nau.setRate(NAU7802_RATE_10SPS);

//   // Flush a few ADC readings
//   for (uint8_t i = 0; i < 10; i++)
//   {
//     while (!nau.available())
//       delay(1);
//     nau.read();
//   }

//   // Start ADC timer
//   adcDelay.start(ADC_READ_INTERVAL);

//   Serial.println(F("Commands:"));
//   Serial.println(F("  ] : Move forward for 0.5 seconds at constant speed"));
//   Serial.println(F("  [ : Move backward for 0.5 seconds at constant speed"));
//   Serial.println(F("  k : Move forward for 60 seconds at speed 5"));

//   // Setup the stepper motor
//   stepper.setMaxSpeed(MAX_SPEED);
// }

// // --------------- Process User Input ---------------
// void processUserInput()
// {
//   // If a command was received, process it.
//   if (sfReader.read())
//   {
//     // We expect single-character commands.
//     char cmd = sfReader.charAt(0);

//     // Only accept new commands if no other action is currently running.
//     if (currentAction != NONE)
//       return;

//     if (cmd == ']')
//     {
//       // Move forward at constantSpeed for 500 ms.
//       currentAction = FORWARD_HALF;
//       actionStartTime = millis();
//       stepper.setSpeed(constantSpeed);
//       bufferedOut.println(F("Command: Move forward 0.5 sec"));
//     }
//     else if (cmd == '[')
//     {
//       // Move backward at constantSpeed for 500 ms.
//       currentAction = BACKWARD_HALF;
//       actionStartTime = millis();
//       stepper.setSpeed(-constantSpeed);
//       bufferedOut.println(F("Command: Move backward 0.5 sec"));
//     }
//     else if (cmd == 'k')
//     {
//       // Move forward at speed 5 for 60 seconds.
//       currentAction = FORWARD_60;
//       actionStartTime = millis();
//       stepper.setSpeed(5);
//       bufferedOut.println(F("Command: Move forward at speed 5 for 60 sec"));
//     }
//   }
// }

// // --------------- Update Motor Action ---------------
// void updateMotorAction()
// {
//   // If no command is active, nothing to do.
//   if (currentAction == NONE)
//     return;

//   unsigned long elapsed = millis() - actionStartTime;

//   // Determine the duration based on the action.
//   if ((currentAction == FORWARD_HALF || currentAction == BACKWARD_HALF) && (elapsed < 500))
//   {
//     stepper.runSpeed();
//   }
//   else if (currentAction == FORWARD_60 && (elapsed < 60000))
//   {
//     stepper.runSpeed();
//   }
//   else
//   {
//     // Command duration is over; stop the motor.
//     currentAction = NONE;
//     stepper.setSpeed(0);
//   }
// }

// // --------------- ADC Reading and Printing ---------------
// void printAdcReading()
// {
//   if (adcDelay.justFinished())
//   {
//     adcDelay.repeat();
//     // Wait until new ADC data is available, then read and print.
//     while (!nau.available())
//     { /* non-blocking wait */
//     }
//     int32_t adcReading = nau.read();
//     bufferedOut.print(F("ADC Reading: "));
//     bufferedOut.println(adcReading);
//   }
// }

// // --------------- Main Loop ---------------
// void loop()
// {
//   bufferedOut.nextByteOut(); // Flush buffered output if needed
//   processUserInput();        // Check for new command input
//   updateMotorAction();       // Run the motor command if one is active
//   printAdcReading();         // Print the ADC reading periodically
// }



#include <Arduino.h>
#include <SafeString.h>
#include <loopTimer.h>
#include <BufferedOutput.h>
#include <millisDelay.h>
#include <SafeStringReader.h>

#include <AccelStepper.h>
#include <Adafruit_NAU7802.h>
#include <Adafruit_MCP9808.h>

// --------------- Configuration ---------------
#define BAUD_RATE 115200
#define READBUFFERTIMEOUT 10 // 10ms non-blocking timeout

// Define motor interface type and pins
#define MOTOR_INTERFACE_TYPE 4 // 4-wire motor
#define AIN1_PIN 4
#define AIN2_PIN 5
#define BIN1_PIN 7
#define BIN2_PIN 6
#define MAX_SPEED 1000   // Maximum allowed speed (for safety)
#define ACCELERATION 200 // (Not used here, but available if needed)
int constantSpeed = 500; // Used for the half-second commands

// ADC read interval (in ms)
#define ADC_READ_INTERVAL 10

// --------------- Global Objects and Variables ---------------

// Create objects for buffered output and safe string reading.
createSafeStringReader(sfReader, 15, " ,\r\n"); // Delimiters: space, comma, CR, LF
createBufferedOutput(bufferedOut, 80, DROP_UNTIL_EMPTY);

// Create the stepper motor object
AccelStepper stepper(MOTOR_INTERFACE_TYPE, AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN);

// Create the ADC object
Adafruit_NAU7802 nau;

// Timer for ADC reading
millisDelay adcDelay;

// Enumeration and variables to manage the motor command actions.
enum MotorAction {
  NONE,
  FORWARD_HALF,
  BACKWARD_HALF,
  FORWARD_60
};
MotorAction currentAction = NONE;
unsigned long actionStartTime = 0; // Stores when the current command started

// --------------- Function Prototypes ---------------
void processUserInput();
void updateMotorAction();
void printAdcReading();

// --------------- Setup ---------------
void setup() {
  Serial.begin(BAUD_RATE);
  // Wait a little for the Serial port to be ready
  for (int i = 0; i < 10; i++) {
    Serial.print(i);
    delay(200);
  }
  Serial.println();

  bufferedOut.connect(Serial);
  sfReader.connect(bufferedOut);
  // sfReader.echoOn();
  sfReader.setTimeout(READBUFFERTIMEOUT);

  // Setup the NAU7802 ADC
  if (!nau.begin()) {
    Serial.println("Failed to find NAU7802");
    while (1) ; // halt if ADC not found
  }
  Serial.println("Found NAU7802");
  nau.setLDO(NAU7802_3V3);
  nau.setGain(NAU7802_GAIN_128);
  nau.setRate(NAU7802_RATE_320SPS);

  // Perform offset calibration using nau.calibrate(NAU7802_CALMOD_OFFSET)
  Serial.println("Performing NAU7802 offset calibration...");
  if (!nau.calibrate(NAU7802_CALMOD_OFFSET)) {
    Serial.println("Offset calibration failed!");
  } else {
    Serial.println("Offset calibration complete!");
  }

  // Flush a few ADC readings after calibration
  for (uint8_t i = 0; i < 10; i++) {
    while (!nau.available())
      delay(1);
    nau.read();
  }

  // Start ADC timer
  adcDelay.start(ADC_READ_INTERVAL);

  Serial.println(F("Commands:"));
  Serial.println(F("  ] : Move forward for 0.5 seconds at constant speed"));
  Serial.println(F("  [ : Move backward for 0.5 seconds at constant speed"));
  Serial.println(F("  k : Move forward at speed 5 for 60 seconds"));
  Serial.println(F("  x : Immediately stop the motor"));

  // Setup the stepper motor
  stepper.setMaxSpeed(MAX_SPEED);
}

// --------------- Process User Input ---------------
void processUserInput() {
  // If a command was received, process it.
  if (sfReader.read()) {
    // We expect a single-character command.
    char cmd = sfReader.charAt(0);

    // Check for the immediate stop command regardless of any current action.
    if (cmd == 'x') {
      currentAction = NONE;
      stepper.setSpeed(0);
      // bufferedOut.println(F("Command: Motor stopped"));
      return;
    }

    // Only accept new commands if no other action is currently running.
    if (currentAction != NONE)
      return;

    if (cmd == ']') {
      // Move forward at constantSpeed for 500 ms.
      currentAction = FORWARD_HALF;
      actionStartTime = millis();
      stepper.setSpeed(constantSpeed);
      // bufferedOut.println(F("Command: Move forward 0.5 sec"));
    }
    else if (cmd == '[') {
      // Move backward at constantSpeed for 500 ms.
      currentAction = BACKWARD_HALF;
      actionStartTime = millis();
      stepper.setSpeed(-constantSpeed);
      // bufferedOut.println(F("Command: Move backward 0.5 sec"));
    }
    else if (cmd == 'k') {
      // Move forward at speed 5 for 60 seconds.
      currentAction = FORWARD_60;
      actionStartTime = millis();
      stepper.setSpeed(-5);
      // bufferedOut.println(F("Command: Move forward at speed 5 for 60 sec"));
    }
  }
}

// --------------- Update Motor Action ---------------
void updateMotorAction() {
  // If no command is active, nothing to do.
  if (currentAction == NONE)
    return;

  unsigned long elapsed = millis() - actionStartTime;

  // Determine the duration based on the action.
  if ((currentAction == FORWARD_HALF || currentAction == BACKWARD_HALF) && (elapsed < 500)) {
    stepper.runSpeed();
  }
  else if (currentAction == FORWARD_60 && (elapsed < 40000)) {
    stepper.runSpeed();
  }
  else {
    // Command duration is over; stop the motor.
    currentAction = NONE;
    stepper.setSpeed(0);
  }
}

// --------------- ADC Reading and Printing ---------------
void printAdcReading() {
  if (adcDelay.justFinished()) {
    adcDelay.repeat();
    // Wait until new ADC data is available, then read and print.
    while (!nau.available()) {
      /* non-blocking wait */
    }
    int32_t adcReading = nau.read();
    bufferedOut.print(F("ADC Reading: "));
    bufferedOut.println(adcReading);
  }
}

// --------------- Main Loop ---------------
void loop() {
  bufferedOut.nextByteOut(); // Flush buffered output if needed
  processUserInput();        // Check for new command input
  updateMotorAction();       // Run the motor command if one is active
  printAdcReading();         // Print the ADC reading periodically
}
