#include <Stepper.h> // include stepper library
#include <Servo.h> // include servo library
#include <math.h> // include math library

// Pin definitions
#define DIR_PIN_1 5 // stepper 1 direction
#define STEP_PIN_1 6 // stepper 1 speed
#define DIR_PIN_2 11 // stepper 2 direction
#define STEP_PIN_2 10 // stepper 2 speed
#define STEPS_PER_REV 3200 // steps per revolution
#define MM_PER_REV 40 // mm per revolution
#define STEPPER_SPEED 200 // rotations per minute
#define LIMIT_SWITCH_PIN1 2 // limit switch 1
#define LIMIT_SWITCH_PIN2 3 // limit switch 1
#define SERVO_PIN 13 // servo

// Global variables
int angle = 0; // servo input angle
char state; // initialize serial data
int d = 0; // intialize factor
int y1 = 0; // intialize stepper1 position
int y2 = 0; // intialize stepper2 position
bool s1h = false;
bool s2h = false;

int conv = STEPS_PER_REV/MM_PER_REV; // conversion factor

// Initialize objects
Stepper stepper1(STEPS_PER_REV, STEP_PIN_1, DIR_PIN_1); // initialize stepper 1
Stepper stepper2(STEPS_PER_REV, STEP_PIN_2, DIR_PIN_2); // initialize stepper 2
Servo servoMotor; // initialize servo motor

// Function prototypes
void openGripper();
void closeGripper();

// Setup
void setup() {
  pinMode(STEP_PIN_1, OUTPUT); // set stepper 1 step pin as output
  pinMode(DIR_PIN_1, OUTPUT); // set stepper 1 direction pin as output
  pinMode(STEP_PIN_2, OUTPUT); // set stepper 2 step pin as output
  pinMode(DIR_PIN_2, OUTPUT); // set stepper 2 direction pin as output
  pinMode(LIMIT_SWITCH_PIN1, INPUT); // set limit switch pin as input
  pinMode(LIMIT_SWITCH_PIN2, INPUT); // set limit switch pin as input
  stepper1.setSpeed(STEPPER_SPEED);
  stepper2.setSpeed(STEPPER_SPEED);
  servoMotor.attach(SERVO_PIN); // attach servo to its pin
  servoMotor.write(105);
  Serial.begin(9600); // Start serial communication
  Serial.println("'b' to go from A to B, 'a' to go from B to A, 'home' to home, 'open' to open gripper, 'close' to close gripper.");
}

void loop() {
  if (Serial.available() > 0) {
    state = Serial.read();
    switch (state) {
      
      case 'b': //full routine from HOME --> OPEN --> A --> CLOSE --> B --> OPEN --> HOME --> CLOSE
        
        openGripper(); // OPEN gripper

        for (int s=0; s<800; s++) {
          stepper1.step(-1);
          stepper2.step(4);
        }
        for (int s=0; s<5440; s++) {
          stepper1.step(-1);
          stepper2.step(5);
        }

        closeGripper();

        stepper2.step(-30800);
        stepper1.step(-13000);

      openGripper(); // OPEN gripper
      delay(1000);

      // HOME
      while (s1h == false || s2h == false) {
        if (digitalRead(LIMIT_SWITCH_PIN1) == LOW && s1h == false) {
          stepper1.step(1);

        } else if (digitalRead(LIMIT_SWITCH_PIN1) == HIGH && s1h == false) {
          stepper1.step(-800);
          Serial.println("LS1 HOMED");
          s1h = true; 
        }

        if (digitalRead(LIMIT_SWITCH_PIN2) == LOW && s2h == false) {
          stepper2.step(-1);
         
        } else if (digitalRead(LIMIT_SWITCH_PIN2) == HIGH && s2h == false) {
          stepper2.step(800);
          Serial.println("LS2 HOMED");
          s2h = true;
        }
      }
      s1h = false;
      s2h = false;
      closeGripper();
      break;
    case 'h': // HOME
      while (s1h == false || s2h == false) {
        if (digitalRead(LIMIT_SWITCH_PIN1) == LOW && s1h == false) {
          stepper1.step(1);

        } else if (digitalRead(LIMIT_SWITCH_PIN1) == HIGH && s1h == false) {
          stepper1.step(-800);
          Serial.println("LS1 HOMED");
          s1h = true; 
        } 

        if (digitalRead(LIMIT_SWITCH_PIN2) == LOW && s2h == false) {
          stepper2.step(-1);
         
        } else if (digitalRead(LIMIT_SWITCH_PIN2) == HIGH && s2h == false) {
          stepper2.step(800);
          Serial.println("LS2 HOMED");
          s2h = true;
        }
      }
      s1h = false;
      s2h = false;
      closeGripper();
      break;
      case 'o': // OPEN gripper
        openGripper();
        Serial.println("OPENED");
        break;
      case 'c': // CLOSE gripper
        closeGripper();
        Serial.println("CLOSED");
        break;
    }
  }
}

// Function to open gripper
void openGripper() {
  servoMotor.write(20);
}

// Function to close gripper
void closeGripper() {
  servoMotor.write(105);
}

