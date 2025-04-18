// Lowest start speed: 120
// Lowest kickstartspeed: 75

// Data sent by each sensor is either a 0 or a 1. 
// 0 = Line detected (no reflection)
// 1 = No line detected (reflection)

// Sensor pins from left to right
const int numSensors = 4; 
const int sensor1 =  13; 
const int sensor2 =  7; 
const int sensor3 = 12; 
const int sensor4 = 4; 

// right motor pins
const int enableRightMotor = 5;
const int rightMotorPin1 = 10;
const int rightMotorPin2 = 11;

// left motor pins
const int enableLeftMotor = 6; 
const int leftMotorPin1 = 8;
const int leftMotorPin2 = 9;

// PWM signal values for driving straight and turning
// Below, 1 is the speed of the faster tire in turns and 2 is the slower tire speed
const int motorSpeedStraight = 100;
const int motorSpeedSlight1 = 110;
const int motorSpeedSlight2 = 90;
const int motorSpeedMedium1 = 120;
const int motorSpeedMedium2 = 80;
const int motorSpeedTight1 = 130;
const int motorSpeedTight2 = 70;

// Kickstart PWM signal values for driving straight and turning
// Kickstart speeds are relational to the normal motor speeds 
const int initMotorSpeedStraight = 230;
const int initMotorSpeedSlight1 = 230;
const int initMotorSpeedSlight2 = round(motorSpeedSlight2/motorSpeedSlight1*initMotorSpeedSlight1);
const int initMotorSpeedMedium1 = 230;
const int initMotorSpeedMedium2 = round(motorSpeedMedium2/motorSpeedMedium1*initMotorSpeedMedium1);
const int initMotorSpeedTight1 = 230;
const int initMotorSpeedTight2 = round(motorSpeedTight2/motorSpeedTight1*initMotorSpeedTight1);

const int initDelay = 50; // Kickstart period
const int controlDelay = 100; // Delay after each loop

int prev_error = 0; // initiate previous state (error)

struct State {
    int sensorData[numSensors]; 
    int error; 
};

// If sensors output any of these states, controller reacts
const State states[] = {
    {{1, 1, 1, 0}, -3}, // Tight right
    {{1, 1, 0, 0}, -2}, // Medium right
    {{1, 0, 0, 0}, -1}, // Slight right
    {{1, 0, 0, 1},  0}, // Straight
    {{0, 0, 0, 1},  1}, // Slight left
    {{0, 0, 1, 1},  2}, // Medium left
    {{0, 1, 1, 1},  3}, // Tight left
    {{1, 1, 1, 1},  4}, // All white, reverse
    {{0, 0, 0, 0},  5}  // All black, stop
};

const int numStates = sizeof(states) / sizeof(states[0]);

void setup() {
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  delay(5000); 5s Delay so the robot can be put into starting position 
  // Start the motors
  rotateMotors(initMotorSpeedStraight, initMotorSpeedStraight);
  delay(initDelay);
  rotateMotors(motorSpeedStraight, motorSpeedStraight);  
}

void loop(){
  int currentSensorData[numSensors];
  int sensors[] = {sensor1, sensor2, sensor3, sensor4};

  // Read current sensor state
  for (int i = 0; i < numSensors; i++) {
      currentSensorData[i] = digitalRead(sensors[i]);
  }

  bool stateFound = false;
  int error = prev_error;

  // If state is not recognized, robot continues with previous state
  for (int j = 0; j < numStates; j++) {
      if (compareSensorData(currentSensorData, states[j].sensorData)) {
          error = states[j].error;
          stateFound = true;
          break;
      }
  }

  controller(error);
  delay(controlDelay);
  prev_error = error;
}

void controller(int error) {
  /*
  A simple logic controller. Given any predefined input, it always outputs 
  the same, predefined control signal. 
  Error is always an int in range [-3, 5]
  */

    if (error == -3) { // Tight right
        rotateMotors(initMotorSpeedTight1, initMotorSpeedTight2);
        delay(initDelay);
        rotateMotors(motorSpeedTight1, motorSpeedTight2);

    } else if (error == -2) { // Medium right
        rotateMotors(initMotorSpeedMedium1, initMotorSpeedMedium2);
        delay(initDelay);
        rotateMotors(motorSpeedMedium1, motorSpeedMedium2);

    } else if (error == -1) { // Slight right
        rotateMotors(initMotorSpeedSlight1, initMotorSpeedSlight2);
        delay(initDelay);
        rotateMotors(motorSpeedSlight1, motorSpeedSlight2);

    } else if (error == 0) { // Straight
        rotateMotors(initMotorSpeedStraight, initMotorSpeedStraight);
        rotateMotors(motorSpeedStraight, motorSpeedStraight);

    } else if (error == 1) { // Slight left
        rotateMotors(initMotorSpeedSlight2, initMotorSpeedSlight1);
        delay(initDelay);
        rotateMotors(motorSpeedSlight2, motorSpeedSlight1);

    } else if (error == 2) { // Medium left
        rotateMotors(initMotorSpeedMedium2, initMotorSpeedMedium1);
        delay(initDelay);
        rotateMotors(motorSpeedMedium2, motorSpeedMedium1);

    } else if (error == 3) { // Tight left
        rotateMotors(initMotorSpeedTight2, initMotorSpeedTight1);
        delay(initDelay);
        rotateMotors(motorSpeedTight2, motorSpeedTight1);

    } else if (error == 4) { // Reverse
        rotateMotors(-initMotorSpeedStraight, -initMotorSpeedStraight);
        delay(initDelay);
        rotateMotors(-motorSpeedStraight, -motorSpeedStraight);

    } else { // Stop
      rotateMotors(0, 0);
    }
}

bool compareSensorData(int* currentData, int* stateData) {
  /*
  Check if sensor data equals to a state
  */

    for (int i = 0; i < numSensors; i++) {
        if (currentData[i] != stateData[i])
            return false;
    }
    return true;
}

void rotateMotors(int leftMotorSpeed, int rightMotorSpeed) {
  /*
  Rotate the motors in speeds defined in controller. This function can be used 
  in applications where the motor speed is negative too.  
  */

    if (rightMotorSpeed > 0) {
        digitalWrite(rightMotorPin1, HIGH);
        digitalWrite(rightMotorPin2, LOW);
    } else {
        digitalWrite(rightMotorPin1, LOW);
        digitalWrite(rightMotorPin2, HIGH);
    }

    if (leftMotorSpeed > 0) {
        digitalWrite(leftMotorPin1, HIGH);
        digitalWrite(leftMotorPin2, LOW);
    } else {
        digitalWrite(leftMotorPin1, LOW);
        digitalWrite(leftMotorPin2, HIGH);
    }

    analogWrite(enableRightMotor, abs(rightMotorSpeed));
    analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}




