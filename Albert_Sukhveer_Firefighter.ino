/*
  Firefighter Robot
  
  This program is used to control a firefighting robot, which will navigate through a
  maze and search for a flame. When it finds the flame, it will extinguish it and then
  try to knock over a dowel. After both these tasks have been completed, then the robot
  will exit the maze. 

   The circuit:
 * Two servos connected to an external 6V power supply, with their control pins going to pins 8 and 9 on the Arduino
 * An ultrasonic sensor (HC-SR04) connected to the +5V and the GND of the Arduino, along with the trigger and echo pins going to pins 2 and 3, respectively, on the Arduino
 * A flame sensor connected to the +5V and the GND of the Arduino, along with the input pin going to the A0 analog input of the Arduino
 * A fan with one end connected to the drain of the MOSFET and the other end connected to the +6V of the external power supply, with the gate of the MOSFET going to pin 6 on the Arduino
 * GNDs of the external power supply and the Arduino connected together

  created April-June 2017
  By: Albert W. and Sukhveer S.

*/

#include <Servo.h>

//*********************************************
// *********** SERVO **************************
//*********************************************
// The "STOP" positions for each of the two servos
#define LEFT_SERVO_STOP 92
#define RIGHT_SERVO_STOP 96
// Speed calibration ratio for the two servos (speed of left
// servo divided by speed of right servo at the same "write #")
#define LEFT_SERVO_OVER_RIGHT_SERVO_SPEED_RATIO 0.75 // Previous value(s): 1.20   0.725

// The time needed for the servo to turn 90 degrees with the default movement speed of 30
#define NINETY_DEGREES_TURN_TIME_FORWARD 1400 // Previous value(s): 2100
#define NINETY_DEGREES_TURN_TIME_BACKWARD 1400

// The pin for the left and right servos
#define leftServoPin 8
#define rightServoPin 9

// Servo objects for each of the two servos
Servo leftServo;
Servo rightServo;

int leftServoWrite = LEFT_SERVO_STOP;
int rightServoWrite = RIGHT_SERVO_STOP;

int movementSpeed = 0;

//*********************************************
// ************** ULTRASONIC SENSOR ***********
//*********************************************
// Values related to the ultrasonic sensor
#define TRIG_PIN 2
#define ECHO_PIN 3
#define OBSTACLE_IN_FRONT_THRESHOLD 18 // Previous value(s): 30

// Variables related to the ultrasonic sensor
int numberOfHIGHUltrasonicReadings = 0;
float previousUltrasonicSensorReading = 0;
float numberOfSameReadings = 0;

//*********************************************
// ************** FLAME SENSOR ****************
//*********************************************
#define fireSensorPin A0
#define minimumFlameDetectionNumber 40
#define optimalFlameDetectionNumber 930 // Previous value(s): 190

// For the fan
#define fanControlPin 6

int numberOfHIGHFlameSensorReadings = 0;

//*********************************************
// ************** DOWEL ***********************
//*********************************************
#define almostTouchingDowelNumber 5

boolean hasDowelBeenKnockedOver = false;

unsigned long timeStarted;

void setup() {
  // Initialize the servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);

  // Initialize serial communications
  Serial.begin(9600);

  // Initialize the pins of the ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize the pin that controls the gate of the MOSFET, which,
  // in turn, controls the fan
  pinMode(fanControlPin, OUTPUT);

  // By default, the movement speed of the servo is 30
  movementSpeed = 30;

  // Initially, the robot will move forward
  calculateServoWriteNumbers("FORWARD");

  stopRobot();
}

void loop() {
  // Function that checks for a flame and performs the appropriate algorithm if it is detected
  checkForFlame();
  // Function that checks for a blockage/obstacle/wall and performs the appropriate algorithm to manouever the robot around it
  checkForBlockage();
  // Move the robot (usually forward)
  moveRobot(25);
}

// Function that moves the robot for a specified number of milliseconds
void moveRobot(int timeToMove) {
  // Write the left/right servo write values to each of the servos
  leftServo.write(leftServoWrite);
  rightServo.write(rightServoWrite);

  // Delay for the specified amount of time to ensure the robot moves for that much time
  delay(timeToMove);
}

// Function that is used to stop the robot
void stopRobot() {
  // Write the STOP values (predetermined and defined at the top) to both servos
  // to stop the robot
  leftServo.write(LEFT_SERVO_STOP);
  rightServo.write(RIGHT_SERVO_STOP);
}

// Function that is used to calculate the numbers that will be written to the servo in order
// to move it in a certain direction
// Possible directions: "FORWARD", "BACKWARD", "F_LEFT", "F_RIGHT", "B_LEFT", "B_RIGHT"
void calculateServoWriteNumbers(String desiredDirectionOfMovement) {
  
  // Based on the desired direction of movement, calculate the appropriate numbers
  // Notice that these calculations take into consideration the difference in speeds between
  // the two servos (i.e. it calibrates the servos so that both move at the same speed)
  
  if (desiredDirectionOfMovement.equals("FORWARD")) {
    // Add the calibrated movement speed to the stop positions of each servo (notice that the 
    // left and right servos must spin in opposite directions (clockwise and counterclockwise)
    // to get the robot to move forward
    leftServoWrite = LEFT_SERVO_STOP - (movementSpeed * LEFT_SERVO_OVER_RIGHT_SERVO_SPEED_RATIO);
    rightServoWrite = RIGHT_SERVO_STOP + (movementSpeed);
    return;
  }
  if (desiredDirectionOfMovement.equals("BACKWARD")) {
    // To go backwards, reverse the numbers from the "FORWARD"
    leftServoWrite = LEFT_SERVO_STOP + (movementSpeed * LEFT_SERVO_OVER_RIGHT_SERVO_SPEED_RATIO);
    rightServoWrite = RIGHT_SERVO_STOP - (movementSpeed);
    return;
  }
  if (desiredDirectionOfMovement.equals("F_LEFT")) {
    // To go to the left, stop the left servo's movement and only move the right servo
    // Make the right servo move forward so that the robot goes "FORWARD LEFT"
    leftServoWrite = LEFT_SERVO_STOP;
    rightServoWrite = RIGHT_SERVO_STOP + (movementSpeed);
    return;
  }
  if (desiredDirectionOfMovement.equals("F_RIGHT")) {
    // To go to the right, stop the right servo's movement and only move the left servo
    // Make the left servo move forward so that the robot goes "FORWARD RIGHT"
    leftServoWrite = LEFT_SERVO_STOP - (movementSpeed * LEFT_SERVO_OVER_RIGHT_SERVO_SPEED_RATIO);
    rightServoWrite = RIGHT_SERVO_STOP;
    return;
  }
  if (desiredDirectionOfMovement.equals("B_LEFT")) {
    // To go to the left, stop the left servo's movement and only move the right servo
    // Make the right servo move backward so that the robot goes "BACKWARD LEFT"
    leftServoWrite = LEFT_SERVO_STOP;
    rightServoWrite = RIGHT_SERVO_STOP - 1 - (movementSpeed);
    return;
  }
  if (desiredDirectionOfMovement.equals("B_RIGHT")) {
    // To go to the right, stop the right servo's movement and only move the left servo
    // Make the left servo move backward so that the robot goes "BACKWARD RIGHT"
    leftServoWrite = LEFT_SERVO_STOP + 5 + (movementSpeed * LEFT_SERVO_OVER_RIGHT_SERVO_SPEED_RATIO);
    rightServoWrite = RIGHT_SERVO_STOP;
    return;
  }
}

// Returns the distance (in cm) of the nearest object or -1 if no nearby object is detected
float readUltrasonicSensor() {
  // Set the trigger pin LOW temporarily, then send a HIGH pulse to it, then
  // pull it LOW again in order to ensure a clean reading
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read in the pulse sent by the sensor on the ECHO_PIN to determine
  // the amount of time that it took for the sound from the transmitter to bounce
  // off the object and then return to the receiver 
  float objectProximity = pulseIn(ECHO_PIN, HIGH, 8000);

  // If the pulse on the ECHO_PIN is 0, that means there is no nearby object
  // so return -1
  if (objectProximity == 0) {
    return -1;
  }
  
  // Convert the time into centimeters using the speed of sound: 29 microseconds per centimeter
  // Divide by 2 since the sound wave travelled twice the distance between the robot and the object (one forward,
  // and once backward)
  objectProximity = objectProximity / 2.0 / 29.0;

  // Return the above calculated object proximity
  return objectProximity;
}

void checkForBlockage() {
  // Get the distance from the closest object by reading the ultrasonic sensor
  float objectProximity = readUltrasonicSensor();

  // This code will check to see if a wall is in front of the robot
  // by reading the ultrasonic sensor and recording the number of times that 
  // it detects that a wall is in front of it (which we call "HIGH" readings). 
  // If the number of consecutive "HIGH" readings is greater than or equal to 5, then there is
  // definitely an object in front of it.. so perform the blockage algorithm. In this way
  // this algorithm is able to successfully ignore false "HIGH"s from the sensor
  while (objectProximity <= OBSTACLE_IN_FRONT_THRESHOLD && objectProximity != -1) {
    stopRobot();
    numberOfHIGHUltrasonicReadings++;

    if (numberOfHIGHUltrasonicReadings >= 5) {
      performBlockageAlgorithm();
      numberOfHIGHUltrasonicReadings = 0;
      numberOfSameReadings = 0;
      return;
    }
    else {
      objectProximity = readUltrasonicSensor();
      delay(15);
    }
  }

  // If the above loop ends, it means that no obstacle was detected in front of the robot.
  numberOfHIGHUltrasonicReadings = 0;

  // It is still possible that the robot is stuck somewhere (but far enough from the walls of 
  // the maze that it doesn't detect the wall being right in front of it). In this case, the code
  // below will be used to check if the robot is stuck (using input from the ultrasonic sensor)
  //
  // If the robot is seeing the exact same number every single time
  // then it means that the robot is stuck somewhere
  // So we use the numberOfSameReadings variable to determine the number
  // of times we are getting the same reading. Then afterwards we determine
  // the appropriate course of action using the if statements that follow.

  // If the previous and current reading are off by less than 1.0, then they are basically
  // the same, so increment the numberOfSameReadings variable
  if (abs(objectProximity - previousUltrasonicSensorReading) <= 1.0) {
    numberOfSameReadings++;
  }
  else {
    numberOfSameReadings = 0;
  }

  // Update the previousUltrasonicSensorReading
  previousUltrasonicSensorReading = objectProximity;

  // If the sensor has reported the same value 100 times (and this value is not -1, meaning
  // there is something in front of the robot but not close by), then the robot must be stuck 
  // so manouever the robot around to allow it to get unstuck
  if (numberOfSameReadings >= 100 && objectProximity != -1) {
    performBlockageAlgorithm();
    numberOfSameReadings = 0;
  }

  // Else if the sensor reports the same value 1000 times and it isn't in front
  // of anything, then it must be stuck from the back but can't detect it. So 
  // manouever the robot around the obstacle
  else if (numberOfSameReadings >= 1000 && objectProximity == -1) {
    performBlockageAlgorithm();
    numberOfSameReadings = 0;
  }
}

// Algorithm to guide the robot around an obstacle/wall
void performBlockageAlgorithm() {

  // Turn to the right in 25 millisecond increments and continue
  // searching for the flame
  calculateServoWriteNumbers("F_RIGHT");
  int timeTurned = 0;
  while (timeTurned < NINETY_DEGREES_TURN_TIME_FORWARD) {
    moveRobot(25);
    timeTurned += 25;
    if (checkForFlame()) {
      stopRobot();
      return; 
    }
  }

  stopRobot();

  // Determine how close any object on the right side is
  float rightTurnObstacleProximity = readUltrasonicSensor();

  // If there is nothing on the right, then continue forward in that direction
  if (rightTurnObstacleProximity == -1) {
    calculateServoWriteNumbers("FORWARD");
    return;
  }

  // Go back and turn left, and once again continue searching for the flame
  // at the same time
  calculateServoWriteNumbers("B_RIGHT");
  moveRobot(NINETY_DEGREES_TURN_TIME_BACKWARD);

  calculateServoWriteNumbers("F_LEFT");
  timeTurned = 0;
  while (timeTurned < NINETY_DEGREES_TURN_TIME_FORWARD) {
    moveRobot(25);
    timeTurned += 25;
    if (checkForFlame()) {
      stopRobot();
      return; 
    }
  }

  stopRobot();

  // Determine how close any object on the left side is
  float leftTurnObstacleProximity = readUltrasonicSensor();

  // If there is nothing on the left, or if the object on the left is further away than the one on the right
  // then continue forward in the left direction
  if (leftTurnObstacleProximity == -1 || leftTurnObstacleProximity > rightTurnObstacleProximity) {
    calculateServoWriteNumbers("FORWARD");
    return;
  }
  else {
    // Otherwise, turn back to the right side and then continue forward in the right direction

    // First turn back to the left
    calculateServoWriteNumbers("B_LEFT");
    timeTurned = 0;
    while (timeTurned < NINETY_DEGREES_TURN_TIME_BACKWARD) {
      moveRobot(25);
      timeTurned += 25;
      if (checkForFlame()) {
        stopRobot();
        return; 
      }
    }
    stopRobot();

    // Then go to the right
    calculateServoWriteNumbers("F_RIGHT");
    timeTurned = 0;
    while (timeTurned < NINETY_DEGREES_TURN_TIME_FORWARD) {
      moveRobot(25);
      timeTurned += 25;
      if (checkForFlame()) {
        stopRobot();
        return; 
      }
    }
    stopRobot();

    // Start going forward to proceed in the right direction
    calculateServoWriteNumbers("FORWARD");
    return;
  }

}

// Function used to read the fireSensor
int readFireSensor() {
  int fireSensorReading = analogRead(fireSensorPin);
  return map(0, 1023, 0, 255, fireSensorReading);
}

// Function used to turn the fan on or off
void switchFan(boolean state) {
  if (state == true) {
    digitalWrite(fanControlPin, HIGH);
  }
  else {
    digitalWrite(fanControlPin, LOW);
  }
}

// Function used to check for the flame, and, if detected,
// perform the appropriate algorithm to extinguish the fire
// Returns true if a flame was detected and then extinguished
// Else returns false
boolean checkForFlame() {
  // Read the flame sensor
  int flameSensorReading = readFireSensor();

  // This code will check to see if a flame is near the robot. It reads the flame
  // sensor and records the number of times that it detects a flame (which we call "HIGH" readings). 
  // If the number of consecutive "HIGH" readings is greater than or equal to 5, then there is
  // definitely a flame near the robot.. so perform the extinguishFlame algorithm. In this way
  // this algorithm is able to successfully ignore false "HIGH"s from the flame sensor
  while (flameSensorReading >= minimumFlameDetectionNumber) {
    // If a flame is detected, then stop the robot and do some extra "sensing" to ensure
    // that the reading is not a false "HIGH"
    stopRobot();
    numberOfHIGHFlameSensorReadings++;

    // If the number of "HIGH" readings reaches or exceeds 5, then perform the flame algorithm
    if (numberOfHIGHFlameSensorReadings >= 5) {
      performFlameAlgorithm();
      numberOfHIGHFlameSensorReadings = 0;
      return true;
    } else {
      flameSensorReading = readFireSensor();
    }
  }

  numberOfHIGHFlameSensorReadings = 0;
  return false;
}

void performFlameAlgorithm() {
  // Call the orientRobotToFlame() method, which will orient the robot towards the flame
  // so that it is able to extinguish the fire
  orientRobotToFlame();

  stopRobot();

  // The code below will move the robot towards the flame so that it is close enough to
  // extinguish it. It will automatically reorient itself towards the flame (if needed) using
  // the following algorithm:
  // Normally, as the robot moves closer to the flame, the sensor's value should increase.
  // However, if the robot is not correctly oriented at the flame, then the values it reports
  // will gradually start to decrease (which is how the robot knows that it should reorient itself).
  // When this is detected, the robot calls the oreintRobotToFlame() function to realign itself with the flame.

  calculateServoWriteNumbers("FORWARD");

  int latestFlameSensorReading = readFireSensor();
  int highestFlameSensorReading = latestFlameSensorReading;
  int numberOfLowerReadings = 0;
  
  while (latestFlameSensorReading < optimalFlameDetectionNumber) {

    // Calculate the difference between the previous (highest) reading and the current reading
    int difference = latestFlameSensorReading - highestFlameSensorReading;
    // If the difference is less than -5, then stop the robot and do some rescanning to ensure that this isn't a 
    // false alarm
    if (difference < -5) {
      stopRobot();
      numberOfLowerReadings++;
    } 
    // If the difference is between 0 and -5 (exclusive), then treat it as if it was the same reading (we need this much tolerance to ensure
    // accuracy in the sensor's readings)
    else if (difference < 0) {
      numberOfLowerReadings = 0;
      moveRobot(250);
      stopRobot();
    }
    // Otherwise if there is no difference or if the current reading is higher than the previous, then continue forward since the robot
    // is going in the correct direction
    else {
      highestFlameSensorReading = latestFlameSensorReading;
      numberOfLowerReadings = 0;
      moveRobot(250);
      stopRobot();
    }

    // If the number of consecutive lower readings is greater than or equal to 5, then the robot is definitely going in the wrong direction
    // so back up and reorient before continuing forward
    if (numberOfLowerReadings >= 5) {
      calculateServoWriteNumbers("BACKWARD");
      moveRobot(250);
      stopRobot();
      orientRobotToFlame();
      calculateServoWriteNumbers("FORWARD");
      numberOfLowerReadings = 0;
      delay(15);
    }

    latestFlameSensorReading = readFireSensor();
    
  }

  stopRobot();

  numberOfLowerReadings = 0;

  // When the robot has reached the appropriate distance from the flame, then switch the fan on
  // and keep it on until the flame is extinguished (i.e. until the flame sensor stops reporting that
  // there is a flame)

  switchFan(true);

  while (numberOfLowerReadings < 5) {
    delay(15);
    if (readFireSensor() >= minimumFlameDetectionNumber) {
      numberOfLowerReadings = 0;
    } else {
      numberOfLowerReadings++;
    }
  }
  // Keep the fan on for an extra 3 seconds (in case the flame was almost extinguished but not fully,the robot would
  // think it is done when it actually isn't, so we need to add this "error checking")
  delay(3000);
  switchFan(false);

  // Make the robot move forward and call the function that will find the dowel and knock it over
  calculateServoWriteNumbers("FORWARD");
  findAndKnockOverDowel();
}

// Function that orients the robot so that it is facing the flame
void orientRobotToFlame() {

  // The algorithm to orient the robot is as follows:
  // Move the robot to the left. As long as the flame sensor reading is increasing, that means
  // that the robot is orienting itself closer to the flame. As soon as the number starts to 
  // decrease, that means that the robot has gone too much to the left. So start moving to the right
  // and do the same thing as above to orient the robot with the flame.
  
  int highestFlameSensorReading = 0;
  int latestFlameSensorReading = 0;

  int numberOfLowerReadings = 0;

  // Turn to the left and continue until the sensor values start to decrease

  calculateServoWriteNumbers("F_LEFT");

  while (numberOfLowerReadings < 5) {
    latestFlameSensorReading = readFireSensor();

    // Calculate the difference between the previous (highest) reading and the current reading
    int difference = latestFlameSensorReading - highestFlameSensorReading;

    // If the difference is less than -5, then stop the robot and do some rescanning to ensure that this isn't a 
    // false alarm    
    if (difference < -5) {
      stopRobot();
      numberOfLowerReadings++;
      delay(15);
    } 
    // If the difference is between 0 and -5 (exclusive), then treat it as if it was the same reading (we need this much tolerance to ensure
    // accuracy in the sensor's readings)
    else if (difference < 0) {
      numberOfLowerReadings = 0;
      moveRobot(250);
      stopRobot();
    } 
    // Otherwise if there is no difference or if the current reading is higher than the previous, then continue turning since the robot
    // is going in the correct direction
    else {
      numberOfLowerReadings = 0;
      highestFlameSensorReading = latestFlameSensorReading;
      moveRobot(250);
      stopRobot();
    }
  }

  numberOfLowerReadings = 0;
  highestFlameSensorReading = 0;

  calculateServoWriteNumbers("B_LEFT");
  moveRobot(250);
  stopRobot();

  // Start moving to the right and continue until the sensor values start to decrease
  calculateServoWriteNumbers("F_RIGHT");

  while (numberOfLowerReadings < 5) {
    latestFlameSensorReading = readFireSensor();

    // Calculate the difference between the previous (highest) reading and the current reading
    int difference = latestFlameSensorReading - highestFlameSensorReading;

    // If the difference is less than -5, then stop the robot and do some rescanning to ensure that this isn't a 
    // false alarm    
    if (difference < -5) {
      stopRobot();
      numberOfLowerReadings++;
      delay(15);
    } 
    // If the difference is between 0 and -5 (exclusive), then treat it as if it was the same reading (we need this much tolerance to ensure
    // accuracy in the sensor's readings)
    else if (difference < 0) {
      numberOfLowerReadings = 0;
      moveRobot(250);
      stopRobot();
    } 
    // Otherwise if there is no difference or if the current reading is higher than the previous, then continue turning since the robot
    // is going in the correct direction    
    else {
      numberOfLowerReadings = 0;
      highestFlameSensorReading = latestFlameSensorReading;
      moveRobot(250);
      stopRobot();
    }
  }

  calculateServoWriteNumbers("B_RIGHT");
  moveRobot(250);
}

// Function used to find the dowel and knock it over
void findAndKnockOverDowel() {

  // Move backwards from the candle to ensure that the robot has space to turn and doesn't
  // keep hitting the candle
  calculateServoWriteNumbers("BACKWARD"); 
  moveRobot(500);
  stopRobot();

  // To find the dowel and knock it over, the following algorithm is used:
  // Turn the robot in small increments (approximately 10 degrees) until the angle turned reaches
  // 360 degrees. In each increment, move the robot forward until it hits an object (a wall or the dowel)
  // If, before hitting the object, the ultrasonic sensor's reading is different than after hitting, then that means
  // that the object has fell over (meaning it was a dowel). Otherwise, if the number is approximately the same, that means
  // that the object is a wall (since it didn't fall over), and so the robot will backup to the original location and then
  // move to the next 10 degrees increment.
  
  int timeTurned = 0; // Variable storing how much the robot has turned
  while (timeTurned < 4*NINETY_DEGREES_TURN_TIME_FORWARD && !hasDowelBeenKnockedOver) {
    // Move approximately 10 degrees to the left
    calculateServoWriteNumbers("F_LEFT");
    moveRobot(750);
    stopRobot();
    timeTurned += 750;

    calculateServoWriteNumbers("FORWARD");

    int timeForward = 0; // Variable storing how long the robot has gone forward (used to help the object get back to its original position)
    int ultrasonicReading = readUltrasonicSensor();
    // Continue moving the robot forward until it hits the dowel/wall and count how long
    // the robot has been moving forward
    while (ultrasonicReading > almostTouchingDowelNumber || ultrasonicReading == -1) {
      moveRobot(100);
      timeForward += 100;
      ultrasonicReading = readUltrasonicSensor();
    }

    // Bump into the object
    moveRobot(250);
    timeForward += 250;

    // Check if the reading from the sensor is different, and if it is less or if the reading is -1,
    // then the dowel was successfully knocked over
    ultrasonicReading = readUltrasonicSensor();
    if (ultrasonicReading == -1 || ultrasonicReading > OBSTACLE_IN_FRONT_THRESHOLD) {
      hasDowelBeenKnockedOver = true;
    }

    calculateServoWriteNumbers("BACKWARD");

    // Reverse to the original position
    int timeBackward = 0;
    while (timeBackward < timeForward) {
      moveRobot(100);
      timeBackward += 100;
    }
    
  }

  hasDowelBeenKnockedOver = true;

}


