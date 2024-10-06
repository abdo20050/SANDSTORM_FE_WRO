#include <Wire.h>
#include <LiquidCrystal_I2C.h> // I2C LCD library
#include <Pixy2.h>             // Pixy Cam library
#include <NewPing.h>           // Library for ultrasonic sensors
#include <Servo.h>             // Servo library
#include <MPU6050.h>           // IMU library for MPU6050

Pixy2 pixy;  // Create a Pixy2 object
MPU6050 imu; // IMU object

// Initialize the I2C LCD (address 0x27 is common for 16x2 LCDs)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Ultrasonic sensor pins
#define TRIG_PIN_FRONT 7
#define ECHO_PIN_FRONT 6
#define TRIG_PIN_LEFT 5
#define ECHO_PIN_LEFT 4
#define TRIG_PIN_RIGHT 9
#define ECHO_PIN_RIGHT 8

// Motor pins for driving
#define MOTOR_DRIVE_PWM 10  // PWM pin for motor speed control
#define MOTOR_DRIVE_FORWARD 11
#define MOTOR_DRIVE_BACKWARD 12

// Servo motor for steering
Servo steeringServo;   // Servo object for steering

// Steering angles
int STRAIGHT_ANGLE = 100;      // Default angles
int SOFT_LEFT_ANGLE = 80;
int HARD_LEFT_ANGLE = 40;
int SOFT_RIGHT_ANGLE = 120;
int HARD_RIGHT_ANGLE = 140;

// Define ultrasonic sensor distance threshold
#define OBSTACLE_DISTANCE 30

// Create ultrasonic sensor objects
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, 200);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, 200);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, 200);

// Lap counting variables
int lapCount = 0;
int motorSpeed = 150; // Initial motor speed (range: 0-255)
String steeringDirection = "Straight"; // Variable to store steering direction

// Motor control functions for driving
void moveForward() {
  analogWrite(MOTOR_DRIVE_PWM, motorSpeed);  // Set motor speed using PWM
  digitalWrite(MOTOR_DRIVE_FORWARD, HIGH);
  digitalWrite(MOTOR_DRIVE_BACKWARD, LOW);
}

void stopMovement() {
  analogWrite(MOTOR_DRIVE_PWM, 0);  // Set speed to 0 to stop
  digitalWrite(MOTOR_DRIVE_FORWARD, LOW);
  digitalWrite(MOTOR_DRIVE_BACKWARD, LOW);
}

// Steering control functions
void steerStraight() {
  steeringServo.write(STRAIGHT_ANGLE);  // Set servo to straight
  steeringDirection = "Straight";
}

void softLeft() {
  steeringServo.write(SOFT_LEFT_ANGLE);  // Slight left turn
  steeringDirection = "Soft Left";
}

void hardLeft() {
  steeringServo.write(HARD_LEFT_ANGLE);  // Sharp left turn
  steeringDirection = "Hard Left";
}

void softRight() {
  steeringServo.write(SOFT_RIGHT_ANGLE);  // Slight right turn
  steeringDirection = "Soft Right";
}

void hardRight() {
  steeringServo.write(HARD_RIGHT_ANGLE);  // Sharp right turn
  steeringDirection = "Hard Right";
}

// Function to read distance from an ultrasonic sensor
int getDistance(NewPing sonar) {
  return sonar.ping_cm();
}

// Pixy Cam object detection (Red = Right, Green = Left)
String detectColor() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) {
        return "red";  // Red color detected
      } else if (pixy.ccc.blocks[i].m_signature == 2) {
        return "green";  // Green color detected
      }
    }
  }
  return "none";  // No color detected
}

// Initialize IMU and calculate initial yaw angle
void initializeIMU() {
  imu.initialize();
}

// Get the current yaw angle from the gyroscope
float getYaw() {
  int16_t gx, gy, gz; // Gyroscope readings
  imu.getRotation(&gx, &gy, &gz); // Read gyroscope data
  return gx / 131.0; // Simplified conversion; adjust as needed
}

// Check for lap completion (crossing 360-degree mark)
void checkLap() {
  // Detect if the robot completes a lap
  if (lapCount >= 3) {
    stopMovement();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Laps: 3 Completed");
    while (1);  // Halt the robot
  }
}

void setup() {
  // Motor setup
  pinMode(MOTOR_DRIVE_FORWARD, OUTPUT);
  pinMode(MOTOR_DRIVE_BACKWARD, OUTPUT);
  pinMode(MOTOR_DRIVE_PWM, OUTPUT);  // PWM pin setup for speed control

  // Servo setup for steering
  steeringServo.attach(12);  // Attach the servo to pin 12
  steerStraight();  // Set the servo to straight position

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Robot Starting");
  delay(2000);  // Show startup message for 2 seconds
  lcd.clear();

  // Initialize Pixy2, Serial for debugging, and IMU
  pixy.init();
  Serial.begin(9600);
  initializeIMU();  // Initialize the IMU

  // Instruct the user to adjust angles via Serial Monitor
  Serial.println("Enter commands like 'straight90', 'softL110', 'hardR30' to adjust servo angles.");
}

void loop() {
  // Get distance from ultrasonic sensors
  int frontDistance = getDistance(sonarFront);
  int leftDistance = getDistance(sonarLeft);
  int rightDistance = getDistance(sonarRight);

  // Update LCD with front distance and steering direction
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(frontDistance);
  lcd.print(" cm");
  lcd.setCursor(0, 1);
  lcd.print("Steer: ");
  lcd.print(steeringDirection);

  // Check for color detection from Pixy2
  String color = detectColor();
  if (color == "green") {
    Serial.println("Green detected, soft left steering.");
    softLeft();
    delay(1000);  // Adjust delay for steering
    steerStraight();  // Return to straight position
  } else if (color == "red") {
    Serial.println("Red detected, soft right steering.");
    softRight();
    delay(1000);
    steerStraight();  // Return to straight position
  }

  // Obstacle avoidance using ultrasonic sensors
  if (frontDistance < OBSTACLE_DISTANCE) {
    stopMovement();
    if (leftDistance > rightDistance) {
      hardLeft();  // Hard left if right side has more obstacles
    } else {
      hardRight();  // Hard right if left side has more obstacles
    }
    delay(500);
    steerStraight();  // Return to straight position
  } else if (leftDistance < OBSTACLE_DISTANCE) {
    softRight();  // Soft right turn
    delay(500);
    steerStraight();  // Return to straight position
  } else if (rightDistance < OBSTACLE_DISTANCE) {
    softLeft();  // Soft left turn
    delay(500);
    steerStraight();  // Return to straight position
  } else {
    moveForward();  // Move forward if no obstacles
  }

  // Check lap completion
  checkLap();  // Check if a lap has been completed

  delay(100);  // Delay between readings
}
