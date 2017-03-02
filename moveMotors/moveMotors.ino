// Import the Arduino Servo library
#include <Servo.h> 

// servo motors
#define pan 5
#define tilt 6

// connect motor controller pins to Arduino digital pins
// right motor
#define enA 11
#define in1 12
#define in2 13
// left motor
#define enB 3
#define in4 2
#define in3 7

// Create a Servo object for each servo
Servo panServo, tiltServo;

// Common servo setup values
int minPulse = 600;   // minimum servo position, us (microseconds)
int maxPulse = 2400;  // maximum servo position, us

// User input for servo and position
int startbyte;       // start byte, begin reading input
int servo;           // which servo to pulse?
int pos;             // servo angle 0-180
int currentPanPosition = 90;
int currentTiltPosition = 60;

int leftPWM = 0;
int rightPWM = 0;


void setup() 
{ 
  // Attach each Servo object to a digital pin
  panServo.attach(pan, minPulse, maxPulse);
  tiltServo.attach(tilt, minPulse, maxPulse);

  // Define start Servo angle position
  panServo.write(currentPanPosition);
  tiltServo.write(currentTiltPosition);

  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Open the serial connection, 9600 baud
  Serial.begin(9600);
} 

void loop() 
{ 
  // Wait for serial input (min 3 bytes in buffer)
  if (Serial.available() > 2 ) {
    // Read the first byte
    startbyte = Serial.read();

    switch (startbyte) {
    case 255:
      movePanTilt();
      break;
    case 254:
      leftPWM = Serial.read();
      Serial.print("Serial.print(leftPWM) ");
      Serial.println(leftPWM);
      rightPWM = Serial.read();
      Serial.print("Serial.print(rightPWM) ");
      Serial.println(rightPWM);
      moveStright(leftPWM, rightPWM);
      break;
    case 253:
      stopMotors();
      break;
    }


  }
}

void movePanTilt() {
  // What servo to move
  servo = Serial.read();
  if (servo == 1) {
    Serial.print("Change current angle of pan servo to: ");
  }
  else {
    Serial.print("Change current angle of tilt servo to: ");
  }

  // For which position
  pos = Serial.read();
  Serial.print(pos);
  Serial.println(" degrees");

  // Assign new position to appropriate servo
  switch (servo) {
  case 1:
    panServo.write(pos);
    break;
  case 2:
    tiltServo.write(pos);
    break;
  }
}

void moveStright(int leftPWM, int rightPWM) {
  // PWM for motors
  analogWrite(enA, rightPWM);
  analogWrite(enB, leftPWM);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


