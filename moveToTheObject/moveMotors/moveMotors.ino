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
int currentPanPosition = 95;
int currentTiltPosition = 45;

int leftPWM = 0;
int rightPWM = 0;
int time = 0;


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
      //map values from serial between 1-255
      leftPWM = Serial.read();
      leftPWM = map(leftPWM, 0, 255, -255, 255);

      rightPWM = Serial.read();
      rightPWM = map(rightPWM, 0, 255, -255, 255);
      
      move(leftPWM, rightPWM);
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

void move(int leftPWM, int rightPWM) {

  // stop the motors
  if (leftPWM == -1 && rightPWM == -1) {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    Serial.println("Stop the motors");
    Serial.print("leftPWM: ");
    Serial.println(leftPWM);
    Serial.print("rightPWM: ");
    Serial.println(rightPWM);
  }

  // move stright
  else if (rightPWM > 0 && leftPWM > 0) {
    analogWrite(enA, rightPWM);
    analogWrite(enB, leftPWM);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    Serial.println("Move stright");
    Serial.print("leftPWM: ");
    Serial.println(leftPWM);
    Serial.print("rightPWM: ");
    Serial.println(rightPWM);
  }

  // turn left on the point
  else if (rightPWM > 0 && leftPWM <= -1) {
    analogWrite(enA, rightPWM);
    analogWrite(enB, abs(leftPWM));
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Serial.println("Turn left on the point");
    Serial.print("leftPWM: ");
    Serial.println(leftPWM);
    Serial.print("rightPWM: ");
    Serial.println(rightPWM);
  }

  // turn right on the point
  else if (rightPWM <= -1 && leftPWM > 0) {
    analogWrite(enA, abs(rightPWM));
    analogWrite(enB, leftPWM);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    Serial.println("Turn right on the point");
    Serial.print("leftPWM: ");
    Serial.println(leftPWM);
    Serial.print("rightPWM: ");
    Serial.println(rightPWM);
  }
  
  // go back
  else if (rightPWM < -1 && leftPWM < -1) {
    analogWrite(enA, abs(rightPWM));
    analogWrite(enB, abs(leftPWM));
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Serial.println("Go back");
    Serial.print("leftPWM: ");
    Serial.println(leftPWM);
    Serial.print("rightPWM: ");
    Serial.println(rightPWM);
  }
  
  else {
    Serial.println("Else statement");
    Serial.print("leftPWM: ");
    Serial.println(leftPWM);
    Serial.print("rightPWM: ");
    Serial.println(rightPWM);
  }

}




