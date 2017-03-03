// Import the Arduino Servo library
#include <Servo.h> 

// servo motors
#define pan 5
#define tilt 6

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


void setup() 
{ 
  // Attach each Servo object to a digital pin
  panServo.attach(pan, minPulse, maxPulse);
  tiltServo.attach(tilt, minPulse, maxPulse);

  // Define start Servo angle position
  panServo.write(currentPanPosition);
  tiltServo.write(currentTiltPosition);

  // Open the serial connection, 9600 baud
  Serial.begin(9600);
} 

void loop() 
{ 
  // Wait for serial input (min 3 bytes in buffer)
  if (Serial.available() > 2 ) {
    // Read the first byte
    startbyte = Serial.read();

    // If it's really the startbyte (255) ...
    if (startbyte == 255) {
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
  }
}