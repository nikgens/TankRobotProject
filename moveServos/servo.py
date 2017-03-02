# send data to Serial port
import serial
import struct

#define port
usbport = '/dev/ttyUSB0'
serialArduino = serial.Serial(usbport, 9600, timeout=1)
def move(servo, angle):
    '''Moves the specified servo to the supplied angle.

    Arguments:
        servo
          the servo number to command, an integer from 1-4
        angle
          the desired servo angle, an integer from 0 to 180

    (e.g.) >>> servo.move(2, 90)
           ... # "move servo #2 to 90 degrees"'''

    if (0 <= angle <= 180):
        serialArduino.write(struct.pack('>B', 255))
        serialArduino.write(struct.pack('>B', servo))
        serialArduino.write(struct.pack('>B', angle))
    else:
        print ("Servo angle must be an integer between 0 and 180.\n")