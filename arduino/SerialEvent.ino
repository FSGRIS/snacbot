/*
  Serial Event example

 When new serial data arrives, this sketch adds it to a String.
 When a newline is received, the loop prints the string and
 clears it.

 A good test for this is to try it with a GPS receiver
 that sends out NMEA 0183 sentences.

 Created 9 May 2011
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/SerialEvent

 */
#include <Servo.h>

#define MAX_DEGREE 175 // max without weird jitter
#define MIN_DEGREE 10   // min without weird jitter

int degrees[4] = {0, 0, 0, 0};
boolean received = false;  // whether the string is complete
int servo0Pin = 9;
int servo1Pin = 10;
int servo2Pin = 11;
int servo3Pin = 6;

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  servo0.attach(servo0Pin);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);  
  servo3.attach(servo3Pin);  
}

void loop() {
  // print the string when a newline arrives:
  if (received) {
  	servo0.write(degrees[0]);
  	servo1.write(degrees[1]);
  	servo2.write(degrees[2]);
  	servo3.write(degrees[3]);	
  
  	received = false;
  
  	int i;
  	for (i = 0; i < 4; i++) {
  		//Serial.println(degrees[i]);
  	}
  	//Serial.println();
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
  	// get the new byte:
    byte buf[8];

    Serial.readBytes(buf, 8);
    int servoNum = *((int *) buf);
    degrees[servoNum] = *((int *)(buf + 4));
    Serial.println(servoNum);
    Serial.println(degrees[servoNum]);
    Serial.println();
  		
  	if (degrees[servoNum] > MAX_DEGREE)
  	  degrees[servoNum] = MAX_DEGREE;
  
  	if (degrees[servoNum] < MIN_DEGREE)
  	  degrees[servoNum] = MIN_DEGREE;		
  
  	received = true;
  }
}


