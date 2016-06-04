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
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

#define MAX_DEGREE 175 // max without weird jitter
#define MIN_DEGREE 10   // min without weird jitter

void lidEvent(const std_msgs::Int32MultiArray& servo_msg);

Servo servos[4];
int pins[4] = {10, 6, 11, 9};
int buttonPin = 12;

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32MultiArray> sub("snacbot/servo", &lidEvent );
std_msgs::String msg;
ros::Publisher pub("snacbot/done", &msg);

void setup() {  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  pinMode(buttonPin, INPUT);

  int i;
  for (i = 0; i < 4; i++) {
    servos[i].attach(pins[i]);
    servos[i].write((i % 2 == 0) ? MAX_DEGREE : MIN_DEGREE);
  }
}

void loop() {
    int buttonStatus = digitalRead(buttonPin);
    if (buttonStatus == HIGH) {
      int i;
      for (i = 0; i < 4; i++) {
        servos[i].write((i % 2 == 0)? MAX_DEGREE : MIN_DEGREE);  
      }
      msg.data = "done";
      pub.publish(&msg);
    }    
    nh.spinOnce();
    delay(1);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void lidEvent(const std_msgs::Int32MultiArray& servo_msg) {
    Serial.println("Event received");
    Serial.print("Servo num: ");
    Serial.println(servo_msg.data[0]  );
    Serial.print("Degree: ");
    Serial.println(servo_msg.data[1]);
    int servoNum = servo_msg.data[0];
    int degree = servo_msg.data[1];
  	if (degree > MAX_DEGREE)
  	  degree = MAX_DEGREE;
  
  	if (degree < MIN_DEGREE)
  	  degree = MIN_DEGREE;
    servos[servoNum].write(degree);
}


