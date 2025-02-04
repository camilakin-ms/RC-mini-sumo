
// this file receives and reads a package via ESP-NOW communication, and runs motors speed and direction with them

#include <esp_now.h>
#include <WiFi.h>
#include <dummy.h>

// structure  to receive data, x and y coordinates from joystick (already mapped from 0-254 for use w/ DC motors)
// must match the sender structure
typedef struct struct_message {
  int jstk_x;
  int jstk_y;
} struct_message;

// create a struct_message called myData
struct_message myData;

// motor pins
#define ENA 33 // control powe (speed)
#define IN1 25 // control direction for ENA 'forward'
#define IN2 26 // control direction for ENA 'backward'

#define ENB 12 // control power (speed)
#define IN3 27 // control direction for ENB 'forward'
#define IN4 14 // control direction for ENB 'backward'


// *********************************************
// THIS IS THE PART THAT NEEDS ATTENTION!!!!!!!
// *********************************************
// we want to have it make backward motion too, and that when it is neutral (x=y=127), it does not move

// director and acceleration control
// currently just for FORWARD motion 
void driveMotors(int xAxis, int yAxis) {
  // this is the math that makes turning smooth
  // understand how it works before altering it
  
    if (xAxis == 127 && yAxis == 127) {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    return;
  }
  // Map the values to the proper range
  int mappedX = map(xAxis, 0, 254, -254, 254);
  int mappedY = map(yAxis, 0, 254, -254, 254);
  
  // Compute motor speeds with mapped values
  int leftMotorSpeed = mappedY + mappedX;
  int rightMotorSpeed = mappedY - mappedX;

  //Constrain the values so they cant exceed what can be read
  leftMotorSpeed = constrain(leftMotorSpeed, -254, 254);
  rightMotorSpeed = constrain(rightMotorSpeed, -254, 254);

  // Motor A (left)
  analogWrite(ENA, abs(leftMotorSpeed)); // Use absolute value for speed
  if (leftMotorSpeed > 0) { // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else { // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }


  // Motor B (right)
  analogWrite(ENB, abs(rightMotorSpeed)); // Use absolute value for speed
  if (rightMotorSpeed > 0) { // Forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else { // Backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
 // digitalWrite(IN3, rightMotorSpeed > 0 ? HIGH : LOW);
  // same with IN4 and 'backward' motion
}


// callback function that will be executed EVERY TIME data is received
// think of this as the loop() function usually used
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  
  // copies the incoming data to the structure myData
  memcpy(&myData, incomingData, sizeof(myData));

  // for validation, serial prints data received
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Joystick X: ");
  Serial.println(myData.jstk_x);
  Serial.print("Joystick Y: ");
  Serial.println(myData.jstk_y);
  Serial.println();

  // calls movement function for the motors with updated x and y values from joystick
  driveMotors(myData.jstk_x, myData.jstk_y);
}


void setup() {
  // initialize Serial Monitor
  Serial.begin(115200);
  
  // set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // set pinmode for all motor driver pins
  // motor A
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // motor B
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  
  // initialize motors to be stopped 
  digitalWrite(IN1, LOW); // motor A 'forward' motion = 0
  digitalWrite(IN2, LOW); // motor A 'backward' motion = 0
  digitalWrite(IN3, LOW); // motor B 'forward' motion = 0
  digitalWrite(IN4, LOW); // motor B 'backward' motion = 0

  // initialize speed to 0 in both motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);






  // once ESPNow is successfully initialized
  // esp_now_register_recv_cb tells the ESP32 "every time you receive data, execute OnDataRecv"
  // effectively updatating motor instructions every time new x and y values are obtained
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}


// the reason loop() is empty is because we don't want any code to run if we don't have x and y values from the joystick
// if you need to troubleshoot motor motion, do it here, but in general
// ALL MOTOR CONTROL SHOULD HAPPEN IN THE FUNCTION OnDataRecv() !!!!!
void loop() {

}
