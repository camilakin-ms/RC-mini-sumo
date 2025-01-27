#include <esp_now.h>
#include <WiFi.h>
#include <dummy.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int jstk_x;
  int jstk_y;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// motor pins
#define IN1 25 // ENA
#define IN2 26 // ENA
#define IN3 27 // ENB
#define IN4 14 // ENB
#define ENA 33
#define ENB 12

// director and acceleration control
void driveMotors(int xAxis, int yAxis) {
    int leftMotorSpeed = yAxis + (xAxis - 127);
    int rightMotorSpeed = yAxis - (xAxis - 127);

    leftMotorSpeed = constrain(leftMotorSpeed, -254, 254);
    rightMotorSpeed = constrain(rightMotorSpeed, -254, 254);

    analogWrite(ENA, abs(leftMotorSpeed));
    digitalWrite(IN1, leftMotorSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN2, leftMotorSpeed < 0 ? HIGH : LOW);

    analogWrite(ENB, abs(rightMotorSpeed));
    digitalWrite(IN3, rightMotorSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN4, rightMotorSpeed < 0 ? HIGH : LOW);

    // analogWrite(ENA, abs(leftMotorSpeed));
    // digitalWrite(IN1, leftMotorSpeed > 0 ? HIGH : LOW);

    // analogWrite(ENB, abs(rightMotorSpeed));
    // digitalWrite(IN3, rightMotorSpeed > 0 ? HIGH : LOW);
}


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Joystick X: ");
  Serial.println(myData.jstk_x);
  Serial.print("Joystick Y: ");
  Serial.println(myData.jstk_y);
  Serial.println();

  driveMotors(myData.jstk_x, myData.jstk_y);
}


void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {

}