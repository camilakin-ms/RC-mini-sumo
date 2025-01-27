#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xA0, 0xB7, 0x65, 0x26, 0x9B, 0xAC};

// joystick values
uint8_t joystick_x_pin = 39;
uint8_t joystick_y_pin = 36;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int jstk_x;
  int jstk_y;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

int mapJoystickValue(int value/*, int minInput, int maxInput*/) {
    int deadZone = 40;  // Ignore minor fluctuations around the center
    int minInput = 0;
    int maxInput = 4095;
    int center = 1860; //(maxInput + minInput) / 2;

    if (abs(value - center) < deadZone) return 127;  // Neutral position
    if (value > center) return map(value, center, maxInput, 127, 254);  // Forward/Right
    return map(value, minInput, center, 0, 127);  // Backward/Left
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  analogReadResolution(12);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}
 
 void loop() {

  // read joystick values
  int x_adc_val, y_adc_val; 
  x_adc_val = analogRead(joystick_x_pin); 
  y_adc_val = analogRead(joystick_y_pin);
  // Serial.print("Joystick x: ");
  // Serial.println(x_adc_val);
  // Serial.print("Joystick y: ");
  // Serial.println(y_adc_val);
  myData.jstk_x = mapJoystickValue(x_adc_val);
  myData.jstk_y = mapJoystickValue(y_adc_val);
  Serial.print("Joystick x: ");
  Serial.println(myData.jstk_x);
  Serial.print("Joystick y: ");
  Serial.println(myData.jstk_y);
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
    Serial.print("Joystick x: ");
    Serial.println(myData.jstk_x);
    Serial.print("Joystick y: ");
    Serial.println(myData.jstk_y);
  }
  else {
    Serial.println("Error sending the data");
  }

}