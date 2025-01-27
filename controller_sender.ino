// this code is for reading joystick values, mapping them and transmitting them to the mini sumo bot car

#include <esp_now.h> // we are using ESP-NOW communication
#include <WiFi.h>

// REPLACE WITH RECEIVER MAC Address (this is needed for pairing the ESPs, to find the MAC address use  Get_MAC_Address_ESP32.ino)
// each sender ESP will have a different receiver MAC address
uint8_t broadcastAddress[] = {0xA0, 0xB7, 0x65, 0x26, 0x9B, 0xAC};

// joystick pin values
uint8_t joystick_x_pin = 39;
uint8_t joystick_y_pin = 36;

// structure to send data (x and y coordinates from joystick reading)
// must match the receiver structure
typedef struct struct_message {
  int jstk_x; 
  int jstk_y;
} struct_message;

// struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
// checks the status of the package sent; if it was delivered or not
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// map raw joystick values (0-4095) to 0-254 for the DC motors to use in the receiver code
int mapJoystickValue(int value) {
    int deadZone = 40;  // ignore minor fluctuations around the center, this can be fine-tuned
    int minInput = 0;
    int maxInput = 4095;
    int center = 1860; // change to each joystick's 'neutral' reading
    // in theory, center should be (maxInput + minInput) / 2;
    // but we calibrated it to the 'neutral' joystick readings

    if (abs(value - center) < deadZone) return 127;  // Neutral position
    if (value > center) return map(value, center, maxInput, 127, 254);  // Forward/Right
    return map(value, minInput, center, 0, 127);  // Backward/Left
}
 
void setup() {
  // initialize serial monitor
  Serial.begin(115200);
  analogReadResolution(12);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // esp_now_register_send_cb is an ESP-NOW function which basically tells the ESP "every time you send a package, run OnDataSent()"
  // thus getting the status of the transmitted package
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer using receiver MAC address
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
  
  // if you want to check the raw readings from the joystick, uncomment this
  // Serial.print("Joystick x: ");
  // Serial.println(x_adc_val);
  // Serial.print("Joystick y: ");
  // Serial.println(y_adc_val);

  // set the data in the package to the mapped x and y coordinates
  myData.jstk_x = mapJoystickValue(x_adc_val);
  myData.jstk_y = mapJoystickValue(y_adc_val);

  //if you want to check the mapped values which are being sent, uncomment this
  // Serial.print("Joystick x: ");
  // Serial.println(myData.jstk_x);
  // Serial.print("Joystick y: ");
  // Serial.println(myData.jstk_y);
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  // confirm if package was sent successfully
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
