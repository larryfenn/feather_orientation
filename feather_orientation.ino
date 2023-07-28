#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
// 0x29 for the little tiny board
// 0x28 for the bigger one
const int button_1_pin = 0;
const int button_2_pin = 16;
const int button_3_pin = 2;

const int DATAGRAM_REPEATS = 10;

int old_button_1_state = 0;
int old_button_2_state = 0;
int old_button_3_state = 0;
int button_1_3_hold_counter = 0;

IPAddress server(192,168,1,52);
WiFiUDP conn;
uint8_t id;
int16_t old_w = 0;
int16_t old_x = 0;
int16_t old_y = 0;
int16_t old_z = 0;
uint8_t action_flag = 0;
int action_flag_repeats = 0;

void setup(void)
{
  Serial.begin(115200);
  Serial.print("Connecting");
  WiFi.begin("aleph", "shibboleth");
  Serial.print("Connected to wifi");
  conn.begin(5004);
  Serial.print("UDP client created");
  pinMode(button_1_pin, INPUT);
  pinMode(button_2_pin, INPUT);
  pinMode(button_3_pin, INPUT);
  bno.begin(OPERATION_MODE_NDOF);
  bno.setExtCrystalUse(true);
}

void loop(void)
{
  int button_1_state = digitalRead(button_1_pin);
  int button_2_state = digitalRead(button_2_pin);
  int button_3_state = digitalRead(button_3_pin);  
  // Actions:
  // 1: Button 1 clicked down (i.e. hold and release is disregarded)
  // 2: Button 2 clicked down
  // 3: Button 3 clicked down
  // 4: press and hold buttons 1 and 3 for a bit (triggers calibration)
  // these set the action flag
  // once the action flag is set, it can't have any other value for some time
  // this is because we can't guarantee any particular UDP datagram gets received
  // so we send the same action flag in a series of datagrams in the hope that it gets picked up
  // action flag unset -- we're free to try to assign to it
  if (action_flag == 0) {
    // first we just check if any button is being pressed; i.e. the current state is HIGH and the old state is LOW
    if (button_1_state != old_button_1_state && button_1_state == LOW) {
      action_flag = 1;
    }
    if (button_2_state != old_button_2_state && button_2_state == LOW) {
      action_flag = 2;
    }
    if (button_3_state != old_button_3_state && button_3_state == LOW) {
      action_flag = 3;
    }
    // if in fact button 1 and 3 are being held then we override the action flag
    if (button_1_state == old_button_1_state && button_3_state == old_button_3_state && button_1_state == LOW && button_3_state == LOW) {
      button_1_3_hold_counter++;
    }
    if (button_1_3_hold_counter > 500) {
      action_flag = 4;
      button_1_3_hold_counter = 0;
    }    
  }

  id = WiFi.localIP()[3];
  imu::Quaternion quat = bno.getQuat();
  int16_t w = quat.w() * 16384;
  int16_t x = quat.x() * 16384;
  int16_t y = quat.y() * 16384;
  int16_t z = quat.z() * 16384;
  if(old_w != w || old_x != x || old_y != y || old_z != z) {
    conn.beginPacket(server, 5005);
    conn.write(reinterpret_cast<char*>(&id), sizeof(id));
    auto time = millis();
    conn.write(reinterpret_cast<const char*>(&time), sizeof(time));
    conn.write(reinterpret_cast<char*>(&w), sizeof(w));
    conn.write(reinterpret_cast<char*>(&x), sizeof(x));
    conn.write(reinterpret_cast<char*>(&y), sizeof(y));
    conn.write(reinterpret_cast<char*>(&z), sizeof(z));
    conn.write(reinterpret_cast<char*>(&action_flag), sizeof(action_flag));
    conn.endPacket();
    if (action_flag != 0 && action_flag_repeats > DATAGRAM_REPEATS) {
      action_flag = 0;
      action_flag_repeats = 0;
    } else if (action_flag != 0) {
      action_flag_repeats++;
    }
    old_w = w;
    old_x = x;
    old_y = y;
    old_z = z;
  }
  old_button_1_state = button_1_state;
  old_button_2_state = button_2_state;
  old_button_3_state = button_3_state;
}
