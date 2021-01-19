/*
  Author: V. Klopfenstein
  
  Overview:
    This arduino code controls a LED strip based on specific RC channel data.
    Such data is obtained using MAVLink.

    Altough over-engineeRed, this project also serves as an exercise to learn a bit more about (Free)RTOS

  Disclaimer:
    Pretty much all of the MAVLink interfacing code has been 'borrowed' from https://www.locarbftw.com/understanding-the-arduino-mavlink-library/ [Jan 2021]
    Their article was the first to actually explain how the arduino mavlink library works.    
    It was also the only public code that wasn't unnecessarily bloated. 
    
    For MAVLink message IDs, refer to:
      https://mavlink.io/en/messages/common.html#messages

  TODO:
    Bugfix turning indicators.
      > The LEFT indicator light does NOT return if still indicating AND reversing.
        Additionally, the RIGHT white LED never returns to RED.
      > It is possible to light both indicator lights if turning RIGHT first, then turning sharply applying LEFT.
        This is most likely due to the low update rate of the MAVLink publisher, as the midzone never gets detected.
        As such, the arduino never knows that it should stop indicating.
*/

// =========================================
// HEAD
// =========================================

#include <mavlink.h>
#include <SoftwareSerial.h>
#include <FastLED.h>

int availableMemory() {
  int size = 1024; // Use 2048 with ATmega328
  byte *buf;

  while ((buf = (byte *) malloc(--size)) == NULL)
    ;

  free(buf);

  return size;
}

SoftwareSerial mlSerial(12, 11); // RX, TX

// Channel settings
uint8_t chThr = 4;          // Channel used for throttle
uint8_t chYaw = 3;          // Channel used for yaw
uint8_t chDeviation = 100;  // Max deviation in ms from channel midpoint.
                            // Calculation is as follows: (chVal - chDeviation && chVal + chDeviation)
/*uint16_t chThrMid = 1500;    // Midpoint of throttle in ms
uint16_t chYawMid = 1500;    // Midpoint of yaw in ms*/
uint16_t chMap[4] = {};

// Misc settings
bool carReversing = false;
bool revLEDsChanged = false;
/*
  bool carIndicating = false;
  bool turnLEDsChanged = false;
*/
uint8_t carTurningDirection = 0;
  // - '1' for RIGHT, '2' for LEFT, 'N' for NONE

// LED settings
#define NUM_LEDS 7
#define LED_TYPE WS2811
CRGB leds[NUM_LEDS];

// =========================================
// MAIN
// =========================================

// Setup
void setup() {
  delay(300);
  Serial.begin(57600);    // Init arduino serial RX/TX
  mlSerial.begin(57600);  // Init serial RX/TX for MAVLink interface

  // LEDs setup
  // -------------------------------------
  // PARAMS: <LED_TYPE>, <LED_PIN>, <COLOR_ORDER>(leds, <NUM_LEDS>)
  FastLED.addLeds<LED_TYPE, 4, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(100);
  
  // Power down LEDs
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  // -------------------------------------

  // Set up foundation for MAVLink communications
  request_datastream();
  Serial.print(F("0 - "));
  Serial.println(availableMemory());

  // Show arduino bootup completion with LEDs
  // -------------------------------------
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      delay(50);
      leds[i] = CRGB::Green;
      FastLED.show();
    }

    delay(500);
    FastLED.setBrightness(50);
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Red;
    }
    FastLED.show();
  // -------------------------------------

  Serial.print(F("d - "));
  Serial.println(availableMemory());
  
  // Wait for FC to have fully booted up.
  delay(1000);
}

void loop() {
  request_MAVLinkData();
  // Channel watcher
      // Check if chThr is lesser than 1500 when accounting with deviation as well
      if (chMap[chThr - 1] < (1500 - chDeviation)) {
        carReversing = true;
        if (!revLEDsChanged) {
          Serial.println(F("LED > White"));
          leds[0] = CRGB::White;
          leds[1] = CRGB::White;
          leds[5] = CRGB::White;
          leds[6] = CRGB::White;
          FastLED.show();
          
          revLEDsChanged = true;
        }
      } else {
        carReversing = false;
        // Only revert LED colors IF car was previously reversing.
        // Otherwise, FastLED.show() gets called all the time.
        if (revLEDsChanged) {
          Serial.println(F("LED > Red"));
          leds[0] = CRGB::Red;
          leds[1] = CRGB::Red;
          leds[5] = CRGB::Red;
          leds[6] = CRGB::Red;
          FastLED.show();
          /*
          if (carIndicating) {
            switch (carTurningDirection) {
              case 2: { 
                Serial.println(F("LED LEFT > Orange"));
                leds[5] = CRGB::Red;
                leds[6] = CRGB::Red;
                FastLED.show();
              };
              break;
              case 1: { 
                  Serial.println(F("LED RIGHT > Orange"));
                  leds[0] = CRGB::Red;
                  leds[1] = CRGB::Red;
                  FastLED.show();
              };
              break;  
            }  
          */
          revLEDsChanged = false;     
        }

      }

      /*
      Serial.println(F("----------"));
      for (uint8_t i = 0; i < 4; i++) {
        Serial.print(i);
        Serial.print(F(" - "));
          Serial.println(chMap[i]);
      }*/

    // Turning check
          //Serial.print(F("[YAW]: "));
          //Serial.println(chMap[chYaw - 1]);
      if (chMap[chYaw - 1] < (1500 - chDeviation)) {
        // LEFT
        carTurningDirection = 2;
      } else if (chMap[chYaw - 1] > (1500 + chDeviation)) {
        // RIGHT
        carTurningDirection = 1;
      } else {
        // NO MIDPOINT DEVIATION
        carTurningDirection = 0;
      }

  // Turn indication
  /*
  switch (carTurningDirection) {
      case 2: { 
        if (!turnLEDsChanged) {
          Serial.println(F("LED LEFT > Orange"));
          leds[5] = CRGB::Orange;
          leds[6] = CRGB::Orange;
          FastLED.show();

          turnLEDsChanged = true;
          carIndicating = true;
        }
      };
      break;
      case 1: { 
        if (!turnLEDsChanged) {
          Serial.println(F("LED RIGHT > Orange"));
          leds[0] = CRGB::Orange;
          leds[1] = CRGB::Orange;
          FastLED.show();

          turnLEDsChanged = false;
          carIndicating = true;
        }
      };
      break;
      case 0: {
        if (carIndicating && !carReversing) {
          Serial.println(F("LED IND > Red"));
          leds[0] = CRGB::Red;
          leds[1] = CRGB::Red;
          leds[5] = CRGB::Red;
          leds[6] = CRGB::Red;
          FastLED.show();

          turnLEDsChanged = false;
          carIndicating = false;
        } else if (carIndicating && carReversing) {
          Serial.println(F("LED IND > Red"));
          leds[0] = CRGB::White;
          leds[1] = CRGB::White;
          leds[5] = CRGB::White;
          leds[6] = CRGB::White;
          FastLED.show();

          turnLEDsChanged = false;
          carIndicating = false;
        }
        //Serial.println(F("T IN: NONE"));
      };
    break;
  }*/
}

// =========================================
// FUNCTIONS
// =========================================
void request_MAVLinkData() {
  mavlink_message_t msg;
  mavlink_status_t status;

  // Only request if mlSerial is alive and well, i.E. not '0'
  while (mlSerial.available()) {
    uint8_t c = mlSerial.read();

    // Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, & msg, & status)) {
      
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: { // # MSG ID: 35    
            mavlink_rc_channels_raw_t packet;
            mavlink_msg_rc_channels_raw_decode( & msg, & packet); 

            /*
            Serial.println(F("---------"));
            Serial.println(packet.chan1_raw);
            Serial.println(packet.chan2_raw);
            Serial.println(packet.chan3_raw);
            Serial.println(packet.chan4_raw);
            Serial.println(F("---------"));*/

            // Populate chMap array
            chMap[0] = packet.chan1_raw;
            chMap[1] = packet.chan2_raw;
            chMap[2] = packet.chan3_raw; // YAW
            chMap[3] = packet.chan4_raw; // THROTTLE
          }
          break;
      }
    }
  }
}

// Due to the nature of MAVLink, we must first request a datastream from the TARGET.
// If not done prior to REQUESTING data, the MAVLink interface will be unable to handle our request.
void request_datastream() {
  // Params used in the request.
  // Primarily used to identify the client for the interface
  uint8_t _system_id = 1; // ID of the client that is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2;
  uint8_t _target_system = 55; // ID of target system, doesn't really matter
  uint8_t _target_component = 0;
  uint8_t _req_stream_id = MAV_DATA_STREAM_RC_CHANNELS; // Type of data stream
  uint16_t _req_message_rate = 0x01; // Amount of requets per sec
  uint8_t _start_stop = 1; // 1 = Start | 0 = Stop

  // Initialize the requiRed buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, & msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, & msg); // Send the message (.write sends as bytes)

  mlSerial.write(buf, len); //Write data to serial port
}
