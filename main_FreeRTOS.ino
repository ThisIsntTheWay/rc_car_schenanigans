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
   
   !!!!!!!!!!!!!!!!      
   INFO:
     The current implementation of FreeRTOS in this code suffers from low SRAM (180 Bytes free) on an Arduino UNO.
     For the moment, development on this code will be halted for the moment.  
   !!!!!!!!!!!!!!!!
*/

// =========================================
// HEAD
// =========================================

#include <mavlink.h>
#include <Arduino_FreeRTOS.h>
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
uint8_t chYaw = 2;          // Channel used for yaw
uint8_t chDeviation = 100;  // Max deviation in ms from channel midpoint.
                        // Calculation is as follows: (chVal - chDeviation && chVal + chDeviation)
/*uint16_t chThrMid = 1500;    // Midpoint of throttle in ms
uint16_t chYawMid = 1500;    // Midpoint of yaw in ms*/
uint16_t chMap[4] = {};

// Misc settings
bool carReversing = false;
uint8_t carTurningDirection = 0;
  // - '1' for RIGHT, '2' for LEFT, 'N' for NONE

// LED settings
#define NUM_LEDS 7
#define LED_TYPE WS2811
CRGB leds[NUM_LEDS];

// =========================================
// RTOS Tasks
// =========================================
void taskReverseIndicator(void *pvParameters);
void taskTurnIndicator(void *pvParameters);
void taskChannelWatcher(void *pvParameters);
void taskMAVLinkTelem(void *pvParameters);

// =========================================
// MAIN
// =========================================

// Setup
void setup() {
  delay(300);
  Serial.begin(57600);    // Init arduino serial RX/TX
  mlSerial.begin(57600);  // Init serial RX/TX for MAVLink interface
  
  Serial.print(F("---"));

    
  // LEDs setup
  // -------------------------------------
    // PARAMS: <LED_TYPE>, <LED_PIN>, <COLOR_ORDER>(leds, <NUM_LEDS>)
    FastLED.addLeds<LED_TYPE, 5, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
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

  // Create Tasks
  // -------------------------------------
  Serial.print(F("1 - "));
      xTaskCreate(
        taskMAVLinkTelem
        ,  "MA"
        ,  128    // Stack size
        ,  NULL   // Task param
        ,  1      // Priority
        ,  NULL   // Task handle
      );
  Serial.println(availableMemory());

  Serial.print(F("2 - "));
      xTaskCreate(
        taskChannelWatcher
        ,  "RC"
        ,  64    // Stack size
        ,  NULL   // Task param
        ,  1      // Priority
        ,  NULL   // Task handle
      );
  Serial.println(availableMemory());
      
  Serial.print(F("3 - "));
      xTaskCreate(
        taskTurnIndicator
        ,  "TI"
        ,  16    // Stack size
        ,  NULL   // Task param
        ,  1      // Priority
        ,  NULL   // Task handle
      );
  Serial.println(availableMemory());

  Serial.print(F("4 - "));
      xTaskCreate(
        taskReverseIndicator
        ,  "RI"
        ,  16    // Stack size
        ,  NULL   // Task param
        ,  1      // Priority
        ,  NULL   // Task handle
      );
  Serial.println(availableMemory());

  // -------------------------------------

  // Show arduino bootup completion with fastLEDs
  // -------------------------------------
    for (byte i = 0; i < NUM_LEDS; i++) {
      delay(100);
      leds[i] = CRGB::Blue;
      FastLED.show();
    }

    delay(750);
    for (byte i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Red;
    }
    FastLED.show();
  // -------------------------------------

  Serial.print(F("d - "));
  Serial.println(availableMemory());
}

void loop() {
  // SUPERSEDED BY TASK ARCHITECTURE
}

// =========================================
// TASKS
// =========================================

void taskMAVLinkTelem(void *pvParameters)
{
  Serial.begin(57600);
  Serial.println(F("mavlink on."));

  while(1) {
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

              // Populate chMap array
              uint16_t chMap[4] = {
                packet.chan1_raw,
                packet.chan2_raw,
                packet.chan3_raw,
                packet.chan4_raw
              };
            break;
            }
        }
      }
    }
    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
}

void taskChannelWatcher(void *pvParameters) {
  (void) pvParameters;
  
  Serial.println(F("ch watch on."));

  for (;;) {
    // Check if chThr is lesser than 1500 when accounting with deviation as well
          //Serial.print(F("[THR]: "));
          //Serial.println(chMap[chThr - 1]);
      if (chMap[chThr - 1] < (1500 - chDeviation)) {
        carReversing = true;
      } else {
        carReversing = false;
      }

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

    vTaskDelay(1);
  }
}

void taskReverseIndicator(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Control REAR LEDs based on value of carReversing
    if (carReversing) {
      leds[0] = CRGB::White;
      leds[1] = CRGB::White;
      FastLED.show();
    } else {
      leds[0] = CRGB::Red;
      leds[1] = CRGB::Red;
      FastLED.show();
    }

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void taskTurnIndicator(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Control SIDE LEDs based on channel value
    switch (carTurningDirection) {
      case 2: { 
        Serial.println(F("T IN: LEFT."));
      };
      break;
      case 1: { 
        Serial.println(F("T IN: RIGHT."));
      };
      break;
      case 0: {
        Serial.println(F("T IN: NONE"));
      };
      break;
    }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

// =========================================
// FUNCTIONS
// =========================================

// Due to the nature of MAVLink, we must first request a datastream from the TARGET.
// If not done prior to REQUESTING data, the MAVLink interface will be unable to handle our request.
void request_datastream() {
  // Params used in the request.
  // Primarily used to identify the client for the interface
  uint8_t _system_id = 1; // ID of the client that is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2;
  uint8_t _target_system = 55; // ID of target system, doesn't really matter
  uint8_t _target_component = 0;
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL; // Type of data stream
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
