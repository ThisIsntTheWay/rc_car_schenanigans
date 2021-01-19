/*
  Author: V. Klopfenstein
  
  Overview:
    This arduino code controls a LED strip based on specific RC channel data.
    Such data is obtained using MAVLink.

    Altough over-engineered, this project also serves as an exercise to learn a bit more about (Free)RTOS

  Disclaimer:
    Pretty much all of the MAVLink interfacing code has been 'borrowed' from https://www.locarbftw.com/understanding-the-arduino-mavlink-library/ [Jan 2021]
    Their article was the first to actually explain how the arduino mavlink library works.    
    It was also the only public code that wasn't unnecessarily bloated. 
    
    For MAVLink message IDs, refer to:
      https://mavlink.io/en/messages/common.html#messages
*/

// =========================================
// HEAD
// =========================================

#include <mavlink.h>
#include <Arduino_FreeRTOS.h>
#include <SoftwareSerial.h>
#include <FastLED.h>

SoftwareSerial mlSerial(12, 11); // RX, TX

// Channel settings
int chThr = 4;          // Channel used for throttle
int chYaw = 2;          // Channel used for yaw
int chDeviation = 100;  // Max deviation in ms from channel midpoint.
                        // Calculation is as follows: (chVal - chDeviation && chVal + chDeviation)
int chThrMid = 1500;    // Midpoint of throttle in ms
int chYawMid = 1500;    // Midpoint of yaw in ms
uint16_t chMap[8] = {};

// Misc settings
bool carReversing = false;
bool carTurning = false;
char carTurningDirection[1] = "N";
  // - 'R' for RIGHT, 'L' for LEFT, 'N' for NONE

// LED settings
#define LED_PIN     5
#define NUM_LEDS    7
#define BRIGHTNESS  100
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
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
  // LEDs setup
  // -------------------------------------
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);
    
    // Power down LEDs
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
    }
    FastLED.show();

    Serial.begin(57600);    // Init arduino serial RX/TX
    mlSerial.begin(57600);  // Init serial RX/TX for MAVLink interface
    
    Serial.println("Boot complete"); //Console output
  // -------------------------------------

  // Set up foundation for MAVLink communications
    request_datastream();

  // Create Tasks
  // -------------------------------------
      xTaskCreate(
        taskMAVLinkTelem
        ,  (const portCHAR *) "MAVLink interface"
        ,  128    // Stack size
        ,  NULL   // Task param
        ,  1      // Priority
        ,  NULL   // Task handle
      );

      xTaskCreate(
        taskChannelWatcher
        ,  (const portCHAR *) "RC Channell observer"
        ,  64     // Stack size
        ,  NULL   // Task param
        ,  2      // Priority
        ,  NULL   // Task handle
      );
      
      xTaskCreate(
        taskTurnIndicator
        ,  (const portCHAR *) "RC Channel observer"
        ,  64     // Stack size
        ,  NULL   // Task param
        ,  3      // Priority
        ,  NULL   // Task handle
      );

      xTaskCreate(
        taskReverseIndicator
        ,  (const portCHAR *) "Car reversing indicator"
        ,  64     // Stack size
        ,  NULL   // Task param
        ,  3      // Priority
        ,  NULL   // Task handle
      );
  // -------------------------------------

  // Show arduino bootup completion with fastLEDs
  // -------------------------------------
    for (int i = 0; i < NUM_LEDS; i++) {
      delay(100);
      leds[i] = CRGB::BLUE;
      FastLED.show();
    }

    delay(750);
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::RED;
    }
    FastLED.show();
  // -------------------------------------
}

void loop() {
  // SUPERSEDED BY TASK ARCHITECTURE
}

// =========================================
// TASKS
// =========================================

void taskMAVLinkTelem(void *pvParameters)
{
  (void) pvParameters;

  for (;;) {
    mavlink_message_t msg;
    mavlink_status_t status;

    // Only request if mlSerial is alive and well, i.E. not '0'
    while (mlSerial.available()) {
      uint8_t c = mlSerial.read();

      // Get new message
      if (mavlink_parse_char(MAVLINK_COMM_0, c, & msg, & status)) {

        // Handle new message from autopilot
        Serial.print("DEBUG: msgid is: ");
          Serial.println(msg.msgid); //Console output
        
        switch (msg.msgid) {
          case MAVLINK_MSG_ID_RC_CHANNELS_RAW: { // # MSG ID: 35    
              mavlink_rc_channels_raw_t packet;
              mavlink_msg_rc_channels_raw_decode( & msg, & packet); 

              // Populate chMap array
              uint16_t chMap[8] = {
                packet.chan1_raw,
                packet.chan2_raw,
                packet.chan3_raw,
                packet.chan4_raw,
                packet.chan5_raw,
                packet.chan6_raw,
                packet.chan7_raw,
                packet.chan8_raw,
              };
            }
          break;
        }
      }
    }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void taskChannelWatcher(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Check if chThr is lesser than ChThrMid when accounting with deviation as well
      Serial.println(chMap[chThr - 1]);
      if (chMap[chThr - 1] < (chThrMid - chDeviation)) {
        carReversing = true;
      } else {
        carReversing = false;
      }

    // Check if Car is 'yawing', meaning that it is turning into a specific direction
    
    // Turning check
    if ((chMap[chYaw - 1] < (chYawMid - chDeviation)) || (chMap[chYaw - 1] > (chYawMid + chDeviation))) {

      // LEFT
      carReversing = true;
    } else {
      // RIGHT
      carReversing = false;
    }

    vTaskDelay(6);
  }
}

void taskReverseIndicator(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Control REAR LEDs based on value of carReversing
    if (carReversing) {
      leds[0] = CRGB::WHITE;
      leds[1] = CRGB::WHITE;
      FastLED.show();
    } else {
      leds[0] = CRGB::RED;
      leds[1] = CRGB::RED;
      FastLED.show();
    }

    vTaskDelay(6);  // one tick delay (15ms) in between reads for stability
  }
}

void taskTurnIndicator(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Control SIDE LEDs based on channel value
    if (carIndicating) {
      leds[0] = CRGB::WHITE;
      leds[1] = CRGB::WHITE;
      FastLED.show();
    } else {
      leds[0] = CRGB::RED;
      leds[1] = CRGB::RED;
      FastLED.show();
    }

    vTaskDelay(6);  // one tick delay (15ms) in between reads for stability
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

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, & msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, & msg); // Send the message (.write sends as bytes)

  mlSerial.write(buf, len); //Write data to serial port
}
