/*
  Author: V. Klopfenstein
  
  Disclaimer:
    A very large portion of this code has been 'borrowed' from https://www.locarbftw.com/understanding-the-arduino-mavlink-library/
    His article was the first to actually explain how the arduino mavlink library works.
    Additionally, it has somewhat explained the anatomy of the MAVLink protocol.
    
    It was also the only public code that wasn't unnecessarily bloated. 
    
    For MAVLink message IDs, refer to:
      https://mavlink.io/en/messages/common.html#messages

*/

#include <mavlink.h>
#include <SoftwareSerial.h>

SoftwareSerial mlSerial(12, 11); // RX / TX

void setup() {
  Serial.begin(57600);    // Init arduino serial RX/TX
  mlSerial.begin(57600);  // Init serial RX/TX for MAVLink interface
  
  Serial.println("Boot complete"); //Console output

  request_datastream();
}

void loop() {  
  MavLink_receive();
  
  // TODO:
   // Handle shit
}

// Receive MAVLink stream
void MavLink_receive() {
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
              
            Serial.print("PX RC CHANNLES (RAW): ");
            Serial.print("[CH1: ");
            Serial.print(packet.chan1_raw);
            Serial.print("], [CH2: ");
            Serial.print(packet.chan2_raw);
            Serial.print("], [CH3: ");
            Serial.print(packet.chan3_raw);
            Serial.print("], [CH4: ");
            Serial.print(packet.chan4_raw);
            Serial.print("], [CH5: ");
            Serial.print(packet.chan5_raw);
            Serial.print("], [CH6: ");
            Serial.print(packet.chan6_raw);
            Serial.print("], [CH7: ");
            Serial.print(packet.chan7_raw);
            Serial.print("], [RSSI: ");
            Serial.print(packet.rssi);
            Serial.println("]");
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
