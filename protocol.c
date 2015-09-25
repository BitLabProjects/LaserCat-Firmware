/*
  protocol.c - controls Grbl execution protocol and procedures
  Part of Grbl v0.9

  Copyright (c) 2012-2014 Sungeun K. Jeon  

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
/* 
  This file is based on work from Grbl v0.8, distributed under the 
  terms of the MIT-license. See COPYING for more details.  
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2011-2012 Sungeun K. Jeon
*/ 

#include "system.h"
#include "serial.h"
#include "settings.h"
#include "protocol.h"
//#include "gcode.h"
//#include "planner.h"
#include "stepper.h"
//#include "motion_control.h"
#include "motor.h"

#define CI_START_CHAR '#' //35 decimal
#define CI_END_CHAR '$' //36 decimal

#define CISTATE_WAITINGFOR_STARTCHAR 0
#define CISTATE_WAITINGFOR_PACKETID 1
#define CISTATE_WAITINGFOR_LENGTH 2
#define CISTATE_READINGDATA 3
#define CISTATE_WAITINGFOR_CHECKSUM 4
#define CISTATE_WAITINGFOR_ENDCHAR 5

#define CIRXERROR_INVALIDPACKETID 1
#define CIRXERROR_INVALIDENDCHAR 2
#define CIRXERROR_INVALIDCHECKSUM 3
#define CIRXERROR_INVALIDARGUMENTS 4

#define CONNECT_COMMAND 1
#define RESET_COMMAND 2
#define SETSETTINGS_COMMAND 3
#define WAKEUP_COMMAND 4
#define GOIDLE_COMMAND 5
#define STOREPLANNERBLOCK_COMMAND 6
#define STORESEGMENTBLOCK_COMMAND 7
#define OK_COMMAND 8
#define ERROR_COMMAND 9
#define ASKPOSITION_COMMAND 10
#define OKPOSITION_COMMAND 11
#define ASKHASMORESEGMENTBUFFER_COMMAND 12
#define OKSEGMENTBUFFER_COMMAND 13
#define OKSTORESEGMENT_COMMAND 14
//TODO Riportare SetSpeed dall'altro progetto
#define MANUALSTEP_COMMAND 16
#define UNDEFINED_COMMAND 99

struct ProtocolMessage {
  uint8_t packet_id;
  uint8_t length;
  uint8_t data_bytes[20];
  uint8_t checksum;
};

struct CommandInterpreter {
  uint8_t state;
  uint8_t previous_packet_id;
  uint8_t data_count;
};

struct CommandInterpreter commandInterpreter;
struct ProtocolMessage rxMessage;
struct ProtocolMessage txMessage;

/*
#define DEBUGARRAYLENGTH 5
uint8_t debugarray[DEBUGARRAYLENGTH];
void debugarray_add(uint8_t data) {
  for(uint8_t i=0; i < DEBUGARRAYLENGTH-1; i++)
    debugarray[i] = debugarray[i+1];
  debugarray[DEBUGARRAYLENGTH-1] = data;
}
*/

uint8_t calculateChksum(uint8_t id, uint8_t length, uint8_t* data_bytes, uint8_t command){
  uint8_t chksum = 0;
  uint8_t i;
  chksum ^= id;
  chksum ^= length;
  chksum ^= command;
  for(i = 0; i < length; i++){
    chksum ^= data_bytes[i];
  }
  return chksum;
}

uint8_t calculateChksumMsg(struct ProtocolMessage* pm) {
  return calculateChksum(pm->packet_id, pm->length, pm->data_bytes, 0);
}

void command_send_do() {
  uint8_t i;
  serial_write(CI_START_CHAR);
  serial_write(txMessage.packet_id);
  serial_write(txMessage.length);
  for(i=0; i<txMessage.length; i++)
    serial_write(txMessage.data_bytes[i]);
  serial_write(txMessage.checksum);
  serial_write(CI_END_CHAR);
}

//uint8_t cmdcount = 0;
void command_send(uint8_t command) {
  //uint8_t crcerror = 0;
  //cmdcount += 1;
  //if (cmdcount == 100) {
  //  cmdcount = 0;
  //  crcerror = 15;
  //}
  txMessage.packet_id = rxMessage.packet_id;
  txMessage.length = 1;
  txMessage.data_bytes[0] = command;
  calculateChksumMsg(&txMessage);
  command_send_do();
}

void command_send_byte(uint8_t command, uint8_t data_bytes) {
  txMessage.packet_id = rxMessage.packet_id;
  txMessage.length = 2;
  txMessage.data_bytes[0] = command;
  txMessage.data_bytes[1] = data_bytes;
  calculateChksumMsg(&txMessage);
  command_send_do();
}

void command_send_rxerror(uint8_t rxerror) {
  command_send_byte(ERROR_COMMAND, rxerror);
}

void command_send_array(uint8_t command, uint8_t* data_bytes, uint8_t data_length) {
  uint8_t i;
  txMessage.packet_id = rxMessage.packet_id;
  txMessage.length = 1 + data_length;
  txMessage.data_bytes[0] = command;
  for(i=0; i<data_length; i++)
    txMessage.data_bytes[1 + i] = data_bytes[i];
  calculateChksumMsg(&txMessage);
  command_send_do();
}

uint8_t isChecksumValid(struct ProtocolMessage* ci) {
  if (calculateChksum(ci->packet_id, ci->length, ci->data_bytes, 0) == ci->checksum){
    return true;
  } else{
    return false;
  }
}

void command_receive_and_execute() {
  uint8_t blockIndex;
  segment_t segment;
  st_block_t block;
  int i;
  uint8_t* dst_ptr;
  uint8_t* src_ptr;
  
  uint8_t execute = false;
  if (serial_has_bytes()) {
    uint8_t c;
    c = serial_read();
    switch(commandInterpreter.state) {
      case CISTATE_WAITINGFOR_STARTCHAR:
        if (c == CI_START_CHAR)
          commandInterpreter.state = CISTATE_WAITINGFOR_PACKETID;
        break;

      case CISTATE_WAITINGFOR_PACKETID:
        rxMessage.packet_id = c;
        commandInterpreter.state = CISTATE_WAITINGFOR_LENGTH;
        break;

      case CISTATE_WAITINGFOR_LENGTH:
        rxMessage.length = c;
        commandInterpreter.data_count = 0;
        commandInterpreter.state = CISTATE_READINGDATA;
        break;

      case CISTATE_READINGDATA:
        rxMessage.data_bytes[commandInterpreter.data_count] = c;
        commandInterpreter.data_count += 1;
        if (commandInterpreter.data_count == rxMessage.length) {
            commandInterpreter.state = CISTATE_WAITINGFOR_CHECKSUM;
        }
        break;

      case CISTATE_WAITINGFOR_CHECKSUM:
        rxMessage.checksum = c;
        commandInterpreter.state = CISTATE_WAITINGFOR_ENDCHAR;
        break;

      case CISTATE_WAITINGFOR_ENDCHAR:
        if (c == CI_END_CHAR)
          execute = true;
        else {
          command_send_rxerror(CIRXERROR_INVALIDENDCHAR);
        }
        commandInterpreter.state = CISTATE_WAITINGFOR_STARTCHAR;
        break;
    }
  }
  if (execute) {
    uint8_t expected_packet_id;
    //Check received packet CRC
    if (!isChecksumValid(&rxMessage)) {
      command_send_rxerror(CIRXERROR_INVALIDCHECKSUM);
      return;
    }

    //Skip packet id check for connect command, it can come at any moment
    //The skip is accomplished by resetting the last packet id to 255, connect command resets the count on the host
    if (rxMessage.data_bytes[0] == CONNECT_COMMAND) {
      commandInterpreter.previous_packet_id = 255; //-1, 0 is the next packet
    }
    
    //Check received packet id
    if (commandInterpreter.previous_packet_id == 255)
      expected_packet_id = 0;
    else
      expected_packet_id = commandInterpreter.previous_packet_id + 1;
    if (rxMessage.packet_id != expected_packet_id) {
      //If the packet id is the same of the last reply, the host did not receive my last reply, so send it again
      if (rxMessage.packet_id == txMessage.packet_id) {
        command_send_do();
        return;
      }
      else
        command_send_rxerror(CIRXERROR_INVALIDPACKETID);
    }

    commandInterpreter.previous_packet_id = rxMessage.packet_id;

    switch (rxMessage.data_bytes[0]) {
      case CONNECT_COMMAND:
        command_send(OK_COMMAND);
        break;

      case RESET_COMMAND:
        asm{reset}
        break;

      case SETSETTINGS_COMMAND:
        if (rxMessage.length != 8) { 
          //7 parameters plus the command
          command_send_rxerror(CIRXERROR_INVALIDARGUMENTS);
          break;
        }
        settings.pulse_microseconds = rxMessage.data_bytes[1];
        settings.step_invert_mask = rxMessage.data_bytes[2];
        settings.dir_invert_mask = rxMessage.data_bytes[3];
        settings.stepper_idle_lock_time = rxMessage.data_bytes[4];
        settings.flags = rxMessage.data_bytes[5];
        stepper_set_settings(rxMessage.data_bytes[6], rxMessage.data_bytes[7]);
        command_send(OK_COMMAND);
        break;

      case WAKEUP_COMMAND:
        if (rxMessage.length != 2) {
          command_send_rxerror(CIRXERROR_INVALIDARGUMENTS);
          break;
        }
        st_wake_up(rxMessage.data_bytes[1]);
        command_send(OK_COMMAND);
        break;

      case GOIDLE_COMMAND:
        if (rxMessage.length != 2) {
          command_send_rxerror(CIRXERROR_INVALIDARGUMENTS);
          break;
        }
        st_go_idle(rxMessage.data_bytes[1]);
        command_send(OK_COMMAND);
        break;

      case STOREPLANNERBLOCK_COMMAND:
        if (rxMessage.length != 19) {
          command_send_rxerror(CIRXERROR_INVALIDARGUMENTS);
          break;
        }
        blockIndex = rxMessage.data_bytes[1];
        dst_ptr = (uint8_t*)&block;
        src_ptr = (uint8_t*)&rxMessage.data_bytes[2];
        for(i=0; i<17; i++)
          *(dst_ptr++) = *(src_ptr++);
        stepper_store_planner_block(blockIndex, &block);
        command_send(OK_COMMAND);
        break;

      case STORESEGMENTBLOCK_COMMAND:
        if (rxMessage.length != 8) { 
          command_send_rxerror(CIRXERROR_INVALIDARGUMENTS);
          break;
        }
        dst_ptr = (uint8_t*)&segment;
        src_ptr = (uint8_t*)&rxMessage.data_bytes[1];
        for(i=0; i<7; i++)
          *(dst_ptr++) = *(src_ptr++);
        stepper_setmode(STEPPERMODE_AUTO);
        stepper_store_segment_block(&segment);
        //command_send(OK_COMMAND);
        command_send_byte(OKSTORESEGMENT_COMMAND, stepper_has_more_segment_buffer());
        break;

      case ASKPOSITION_COMMAND:
        command_send_array(OKPOSITION_COMMAND, (uint8_t*)&sys.position, 12); //3*int32_t
        break;

      case ASKHASMORESEGMENTBUFFER_COMMAND:
        command_send_byte(OKSEGMENTBUFFER_COMMAND, stepper_has_more_segment_buffer());
        break;
        
      case MANUALSTEP_COMMAND:
        if (rxMessage.length != 3) {
          command_send_rxerror(CIRXERROR_INVALIDARGUMENTS);
          break;
        }
        stepper_setmode(STEPPERMODE_MANUAL);
        stepper_manualstep(rxMessage.data_bytes[1], rxMessage.data_bytes[2]);
        command_send(OK_COMMAND);
        break;

      case UNDEFINED_COMMAND:
        command_send_rxerror(CIRXERROR_INVALIDARGUMENTS);
        break;

      default:
        command_send_rxerror(CIRXERROR_INVALIDARGUMENTS);
        break;
    }
    rxMessage.data_bytes[0] = UNDEFINED_COMMAND;
  }
}

void protocol_main_loop()
{
  commandInterpreter.state = CISTATE_WAITINGFOR_STARTCHAR;
  commandInterpreter.previous_packet_id = 255; //-1, 0 is the next packet
  
  rxMessage.packet_id = 255;
  command_send(OK_COMMAND);

  while(true)
  {
    command_receive_and_execute();
  }

  return; /* Never reached */
}