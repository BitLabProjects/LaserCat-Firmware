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
#include "report.h"


static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also directs and executes Grbl internal commands,
// such as settings, initiating the homing cycle, and toggling switch states.
static void protocol_execute_line(char *line) 
{      
  protocol_execute_runtime(); // Runtime command check point.
  if (sys.abort) { return; } // Bail to calling function upon system abort  

  if (line[0] == 0) {
    // Empty or comment line. Send status message for syncing purposes.
    report_status_message(STATUS_OK);

  } else if (line[0] == '$') {
    // Grbl '$' system command
    report_status_message(system_execute_line(line));
    
  } else if (sys.state == STATE_ALARM) {
    // Everything else is gcode. Block if in alarm mode.
    report_status_message(STATUS_ALARM_LOCK);

  } else {
    // Parse and execute g-code block!
    //TODO ASSERT FALSE report_status_message(gc_execute_line(line));
  }
}

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

#define INIT_COMMAND 1
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
    uint8_t c = serial_read();
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

    //Check received packet id
    uint8_t expected_packet_id;
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
    //Check received packet CRC
    if (!isChecksumValid(&rxMessage)) {
      command_send_rxerror(CIRXERROR_INVALIDCHECKSUM);
      return;
    }

    commandInterpreter.previous_packet_id = rxMessage.packet_id;

    switch (rxMessage.data_bytes[0]) {
      case INIT_COMMAND:
        //Already done on startup stepper_init();
        command_send(OK_COMMAND);
        break;

      case RESET_COMMAND:
        st_reset();
        command_send(OK_COMMAND);
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
        stepper_store_segment_block(&segment);
        command_send(OK_COMMAND);
        break;

      case ASKPOSITION_COMMAND:
        command_send_array(OKPOSITION_COMMAND, (uint8_t*)&sys.position, 12); //3*int32_t
        break;

      case ASKHASMORESEGMENTBUFFER_COMMAND:
        command_send_byte(OKSEGMENTBUFFER_COMMAND, stepper_has_more_segment_buffer());
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

/* 
  GRBL PRIMARY LOOP:
*/
void protocol_main_loop()
{
  // ------------------------------------------------------------
  // Complete initialization procedures upon a power-up or reset.
  // ------------------------------------------------------------
  uint8_t c;

  commandInterpreter.previous_packet_id = 255; //-1, 0 is the next packet
  // Print welcome message   
  //report_init_message();
  while(true)
  {
    command_receive_and_execute();
  }

/*
  // Check for and report alarm state after a reset, error, or an initial power up.
  if (sys.state == STATE_ALARM) {
    report_feedback_message(MESSAGE_ALARM_LOCK); 
  } else {
    // All systems go!
    sys.state = STATE_IDLE; // Set system to ready. Clear all state flags.
    system_execute_startup(line); // Execute startup script.
  }
    
  // ---------------------------------------------------------------------------------  
  // Primary loop! Upon a system abort, this exits back to main() to reset the system. 
  // ---------------------------------------------------------------------------------  
  
  uint8_t iscomment = false;
  uint8_t char_counter = 0;
  
  for (;;) {

    // Process one line of incoming serial data, as the data becomes available. Performs an
    // initial filtering by removing spaces and comments and capitalizing all letters.
    
    // NOTE: While comment, spaces, and block delete(if supported) handling should technically 
    // be done in the g-code parser, doing it here helps compress the incoming data into Grbl's
    // line buffer, which is limited in size. The g-code standard actually states a line can't
    // exceed 256 characters, but the Arduino Uno does not have the memory space for this.
    // With a better processor, it would be very easy to pull this initial parsing out as a 
    // seperate task to be shared by the g-code parser and Grbl's system commands.
    
    while((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // End of line reached
        line[char_counter] = 0; // Set string termination character.
        protocol_execute_line(line); // Line is complete. Execute it!
        iscomment = false;
        char_counter = 0;
      } else {
        if (iscomment) {
          // Throw away all comment characters
          if (c == ')') {
            // End of comment. Resume line.
            iscomment = false;
          }
        } else {
          if (c <= ' ') { 
            // Throw away whitepace and control characters  
          } else if (c == '/') { 
            // Block delete NOT SUPPORTED. Ignore character.
            // NOTE: If supported, would simply need to check the system if block delete is enabled.
          } else if (c == '(') {
            // Enable comments flag and ignore all characters until ')' or EOL.
            // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
            // In the future, we could simply remove the items within the comments, but retain the
            // comment control characters, so that the g-code parser can error-check it.
            iscomment = true;
          // } else if (c == ';') {
            // Comment character to EOL NOT SUPPORTED. LinuxCNC definition. Not NIST.
            
          // TODO: Install '%' feature 
          // } else if (c == '%') {
            // Program start-end percent sign NOT SUPPORTED.
            // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
            // where, during a program, the system auto-cycle start will continue to execute 
            // everything until the next '%' sign. This will help fix resuming issues with certain
            // functions that empty the planner buffer to execute its task on-time.

          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // Detect line buffer overflow. Report error and reset line buffer.
            report_status_message(STATUS_OVERFLOW);
            iscomment = false;
            char_counter = 0;
          } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }
      }
    }
    
    // If there are no more characters in the serial read buffer to be processed and executed,
    // this indicates that g-code streaming has either filled the planner buffer or has 
    // completed. In either case, auto-cycle start, if enabled, any queued moves.
    protocol_auto_cycle_start();

    protocol_execute_runtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.
              
  }*/
  
  return; /* Never reached */
}
/*

// Executes run-time commands, when required. This is called from various check points in the main
// program, primarily where there may be a while loop waiting for a buffer to clear space or any
// point where the execution time from the last check point may be more than a fraction of a second.
// This is a way to execute runtime commands asynchronously (aka multitasking) with grbl's g-code
// parsing and planning functions. This function also serves as an interface for the interrupts to 
// set the system runtime flags, where only the main program handles them, removing the need to
// define more computationally-expensive volatile variables. This also provides a controlled way to 
// execute certain tasks without having two or more instances of the same task, such as the planner
// recalculating the buffer upon a feedhold or override.
// NOTE: The sys.execute variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
void protocol_execute_runtime()
{
  uint8_t rt_exec = sys.execute; // Copy to avoid calling volatile multiple times
  if (rt_exec) { // Enter only if any bit flag is true
    
    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    if (rt_exec & (EXEC_ALARM | EXEC_CRIT_EVENT)) {      
      sys.state = STATE_ALARM; // Set system alarm state

      // Critical events. Hard/soft limit events identified by both critical event and alarm exec
      // flags. Probe fail is identified by the critical event exec flag only.
      if (rt_exec & EXEC_CRIT_EVENT) {
        if (rt_exec & EXEC_ALARM) { report_alarm_message(ALARM_LIMIT_ERROR); }
        else { report_alarm_message(ALARM_PROBE_FAIL); }
        report_feedback_message(MESSAGE_CRITICAL_EVENT);
        bit_false_atomic(sys.execute,EXEC_RESET); // Disable any existing reset
        do { 
          // Nothing. Block EVERYTHING until user issues reset or power cycles. Hard limits
          // typically occur while unattended or not paying attention. Gives the user time
          // to do what is needed before resetting, like killing the incoming stream. The 
          // same could be said about soft limits. While the position is not lost, the incoming
          // stream could be still engaged and cause a serious crash if it continues afterwards.
        } while (bit_isfalse(sys.execute,EXEC_RESET));

      // Standard alarm event. Only abort during motion qualifies.
      } else {
        // Runtime abort command issued during a cycle, feed hold, or homing cycle. Message the
        // user that position may have been lost and set alarm state to enable the alarm lockout
        // to indicate the possible severity of the problem.
        report_alarm_message(ALARM_ABORT_CYCLE);
      }
      bit_false_atomic(sys.execute,(EXEC_ALARM | EXEC_CRIT_EVENT));
    } 
  
    // Execute system abort. 
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // Only place this is set true.
      return; // Nothing else to do but exit.
    }
    
    // Execute and serial print status
    if (rt_exec & EXEC_STATUS_REPORT) { 
      report_realtime_status();
      bit_false_atomic(sys.execute,EXEC_STATUS_REPORT);
    }
    
    // Execute a feed hold with deceleration, only during cycle.
    if (rt_exec & EXEC_FEED_HOLD) {
      // !!! During a cycle, the segment buffer has just been reloaded and full. So the math involved
      // with the feed hold should be fine for most, if not all, operational scenarios.
      if (sys.state == STATE_CYCLE) {
        sys.state = STATE_HOLD;
        //TODO st_update_plan_block_parameters();
        //TODO st_prep_buffer();
        sys.auto_start = false; // Disable planner auto start upon feed hold.
      }
      bit_false_atomic(sys.execute,EXEC_FEED_HOLD);
    }
        
    // Execute a cycle start by starting the stepper interrupt begin executing the blocks in queue.
    if (rt_exec & EXEC_CYCLE_START) { 
      if (sys.state == STATE_QUEUED) {
        sys.state = STATE_CYCLE;
        //TODO st_prep_buffer(); // Initialize step segment buffer before beginning cycle.
        //TODO st_wake_up();
        if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) {
          sys.auto_start = true; // Re-enable auto start after feed hold.
        } else {
          sys.auto_start = false; // Reset auto start per settings.
        }
      }    
      bit_false_atomic(sys.execute,EXEC_CYCLE_START);
    }
    
    // Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by 
    // runtime command execution in the main program, ensuring that the planner re-plans safely.
    // NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
    // cycle reinitializations. The stepper path should continue exactly as if nothing has happened.   
    // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
    if (rt_exec & EXEC_CYCLE_STOP) {
      //TODO if ( plan_get_current_block() ) { sys.state = STATE_QUEUED; }
      //TODO else { sys.state = STATE_IDLE; }
      bit_false_atomic(sys.execute,EXEC_CYCLE_STOP);
    }

  }
  
  // Overrides flag byte (sys.override) and execution should be installed here, since they 
  // are runtime and require a direct and controlled interface to the main stepper program.

  // Reload step segment buffer
  //TODO if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_HOMING)) { st_prep_buffer(); }  
  
}  
*/
/*
// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
void protocol_buffer_synchronize()
{
//TODO 

  // If system is queued, ensure cycle resumes if the auto start flag is present.
  protocol_auto_cycle_start();
  // Check and set auto start to resume cycle after synchronize and caller completes.
  if (sys.state == STATE_CYCLE) { sys.auto_start = true; }
  while (plan_get_current_block() || (sys.state == STATE_CYCLE)) { 
    protocol_execute_runtime();   // Check and execute run-time commands
    if (sys.abort) { return; } // Check for system abort
  }
   
}
*/ 

// Auto-cycle start has two purposes: 1. Resumes a plan_synchronize() call from a function that
// requires the planner buffer to empty (spindle enable, dwell, etc.) 2. As a user setting that 
// automatically begins the cycle when a user enters a valid motion command manually. This is 
// intended as a beginners feature to help new users to understand g-code. It can be disabled
// as a beginner tool, but (1.) still operates. If disabled, the operation of cycle start is
// manually issuing a cycle start command whenever the user is ready and there is a valid motion 
// command in the planner queue.
// NOTE: This function is called from the main loop and mc_line() only and executes when one of
// two conditions exist respectively: There are no more blocks sent (i.e. streaming is finished, 
// single commands), or the planner buffer is full and ready to go.
//void protocol_auto_cycle_start() { if (sys.auto_start) { bit_true_atomic(sys.execute, EXEC_CYCLE_START); } } 