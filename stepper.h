/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
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
    Copyright (c) 2011 Sungeun K. Jeon
*/ 

#ifndef stepper_h
#define stepper_h 

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

// Initialize and setup the stepper motor subsystem
void stepper_init();

// Enable steppers, but cycle does not start unless called by motion control or runtime command.
void st_wake_up(uint8_t setup_and_enable_motors);

// Immediately disables steppers
void st_go_idle(uint8_t delay_and_disable_steppers);

// Reset the stepper subsystem variables       
void st_reset();

typedef struct {  
  uint8_t direction_bits;
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
} st_block_t;
typedef struct {
  uint16_t n_step;          // Number of step events to be executed for this segment
  uint8_t st_block_index;   // Stepper block data index. Uses this information to execute this segment.
  uint16_t cycles_per_tick; // Step distance traveled per ISR tick, aka step rate.
  //#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
  //#else
  uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
  //#endif
} segment_t;

void stepper_set_settings(uint8_t step_invert_mask, uint8_t dir_invert_mask);

void stepper_store_planner_block(uint8_t blockIndex, st_block_t* block);
void stepper_store_segment_block(segment_t* segment);
void stepper_interrupt();
uint8_t stepper_has_more_segment_buffer();

#endif
