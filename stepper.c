/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
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
#include "nuts_bolts.h"
#include "stepper.h"
#include "settings.h"
//#include "planner.h"
#include "probe.h"
#include "motor.h"


// Some useful constants.
#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment 
#define REQ_MM_INCREMENT_SCALAR 1.25                                   
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2

// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the stepper ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the 
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
#define MAX_AMASS_LEVEL 3
// AMASS_LEVEL0: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
#define AMASS_LEVEL1 (F_CPU/8000) // Over-drives ISR (x2). Defined as F_CPU/(Cutoff frequency in Hz)
#define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
#define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)


// Stores the planner block Bresenham algorithm execution data for the segments in the segment 
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use. 
/*typedef struct {  
  uint8_t direction_bits;
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
} st_block_t;*/
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper 
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by 
// the planner, where the remaining planner block steps still can.
/*typedef struct {
  uint16_t n_step;          // Number of step events to be executed for this segment
  uint8_t st_block_index;   // Stepper block data index. Uses this information to execute this segment.
  uint16_t cycles_per_tick; // Step distance traveled per ISR tick, aka step rate.
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
  #else
    uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
  #endif
} segment_t;*/
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct {
  // Used by the bresenham line algorithm
  uint32_t counter_x,        // Counter variables for the bresenham line tracer
           counter_y, 
           counter_z;
  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // Stores out_bits output to complete the step pulse delay
  #endif
  
  uint8_t execute_step;     // Flags step execution for each interrupt.
  uint8_t step_pulse_time;  // Step pulse reset time after step rise
  uint8_t step_outbits;         // The next stepping-bits to be output
  uint8_t dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif

  uint16_t step_count;       // Steps remaining in line segment motion  
  uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
  st_block_t *exec_block;   // Pointer to the block data for the segment being executed
  segment_t *exec_segment;  // Pointer to the segment being executed
} stepper_t;
static stepper_t st;

// Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

// Step and direction port invert masks.
static uint8_t step_port_invert_mask;
static uint8_t dir_port_invert_mask;

// Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
static volatile uint8_t busy;   

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
//TODO SB!Understand if this is needed -> static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped 

// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
  uint8_t st_block_index;  // Index of stepper common data block being prepped
  uint8_t flag_partial_block;  // Flag indicating the last block completed. Time to load a new one.

  float steps_remaining;
  float step_per_mm;           // Current planner block step/millimeter conversion scalar
  float req_mm_increment;
  float dt_remainder;
  
  uint8_t ramp_type;      // Current segment ramp state
  float mm_complete;      // End of velocity profile from end of current planner block in (mm).
                          // NOTE: This value must coincide with a step(no mantissa) when converted.
  float current_speed;    // Current speed at the end of the segment buffer (mm/min)
  float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
  float exit_speed;       // Exit speed of executing block (mm/min)
  float accelerate_until; // Acceleration ramp end measured from end of block (mm)
  float decelerate_after; // Deceleration ramp start measured from end of block (mm)
} st_prep_t;
static st_prep_t prep;


/*    BLOCK VELOCITY PROFILE DEFINITION 
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  time ----->      EXAMPLE: Block 2 entry speed is at max junction velocity
  
  The planner block buffer is planned assuming constant acceleration velocity profiles and are
  continuously joined at block junctions as shown above. However, the planner only actively computes
  the block entry speeds for an optimal velocity plan, but does not compute the block internal
  velocity profiles. These velocity profiles are computed ad-hoc as they are executed by the 
  stepper algorithm and consists of only 7 possible types of profiles: cruise-only, cruise-
  deceleration, acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and 
  triangle(no cruise).

                                        maximum_speed (< nominal_speed) ->  + 
                    +--------+ <- maximum_speed (= nominal_speed)          /|\                                         
                   /          \                                           / | \                      
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |                       
                  +-------------+                     current_speed -> +----+--+                   
                   time -->  ^  ^                                           ^  ^                       
                             |  |                                           |  |                       
                decelerate_after(in mm)                             decelerate_after(in mm)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in mm)                             accelerate_until(in mm)
                    
  The step segment buffer computes the executing block velocity profile and tracks the critical
  parameters for the stepper algorithm to accurately trace the profile. These critical parameters 
  are shown and defined in the above illustration.
*/

void stepper_set_settings(uint8_t step_invert_mask, uint8_t dir_invert_mask)
{
  step_port_invert_mask = step_invert_mask;
  dir_port_invert_mask = dir_invert_mask;
}
void stepper_store_planner_block(uint8_t blockIndex, st_block_t* block)
{
  st_block_buffer[blockIndex] = *block;
}
void stepper_store_segment_block(segment_t* segment)
{
  // Segment complete! Increment segment buffer indices.
  segment_buffer[segment_buffer_head] = *segment;
  segment_buffer_head = segment_next_head;
  if (++segment_next_head == SEGMENT_BUFFER_SIZE) { segment_next_head = 0; }
}
uint8_t stepper_has_more_segment_buffer()
{
  //return segment_buffer_tail != segment_next_head;
  uint8_t count;
  if (segment_buffer_head >= segment_buffer_tail)
    count = segment_buffer_head - segment_buffer_tail;
  else
    count = SEGMENT_BUFFER_SIZE - (segment_buffer_tail - segment_buffer_head);
  return SEGMENT_BUFFER_SIZE - count - 1;
}

// Stepper state initialization. Cycle should only start if the st.cycle_start flag is
// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up(uint8_t setup_and_enable_motors) 
{
  //TODO
  /*
    // Enable stepper drivers.
    if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
    else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }
  */
  //if (sys.state & (STATE_CYCLE | STATE_HOMING)) {
  if (setup_and_enable_motors) {
    // Initialize stepper output bits
    st.dir_outbits = dir_port_invert_mask; 
    //st.step_outbits = step_port_invert_mask;
    
    // Initialize step pulse timing from settings. Here to ensure updating after re-writing.
    #ifdef STEP_PULSE_DELAY
      // Set total step pulse time after direction pin set. Ad hoc computation from oscilloscope.
      st.step_pulse_time = -(((settings.pulse_microseconds+STEP_PULSE_DELAY-2)*TICKS_PER_MICROSECOND) >> 3);
      // Set delay between direction pin write and step command.
      //TODO OCR0A = -(((settings.pulse_microseconds)*TICKS_PER_MICROSECOND) >> 3);
    #else // Normal operation
      // Set step pulse time. Ad hoc computation from oscilloscope. Uses two's complement.
      st.step_pulse_time = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND) >> 3);
    #endif

    // Enable Stepper Driver Interrupt
    //TODO
    //TIMSK1 |= (1<<OCIE1A);
    TMR1ON_bit = 1;
    //T1CONbits.TMR1ON = 1;
  }
}


// Stepper shutdown
void st_go_idle(uint8_t delay_and_disable_steppers) 
{
  bool pin_state;
  
  // Disable Stepper Driver Interrupt. Allow Stepper Port Reset Interrupt to finish, if active.
  //TODO
  //TIMSK1 &= ~(1<<OCIE1A); // Disable Timer1 interrupt
  //TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Reset clock to no prescaling.
  TMR1ON_bit = 0;
  busy = false;
  
  // Set stepper driver idle state, disabled or enabled, depending on settings and circumstances.
  pin_state = false; // Keep enabled.
  //if (((settings.stepper_idle_lock_time != 0xff) || bit_istrue(sys.execute,EXEC_ALARM)) && sys.state != STATE_HOMING) {
  if (delay_and_disable_steppers) {
    // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
    // stop and not drift from residual inertial forces at the end of the last movement.
    //TODO delay_ms(settings.stepper_idle_lock_time);
    pin_state = true; // Override. Disable steppers.
  }
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { pin_state = !pin_state; } // Apply pin invert.
  if (pin_state) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
  else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }

}


/* "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. Grbl employs
   the venerable Bresenham line algorithm to manage and exactly synchronize multi-axis moves.
   Unlike the popular DDA algorithm, the Bresenham algorithm is not susceptible to numerical
   round-off errors and only requires fast integer counters, meaning low computational overhead
   and maximizing the Arduino's capabilities. However, the downside of the Bresenham algorithm
   is, for certain multi-axis motions, the non-dominant axes may suffer from un-smooth step 
   pulse trains, or aliasing, which can lead to strange audible noises or shaking. This is 
   particularly noticeable or may cause motion issues at low step frequencies (0-5kHz), but 
   is usually not a physical problem at higher frequencies, although audible.
     To improve Bresenham multi-axis performance, Grbl uses what we call an Adaptive Multi-Axis
   Step Smoothing (AMASS) algorithm, which does what the name implies. At lower step frequencies,
   AMASS artificially increases the Bresenham resolution without effecting the algorithm's 
   innate exactness. AMASS adapts its resolution levels automatically depending on the step
   frequency to be executed, meaning that for even lower step frequencies the step smoothing 
   level increases. Algorithmically, AMASS is acheived by a simple bit-shifting of the Bresenham
   step count for each AMASS level. For example, for a Level 1 step smoothing, we bit shift 
   the Bresenham step event count, effectively multiplying it by 2, while the axis step counts 
   remain the same, and then double the stepper ISR frequency. In effect, we are allowing the
   non-dominant Bresenham axes step in the intermediate ISR tick, while the dominant axis is 
   stepping every two ISR ticks, rather than every ISR tick in the traditional sense. At AMASS
   Level 2, we simply bit-shift again, so the non-dominant Bresenham axes can step within any 
   of the four ISR ticks, the dominant axis steps every four ISR ticks, and quadruple the 
   stepper ISR frequency. And so on. This, in effect, virtually eliminates multi-axis aliasing 
   issues with the Bresenham algorithm and does not significantly alter Grbl's performance, but 
   in fact, more efficiently utilizes unused CPU cycles overall throughout all configurations.
     AMASS retains the Bresenham algorithm exactness by requiring that it always executes a full
   Bresenham step, regardless of AMASS Level. Meaning that for an AMASS Level 2, all four 
   intermediate steps must be completed such that baseline Bresenham (Level 0) count is always 
   retained. Similarly, AMASS Level 3 means all eight intermediate steps must be executed. 
   Although the AMASS Levels are in reality arbitrary, where the baseline Bresenham counts can
   be multiplied by any integer value, multiplication by powers of two are simply used to ease 
   CPU overhead with bitshift integer operations. 
     This interrupt is simple and dumb by design. All the computational heavy-lifting, as in
   determining accelerations, is performed elsewhere. This interrupt pops pre-computed segments,
   defined as constant velocity over n number of steps, from the step segment buffer and then 
   executes them by pulsing the stepper pins appropriately via the Bresenham algorithm. This 
   ISR is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port
   after each pulse. The bresenham line tracer algorithm controls all stepper outputs
   simultaneously with these two interrupts.
   
   NOTE: This interrupt must be as efficient as possible and complete before the next ISR tick, 
   which for Grbl must be less than 33.3usec (@30kHz ISR rate). Oscilloscope measured time in 
   ISR is 5usec typical and 25usec maximum, well below requirement.
   NOTE: This ISR expects at least one step to be executed per segment.
*/

// TODO: Replace direct updating of the int32 position counters in the ISR somehow. Perhaps use smaller
// int8 variables and update position counters only when a segment completes. This can get complicated 
// with probing and homing cycles that require true real-time positions.
//TODO
//ISR(TIMER1_COMPA_vect)
void stepper_interrupt()
{      
  uint32_t local;
  uint32_t stepsBefore;
  uint16_t TMR1;
  uint8_t amass_level;
  // SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; // Debug: Used to time ISR
  //if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt
  
  // Set the direction pins a couple of nanoseconds before we step the steppers
  //TODO DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);

  // Then pulse the stepping pins
//TODO 
/*
  #ifdef STEP_PULSE_DELAY
    st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // Store out_bits to prevent overwriting.
  #else  // Normal operation
    STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_outbits;
  #endif  
*/


  // Enable step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
  // exactly settings.pulse_microseconds microseconds, independent of the main Timer1 prescaler.
  //TODO
  //TCNT0 = st.step_pulse_time; // Reload Timer0 counter
  //TCCR0B = (1<<CS01); // Begin Timer0. Full speed, 1/8 prescaler

  //busy = true;
  //sei(); // Re-enable interrupts to allow Stepper Port Reset Interrupt to fire on-time. 
         // NOTE: The remaining code in this ISR will finish before returning to main program.
    
  // If there is no step segment, attempt to pop one from the stepper buffer
  if (st.exec_segment == NULL) {
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
      // Initialize new step segment and load number of steps to execute
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      /*
      #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS is disabled, set timer prescaler for segments with slow step frequencies (< 250Hz).
        TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (st.exec_segment->prescaler<<CS10);
      #endif
      */

      // Initialize step segment timing per step and load number of steps to execute.
      //OCR1A = st.exec_segment->cycles_per_tick;
      //TMR1 = 0xFFFF - st.exec_segment->cycles_per_tick;

      st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.
      // If the new segment starts a new planner block, initialize stepper variables and counters.
      // NOTE: When the segment data index changes, this indicates a new planner block.
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];

        if (st.exec_block_index > 1) {
          st.exec_block_index += 1;
          st.exec_block_index -= 1;
        }
        
        // Initialize Bresenham line and distance counters
        local = st.exec_block->step_event_count;
        st.counter_x = (local >> 1);
        st.counter_y = st.counter_x;
        st.counter_z = st.counter_x;        
      }

      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask; 

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
        amass_level = st.exec_segment->amass_level;
        //<Magic>
        //SB!This code does not shift property
        
        //Topolino---------------------

        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> amass_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> amass_level;
        st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> amass_level;

        //Pluto---------------------
         /*
        stepsBefore = st.exec_block->steps[X_AXIS];
        st.steps[X_AXIS] = stepsBefore >> amass_level;

        //Pippo---------------------

        stepsBefore = st.exec_block->steps[Y_AXIS];
        st.steps[Y_AXIS] = stepsBefore >> amass_level;

        stepsBefore = st.exec_block->steps[Z_AXIS];
        st.steps[Z_AXIS] = stepsBefore >> amass_level;
        //</Magic>
           */
        
        //st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
        //st.steps[Z_AXIS] = (st.exec_block->steps[Z_AXIS]) >> (st.exec_segment->amass_level);
      #endif
      
    } else {
      // Segment buffer empty. Shutdown.
      //st_go_idle(false);
      //bit_true_atomic(sys.execute,EXEC_CYCLE_STOP); // Flag main program for cycle end
      //busy = false;
      return; // Nothing to do but exit.
    }
  }

  //SB!Moved the period set because we don't have a period register, and must initialize the counter to max - desired period
  //TMR1 = 0xFFFF - (st.exec_segment->cycles_per_tick - TMR1);
  //TMR1 = 0xFFFF;// - (((st.exec_segment->cycles_per_tick)>>4) - TMR1);
  //TMR1 = 0xF000;
  //TMR1H = (TMR1 >> 8);
  //TMR1L = TMR1 & 0xFF;
  //TMR1 = 1;
    
  // Check probing state.
  //TODO probe_state_monitor();
   
  // Reset step out bits.
  st.step_outbits = 0; 

  // Execute step displacement profile by Bresenham line algorithm
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_x += st.steps[X_AXIS];
  #else
    st.counter_x += st.exec_block->steps[X_AXIS];
  #endif  
  if (st.counter_x > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<X_STEP_BIT);
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
    else { sys.position[X_AXIS]++; }
  }
  

  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_y += st.steps[Y_AXIS];
  #else
    st.counter_y += st.exec_block->steps[Y_AXIS];
  #endif    
  if (st.counter_y > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Y_STEP_BIT);
    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
    else { sys.position[Y_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_z += st.steps[Z_AXIS];
  #else
    st.counter_z += st.exec_block->steps[Z_AXIS];
  #endif  
  if (st.counter_z > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Z_STEP_BIT);
    st.counter_z -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
    else { sys.position[Z_AXIS]++; }
  }

  
  // During a homing cycle, lock out and prevent desired axes from moving.
  //if (sys.state == STATE_HOMING) { st.step_outbits &= sys.homing_axis_lock; }

  st.step_count--; // Decrement step events count 
  if (st.step_count == 0) {
    // Segment is complete. Discard current segment and advance segment indexing.
    st.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }

  //st.step_outbits ^= step_port_invert_mask;  // Apply step port invert mask
  
  if (bit_istrue(st.step_outbits, (1<<X_STEP_BIT))) {
    if (bit_istrue(st.dir_outbits, (1<<X_DIRECTION_BIT)))
      motor_step(IDX_MOTOR1, MOTOR_DIRECTION_FORWARD);
    else
      motor_step(IDX_MOTOR1, MOTOR_DIRECTION_BACKWARD);
  }
  
  //busy = false;
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; // Debug: Used to time ISR
}


/* The Stepper Port Reset Interrupt: Timer0 OVF interrupt handles the falling edge of the step
   pulse. This should always trigger before the next Timer1 COMPA interrupt and independently
   finish, if Timer1 is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is 
   added to Grbl.
*/
// This interrupt is enabled by ISR_TIMER1_COMPAREA when it sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds) 
// completing one step cycle.
//TODO
/*
ISR(TIMER0_OVF_vect)
{
  // Reset stepping pins (leave the direction pins)
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK); 
  TCCR0B = 0; // Disable Timer0 to prevent re-entering this interrupt when it's not needed. 
}
#ifdef STEP_PULSE_DELAY
  // This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
  // initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
  // will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
  // The new timing between direction, step pulse, and step complete events are setup in the
  // st_wake_up() routine.
  ISR(TIMER0_COMPA_vect) 
  { 
    STEP_PORT = st.step_bits; // Begin step pulse.
  }
#endif
*/

// Reset and clear stepper subsystem variables
void st_reset()
{
  // Initialize stepper driver idle state.
  st_go_idle(false);
  
  // Initialize stepper algorithm variables.
  //memset(&prep, 0, sizeof(prep));
  //pl_block = NULL;  // Planner block pointer used by segment buffer

  memset(&st, 0, sizeof(st));
  st.exec_segment = NULL;
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;
  
  //TODO farle arrivare dai settings
  //st_generate_step_dir_invert_masks();
      
  // Initialize step and direction port pins.

  STEP_PORT = (STEP_PORT & ~STEP_MASK) | step_port_invert_mask;
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | dir_port_invert_mask;
}


// Initialize and start the stepper motor subsystem
void stepper_init()
{
  // Configure step and direction interface pins
  STEP_DDR |= STEP_MASK;
  STEPPERS_DISABLE_DDR |= 1<<STEPPERS_DISABLE_BIT;
  DIRECTION_DDR |= DIRECTION_MASK;

  TMR1ON_bit = 0; //Turn timer off
  T1CKPS0_bit = 0; //Prescaler 1:1
  T1CKPS1_bit = 0;
  TMR1CS_bit = 0; //Fosc/4, uses the CPU clock source
  TMR1IF_bit = 0;
  TMR1IE_bit = 1;

//TODO
/*
  // Configure Timer 1: Stepper Driver Interrupt
  TCCR1B &= ~(1<<WGM13); // waveform generation = 0100 = CTC
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~((1<<WGM11) | (1<<WGM10)); 
  TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0)); // Disconnect OC1 output
  // TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Set in st_go_idle().
  // TIMSK1 &= ~(1<<OCIE1A);  // Set in st_go_idle().
  
  // Configure Timer 0: Stepper Port Reset Interrupt
  TIMSK0 &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0)); // Disconnect OC0 outputs and OVF interrupt.
  TCCR0A = 0; // Normal operation
  TCCR0B = 0; // Disable Timer0 until needed
  TIMSK0 |= (1<<TOIE0); // Enable Timer0 overflow interrupt
  #ifdef STEP_PULSE_DELAY
    TIMSK0 |= (1<<OCIE0A); // Enable Timer0 Compare Match A interrupt
  #endif
*/
}