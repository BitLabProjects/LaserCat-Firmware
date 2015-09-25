#include "system.h"
#include "motor.h"

#define COIL_OFF 0
#define COIL_FORWARD 1
#define COIL_BACKWARD 2

//-- Motor1
//Coil 1
#define Motor1_Coil1_Direction_TrisBit TRISB4_bit
#define Motor1_Coil1_Direction_PortBit LATB4_bit
#define Motor1_Coil1_OnOff1_TrisBit TRISB5_bit
#define Motor1_Coil1_OnOff1_PortBit LATB5_bit
#define Motor1_Coil1_OnOff2_TrisBit TRISB3_bit
#define Motor1_Coil1_OnOff2_PortBit LATB3_bit
//Coil 2
#define Motor1_Coil2_Direction_TrisBit TRISB1_bit
#define Motor1_Coil2_Direction_PortBit LATB1_bit
#define Motor1_Coil2_OnOff1_TrisBit TRISB2_bit
#define Motor1_Coil2_OnOff1_PortBit LATB2_bit
#define Motor1_Coil2_OnOff2_TrisBit TRISB0_bit
#define Motor1_Coil2_OnOff2_PortBit LATB0_bit

//-- Motor2
//Coil 1
#define Motor2_Coil1_Direction_TrisBit TRISD3_bit
#define Motor2_Coil1_Direction_PortBit LATD3_bit
#define Motor2_Coil1_OnOff1_TrisBit TRISD4_bit
#define Motor2_Coil1_OnOff1_PortBit LATD4_bit
#define Motor2_Coil1_OnOff2_TrisBit TRISD2_bit
#define Motor2_Coil1_OnOff2_PortBit LATD2_bit
//Coil 2
#define Motor2_Coil2_Direction_TrisBit TRISD6_bit
#define Motor2_Coil2_Direction_PortBit LATD6_bit
#define Motor2_Coil2_OnOff1_TrisBit TRISD7_bit
#define Motor2_Coil2_OnOff1_PortBit LATD7_bit
#define Motor2_Coil2_OnOff2_TrisBit TRISD5_bit
#define Motor2_Coil2_OnOff2_PortBit LATD5_bit

void Motor_Coil1(uint8_t idxMotor, uint8_t state);
void Motor_Coil2(uint8_t idxMotor, uint8_t state);

void Motor1_Coil1(uint8_t state);
void Motor1_Coil2(uint8_t state);
void Motor2_Coil1(uint8_t state);
void Motor2_Coil2(uint8_t state);

uint8_t motor_current_step[2];

int motor_init()
{
  Motor1_Coil1_Direction_TrisBit = 0;
  Motor1_Coil1_OnOff1_TrisBit = 0;
  Motor1_Coil1_OnOff2_TrisBit = 0;
  Motor1_Coil2_Direction_TrisBit = 0;
  Motor1_Coil2_OnOff1_TrisBit = 0;
  Motor1_Coil2_OnOff2_TrisBit = 0;

  Motor2_Coil1_Direction_TrisBit = 0;
  Motor2_Coil1_OnOff1_TrisBit = 0;
  Motor2_Coil1_OnOff2_TrisBit = 0;
  Motor2_Coil2_Direction_TrisBit = 0;
  Motor2_Coil2_OnOff1_TrisBit = 0;
  Motor2_Coil2_OnOff2_TrisBit = 0;

  //Codice duplicato perché chiamando Motor1_Coil1 il compilatore si lamenta di codice non rientrante
  //Motor1_Coil1(COIL_OFF);
  //Motor1_Coil2(COIL_OFF);
  Motor1_Coil1_OnOff1_PortBit = 1;
  Motor1_Coil1_OnOff2_PortBit = 1;
  Motor1_Coil2_OnOff1_PortBit = 1;
  Motor1_Coil2_OnOff2_PortBit = 1;
  
  Motor2_Coil1_OnOff1_PortBit = 1;
  Motor2_Coil1_OnOff2_PortBit = 1;
  Motor2_Coil2_OnOff1_PortBit = 1;
  Motor2_Coil2_OnOff2_PortBit = 1;
  
  motor_current_step[0] = 0;
  motor_current_step[1] = 0;
}

void motor_step(uint8_t idx_motor, uint8_t motor_direction) {
  uint8_t motor_current_step_local;
  motor_current_step_local = motor_current_step[idx_motor];
  if (motor_direction == MOTOR_DIRECTION_FORWARD) {
    if (motor_current_step_local == 7)
      motor_current_step_local = 0;
    else
      motor_current_step_local += 1;
  } else {
    if (motor_current_step_local == 0)
      motor_current_step_local = 7;
    else
      motor_current_step_local -= 1;
  }
  motor_current_step[idx_motor] = motor_current_step_local;
  
  /*switch(motor_current_step_local) {
    case 0: {
      Motor_Coil1(COIL_FORWARD);
      Motor_Coil2(COIL_BACKWARD);
    } break;
    case 1: {
      Motor_Coil1(COIL_FORWARD);
      Motor_Coil2(COIL_OFF);
    } break;
    case 2: {
      Motor_Coil1(COIL_FORWARD);
      Motor_Coil2(COIL_FORWARD);
    } break;
    case 3: {
      Motor_Coil1(COIL_OFF);
      Motor_Coil2(COIL_FORWARD);
    } break;
    case 4: {
      Motor_Coil1(COIL_BACKWARD);
      Motor_Coil2(COIL_FORWARD);
    } break;
    case 5: {
      Motor_Coil1(COIL_BACKWARD);
      Motor_Coil2(COIL_OFF);
    } break;
    case 6: {
      Motor_Coil1(COIL_BACKWARD);
      Motor_Coil2(COIL_BACKWARD);
    } break;
    case 7: {
      Motor_Coil1(COIL_OFF);
      Motor_Coil2(COIL_BACKWARD);
    } break;
  }*/
  
  switch(motor_current_step_local) {
    case 0: {
      Motor_Coil1(idx_motor, COIL_FORWARD);
      Motor_Coil2(idx_motor, COIL_BACKWARD);
    } break;
    case 1: {
      Motor_Coil1(idx_motor, COIL_FORWARD);
      Motor_Coil2(idx_motor, COIL_BACKWARD);
    } break;
    case 2: {
      Motor_Coil1(idx_motor, COIL_FORWARD);
      Motor_Coil2(idx_motor, COIL_FORWARD);
    } break;
    case 3: {
      Motor_Coil1(idx_motor, COIL_FORWARD);
      Motor_Coil2(idx_motor, COIL_FORWARD);
    } break;
    case 4: {
      Motor_Coil1(idx_motor, COIL_BACKWARD);
      Motor_Coil2(idx_motor, COIL_FORWARD);
    } break;
    case 5: {
      Motor_Coil1(idx_motor, COIL_BACKWARD);
      Motor_Coil2(idx_motor, COIL_FORWARD);
    } break;
    case 6: {
      Motor_Coil1(idx_motor, COIL_BACKWARD);
      Motor_Coil2(idx_motor, COIL_BACKWARD);
    } break;
    case 7: {
      Motor_Coil1(idx_motor, COIL_BACKWARD);
      Motor_Coil2(idx_motor, COIL_BACKWARD);
    } break;
  }
}

/*void aspetta(uint32_t step) {
  uint32_t i,j;
  for(i=0; i<step; i++) {
    for(j=0; j<10; j++) {

    }
  }
}

void rotazione1(){
  uint32_t d = 100;
  //Motor1_Coil1(COIL_FORWARD);
  //Motor1_Coil2(COIL_BACKWARD);
  //return;
  

  Motor1_Coil1(COIL_FORWARD);
  Motor1_Coil2(COIL_BACKWARD);
  aspetta(d);
  Motor1_Coil1(COIL_FORWARD);
  Motor1_Coil2(COIL_OFF);
  aspetta(d);
  Motor1_Coil1(COIL_FORWARD);
  Motor1_Coil2(COIL_FORWARD);
  aspetta(d);
  Motor1_Coil1(COIL_OFF);
  Motor1_Coil2(COIL_FORWARD);
  aspetta(d);
  Motor1_Coil1(COIL_BACKWARD);
  Motor1_Coil2(COIL_FORWARD);
  aspetta(d);
  Motor1_Coil1(COIL_BACKWARD);
  Motor1_Coil2(COIL_OFF);
  aspetta(d);
  Motor1_Coil1(COIL_BACKWARD);
  Motor1_Coil2(COIL_BACKWARD);
  aspetta(d);
  Motor1_Coil1(COIL_OFF);
  Motor1_Coil2(COIL_BACKWARD);
  aspetta(d);
} */

/*void rotazione2(){
  Motor1_Coil1(COIL_OFF);
  Motor1_Coil2(COIL_BACKWARD);

  Motor1_Coil1(COIL_BACKWARD);
  Motor1_Coil2(COIL_BACKWARD);

  Motor1_Coil1(COIL_BACKWARD);
  Motor1_Coil2(COIL_OFF);

  Motor1_Coil1(COIL_BACKWARD);
  Motor1_Coil2(COIL_FORWARD);

  Motor1_Coil1(COIL_OFF);
  Motor1_Coil2(COIL_FORWARD);

  Motor1_Coil1(COIL_FORWARD);
  Motor1_Coil2(COIL_FORWARD);

  Motor1_Coil1(COIL_FORWARD);
  Motor1_Coil2(COIL_OFF);

  Motor1_Coil1(COIL_FORWARD);
  Motor1_Coil2(COIL_BACKWARD);

}*/

void Motor_Coil1(uint8_t idxMotor, uint8_t state)
{
  switch (idxMotor){
    case 0:
      Motor1_Coil1(state);
      break;
    case 1:
      Motor2_Coil1(state);
      break;
  }
}
void Motor_Coil2(uint8_t idxMotor, uint8_t state)
{
  switch (idxMotor){
    case 0:
      Motor1_Coil2(state);
      break;
    case 1:
      Motor2_Coil2(state);
      break;
  }
}

void Motor1_Coil1(uint8_t state) {
  switch(state) {
    case COIL_OFF:
      Motor1_Coil1_OnOff1_PortBit = 1;
      Motor1_Coil1_OnOff2_PortBit = 1;
    break;
    case COIL_FORWARD:
      Motor1_Coil1_OnOff1_PortBit = 0;
      Motor1_Coil1_OnOff2_PortBit = 0;
      Motor1_Coil1_Direction_PortBit = 0;
    break;
    case COIL_BACKWARD:
      Motor1_Coil1_OnOff1_PortBit = 0;
      Motor1_Coil1_OnOff2_PortBit = 0;
      Motor1_Coil1_Direction_PortBit = 1;
    break;
  }
}

void Motor1_Coil2(uint8_t state) {
    switch(state) {
      case COIL_OFF:
        Motor1_Coil2_OnOff1_PortBit = 1;
        Motor1_Coil2_OnOff2_PortBit = 1;
      break;
      case COIL_FORWARD:
        Motor1_Coil2_OnOff1_PortBit = 0;
        Motor1_Coil2_OnOff2_PortBit = 0;
        Motor1_Coil2_Direction_PortBit = 0;
      break;
      case COIL_BACKWARD:
        Motor1_Coil2_OnOff1_PortBit = 0;
        Motor1_Coil2_OnOff2_PortBit = 0;
        Motor1_Coil2_Direction_PortBit = 1;
      break;
    }
}


void Motor2_Coil1(uint8_t state) {
  switch(state) {
    case COIL_OFF:
      Motor2_Coil1_OnOff1_PortBit = 1;
      Motor2_Coil1_OnOff2_PortBit = 1;
    break;
    case COIL_FORWARD:
      Motor2_Coil1_OnOff1_PortBit = 0;
      Motor2_Coil1_OnOff2_PortBit = 0;
      Motor2_Coil1_Direction_PortBit = 0;
    break;
    case COIL_BACKWARD:
      Motor2_Coil1_OnOff1_PortBit = 0;
      Motor2_Coil1_OnOff2_PortBit = 0;
      Motor2_Coil1_Direction_PortBit = 1;
    break;
  }
}

void Motor2_Coil2(uint8_t state) {
    switch(state) {
      case COIL_OFF:
        Motor2_Coil2_OnOff1_PortBit = 1;
        Motor2_Coil2_OnOff2_PortBit = 1;
      break;
      case COIL_FORWARD:
        Motor2_Coil2_OnOff1_PortBit = 0;
        Motor2_Coil2_OnOff2_PortBit = 0;
        Motor2_Coil2_Direction_PortBit = 0;
      break;
      case COIL_BACKWARD:
        Motor2_Coil2_OnOff1_PortBit = 0;
        Motor2_Coil2_OnOff2_PortBit = 0;
        Motor2_Coil2_Direction_PortBit = 1;
      break;
    }
}