#include "motor.c"

void setup() {
    motor_init();    
}

void loop() {
     motor_step(IDX_MOTOR1, MOTOR_DIRECTION_FORWARD);
}
