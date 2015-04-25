#ifndef motor_h
#define motor_h

#define IDX_MOTOR1 0
#define IDX_MOTOR2 1

#define MOTOR_DIRECTION_FORWARD 1
#define MOTOR_DIRECTION_BACKWARD 2

int motor_init();
void motor_step(uint8_t idx_motor, uint8_t motor_direction);
void rotazione1();
//void rotazione2();

#endif