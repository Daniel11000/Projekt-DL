#pragma once

/*
 	 commands left motor to drive forwards or backwards

 	 PWM : 0 - 100 integer value
 	 command: "FORWARD", "BACKWARD","FAST STOP", "SOFT STOP"
 */
void left_motor_drive(int PWM, char command[]);
/*
 	 commands right motor to drive forwards or backwards

 	 PWM : 0 - 100 integer value
 	 command: "FORWARD", "BACKWARD","FAST STOP", "SOFT STOP"
 */
void right_motor_drive(int PWM, char command[]);
