#include "motor_control_demos.h"

#define PI 3.14159265
#define G 9.81
#define KT_ak60_6 0.068 // 0.068Nm/A(from datasheet), 0.087(KT real from Neurobionics)



void gravity_compensation_initialize(uint16_t ID){
    enter_motor_mode_servo(ID);

    comm_can_set_origin(ID, 0);
    
    
}

//mass: kg, arm_length: m
void gravity_compensation(uint16_t ID, float arm_length, float mass, float arm_current){
    extern float motor_pos;
    extern float current;
    extern float duty;
    current = -(((mass*arm_length)*sin(motor_pos*PI/180))/(KT_ak60_6)+arm_current*sin(motor_pos*PI/180));
    comm_can_set_current(ID,current);
    
    /*
    if (fabsf(current)>0.5){
        comm_can_set_current(ID, current);
    }

    else {
        if (0 < motor_pos < 20){
            duty = -0.01*motor_pos/6;
            comm_can_set_duty(ID, duty);
        }
        else if (340 < motor_pos < 360){
            duty = 0.01*(360-motor_pos)/6;
            comm_can_set_duty(ID, duty);
        }
    }
    */
}

