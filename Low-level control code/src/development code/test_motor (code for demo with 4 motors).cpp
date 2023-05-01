#include "serialPort/SerialPort.h"
#include <csignal>
#include <iostream>
#include <fstream>

#define KP 0.01f // gain of the proportional controller
#define KI 0.0f // gain of the integral controller
#define KD 0.0f // gain of the derivative controller
#define frequency 100.0f // frequency of sending commands [Hz]
#define run_time 15.0f // run time of the motor [s] 
#define T_limit 0.5f  // maximum output torque limit
#define initial_position_difference00 6.28/1 // value added to the starting position of the motor to get target position [rad]
#define initial_position_difference01 6.28/1
#define initial_position_difference10 6.28/1
#define initial_position_difference11 6.28/1
#define load "unloaded"

//hardware setup
//#define lines 2 // number of connected lines
//#define 

void motor_send_init(MOTOR_send *motor, int ID, int Mode){
    // set the id of motor
    motor->id = ID;
    // set the motor type, A1Go1 or B1
    //motor->motorType = MotorType::A1Go1; don't know how to pass a class enum to a function
    motor->mode = Mode;   //0: idle, 5: open loop rotation, 10: closed loop FOC control
    motor->T = 0.0;     //forward fetched torque [Nm]
    motor->W = 0.0;     //desired angular velocity
    motor->Pos = 0.0;   //desired position
    motor->K_P = 0.0;   //gain of position error
    motor->K_W = 0.0;   //gain of velocity error
}

void print_init_values(MOTOR_send *motor, int line){
    std::cout << "line: " << line << std::endl;
    std::cout << "ID: " << motor->id << std::endl;
    std::cout << "Mode: " << motor->mode << std::endl;
    std::cout << "T: " << motor->T << std::endl;
    std::cout << "W: " << motor->W << std::endl;
    std::cout << "Pos: " << motor->Pos/9.1 << std::endl;
    std::cout << "Kp: " << motor->K_P << std::endl;
    std::cout << "Kw: " << motor->K_W << std::endl;
    std::cout << "******************" << std::endl;
}

class PIDcontroller {        // The class
  public:              // Access specifier
    PIDcontroller(float initPos, float initDifference);
    float nextValue(float initPos);   // Method/function declaration

  private:
    float gear_ratio = 9.1;
    float error = 0;                                                      //set the initial error to 0 
    float targetPosition = 0; //set the target position to be different to initial position by specified value
    float measuredPosition = 0;                             //set the initial position
    
    //set initial values to all the variables of the PID controller
    float integral = 0;                                                   
    float derivative = 0;
    float prevError = 0; //previous error is set to the initial error to avoid extremely high initial value
    float PIDoutput = 0;
    float timeDelta = 1/frequency;                 //time delta is equal to the period of sending messages
    
};

PIDcontroller::PIDcontroller(float initPos, float initDifference) {
    targetPosition = initPos/gear_ratio + initDifference;
    measuredPosition = initPos/gear_ratio;
    prevError = initDifference;
}

// Method/function definition outside the class
float PIDcontroller::nextValue(float initPos) {
    measuredPosition = initPos/9.1;
    error = targetPosition - measuredPosition;
    integral = integral + error * timeDelta;
    derivative = (error - prevError) / timeDelta;
    prevError = error;
    PIDoutput = KP * error + KI * integral + KD * derivative;
    PIDoutput = std::min(std::max(PIDoutput, -T_limit), T_limit);
    return PIDoutput;
}


int main(){
    // set the serial port name
    SerialPort serial0("/dev/ttyUSB0");
    SerialPort serial1("/dev/ttyUSB1");

    // send message struct
    MOTOR_send motor00_run, motor00_stop, motor01_run, motor01_stop, motor10_run, motor10_stop, motor11_run, motor11_stop; // motorXY, X=line ID, Y=motor ID
    // receive message struct
    MOTOR_recv motor00_r, motor01_r, motor10_r, motor11_r;

    // set the id of motor
    //motor_run.id = 0;
    // set the motor type, A1Go1 or B1
    motor00_run.motorType = MotorType::A1Go1;
    motor_send_init(&motor00_run, 0, 10);
    std::cout << "ID: " << motor00_run.id << std::endl;
    motor01_run.motorType = MotorType::A1Go1;
    motor_send_init(&motor01_run, 1, 10);
    motor10_run.motorType = MotorType::A1Go1;
    motor_send_init(&motor10_run, 0, 10);
    motor11_run.motorType = MotorType::A1Go1;
    motor_send_init(&motor11_run, 2, 10);
    
/*  motor_run.mode = 10;   //0: idle, 5: open loop rotation, 10: closed loop FOC control
    motor_run.T = 0.0;     //forward fetched torque [Nm]
    motor_run.W = 0.0;     //desired angular velocity
    motor_run.Pos = 0.0;   //desired position
    motor_run.K_P = 0.0;   //gain of position error
    motor_run.K_W = 0.0;   //gain of velocity error
*/
//    motor_stop.id = motor_run.id;               //set the same ID for run and stop commands
    motor00_stop.motorType = motor00_run.motorType; //set the same type of the motor for run and stop commands
    motor_send_init(&motor00_stop, 0, 0);
    motor01_stop.motorType = motor01_run.motorType; //set the same type of the motor for run and stop commands
    motor_send_init(&motor01_stop, 1, 0);
    motor10_stop.motorType = motor10_run.motorType; //set the same type of the motor for run and stop commands
    motor_send_init(&motor10_stop, 0, 0);
    motor11_stop.motorType = motor11_run.motorType; //set the same type of the motor for run and stop commands
    motor_send_init(&motor11_stop, 2, 0);
//    motor_stop.mode = 0;                        //set the mode of the stop command to idle

    motor00_r.motorType = motor00_run.motorType;    //set the same motor type for receive and send structs
    motor01_r.motorType = motor01_run.motorType;
    motor10_r.motorType = motor10_run.motorType;    //set the same motor type for receive and send structs
    motor11_r.motorType = motor11_run.motorType;

    //print initial values
    print_init_values(&motor00_run, 0);
    print_init_values(&motor01_run, 0);
    print_init_values(&motor10_run, 1);
    print_init_values(&motor11_run, 1);

    // encode data into motor commands
    modify_data(&motor00_run);
    modify_data(&motor00_stop);
    modify_data(&motor01_run);
    modify_data(&motor01_stop);
    modify_data(&motor10_run);
    modify_data(&motor10_stop);
    modify_data(&motor11_run);
    modify_data(&motor11_stop);

    //send dummy message with zeros to get readings from the motor
    serial0.sendRecv(&motor00_run, &motor00_r); 
    extract_data(&motor00_r);
    sleep(1/frequency);
    serial0.sendRecv(&motor01_run, &motor01_r); 
    extract_data(&motor01_r);
    sleep(1/frequency);  
    serial1.sendRecv(&motor10_run, &motor10_r); 
    extract_data(&motor10_r);  
    sleep(1/frequency);
    serial1.sendRecv(&motor11_run, &motor11_r); 
    extract_data(&motor11_r);
    sleep(1/frequency); 
    std::cout << "dummy messages went through" << std::endl;     
 
    PIDcontroller PID00(motor00_r.Pos, initial_position_difference00);
    PIDcontroller PID01(motor01_r.Pos, initial_position_difference01);
    PIDcontroller PID10(motor10_r.Pos, initial_position_difference10);
    PIDcontroller PID11(motor11_r.Pos, initial_position_difference11);

    int i=0;                                                              //set iterator to 0
    int iterations= run_time*frequency;                                   //get the number of iterations for while loop

    while(i<iterations){
    	serial0.sendRecv(&motor00_run, &motor00_r); 
        extract_data(&motor00_r);
        

        motor00_run.T = PID00.nextValue(motor00_r.Pos);
        

        i++;
	    sleep(1/frequency); // sleep for a set period
    }
   
    serial0.sendRecv(&motor00_stop, &motor00_r);
    serial0.sendRecv(&motor01_stop, &motor01_r);
    serial1.sendRecv(&motor10_stop, &motor10_r);
    serial1.sendRecv(&motor11_stop, &motor11_r);
    

    i=0;                                                              //set iterator to 0
                                      //get the number of iterations for while loop

    while(i<iterations){
    	
        serial0.sendRecv(&motor01_run, &motor01_r); 
        extract_data(&motor01_r);  
        

        
        motor01_run.T = PID01.nextValue(motor01_r.Pos);
        

        i++;
	    sleep(1/frequency); // sleep for a set period
    }
   
    serial0.sendRecv(&motor00_stop, &motor00_r);
    serial0.sendRecv(&motor01_stop, &motor01_r);
    serial1.sendRecv(&motor10_stop, &motor10_r);
    serial1.sendRecv(&motor11_stop, &motor11_r);

    i=0;                                                              //set iterator to 0
                                     //get the number of iterations for while loop

    while(i<iterations){
    	
        serial1.sendRecv(&motor10_run, &motor10_r); 
        extract_data(&motor10_r);  
        

        
        motor10_run.T = PID10.nextValue(motor10_r.Pos);
        

        i++;
	    sleep(1/frequency); // sleep for a set period
    }
   
    serial0.sendRecv(&motor00_stop, &motor00_r);
    serial0.sendRecv(&motor01_stop, &motor01_r);
    serial1.sendRecv(&motor10_stop, &motor10_r);
    serial1.sendRecv(&motor11_stop, &motor11_r);

    i=0;                                                              //set iterator to 0
                                    //get the number of iterations for while loop

    while(i<iterations){
    	
        serial1.sendRecv(&motor11_run, &motor11_r); 
        extract_data(&motor11_r); 

        motor11_run.T = PID11.nextValue(motor11_r.Pos); 

        i++;
	    sleep(1/frequency); // sleep for a set period
    }
   
    serial0.sendRecv(&motor00_stop, &motor00_r);
    serial0.sendRecv(&motor01_stop, &motor01_r);
    serial1.sendRecv(&motor10_stop, &motor10_r);
    serial1.sendRecv(&motor11_stop, &motor11_r);


    PIDcontroller PID001(motor00_r.Pos, initial_position_difference00);
    PIDcontroller PID011(motor01_r.Pos, initial_position_difference01);
    PIDcontroller PID101(motor10_r.Pos, initial_position_difference10);
    PIDcontroller PID111(motor11_r.Pos, initial_position_difference11);


    i=0;                                                              //set iterator to 0
                                      //get the number of iterations for while loop

    while(i<iterations){
    	serial0.sendRecv(&motor00_run, &motor00_r); 
        extract_data(&motor00_r);
        serial0.sendRecv(&motor01_run, &motor01_r); 
        extract_data(&motor01_r);  
        serial1.sendRecv(&motor10_run, &motor10_r); 
        extract_data(&motor10_r);  
        serial1.sendRecv(&motor11_run, &motor11_r); 
        extract_data(&motor11_r); 

        motor00_run.T = PID001.nextValue(motor00_r.Pos);
        motor01_run.T = PID011.nextValue(motor01_r.Pos);
        motor10_run.T = PID101.nextValue(motor10_r.Pos);
        motor11_run.T = PID111.nextValue(motor11_r.Pos); 

        i++;
	    sleep(1/frequency); // sleep for a set period
    }
   
    serial0.sendRecv(&motor00_stop, &motor00_r);
    serial0.sendRecv(&motor01_stop, &motor01_r);
    serial1.sendRecv(&motor10_stop, &motor10_r);
    serial1.sendRecv(&motor11_stop, &motor11_r);

    return 0;
}
