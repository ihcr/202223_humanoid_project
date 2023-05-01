#ifndef MOTOR_H
#define MOTOR_H

#define Mode 10 //operation mode of motors, 0=stall, 5=slow motion, 10=torque mode

#include <iostream>
#include <string>
#include <list>
#include "yaml-cpp/yaml.h"
#include "PIDcontroller.h"
#include "unitreeMotor/unitreeMotor.h"

//class for an A1 motor
class Motor : public PIDcontroller {
  public:
    
    //constructors
    Motor();
    Motor(float torque_limit, float temp_limit, float refresh_frequency, int line_id, int ID, YAML::Node PID_gains);

    //public member methods
    void print_init_values();                           //function to print initial values of motor's variables
    void new_torque();                                  //calculate new torque required for the motor
    void desired_position_update();                     //get new desired position from terminal (testing purposes)
    void send_desired_position(float desired_position); //set the desired position of the motor
    float read_actual_position();                       //get the actual position of the motor
    float read_desired_position();
    float read_torque();

    //public member variables 
    MOTOR_send motor_run, motor_stop; //motor send message structs for run and stop instructions
    MOTOR_recv motor_r;               //motor receive message struct for obtainig data from the motor

  private:
    
    //private member variables
    int _line_id = 0;                        //default value for line id, will be changed in the constructor
    MotorType _motorType = MotorType::A1Go1; //type of the connected motor (in this case always A1)
    float _T = 0.0;                          //forward fetched torque [Nm]
    float _W = 0.0;                          //desired angular velocity
    float _Pos = 0.0;                        //desired position
    float _K_P = 0.0;                        //gain of position error
    float _K_W = 0.0;                        //gain of velocity error
    float _gear_ratio = 9.1;                 //internal gear ratio of the motor
    float _temp_limit;
};

//default contructor
Motor::Motor() : PIDcontroller{}{

}

Motor::Motor(float torque_limit, float temp_limit, float refresh_frequency, int line_id, int ID, YAML::Node PID_gains) : PIDcontroller{torque_limit, refresh_frequency, PID_gains} {
    std::cout << "Motor.h - constructor()" << std::endl;
    _line_id = line_id;
    _temp_limit = temp_limit;
    // set the id of motor
    motor_run.motorType = _motorType; //set the motor type in the message struct
    motor_run.id = ID;                //set message ID to be matching with motor's one
    motor_run.mode = Mode;            //0: idle, 5: open loop rotation, 10: closed loop FOC control
    motor_run.T = _T;                 //forward fetched torque [Nm]
    motor_run.W = _W;                 //desired angular velocity
    motor_run.Pos = _Pos;             //desired position
    motor_run.K_P = _K_P;             //gain of position error
    motor_run.K_W = _K_W;             //gain of velocity error

    motor_stop.motorType = _motorType; //set the motor type in the message struct
    motor_stop.id = ID;                //set message ID to be matching with motor's one
    motor_stop.mode = 0;               //0: idle, 5: open loop rotation, 10: closed loop FOC control
    motor_stop.T = _T;                 //forward fetched torque [Nm]
    motor_stop.W = _W;                 //desired angular velocity
    motor_stop.Pos = _Pos;             //desired position
    motor_stop.K_P = _K_P;             //gain of position error
    motor_stop.K_W = _K_W;             //gain of velocity error

    motor_r.motorType = _motorType;    //set the motor type in the message struct

    print_init_values();               //print values of initialised motor

    //encode the data so that it can be sent
    modify_data(&motor_run);
    modify_data(&motor_stop);
}

//function to print initial values of motor's variables
void Motor::print_init_values(){
    std::cout << "Motor.h - print_init_values()" << std::endl;
    std::cout << "line: " << _line_id << std::endl;
    std::cout << "ID: " << motor_run.id << std::endl;
    std::cout << "Mode: " << motor_run.mode << std::endl;
    std::cout << "T: " << motor_run.T << std::endl;
    std::cout << "W: " << motor_run.W << std::endl;
    std::cout << "Pos: " << motor_run.Pos/_gear_ratio << std::endl;
    std::cout << "Kp: " << motor_run.K_P << std::endl;
    std::cout << "Kw: " << motor_run.K_W << std::endl;
    std::cout << "******************" << std::endl;
}

//calculate new torque required for the motor
void Motor::new_torque(){
  if(motor_r.Temp > _temp_limit){
    std::cout << "ID: " << motor_run.id << std::endl;
    std::cout << "Motor is overheating" << std::endl;
    motor_run.T = 0.0;
  } else {
    //std::cout << "ID: " << motor_run.id << std::endl;
    motor_run.T = nextValue(motor_r.Pos);
  }
}

//get new desired position from terminal (testing purposes)
void Motor::desired_position_update() {
  std::string obtained_string;
  float new_desired_position = 0;
  std::cout << "\nEnter desired position: ";
  std::getline(std::cin, obtained_string);
  new_desired_position = std::stof(obtained_string);  // convert to float
  if(get_targetPosition() != new_desired_position){
      set_targetPosition(new_desired_position);
  }
}

//set the desired position of the motor
void Motor::send_desired_position(float desired_position){
  set_targetPosition(desired_position);
}

//function to access actual position of the motor
float Motor::read_actual_position(){
  return motor_r.Pos/_gear_ratio; //divide encoder's position by gear ratio to obtain actual position of a rotor
}

float Motor::read_torque(){
  return get_PIDoutput();
}

float Motor::read_desired_position(){
  return get_desired_pos();
}
#endif