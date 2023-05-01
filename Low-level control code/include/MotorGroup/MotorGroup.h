#ifndef MOTORGROUP_H
#define MOTORGROUP_H

#include <iostream>
#include <string>
#include <string.h>
#include <list>
#include <chrono>
#include <thread>
#include "yaml-cpp/yaml.h"
#include "Motor/Motor.h"
#include "serialPort/SerialPort.h"

//class for a group of motors on one USB port
class MotorGroup : private SerialPort {
  public:
    //constructor 
    MotorGroup(const std::string &portName);
    MotorGroup(float torque_limit, float temp_limit, float refresh_frequency, std::string &portName, std::list<int> IDs, YAML::Node PID_gains);
    
    //destructor is needed to free up memory after dynamic creation of an array  
    ~MotorGroup();                                                                 

    //member methods
    void update_torque();           //loop through all the instantiated motors to send a message, update data and calculate new torque
    void update_desired_position(); //loop through all the instantiated motors to update their desired positions
    void stop_motors();             //send stop message to all motors
    void print_info();              //print info for every motor
    int get_number_of_motors();     //accessor of _number_of_motors private variable
    void send_desired_positions(std::vector<float> desired_positions); //function to pass desired positions to all motors connected to one converter
    std::list<float> read_actual_positions(); //function to obtain actual positions of all motors connected to one converter
    std::vector<float> read_torque();
    std::vector<float> read_desired_position();
    

    //member variables 
  private:
    int _number_of_motors;          //number of motors connected to the USB converter
    int _converter_id;              //ID of instantiated USB port, e.g. if USB port name is /dev/ttyUSB0, _line_id is going to be 0
    float _gear_ratio = 9.1;        //internal gear ratio of the motor
    Motor* _motors;                 //create pointers to motor objects
};

//default constructor
MotorGroup::MotorGroup(const std::string &portName) : SerialPort{portName}{

}

MotorGroup::MotorGroup(float torque_limit, float temp_limit, float refresh_frequency, std::string &portName, std::list<int> IDs, YAML::Node PID_gains) : SerialPort{portName}  {
    std::cout << "MotorGroup.h - constructor()" << std::endl;                  //print info that the program entered the constructor of this class
    _converter_id = (int)portName[11] - 48;                                    //when casted to int a character is assigned its ASCII value, so by subtracting 48 we can get desired value
    std::cout << "_converter_id: " << _converter_id << std::endl;              //print id of instantiated line
    _number_of_motors = IDs.size();                                            //get the number of motors connected to the USB converter
    std::cout << "_number_of_motors " << _number_of_motors << std::endl;       //print number of connected motors
    _motors = new Motor[_number_of_motors];                                    //allocate memory for array of Motor
    
    //create a loop to instantiate all the motors
    for (int i = 0; i < _number_of_motors; i++) {
        int ID = IDs.front();
        _motors[i] = Motor(torque_limit, temp_limit, refresh_frequency, _converter_id, ID, PID_gains[i]); //initialize each object in the array
        IDs.pop_front(); 
        sendRecv(&_motors[i].motor_run, &_motors[i].motor_r);                          //send an initial message to the motor and receive data from it
        extract_data(&_motors[i].motor_r);                                             //extract data from received message
        _motors[i].set_targetPosition(_motors[i].motor_r.Pos/_gear_ratio);             //set desired position of the PIDcontroller to the initial position of the motor to make sure it does not move when instantiated
        _motors[i].nextValue(_motors[i].motor_r.Pos);                                  //update the actual position of the PIDcontroller to the actual position of the motor, so that error=0 in PIDcontroller
        std::cout << "dummy message " << i << " went through and desired position set to initial position" << std::endl;
    }
}

MotorGroup::~MotorGroup(){
    delete[] _motors; // Free memory allocated for array of MyObject
}

void MotorGroup::update_torque(){
    //a loop to iterate through all the motors
    //std::cout << "Entered MotorGroup::update_torque()"<< std::endl;
    for(int i = 0; i < _number_of_motors; i++){
        //auto start = std::chrono::high_resolution_clock::now();
        sendRecv(&_motors[i].motor_run, &_motors[i].motor_r); //send an initial message to the motor and receive data from it
        extract_data(&_motors[i].motor_r);                    //extract data from the received message
        //auto end = std::chrono::high_resolution_clock::now();
        //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        //std::cout << "Execution time of sendRecv() and extract_data(): " << duration.count() << " microseconds" << std::endl;
        _motors[i].new_torque();                              //update motor's torque
    }
}

void MotorGroup::update_desired_position(){
    //a loop to iterate through all the motors
    for(int i = 0; i < _number_of_motors; i++){
        _motors[i].desired_position_update(); //update desired position of a motor
    }
}

void MotorGroup::stop_motors(){
    //a loop to iterate through all the motors
    for(int i = 0; i < _number_of_motors; i++){
        sendRecv(&_motors[i].motor_stop, &_motors[i].motor_r); //send a stop message to a motor
    }
}

void MotorGroup::print_info(){
    //a loop to iterate through all the motors and print info about it
    for(int i = 0; i < _number_of_motors; i++){
        std::cout << "motorGroup.print_info(): " << std::endl;
        std::cout << "id: " << _motors[i].motor_run.id << std::endl;
        std::cout << "Pos: " << _motors[i].motor_r.Pos/9.1 << std::endl;
        std::cout << "Target Pos: " << _motors[i].get_targetPosition() << std::endl;
        std::cout << "error: " << _motors[i].get_error() << std::endl;
        std::cout << "******************" << std::endl;
    }
}

int MotorGroup::get_number_of_motors(){
    return _number_of_motors;
}

void MotorGroup::send_desired_positions(std::vector<float> desired_positions){
    for(int i = 0; i < _number_of_motors; i++){
        _motors[i].send_desired_position(desired_positions[i]);
    }
}

//function to obtain actual positions of all motors connected to one converter
std::list<float> MotorGroup::read_actual_positions(){
    std::list<float> actual_positions; //a list to be populated with obtained positions of all motors
    //iterate through all Motor objects
    for(int i = 0; i < _number_of_motors; i++){
        actual_positions.push_back(_motors[i].read_actual_position()); //get actual position of every motor and add it to the list
    }
    return actual_positions;
}

std::vector<float> MotorGroup::read_torque(){
    std::vector<float> temp_vector;
  //iterate through all MotorGroup objects
  for(int i = 0; i < _number_of_motors; i++){
      temp_vector.push_back(_motors[i].read_torque());
  }
  return temp_vector;
}

std::vector<float> MotorGroup::read_desired_position(){
    std::vector<float> temp_vector;
  //iterate through all MotorGroup objects
  for(int i = 0; i < _number_of_motors; i++){
      temp_vector.push_back(_motors[i].read_desired_position());
  }
  return temp_vector;
}
#endif