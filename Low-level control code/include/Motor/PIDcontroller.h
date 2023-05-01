/*
* PID controller class for a position control of a motor through output torque
* Values needed for setup: initial position of the rotor (initPos), frequency of output update (frequency)
* Input: desired position
* Output: magnitude of output torque
*/


#ifndef PIDcontroller_H
#define PIDcontroller_H

class PIDcontroller {
  public:

    //constructors
    PIDcontroller();
    PIDcontroller(float torque_limit, float refresh_frequency, YAML::Node PID_gains);

    //public member methods
    float nextValue(float Pos);                       //calculate next value of generated torque based on the measured position

    //accessors
    float get_targetPosition();
    float get_error();
    float get_PIDoutput();
    float get_desired_pos();

    //mutators
    void set_targetPosition(float new_targetPosition);

  private:
    float _gear_ratio = 9.1;      //internal gear ratio of Unitree A1 motor
    float _error = 0;             //position error
    float _targetPosition = 0;    //target/desired position
    float _measuredPosition = 0;  //rotor's actual position (encoder output/gear ratio)
    float _integral = 0;          //error integral value of integral controller                                         
    float _derivative = 0;        //gradient value of derivative controller
    float _prevError = 0;         //position error from the previous iteration for derivative controller
    float _PIDoutput = 0;         //output torque obtained from the PID controller
    float _timeDelta = 0;         //time delta is equal to the period of sending messages
    float _T_limit = 0;           //maximum absolute torque output of the PID controler
    float _P_gain = 0;
    float _I_gain = 0;
    float _D_gain = 0;
    
};

PIDcontroller::PIDcontroller(){
    
}

//contructor
PIDcontroller::PIDcontroller(float torque_limit, float refresh_frequency, YAML::Node PID_gains) {
    _T_limit = torque_limit/_gear_ratio;
    _timeDelta = 1/refresh_frequency;                //setup time delta needed for derivative and integral controllers
    _P_gain = PID_gains[0].as<float>();
    _I_gain = PID_gains[1].as<float>();
    _D_gain = PID_gains[2].as<float>();
    std::cout << "PIDcontroller.h - constructor" << std::endl;
    std::cout << "_T_limit: " << _T_limit << std::endl;
    std::cout << "_timeDelta: " << _timeDelta << std::endl;
    std::cout << "_P_gain: " << _P_gain << std::endl;
    std::cout << "_I_gain: " << _I_gain << std::endl;
    std::cout << "_D_gain: " << _D_gain << std::endl;
}

//calculate next value of generated torque based on the measured position
float PIDcontroller::nextValue(float Pos) {
    _measuredPosition = Pos/_gear_ratio;                             //Divide input from motor's encoder by the gear ratio to get actual position of the rotor              
    _error = _targetPosition - _measuredPosition;                    //Calculate position error
    _integral = _integral + _error * _timeDelta;                     //Update value of integral
    _derivative = (_error - _prevError) / _timeDelta;                //Calculate gradient of error
    _prevError = _error;                                             //Update error for the next iteration
    _PIDoutput = _P_gain * _error + _I_gain * _integral + _D_gain * _derivative;    //Calculate value of the output torque
    _PIDoutput = std::min(std::max(_PIDoutput, -_T_limit), _T_limit);  //Limit torque output for safety and reduction of heating
/*  std::cout << "PIDcontroller.h - nextValue()" << std::endl;
    std::cout << "_PIDoutput: " << _PIDoutput << std::endl;
    std::cout << "_measuredPosition: " << _measuredPosition << std::endl;
    std::cout << "_targetPosition: " << _targetPosition << std::endl;
    std::cout << "_error: " << _error << std::endl;
    std::cout << "******************" << std::endl;
*/
    return _PIDoutput;
}

//accessors
float PIDcontroller::get_targetPosition() {
    return _targetPosition;
}

float PIDcontroller::get_error() {
    return _error;
}

//mutators
void PIDcontroller::set_targetPosition(float new_targetPosition) {
    _targetPosition = new_targetPosition;
}

float PIDcontroller::get_PIDoutput(){
    return _PIDoutput;
}

float PIDcontroller::get_desired_pos(){
    return _targetPosition;
}
#endif