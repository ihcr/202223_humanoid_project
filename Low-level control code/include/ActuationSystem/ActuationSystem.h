#ifndef ACTUATIONSYSTEM_H
#define ACTUATIONSYSTEM_H

//maximum number of converters needed for the entire robot 
//corresponds to total number of rows (both commented out and not) in joint names in configuration.yml
#define length_of_MotorGroup_array 9 

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include "yaml-cpp/yaml.h"
#include "unitreeMotor/unitreeMotor.h"
#include "MotorGroup/MotorGroup.h"

//class for the entire actuation system of a humanoid
class ActuationSystem {
  public:
    
    //constructor
    ActuationSystem();

    //destructor
    ~ActuationSystem(); //an explicit destructor is needed to free up memory occupied by _motor_groups array

    //public member methods
    void  send_desired_positions(std::list<float> desired_positions); //function sending a list of desired positions to all connected motors
    std::list<float> read_actual_positions();                         //function returning a list of actual positions of all connected motors 
    void  update_torque();                                            //function to loop through all MotorGroup objects to calculate new value of torque
    void  update_desired_position_from_terminal();                    //function to loop through all MotorGroup objects to update the desired position from terminal (used for testing)
    void  stop_motors();                                              //function to loop through all MotorGroup objects to send a stop message, i.e., mode=0, so that all motors go to idel state
    std::vector<float> read_torque();
    std::vector<float> read_desired_position();

  private:

    //private member methods
    void read_configuration_file(); //function to extract configuratoin data from configuration.yml file

    //private member variables 
    
    //data extracted from configuration.yml file
    float _torque_limit;          //maximum magnitude of torque that can be send to a motor
    float _temp_limit;            //temperature threshold of a motor, once exceeded the motors should shutdown 
    float _refresh_frequency;     //frequency at which arrays of new desired positions will be sent
    int   _number_of_converters;  //number of connected USB-RS485 converters
    YAML::Node _joint_names_node; //names related to positions of all connected motors
    YAML::Node _PID_gains_node;   //values of P, I and D gains for every connected motor 

    
    MotorGroup* _motor_groups[length_of_MotorGroup_array]; //fixed length array of pointers to MotorGroup objects
};

ActuationSystem::ActuationSystem() {

    std::cout << "ActuationSystem.h - constructor" << std::endl; //display a message that the function was entered
    
    read_configuration_file(); //get data from the configuration file
    
    //a loop to instantiate specified number of MotorGroup objects
    for(int i = 0; i < _number_of_converters; i++){
      //try-catch block to display error message in case of unsuccessful instantiation of a MotorGroup object
      try{
        std::string port_name = "/dev/ttyUSB" + std::to_string(i); //create a name for a SerialPort object controlling a converter
        
        //a loop to populate IDs list with as many subsequent ID numbers as there is motors in a given MotorGroup object, 
        //i.e., names in a row of _joint_names_node
        std::list<int> IDs;                                        //create an empty list of motor IDs
        for (int j = 0; j < int(_joint_names_node[i].size()); j++){
          IDs.push_back(j); //add subsequent ID to the list
        }
        //IDs.push_back(i+1);

        //create a pointer to an instantiated MotorGroup object 
        //IMPORTANT: "new" keyword makes sure that the memory occupied by the pointer will not be freed up at any point of the code 
        //until explicit calling of the destructor. Without this keyword the code does not work.
        MotorGroup* _motor_group = new MotorGroup(_torque_limit, _temp_limit, _refresh_frequency, port_name, IDs, _PID_gains_node[i]);

        _motor_groups[i] = _motor_group; //add the pointer to instantiated MotorGroup object to the array with other objects of this class
        std::cout << "_motor_groups[" << i << "] assigned a pointer:" << _motor_groups[i] << std::endl; //display value of the MotorGroup object pointer
      } catch (const IOException& e) {
        std::cerr << "Error opening serial port: " << e.what() << std::endl;  //display error message in case of unsuccessful reading of the file
      }
    }
    std::cout << "ActuationSystem.h - constructor END" << std::endl; //display a message that the function ends here
    std::cout << "********************************" << std::endl;    //display a breaker at the end for better readability
}

ActuationSystem::~ActuationSystem(){
  for(int i = 0; i < _number_of_converters; i++){
    delete _motor_groups[i]; // Free memory allocated for array of MyObject
  }
}

//function to extract configuratoin data from configuration.yml file
void ActuationSystem::read_configuration_file(){
  
  std::cout << "ActuationSystem.h - read_configuration_file" << std::endl; //display a message that the function was entered
  
  //try-catch block to display error message in case of unsuccessful reading of the file
  try {
        YAML::Node configuration = YAML::LoadFile("configuration.yml");              //load configuration.yml file
        
        _torque_limit = configuration["torque_limit"].as<float>();                   //copy value of torque_limit as float type
        std::cout << "torque_limit: " << _torque_limit << std::endl;                 //display obtained value 
        
        _temp_limit = configuration["temp_limit"].as<float>();                       //copy value of temp_limit as float type
        std::cout << "temp_limit: " << _temp_limit << std::endl;                     //display obtained value
        
        _refresh_frequency = configuration["refresh_frequency"].as<float>();         //copy value of refresh_frequency as float type
        std::cout << "refresh_frequency: " << _refresh_frequency << std::endl;       //display obtained value
        
        _number_of_converters = configuration["number_of_converters"].as<int>();     //copy value of number_of_converters as int type
        std::cout << "number_of_converters: " << _number_of_converters << std::endl; //display obtained value
        
        _joint_names_node = configuration["joint_names"];                            //copy joint_names as YAML::Node type
        std::cout << "joint_names_list: " << _joint_names_node << std::endl;         //display obtained value

        _PID_gains_node = configuration["PID_gains"];                                //copy PID_gains as YAML::Node type
        std::cout << "PID_gains_list: " << _PID_gains_node << std::endl;             //display obtained value
        std::cout << "********************************" << std::endl;                //display a breaker at the end for better readability 
    } catch (const YAML::BadFile& e) {
        std::cerr << "Error loading configuration file: " << e.what() << std::endl;  //display error message in case of unsuccessful reading of the file
    }
}

//function sending a list of desired positions to all connected motors
void ActuationSystem::send_desired_positions(std::list<float> desired_positions) {
  int number_of_motors = 0;                       //variable needed to store number of motors in every converter
  std::vector<float> desired_positions_v;         //vector to store desired position for a MotorGroup. Vector was chosen because list can be easily copied to it, it can be passed to a function and has random access operator []
  
  //iterate through all MotorGroup objects
  for(int i = 0; i < _number_of_converters; i++){
    number_of_motors = _motor_groups[i]->get_number_of_motors();   //get number of motors in a MotorGroup
    desired_positions_v.assign(desired_positions.begin(), std::next(desired_positions.begin(), number_of_motors)); //copy as many firt elements from the list as there is motors in a MotorGroup
    _motor_groups[i]->send_desired_positions(desired_positions_v); //pass vector with copied values to a MotorGroup
    
    //for loop to delete positions of motors that were already passed
    for (int i = 0; i < number_of_motors; i++){
      desired_positions.pop_front(); //delete first element of the list
    }
  }
}

//function returning a list of actual positions of all connected motors 
//for one motor connected to converter 0 and three motors connected to converter 1
//the list has following format: [USB0 motor 0, USB1 motor 0, USB1 motor 1, USB1 motor 2]
std::list<float> ActuationSystem::read_actual_positions(){
  std::list<float> actual_positions;      //a list to be populated with obtained positions of all motors
  std::list<float> actual_positions_temp; //a list for temporary storage of a list received from every MotorGroup
  
  //iterate through all MotorGroup objects
  for(int i = 0; i < _number_of_converters; i++){
        actual_positions_temp = _motor_groups[i]->read_actual_positions();      //get list of actual positions from all motors in a given MotorGroup object
        actual_positions.splice(actual_positions.end(), actual_positions_temp); //concatenate newly obtained list of positions with list accumulating values from all MotorGroup objects
  }
  return actual_positions;
}

//function to loop through all MotorGroup objects to calculate new value of torque
void ActuationSystem::update_torque(){

  std::vector<std::thread> threads; //vector to store threads

  //loop to put torque update function threads into the vector
  for (int i = 0; i < _number_of_converters; i++) {
      threads.emplace_back(std::thread(&MotorGroup::update_torque, _motor_groups[i]));
  }

  //loop to merge the threads
  for (auto& t : threads) {
      t.join();
  }

}
//function to loop through all MotorGroup objects to update the desired position from terminal (used for testing)
void ActuationSystem::update_desired_position_from_terminal(){
    //iterate through all MotorGroup objects
    for(int i = 0; i < _number_of_converters; i++){
        _motor_groups[i]->update_desired_position(); //update desired position of all MotorGroup objects
    }
}
//function to loop through all MotorGroup objects to send a stop message, i.e., mode=0, so that all motors go to idel state
void ActuationSystem::stop_motors(){
    //iterate through all MotorGroup objects
    for(int i = 0; i < _number_of_converters; i++){
        _motor_groups[i]->stop_motors();  //send a STOP message to every MotorGroup object
    }
}

std::vector<float> ActuationSystem::read_torque(){
  std::vector<float> temp_vector1;
  std::vector<float> temp_vector2;

  //iterate through all MotorGroup objects
  for(int i = 0; i < _number_of_converters; i++){
    temp_vector1 = _motor_groups[i]->read_torque();
    temp_vector2.insert(temp_vector2.end(), temp_vector1.begin(), temp_vector1.end());
  }
  return temp_vector2;
}

std::vector<float> ActuationSystem::read_desired_position(){
 std::vector<float> temp_vector1;
 std::vector<float> temp_vector2;

  //iterate through all MotorGroup objects
  for(int i = 0; i < _number_of_converters; i++){
    temp_vector1 = _motor_groups[i]->read_desired_position();
    temp_vector2.insert(temp_vector2.end(), temp_vector1.begin(), temp_vector1.end());
  }
  return temp_vector2;
}

#endif




