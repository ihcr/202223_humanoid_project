#ifndef CSVGENERATOR_H
#define CSVGENERATOR_H

#include <iostream>
#include <csignal>
#include <fstream>
#include "yaml-cpp/yaml.h"

//class for the entire actuation system of a humanoid
class CSVGenerator {
  public:
    
    //constructor
    CSVGenerator();

    //public member methods
    void append_data(int iteration, std::vector<float> torque, std::list<float> actual_positions, std::vector<float> desired_position);
    void safe_data();

     
  private:

    //private member methods
    
    //private member variables 
    
    //data extracted from configuration.yml file
    float _torque_limit;          //maximum magnitude of torque that can be send to a motor
    float _refresh_frequency;
    int _number_of_converters;
    YAML::Node _PID_gains_node;

    std::vector<std::vector<float>> data;
};

CSVGenerator::CSVGenerator(){
      //try-catch block to display error message in case of unsuccessful reading of the file
  try {
        YAML::Node configuration = YAML::LoadFile("configuration.yml");              //load configuration.yml file
        _torque_limit = configuration["torque_limit"].as<float>();                   //copy value of torque_limit as float type
        _refresh_frequency = configuration["refresh_frequency"].as<float>();         //copy value of refresh_frequency as float type
        _number_of_converters = configuration["number_of_converters"].as<int>();     //copy value of number_of_converters as int type
        _PID_gains_node = configuration["PID_gains"];                                //copy PID_gains as YAML::Node type
    } catch (const YAML::BadFile& e) {
        std::cerr << "Error loading configuration file: " << e.what() << std::endl;  //display error message in case of unsuccessful reading of the file
    }
}

void CSVGenerator::append_data(int iteration, std::vector<float> torque, std::list<float> actual_positions, std::vector<float> desired_position){
    std::vector<float> tempVector;

    tempVector.push_back(iteration);

    //copy torque values
    for (int i = 0; i < int(torque.size()); i++) {
        tempVector.push_back(torque[i]);
    }

    //copy actual_positions values
    for (int i = 0; i < int(actual_positions.size()); i++) {
        tempVector.push_back(actual_positions.front());
        actual_positions.pop_front();
    }

    //copy desired_positions
    for (int i = 0; i < int(desired_position.size()); i++) {
        tempVector.push_back(desired_position[i]);
    }

    data.push_back(tempVector);

}

void CSVGenerator::safe_data(){
    // create an output file stream
    std::ofstream output;
    // use it to open a file named 'logData.csv'
    float KP = _PID_gains_node[0][0][0].as<float>();
    float KI = _PID_gains_node[0][0][1].as<float>();
    float KD = _PID_gains_node[0][0][2].as<float>();
    output.open("KP=" + std::to_string(KP) + " KI=" + std::to_string(KI) + " KD=" + std::to_string(KD) + " f[Hz]=" + std::to_string(_refresh_frequency) + " T_limit[Nm]=" + std::to_string(_torque_limit) + ".csv");
    // check if the file is not open
    if (!output.is_open()) {
        // print error message and quit if a problem occurred
        std::cerr << "Error creating file!\n";
        exit(1);
    }

    //add headers conatining information about conducted test
    output <<"iteration,";
    for (int i = 0; i < (int(data[0].size())-1)/3; i++){
        output << "torque " + std::to_string(i) + " [Nm],";
    }
    for (int i = 0; i < (int(data[0].size())-1)/3; i++){
        output << "actual position " + std::to_string(i) + " [rad],";
    }
    for (int i = 0; i < (int(data[0].size())-1)/3; i++){
        if(i == ((int(data[0].size())-1)/3) - 1){
            output << "desired position " + std::to_string(i) + " [rad]" << std::endl;
        } else {
            output << "desired position " + std::to_string(i) + " [rad],";
        }
    }    

    for (int i = 0; i < int(data.size()); i++) {
        std::vector<float> element = data[i];
        // do something with the element
        for (int i = 0; i < int(element.size()); i++){
            if(i == (int(element.size()) - 1)){
                output << element[i] << std::endl;
            } else {
                output << element[i] << ",";
            }
        }     
    }
    output.close();
}
#endif