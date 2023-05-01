#include <csignal>
#include <iostream>
#include <fstream>
#include <string>
#include <list> 
#include <chrono>
#include "serialPort/SerialPort.h"
#include "Motor/Motor.h"
#include "Motor/PIDcontroller.h"
#include "MotorGroup/MotorGroup.h"
#include "ActuationSystem/ActuationSystem.h"
#include "CSVGenerator/CSVGenerator.h"

#define update_frequency 1000.0f // frequency of sending commands [Hz]

//function to get inputs from terminal and return them in a list format
std::list<float> get_desired_positions_list(int number_of_motors) {
  
  std::string obtained_string;            //string to store raw input from terminal
  float new_desired_position = 0;         //variable to store converted value from terminal
  std::list<float> new_desired_positions; //list to store desired positions
  
  //for loop to get an input from terminal for every motor
  for(int i = 0; i < number_of_motors; i++) {
    std::cout << "\nEnter desired position: ";
    std::getline(std::cin, obtained_string);
    new_desired_position = std::stof(obtained_string);  // convert to float
    new_desired_positions.push_back(new_desired_position);
  }
  return new_desired_positions;
};

//simple function to stop the code
bool ask_whether_continue() {
  std::string obtained_string;
  std::cout << "\nWant to continue? y or n?";
  std::getline(std::cin, obtained_string);
  if(obtained_string == "y"){
    return true;
  }else{
    return false;
  }
};

int main(){

  //variables needed for the main function
  bool execution_flag = false;             //flag for if block terminating the code
  int i = 0;                               //iterator for the inner do while loop
  std::list<float> desired_positions_list; //list to store input from the terminal
  std::list<float> actual_positions;       //list to store positions read from the motors
  int number_of_motors = 0;

  //CSVGenerator csvgenerator;

  //constructor with execution time measurement 
  auto start = std::chrono::high_resolution_clock::now();
  ActuationSystem actuationSystem;
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "Execution time of ActuationSystem constructor: " << duration.count() << " microseconds" << std::endl;
  
  //option to stop execution in this moment
  if (!(ask_whether_continue())){
    actuationSystem.stop_motors();
    std::cout << "end of run" << std::endl;
    return 0;
  } 
  
  //the main loop to run the motor and update desired positions 
  do {
    std::cout << "Entered the first loop"<< std::endl;
    std::cout << "******************" << std::endl;

    //option to stop execution in this moment
    if (!(ask_whether_continue())){
      actuationSystem.stop_motors();
      std::cout << "end of run" << std::endl;
      return 0;
    } 
    
    auto start2 = std::chrono::high_resolution_clock::now();
    //inner loop for updating output torque, running for a fixed number of iterations 
    do{
      //std::cout << "Entered the second loop"<< std::endl;
      //std::cout << "******************" << std::endl;
      
      //try-catch block for upadting output torque
      try{

        //output torque update function with execution time measurement 
        //start = std::chrono::high_resolution_clock::now();
        actuationSystem.update_torque(); //update torque output for all connected motors
        //end = std::chrono::high_resolution_clock::now();
        //duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        //std::cout << "Execution time of update_torque(): " << duration.count() << " microseconds. Max f: " << (1.0f/float(duration.count()))*1000000 << std::endl;

        //csvgenerator.append_data(i, actuationSystem.read_torque(), actuationSystem.read_actual_positions(), actuationSystem.read_desired_position());

      } catch (const IOException& e) {
        std::cerr << "Error opening serial port: " << e.what() << std::endl;
      }
      i++;                       // increase iterator
      sleep(1/update_frequency); // sleep for a set period
    }while(i<2000);
    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
    std::cout << "Execution time of 2000 iterations:: " << duration2.count() << " milliseconds." << std::endl;
    i=0; // reset the iterator for the next entry into the inner loop
    

    
    //read actual positions of connected motors with execution time measurement
    start = std::chrono::high_resolution_clock::now();
    actual_positions = actuationSystem.read_actual_positions(); //read actual positions of connected motors
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Execution time of read_actual_positions(): " << duration.count() << " microseconds. Max f: " << (1.0f/float(duration.count()))*1000000 << std::endl;
    
    number_of_motors = int(actual_positions.size());

    //print read positions to terminal
    std::cout << "actual_positions: ";
    for (const auto& element : actual_positions) {
        std::cout << element*(180/3.14159265359) << " ";
    }
    std::cout << std::endl;

    execution_flag = ask_whether_continue(); //option to terminate execution of the code
    
    //update desired positions of all connected motors if execution_flag is true
    if(execution_flag == true){
      std::list<float> desired_positions = get_desired_positions_list(number_of_motors); //get new desired positions from terminal in a list format

      for (auto it = desired_positions.begin(); it != desired_positions.end(); ++it) {
          *it = *it*(3.14159265359/180);
      }

      std::cout << "converted desired_positions: ";
      for (const auto& element : desired_positions) {
          std::cout << element << " ";
      }
      std::cout << std::endl;
      
      //send the new desired positions to all connected motors with execution time measurement
      start = std::chrono::high_resolution_clock::now();
      actuationSystem.send_desired_positions(desired_positions); //update desired positions of all connected motors
      end = std::chrono::high_resolution_clock::now();
      duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      std::cout << "Execution time of send_desired_positions(): " << duration.count() << " microseconds. Max f: " << (1.0f/float(duration.count()))*1000000 << std::endl;
    }

  }while(execution_flag);

  actuationSystem.stop_motors(); //set motors to idle mode
  //csvgenerator.safe_data();
  std::cout << "end of run" << std::endl;
  return 0;
}