# Introduction
This library is used for the communication between PC and multiple motor control boards via multiple converters, so that the user can control the motors by PC. This library only includes Linux version, and we offer the usage example of C++. 

# Dependencies
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
  
# Files
## /lib
Including the library files of Linux. Please modify the CMakeList manually to select the correst `.so` file.
## /include
Including the head files. 
## /src
Including the source files of example, changeID tool, code used for motor testing. The example can control motors to run under desired command for desired time, and then stop. The main control file is motor_control.cpp which allows for testing the developed ActuationSystem class.
## /build
Build directory. The final executable files are also in this directory.

# Usage
## Setup
Go to configuration.yml and make sure that appropriate lines are commented out as per description above a list. The content of the file should reflect connected setup, i.e. number of converters and number of motors per converter.

## Build
```bash
mkdir build
cd build
cmake ..
make
```
## Run test code
```bash
cd build
sudo ./test_motor
```
## Run the main control
```bash
cd build
sudo ./motor_control
```
## Change ID
```bash
cd build
sudo ./changeID
```
