#include "serialPort/SerialPort.h"
#include <csignal>
#include <iostream>
#include <fstream>

#define KP 0.0f // gain of the proportional controller
#define KI 0.0f // gain of the integral controller
#define KD 0.0f // gain of the derivative controller
#define frequency 500.0f // frequency of sending commands [Hz]
#define run_time 20.0f // run time of the motor [s] 
#define T_limit 70.0f/9.1f  // maximum output torque limit
#define initial_position_difference 3.14*0 // value added to the starting position of the motor to get target position [rad]
#define load "lever only"
#define Temp_cutoff 50.0f // motor temp limit [deg C]
#define desired_velocity 6.28f*9.1 // desired final velocity [rad]
#define stages 100 // number of velocity increase steps

int main(){
    // set the serial port name
    SerialPort serial("/dev/ttyUSB0");

    // send message struct
    MOTOR_send motor_run, motor_stop;
    // receive message struct
    MOTOR_recv motor_r;

    // set the id of motor
    motor_run.id = 1;
    // set the motor type, A1Go1 or B1
    motor_run.motorType = MotorType::A1Go1;
    motor_run.mode = 10;   //0: idle, 5: open loop rotation, 10: closed loop FOC control
    motor_run.T = 0.0;     //forward fetched torque [Nm]
    motor_run.W = 0.0;     //desired angular velocity
    motor_run.Pos = 0.0;   //desired position
    motor_run.K_P = 0.0;   //gain of position error
    motor_run.K_W = 10.0;   //gain of velocity error

    motor_stop.id = motor_run.id;               //set the same ID for run and stop commands
    motor_stop.motorType = motor_run.motorType; //set the same type of the motor for run and stop commands
    motor_stop.mode = 0;                        //set the mode of the stop command to idle

    motor_r.motorType = motor_run.motorType;    //set the same motor type for receive and send structs
    
    //print initial values
    std::cout << "Mode: " << motor_run.mode << std::endl;
    std::cout << "T: " << motor_run.T << std::endl;
    std::cout << "W: " << motor_run.W << std::endl;
    std::cout << "Pos: " << motor_run.Pos/9.1 << std::endl;
    std::cout << "Kp: " << motor_run.K_P << std::endl;
    std::cout << "Kw: " << motor_run.K_W << std::endl;
    std::cout << "******************" << std::endl;

    // encode data into motor commands
    modify_data(&motor_run);
    modify_data(&motor_stop);

    //send dummy message with zeros to get readings from the motor
    serial.sendRecv(&motor_run, &motor_r); 
    extract_data(&motor_r);
    
    
    int i=0;                                                              //set iterator to 0
    int iterations= run_time*frequency;                                   //get the number of iterations for while loop
    int stage_length = (iterations/2)/stages;                             //length of velocity increase step
    float error = 0;                                                      //set the initial error to 0 
    float targetPosition = motor_r.Pos/9.1 + initial_position_difference; //set the target position to be different to initial position by specified value
    float measuredPosition = motor_r.Pos/9.1;                             //set the initial position
    
    //set initial values to all the variables of the PID controller
    float integral = 0;                                                   
    float derivative = 0;
    float prevError = initial_position_difference; //previous error is set to the initial error to avoid extremely high initial value
    float PIDoutput = 0;
    float timeDelta = 1/frequency;                 //time delta is equal to the period of sending messages
    
    
    auto *logData = new float[iterations][11]; //create a new array for storing measured data
    
    while(i<iterations){
    	serial.sendRecv(&motor_run, &motor_r);
    	extract_data(&motor_r);

        if(i%stage_length == 0 && i < iterations/2){
            motor_run.W = motor_run.W + desired_velocity/stages;
            std::cout << "W: " << motor_run.W/9.1 << std::endl;
        }

        //if statement to power down the motor when temp is too high
        if(motor_r.Temp >= Temp_cutoff){
            std::cout << "Warning! Temp: " << motor_r.Temp << std::endl;
            break;
        }

        //PID controller
        measuredPosition = motor_r.Pos/9.1;
        error = targetPosition - measuredPosition;
        integral = integral + error * timeDelta;
        derivative = (error - prevError) / timeDelta;
        prevError = error;
        PIDoutput = KP * error + KI * integral + KD * derivative;

        //motor_run.T = std::min(std::max(PIDoutput, -T_limit), T_limit); // limit the torque output to a safe value
        //std::cout << "T: " << motor_r.T << std::endl;
/*
    	if(i%100==0){
            std::cout << "T: " << motor_r.T << std::endl;
            std::cout << "W: " << motor_r.W << std::endl;
            std::cout << "Pos: " << motor_r.Pos << std::endl;
            std::cout << "LW: " << motor_r.LW << std::endl;
            std::cout << "Temp: " << motor_r.Temp << std::endl;
            std::cout << "PIDoutput: " << PIDoutput << std::endl;
            std::cout << "i: " << i << std::endl;
            std::cout << "timeDelta: " << timeDelta << std::endl;
            std::cout << "derivative: " << derivative << std::endl;
            std::cout << "******************" << std::endl;
	    } 
*/
	    //save the data to the array
        logData[i][0]= i*timeDelta; 
        logData[i][1]= error; 
        logData[i][2]= targetPosition; 
        logData[i][3]= measuredPosition; 
        logData[i][4]= integral;
        logData[i][5]= derivative; 
        logData[i][6]= PIDoutput;
        logData[i][7]= motor_r.Temp;
        logData[i][8]= motor_r.Acc/9.1; // the internal shaft rotates faster than the output shaft, so the acceleration has to be divided by 9.1
        logData[i][9]= motor_r.W/9.1;   // the internal shaft rotates faster than the output shaft, so the velocity has to be divided by 9.1
        logData[i][10]= motor_r.T*9.1;  // the internal shaft rotates faster than the output shaft, so the torque has to be multiplied by 9.1 (the slower the motor output, the bigger the torque)
        i++;
	    sleep(timeDelta); // sleep for a set period
    }
    
    serial.sendRecv(&motor_stop, &motor_r);

    // create an output file stream
    std::ofstream output;
    // use it to open a file named 'logData.csv'
    output.open("logData.csv");
    // check if the file is not open
    if (!output.is_open()) {
        // print error message and quit if a problem occurred
        std::cerr << "Error creating file!\n";
        exit(1);
    }

    //add headers conatining information about conducted test
    output << "KP=" << KP << " KI=" << KI << " KD=" << KD << " f[Hz]=" << frequency << " T_limit[Nm]=" << T_limit << " Initial error[rad]=" << initial_position_difference << " " << load << ", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0" << std::endl;
    output <<"time[s], Position Error[rad], Target Position[rad], Measured Position[rad], Integral, Derivative, PID Output[Nm], Motor Temperature[°C], Acceleration [rad/s^2], Velocity [rad/s], Torque [Nm]" << std::endl;

    // loop for adding all the rows to the csv
    for (int i = 0; i < iterations; i++) {
        // print data in CSV format
        output << logData[i][0] << "," << logData[i][1] << "," << logData[i][2] << "," << logData[i][3] << "," << logData[i][4] << "," << logData[i][5] << "," << logData[i][6] << "," << logData[i][7] << "," << logData[i][8] << "," << logData[i][9] << "," << logData[i][10] << std::endl;
    }
    output.close();

    delete[] logData;
    logData = nullptr;
    return 0;
}
