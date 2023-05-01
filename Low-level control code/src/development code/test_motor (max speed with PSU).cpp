#include "serialPort/SerialPort.h"
#include <csignal>

int main(){
    // set the serial port name
    SerialPort serial("/dev/ttyUSB0");

    // send message struct
    MOTOR_send motor_run, motor_stop;
    // receive message struct
    MOTOR_recv motor_r;

    // set the id of motor
    motor_run.id = 0;
    // set the motor type, A1Go1 or B1
    motor_run.motorType = MotorType::A1Go1;
    motor_run.mode = 10;
    motor_run.T = 0.0;
    motor_run.W = 7.0*9.1; //7rad/s
    motor_run.Pos = 0.0*9.1;  //must be 0
    motor_run.K_P = 0.0;
    motor_run.K_W = 3.0;

    motor_stop.id = motor_run.id;
    motor_stop.motorType = motor_run.motorType;
    motor_stop.mode = 0;

    motor_r.motorType = motor_run.motorType;
    
    std::cout << "Mode: " << motor_run.mode << std::endl;
    std::cout << "T: " << motor_run.T << std::endl;
    std::cout << "W: " << motor_run.W << std::endl;
    std::cout << "Pos: " << motor_run.Pos << std::endl;
    std::cout << "Kp: " << motor_run.K_P << std::endl;
    std::cout << "Kw: " << motor_run.K_W << std::endl;

    // encode data into motor commands
    modify_data(&motor_run);
    modify_data(&motor_stop);
    
    int i=0; 
    while(i<5){
    	serial.sendRecv(&motor_run, &motor_r);
    	extract_data(&motor_r);
    	std::cout << "Pos: " << motor_r.Pos << std::endl;
	std::cout << "W: " << motor_r.W << std::endl;
	std::cout << "LW: " << motor_r.LW << std::endl;
	std::cout << "T: " << motor_r.T << std::endl;
	i++;
	usleep(1000000);
    }
    
    serial.sendRecv(&motor_stop, &motor_r);

    // turn for 3 second
/*    float targetposition = 15.0;
    float range = 0.2;
	while(1){
		serial.sendRecv(&motor_run, &motor_r);
		// decode data from motor states
		extract_data(&motor_r);
		std::cout << "Pos: " << motor_r.Pos << std::endl;
		std::cout << "W: " << motor_r.W << std::endl;
		std::cout << "LW: " << motor_r.LW << std::endl;
		std::cout << "T: " << motor_r.T << std::endl;
		usleep(100000);
		if(motor_r.Pos > targetposition-range && motor_r.Pos < targetposition+range){
			break;
			}
    }

    // stop the motor

    while(!serial.sendRecv(&motor_stop, &motor_r)){
	usleep(100000);
    }
*/
    return 0;
}
