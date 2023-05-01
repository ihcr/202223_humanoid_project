#include "serialPort/SerialPort.h"
#include <csignal>

#define KP 0.001f
#define KI 0.0005f
#define KD 0.0001f
#define frequency 500.0f // frequency of sending commands [Hz]


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
    motor_run.T = 0.0; //torque that the motor reaches
    motor_run.W = 0.0*9.1;
    motor_run.Pos = 0.0;
    motor_run.K_P = 0.0;
    motor_run.K_W = 0.0; //the higher K_W, the lower the max speed with no load

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
    std::cout << "******************" << std::endl;

    // encode data into motor commands
    modify_data(&motor_run);
    modify_data(&motor_stop);
    
    int i=0; 
    float error = 0;
    float targetPosition = 100;
    float measuredPosition = 0;
    float integral = 0;
    float derivative = 0;
    float prevError = 0;
    float PIDoutput = 0;
    float timeDelta = 1/frequency;
    std::cout << "timeDelta: " << timeDelta << std::endl;

    while(1){
    	serial.sendRecv(&motor_run, &motor_r);
    	extract_data(&motor_r);

        //PID controller
        measuredPosition = motor_r.Pos;
        error = targetPosition - measuredPosition;
        integral = integral + error * timeDelta;
        derivative = (error - prevError) / timeDelta; //timeDelta is 0, why?
        prevError = error;
        PIDoutput = KP * error + KI * integral + KD * derivative;

        motor_run.T = PIDoutput;

    	if(i%100==0){
            std::cout << "T: " << motor_r.T << std::endl;
            std::cout << "W: " << motor_r.W << std::endl;
            std::cout << "Pos: " << motor_r.Pos << std::endl;
            std::cout << "LW: " << motor_r.LW << std::endl;
            std::cout << "Temp: " << motor_r.Temp << std::endl;
            std::cout << "PIDoutput: " << PIDoutput << std::endl;
            //std::cout << "error: " << error << std::endl;
            //std::cout << "timeDelta: " << timeDelta << std::endl;
            //std::cout << "derivative: " << derivative << std::endl;
            //std::cout << "error - prevError: " << error - prevError << std::endl;
            std::cout << "******************" << std::endl;
	}
	i++;
	sleep(timeDelta); // sets the frequency 
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
