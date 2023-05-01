#ifndef UNITREEMOTOR_H
#define UNITREEMOTOR_H

#include "unitreeMotor/include/motor_msg.h"  // Motor communication protocol
#include "CRC/crc32.h"      // CRC32 check algorithm
#include <stdint.h>
#include <iostream>

enum class MotorType{
    A1Go1,      // 4.8M baudrate, K_W x1024
    B1          // 6.0M baudrate, K_W x512
};

struct MOTOR_send{
	// Definition Send formatted data
    MasterComdDataV3  motor_send_data;  //Motor control data structure, see motor_msg.h for details
    MotorType motorType = MotorType::A1Go1;
	int hex_len = 34;                    //The length of the sent hexadecimal command array, 34
    // long long send_time;            //发送该命令的时间, 微秒(us)
    // 待发送的各项数据
    unsigned short id;              //Motor ID, 0xBB represents all motors
    unsigned short mode;            //0: idle, 5: open loop rotation, 10: closed loop FOC control
    //实际给FOC的指令力矩为：
    //K_P*delta_Pos + K_W*delta_W + T
    float T;                        //期望关节的输出力矩（电机本身的力矩）（Nm）
    float W;                        //期望关节速度（电机本身的速度）(rad/s)
    float Pos;                      //desired joint position（rad）
    float K_P;                      //关节刚度系数
    float K_W;                      //关节速度系数
    COMData32 Res;                  // 通讯 保留字节  用于实现别的一些通讯内容
};

struct MOTOR_recv{
    // Definition Receive data
    ServoComdDataV3 motor_recv_data;     //Motor receiving data structure, see motor_msg.h for details
    MotorType motorType = MotorType::A1Go1;
    int hex_len = 78;                    //Received hexadecimal command array length, 78
    // long long resv_time;            //The time to receive the command, in microseconds (us)
    bool correct = false;                   //Whether the received data is complete (true is complete, false is incomplete)
    //Interpreting the resulting motor data
    unsigned char motor_id;         //Motor ID
    unsigned char mode;             //0: idle, 5: open loop rotation, 10: closed loop FOC control
    int Temp;                       //temperature
    unsigned char MError;           //error code

    float T;                        // Current actual motor output torque
    float W;                        // Current actual motor speed (high speed)
    float LW;                       // Current actual motor speed (low speed)
    int Acc;                      // Motor rotor acceleration
    float Pos;                      // Current motor position (main control 0 point correction, motor joint is still based on encoder 0 point)

    float gyro[3];                  // Motor driver board 6-axis sensor data
    float acc[3];
};

inline void modify_data(MOTOR_send* motor_s){
    motor_s->hex_len = 34;
    motor_s->motor_send_data.head.start[0] = 0xFE;
    motor_s->motor_send_data.head.start[1] = 0xEE;
    motor_s->motor_send_data.head.motorID = motor_s->id;
    motor_s->motor_send_data.head.reserved = 0x0;
    motor_s->motor_send_data.Mdata.mode = motor_s->mode;
    motor_s->motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_s->motor_send_data.Mdata.ReadBit = 0x0;
    motor_s->motor_send_data.Mdata.reserved = 0x0;
    motor_s->motor_send_data.Mdata.Modify.L = 0;
    motor_s->motor_send_data.Mdata.T = motor_s->T*256;
    motor_s->motor_send_data.Mdata.W = motor_s->W*128;
    motor_s->motor_send_data.Mdata.Pos = (int)((motor_s->Pos/6.2832)*16384.0);
    motor_s->motor_send_data.Mdata.K_P = motor_s->K_P*2048;
    
    if(motor_s->motorType == MotorType::A1Go1){
        motor_s->motor_send_data.Mdata.K_W = motor_s->K_W*1024;
    }
    else if(motor_s->motorType == MotorType::B1){
        motor_s->motor_send_data.Mdata.K_W = motor_s->K_W*512;       
    }
    
    motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    motor_s->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    motor_s->motor_send_data.Mdata.Res[0] = motor_s->Res;
    motor_s->motor_send_data.CRCdata.u32 = crc32_core((uint32_t*)(&(motor_s->motor_send_data)), 7);
}

inline bool extract_data(MOTOR_recv* motor_r){
    if(motor_r->motor_recv_data.CRCdata.u32 !=
        crc32_core((uint32_t*)(&(motor_r->motor_recv_data)), 18)){
        std::cout << "[WARNING] Receive data CRC error" << std::endl;
        motor_r->correct = false;
        return motor_r->correct;
    }else{
        motor_r->motor_id = motor_r->motor_recv_data.head.motorID;
        motor_r->mode = motor_r->motor_recv_data.Mdata.mode;
        motor_r->Temp = motor_r->motor_recv_data.Mdata.Temp;
        motor_r->MError = motor_r->motor_recv_data.Mdata.MError;
        motor_r->T = ((float)motor_r->motor_recv_data.Mdata.T) / 256;
        motor_r->W = ((float)motor_r->motor_recv_data.Mdata.W) / 128;
        motor_r->LW = motor_r->motor_recv_data.Mdata.LW;

        motor_r->Acc = (int)motor_r->motor_recv_data.Mdata.Acc;
        motor_r->Pos = 6.2832*((float)motor_r->motor_recv_data.Mdata.Pos) / 16384;
        
        motor_r->gyro[0] = ((float)motor_r->motor_recv_data.Mdata.gyro[0]) * 0.00107993176;
        motor_r->gyro[1] = ((float)motor_r->motor_recv_data.Mdata.gyro[1]) * 0.00107993176;
        motor_r->gyro[2] = ((float)motor_r->motor_recv_data.Mdata.gyro[2]) * 0.00107993176;
        
        motor_r->acc[0] = ((float)motor_r->motor_recv_data.Mdata.acc[0]) * 0.0023911132;
        motor_r->acc[1] = ((float)motor_r->motor_recv_data.Mdata.acc[1]) * 0.0023911132;
        motor_r->acc[2] = ((float)motor_r->motor_recv_data.Mdata.acc[2]) * 0.0023911132;

        motor_r->correct = true;
        return motor_r->correct;
    }
}

#endif  // UNITREEMOTOR_H