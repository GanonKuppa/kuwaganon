#pragma once

#include "communication.h"
#include "stdint.h"
#include "spi.h"
#include "parameterManager.h"
#include "timer.h"
#include "myUtil.h"
#include <deque>
#include <functional>

namespace umouse {

class adis16470{

public:
    int16_t omega_raw[3];
    int32_t omega_raw_32bit[3];
    int16_t acc_raw[3];
    int16_t temp_raw;

    float omega_ref[3];
    float acc_ref[3];

    float omega_c[3];
    float acc_c[3];

    float omega_f[3];
    float acc_f[3];

    uint16_t whoAmI(){
        uint16_t dummy = readReg(0x7200);
        return readReg(0x7200);
    }
    void init(){
        ParameterManager &pm = ParameterManager::getInstance();
        //printfAsync("adis who am i: %x\n",whoAmI());
        for(int i=0;i<3;i++){
            omega_f[i] = 0.0f;
            acc_f[i] = 0.0f;
        }

        omega_ref[2] = pm.gyro2_z_ref;
    }

    static adis16470& getInstance() {
        static adis16470 instance;
        return instance;
    }


    void update(std::function< void(void) > w1,
                std::function< void(void) > w2,
                std::function< void(void) > w3,
                std::function< void(void) > w4){

        ParameterManager &pm = ParameterManager::getInstance();

        uint16_t dummy       = readReg(0x0C00);
        w1();
        uint16_t Z_GYRO_LOW  = readReg(0x0E00);
        w2();
        uint16_t Z_GYRO_OUT  = readReg(0x1200);
        w3();
        uint16_t X_ACCEL_OUT = readReg(0x1600);
        w4();
        uint16_t Y_ACCEL_OUT = readReg(0x0000);


        // GYRO_Z
        omega_raw[2] = Z_GYRO_OUT;
        omega_raw[1] = Z_GYRO_LOW;
        omega_raw_32bit[2] = ((Z_GYRO_OUT << 16) + Z_GYRO_LOW);
        omega_ref[2] = pm.gyro2_z_ref;
        omega_c[2] = (omega_raw_32bit[2] * GYRO32BIT_SCALE) - omega_ref[2];

        float scaler = 0.0f;
        if(omega_c[2] >= 0) scaler= pm.gyro2_scaler_left;
        else scaler = pm.gyro2_scaler_right;

        omega_f[2] = scaler * omega_c[2];

        // ACC Y
        acc_raw[1] = Y_ACCEL_OUT;
        acc_c[1] = Y_ACCEL_OUT * ACC_SCALE;// - acc_ref[1];
        acc_ref[1] = pm.acc2_y_ref;
        acc_f[1] = acc_c[1];

        // ACC X
        acc_raw[0] = X_ACCEL_OUT;
        acc_c[0] = X_ACCEL_OUT * ACC_SCALE;// - acc_ref[1];
        acc_ref[0] = pm.acc2_x_ref;
        acc_f[0] = acc_c[2];


    };




    void calibOmegaOffset(uint32_t ref_num){

        float omega_z_sum = 0.0;
        float ref_z;

        for (uint32_t i = 0; i < ref_num; i++) {
            waitusec(500);
            omega_z_sum += (omega_raw_32bit[2]) * GYRO32BIT_SCALE;
        }

        ref_z = (omega_z_sum / (float) (ref_num));

        printfAsync("=====adis16470=====\n gyro offset %f\n", ref_z);

        ParameterManager &pm = ParameterManager::getInstance();
        pm.write<float>(192, ref_z);

        pm.gyro2_z_ref = ref_z;
    }


private:
    const float CONSTANT_G = 9.80665;
    const float GYRO32BIT_SCALE = - 1/655360.0f;
    const float ACC_SCALE = 0.00125f * CONSTANT_G;

/*
    uint16_t readReg(uint16_t adress){
        useSSLA1RSPI0();
        uint8_t sendBuf[2];
        uint8_t recvBuf[2];

        sendBuf[0] = adress;
        sendBuf[1] = adress;

        communicateNbyteRSPI0(sendBuf, recvBuf, 2);
        waitusec_sub(10);
        // なぜか前回読み取り値が次の受信時に取得される
        communicateNbyteRSPI0(sendBuf, recvBuf, 2);
        useSSLA0RSPI0();

        return (recvBuf[0] << 8)+ (recvBuf[1]);
    };
*/

    uint16_t readReg(uint16_t adress){
        useSSLA1RSPI0();

        uint8_t recvBuf[2];
        uint16_t recv = communicate16bitRSPI0(adress);
        useSSLA0RSPI0();
        return recv;
    };


    void writeReg(uint16_t adress, uint16_t data){
        useSSLA1RSPI0();
        uint8_t sendBuf[2];
        uint8_t recvBuf[2];

        sendBuf[0] = adress;
        sendBuf[1] = data;
        communicateNbyteRSPI0(sendBuf, recvBuf, 2);
    };




    adis16470(void) {

    };

    ~adis16470(void) {    };

};

}
