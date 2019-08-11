#pragma once

#include "communication.h"
#include "stdint.h"
#include "spi.h"
#include "parameterManager.h"
#include "timer.h"
#include "myUtil.h"
#include <deque>

namespace umouse {

class adis16470{

public:
    const uint8_t BUFF_SIZE = 10;
    const float CONSTANT_G = 9.80665;
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

    uint8_t whoAmI(){
        return readReg(0x72);
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

    void update(){
        ParameterManager &pm = ParameterManager::getInstance();
        uint16_t Z_GYRO_OUT = readReg(0x0C);
        waitusec_sub(10);
        uint16_t Z_GYRO_LOW = readReg(0x0E);
        omega_raw[2] = Z_GYRO_OUT;
        omega_raw[1] = Z_GYRO_LOW;
        omega_raw_32bit[2] = ((Z_GYRO_OUT << 16) + Z_GYRO_LOW);

        omega_ref[2] = pm.gyro2_z_ref;

        omega_c[2] = (- (omega_raw_32bit[2]) / 655360.0 ) - pm.gyro2_z_ref;

        float scaler = 0.0f;
        if(omega_c[2] >= 0) scaler= pm.gyro2_scaler_left;
        else scaler = pm.gyro2_scaler_right;


        //omega_f[2] =  0.5 * (scaler * 0.1f * 0.01f * omega_c[2]) + 0.5 * omega_f[2];
        omega_f[2] = scaler * omega_c[2];



    };


    void calibOmegaOffset(uint32_t ref_num){

        float omega_z_sum = 0.0;
        float ref_z;

        for (uint32_t i = 0; i < ref_num; i++) {
            waitusec(500);
            omega_z_sum += -(omega_raw_32bit[2]) / 655360.0f;
        }

        ref_z = (omega_z_sum / (float) (ref_num));

        printfAsync("=====adis16470=====\n gyro offset %f\n", ref_z);

        ParameterManager &pm = ParameterManager::getInstance();
        pm.write<float>(192, ref_z);

        pm.gyro2_z_ref = ref_z;
    }


private:

    uint16_t readReg(uint16_t adress){
        useSSLA1RSPI0();
        uint8_t sendBuf[2];
        uint8_t recvBuf[2];

        sendBuf[0] = adress;
        sendBuf[1] = adress + 1;
        //sendBuf[2] = 0;
        communicateNbyteRSPI0(sendBuf, recvBuf, 2);
        useSSLA0RSPI0();

        return (recvBuf[0] << 8)+ (recvBuf[1]);
    };

    uint64_t readGyro32bit(){
        useSSLA1RSPI0();
        uint8_t sendBuf[6];
        uint8_t recvBuf[6];

        sendBuf[0] = 0x0C;
        sendBuf[1] = 0x0D;
        sendBuf[2] = 0x0E;
        sendBuf[3] = 0x0F;
        sendBuf[4] = 0x0E;
        sendBuf[5] = 0x0F;
        communicateNbyteRSPI0(sendBuf, recvBuf, 6);
        useSSLA0RSPI0();
        return ((recvBuf[0]) + (recvBuf[1] << 8) + (recvBuf[2] << 16) + (recvBuf[3] << 24)+ (recvBuf[4] << 32) + (recvBuf[5] << 40));

    }


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
