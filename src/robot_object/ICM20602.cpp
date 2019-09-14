/*
 * ICM20602.cpp
 *
 *  Created on: 2018/06/17
 *      Author: ryota
 */

#include <ICM20602.h>
#include <myUtil.h>
#include "spi.h"
#include "parameterManager.h"
#include "communication.h"
#include "timer.h"



namespace umouse {


void ICM20602::writeReg(uint8_t adress, uint8_t data) {
    useSSLA0RSPI0();
    uint8_t sendBuf[2];
    uint8_t recvBuf[2];

    sendBuf[0] = adress;
    sendBuf[1] = data;
    communicateNbyteRSPI0(sendBuf, recvBuf, 2);
}

uint8_t ICM20602::readReg(uint8_t adress) {
    useSSLA0RSPI0();
    uint8_t sendBuf[2];
    uint8_t recvBuf[2];

    sendBuf[0] = READ_FLAG | adress;
    sendBuf[1] = 0x00;
    communicateNbyteRSPI0(sendBuf, recvBuf, 2);
    return recvBuf[1];
}

void ICM20602::init() {
    //writeReg(107, 0x81);
    //waitmsec(10);
/*    writeReg(17, 0xc9);
    waitmsec(10);
    writeReg(26, 0x00);
    waitmsec(10);
    writeReg(27, 0x18);
    waitmsec(10);
    writeReg(28, 0x18);
    waitmsec(10);
    writeReg(29, 0x04);
    waitmsec(10);
    writeReg(107, 0x01);*/
/*
    writeReg(0x6B,0x80);
    waitmsec(10);
    writeReg(0x6B,0x00);
    waitmsec(10);
    writeReg(0x1A,0x00);
    waitmsec(10);
    writeReg(0x1B,0x18);
    waitmsec(10);
*/

    writeReg(0x6B, 0x80);
    waitmsec(10);
    writeReg(0x6B, 0x00);
    waitmsec(10);
    writeReg(0x6C, 0x00);
    waitmsec(10);

    //55 INT Pin / Bypass Enable Configuration
    //writeReg(0x37, 0x02);
    //waitmsec(10);

    //Register 26 – Configuration
    //communicateNbyteRSPI1(0x1A, 0x01);
    //waitmsec(10);

    //lpf gyro
    writeReg(0x1A, 0x02); //
    waitmsec(10);

    //range gyro
    writeReg(0x1B, 0x18); //0b0001 1000
    waitmsec(10);

    //range acc
    writeReg(0x1C, 0x00); //0b0000 0000
    waitmsec(10);

    //lpf acc
    writeReg(0x1D, 0x04); //0b0000 0100
    waitmsec(10);

    useSSLA0RSPI0();
    uint8_t dummy1, dummy2;
    setEnableRSPI0(1);
    dummy1 = communicate8bitRSPI0(READ_FLAG | 0x1c);
    dummy2 = communicate8bitRSPI0(0x00);
    setEnableRSPI0(0);
    //printfAsync("0x1c: %x %x \n\n",dummy1, dummy2);
    waitmsec(10);
    //書き込み時と読み出し時でSPIの通信速度が異なる
    //RSPI0.SPBR = 1;

    //パラメーターマネージャーからゼロ点を呼び出し
    ParameterManager &pm = ParameterManager::getInstance();
    omega_ref[0] = pm.gyro_x_ref;
    omega_ref[1] = pm.gyro_y_ref;
    omega_ref[2] = pm.gyro_z_ref;

    acc_ref[0] = pm.acc_x_ref;
    acc_ref[1] = pm.acc_y_ref;
    acc_ref[2] = pm.acc_z_ref;



}

void ICM20602::update() {
    ParameterManager &pm = ParameterManager::getInstance();

    uint8_t accel_xout_h = readReg(0x3b);
    uint8_t accel_xout_l = readReg(0x3c);

    uint8_t accel_yout_h = readReg(0x3d);
    uint8_t accel_yout_l = readReg(0x3e);

    uint8_t accel_zout_h = readReg(0x3f);
    uint8_t accel_zout_l = readReg(0x40);

    uint8_t temp_out_h = readReg(0x41);
    uint8_t temp_out_l = readReg(0x42);

    uint8_t gyro_xout_h = readReg(0x43);
    uint8_t gyro_xout_l = readReg(0x44);

    uint8_t gyro_yout_h = readReg(0x45);
    uint8_t gyro_yout_l = readReg(0x46);
    uint8_t gyro_zout_h = readReg(0x47);
    uint8_t gyro_zout_l = readReg(0x48);
    acc_raw[0] = concatenate2Byte_int(accel_xout_h, accel_xout_l);
    acc_raw[1] = concatenate2Byte_int(accel_yout_h, accel_yout_l);
    acc_raw[2] = - concatenate2Byte_int(accel_zout_h, accel_zout_l);

    omega_raw[0] = concatenate2Byte_int(gyro_xout_h, gyro_xout_l);
    omega_raw[1] = - concatenate2Byte_int(gyro_yout_h, gyro_yout_l);
    omega_raw[2] = concatenate2Byte_int(gyro_zout_h, gyro_zout_l);

    omega_ref[0] = pm.gyro_x_ref;
    omega_ref[1] = pm.gyro_y_ref;
    omega_ref[2] = pm.gyro_z_ref;

    acc_ref[0] = pm.acc_x_ref;
    acc_ref[1] = pm.acc_y_ref;
    acc_ref[2] = pm.acc_z_ref;

    for (int i = 0; i < 3; i++) {
        omega_c[i] = (omega_raw[i] * 100 - omega_ref[i]) / 100;
        omega_f[i] = omega_c[i] * GYRO_2000dps;
    }


    if(omega_c[2] >= 0) omega_f[2] *= pm.gyro_scaler_left;
    else omega_f[2] *= pm.gyro_scaler_right;


    for (int i = 0; i < 3; i++) {
        acc_c[i] = acc_raw[i] - acc_ref[i];
        acc_f[i] = 0.02 * (acc_c[i] * ACC_2g * G) + 0.98 * acc_f[i];
    }

    temp_raw = concatenate2Byte_int(temp_out_h, temp_out_l);
    temp_f =  temp_raw * T_25degC + RoomTemp_Offset;

    if(ABS(omega_f[0]) < 5.0 && ABS(omega_f[1]) < 5.0 && ABS(omega_f[2]) < 5.0) stop_count ++;
    else stop_count = 0;

    if(acc_f[2]> G * 0.85) upsideDown_count++;
    else upsideDown_count = 0;
}

bool ICM20602::isStop(){
    if( stop_count > 25) return true;
    else return false;
}

bool ICM20602::isUpsideDown(){
    if(upsideDown_count> 50   ) return true;
    else return false;
}

/**
 * ジャイロオフセット設定用関数<br>
 * 静止状態でNUM_REF回のサンプルを取り, そのデータの上位25%と下位25%の値を
 * クイックソートによって取り除く残りの50%で平均値を求めオフセットとする.(メディアンフィルタと平均の組み合わせ)
 *
 */
void ICM20602::calibOmegaOffset(uint32_t ref_num) {
    uint32_t i = 0;


    int16_t omega_x[ref_num];
    int16_t omega_y[ref_num];
    int16_t omega_z[ref_num];

    float omega_x_sum = 0.0;
    float omega_y_sum = 0.0;
    float omega_z_sum = 0.0;

    int16_t ref_x = 0;
    int16_t ref_y = 0;
    int16_t ref_z = 0;

    for (uint32_t i = 0; i < 3; i++) {
        omega_ref[i] = 0;
    }


    for (uint32_t i = 0; i < ref_num; i++) {
        omega_x[i] = omega_raw[0];
        omega_y[i] = omega_raw[1];
        omega_z[i] = omega_raw[2];
        waitusec(500);
        //printfAsync("%d| calibrating... %d, %d, %d\n", i, omega_x[i],
        //        omega_y[i], omega_z[i]);
        //printfAsync("%d\n",i);
    }
    quickSort_int16(omega_x, 0, ref_num - 1);
    quickSort_int16(omega_y, 0, ref_num - 1);
    quickSort_int16(omega_z, 0, ref_num - 1);

    for (i = ref_num / 4; i < (ref_num * 3 / 4 + 1); i++) {
        omega_x_sum += (float) (omega_x[i]);
        omega_y_sum += (float) (omega_y[i]);
        omega_z_sum += (float) (omega_z[i]);
    }

    ref_x = (int16_t) (100.0 * 2.0 * omega_x_sum / (float) ref_num);
    ref_y = (int16_t) (100.0 * 2.0 * omega_y_sum / (float) ref_num);
    ref_z = (int16_t) (100.0 * 2.0 * omega_z_sum / (float) ref_num);

    //writeEEPROMOffsetOmegaInt(&omega_offset_vec[0]);
    printfAsync("=====ICM20602=====\n gyro offset %d, %d, %d\n", ref_x,
            ref_y, ref_z);

    ParameterManager &pm = ParameterManager::getInstance();
    pm.write<int16_t>(150, ref_x);
    pm.write<int16_t>(151, ref_y);
    pm.write<int16_t>(152, ref_z);

    pm.gyro_x_ref = ref_x;
    pm.gyro_y_ref = ref_y;
    pm.gyro_z_ref = ref_z;


}

void ICM20602::calibAccOffset(uint32_t ref_num) {
    uint32_t i = 0;


    int16_t acc_x[ref_num];
    int16_t acc_y[ref_num];
    int16_t acc_z[ref_num];

    float acc_x_sum = 0.0;
    float acc_y_sum = 0.0;
    float acc_z_sum = 0.0;

    for (uint32_t i = 0; i < 3; i++) {
        acc_ref[i] = 0;
    }

    for (uint32_t i = 0; i < ref_num; i++) {
        acc_x[i] = acc_raw[0];
        acc_y[i] = acc_raw[1];
        acc_z[i] = acc_raw[2];
        waitusec(500);
        //printfAsync("%d| calibrating... %d, %d, %d\n", i, acc_x[i],
        //        acc_y[i], acc_z[i]);
        //printfAsync("%d\n",i);
    }

    for (i = 0; i < ref_num; i++) {
        acc_x_sum += (float) (acc_x[i]);
        acc_y_sum += (float) (acc_y[i]);
        acc_z_sum += (float) (acc_z[i]);
    }
    acc_ref[0] = (int16_t) (acc_x_sum / (float) ref_num);
    acc_ref[1] = (int16_t) (acc_y_sum / (float) ref_num);
    acc_ref[2] = (int16_t) (acc_z_sum / (float) ref_num);

    //writeEEPROMOffsetaccInt(&acc_offset_vec[0]);
    printfAsync("=====ICM20602=====\n gyro offset %d, %d, %d\n", acc_ref[0],
            acc_ref[1], acc_ref[2]);

    ParameterManager &pm = ParameterManager::getInstance();
    pm.write<int16_t>(153, acc_ref[0]);
    pm.write<int16_t>(154, acc_ref[1]);
    pm.write<int16_t>(155, 0);
    pm.acc_x_ref = acc_ref[0];
    pm.acc_y_ref = acc_ref[1];
    pm.acc_z_ref = 0;


}


uint8_t ICM20602::whoAmI(void) {
    return readReg(REG_WHOAMI);
}




}
