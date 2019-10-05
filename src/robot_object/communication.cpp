/*
 * communication.cpp
 *
 *  Created on: 2017/08/13
 *      Author: ryota
 */



#include <ICM20602.h>
#include "adis16470.h"
#include <mouse.h>
#include <myUtil.h>
#include "communication.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <wallsensor.h>
#include <wheelOdometry.h>

#include "uart.h"
#include "timeInterrupt.h"
#include "timer.h"
#include "ad.h"
#include "phaseCounting.h"
#include "pwm.h"
#include "da.h"

#include "sound.h"
#include "fcled.h"
#include "gamepad.h"
#include "maze.h"
#include "parameterManager.h"
#include "powerTransmission.h"


#include <queue>






static void set2ByteVal(uint8_t *buf, uint16_t index, float val,float prop);
static void set2ByteVal(uint8_t *buf, uint16_t index, uint16_t val);
static void set2ByteVal(uint8_t *buf, uint16_t index, int16_t val);
static void set2ByteVal(uint8_t *buf, uint16_t index, uint32_t val);
static void set2ByteVal(uint8_t *buf, uint16_t index, int32_t val);
static void set4ByteVal(uint8_t *buf, uint16_t index, uint32_t val);


namespace umouse{

static const uint16_t PERIODIC_MSG_LEN = 400;
static const uint16_t PARAM_MNG_PART_NUM = 40;
static const uint16_t CMD_SIZE  = 16;
static std::queue<uint8_t> printfBuff;
static uint8_t periodicMsg[PERIODIC_MSG_LEN];

static void packDataMaze(uint8_t part_num, uint8_t *buf);
static void packData(uint8_t *buf);
static void packPidPrmAdjData(uint8_t *buf);
static void packWallSenAdjData(uint8_t *buf);
static void packVelocityAdjData(uint8_t *buf);

/***********同期printf関数******************/
int printfSync(const char *fmt, ...) {
    static char buffer[1000];
    int len;

    va_list ap;
    va_start(ap, fmt);

    len = vsprintf(buffer, fmt, ap);
    putnbyteSCI1(buffer, len);
    va_end(ap);
    return len;
}


/***********非同期printf関数******************/
int printfAsync(const char *fmt, ...) {
    static char buffer[1000];
    int len;

    va_list ap;
    va_start(ap, fmt);

    len = vsprintf(buffer, fmt, ap);

    for (int c = 0; c < len; c++) {
        printfBuff.push(buffer[c]);
    }

    va_end(ap);
    return len;
}


void fetchCommand() {
    static bool first_recieve_flag = false;
    int16_t last_cmd_index = -1;
    for(int i=0;i<recieveBuffCount-4;i++){
        if(recieveBuffCount-i > CMD_SIZE &&
            recieveBuff[i+0]==99 &&
            recieveBuff[i+1]==109 &&
            recieveBuff[i+2]==100
        ){

            if(first_recieve_flag == false){
                SEE();
                first_recieve_flag = true;
            }

            if(recieveBuff[i+3] == 254 && recieveBuff[i+4] == 253){
                Gamepad &gamepad = Gamepad::getInstance();
                gamepad.updateCommand(&recieveBuff[i]);
            }

            if(recieveBuff[i+3] == 251){
                ParameterManager &pm = ParameterManager::getInstance();

                pm.writeCommand(&recieveBuff[i]);
            }

            last_cmd_index = i;
            break;
        }
    }

    if (last_cmd_index != -1){
        for(int ind = last_cmd_index + CMD_SIZE;
                ind < recieveBuffCount;
                ind++){
            recieveBuff[ind -(last_cmd_index + CMD_SIZE)] = recieveBuff[ind];
        }
        recieveBuffCount = recieveBuffCount - (last_cmd_index + CMD_SIZE);
    }
}

/***********periodicMsgを送る******************/
void sendPeriodicMsg() {
    ParameterManager &pm = ParameterManager::getInstance();

    if(pm.send_data_mode == 0) packData(periodicMsg);
    else if(pm.send_data_mode == 1) packPidPrmAdjData(periodicMsg);
    else if(pm.send_data_mode == 2) packWallSenAdjData(periodicMsg);
    else if(pm.send_data_mode == 3) packVelocityAdjData(periodicMsg);
    else packData(periodicMsg);
    putnbyteSCIFA9(periodicMsg, PERIODIC_MSG_LEN);
}

//迷路の壁情報を送る
//32 x 32の迷路データを4つに分割
void packDataMaze(uint8_t part_num, uint8_t *buf) {
    buf[0] = part_num;
    UMouse &mouse = UMouse::getInstance();
    uint8_t ind = 1;

    if (part_num == 0) {
        for (int i = 0; i < 16; i++) {
            buf[ind + 0 + i * 4] = ((mouse.maze.walls_vertical[i] & 0x000000ff)
                    >> 0);
            buf[ind + 1 + i * 4] = ((mouse.maze.walls_vertical[i] & 0x0000ff00)
                    >> 8);
            buf[ind + 2 + i * 4] = ((mouse.maze.walls_vertical[i] & 0x00ff0000)
                    >> 16);
            buf[ind + 3 + i * 4] = ((mouse.maze.walls_vertical[i] & 0xff000000)
                    >> 24);
        }
    }

    if (part_num == 1) {
        for (int i = 0; i < 15; i++) {
            buf[ind + 0 + i * 4] = ((mouse.maze.walls_vertical[i + 16]
                    & 0x000000ff) >> 0);
            buf[ind + 1 + i * 4] = ((mouse.maze.walls_vertical[i + 16]
                    & 0x0000ff00) >> 8);
            buf[ind + 2 + i * 4] = ((mouse.maze.walls_vertical[i + 16]
                    & 0x00ff0000) >> 16);
            buf[ind + 3 + i * 4] = ((mouse.maze.walls_vertical[i + 16]
                    & 0xff000000) >> 24);
        }
    }

    if (part_num == 2) {
        for (int i = 0; i < 16; i++) {
            buf[ind + 0 + i * 4] =
                    ((mouse.maze.walls_horizontal[i] & 0x000000ff) >> 0);
            buf[ind + 1 + i * 4] =
                    ((mouse.maze.walls_horizontal[i] & 0x0000ff00) >> 8);
            buf[ind + 2 + i * 4] =
                    ((mouse.maze.walls_horizontal[i] & 0x00ff0000) >> 16);
            buf[ind + 3 + i * 4] =
                    ((mouse.maze.walls_horizontal[i] & 0xff000000) >> 24);
        }
    }

    if (part_num == 3) {
        for (int i = 0; i < 15; i++) {
            buf[ind + 0 + i * 4] = ((mouse.maze.walls_horizontal[i + 16]
                    & 0x000000ff) >> 0);
            buf[ind + 1 + i * 4] = ((mouse.maze.walls_horizontal[i + 16]
                    & 0x0000ff00) >> 8);
            buf[ind + 2 + i * 4] = ((mouse.maze.walls_horizontal[i + 16]
                    & 0x00ff0000) >> 16);
            buf[ind + 3 + i * 4] = ((mouse.maze.walls_horizontal[i + 16]
                    & 0xff000000) >> 24);
        }
    }
}

//パラメータマネージャーに登録された変数を配列に格納
//52byteを占有 型情報  パート番号2byte + (1byte + 変数値 4byte) *10 = 52byte
void packDataParamMng(uint8_t part_num, uint8_t *buf) {
    buf[0] = part_num;
    buf[1] = 255;
    ParameterManager &pm = ParameterManager::getInstance();
    for(uint8_t i=0;i<10;i++){
        if(pm.typeMap.find(10*part_num + i) != pm.typeMap.end()){
            uint16_t val_num = 10*part_num +i;
            Type_e type = pm.typeMap[val_num];
            buf[2+i*5] = (uint8_t)type;
            if(type == Type_e::FLOAT) *reinterpret_cast<float*>(&buf[2+i*5+1]) = *reinterpret_cast<float*>(pm.adrMap[val_num]);
            if(type == Type_e::UINT8) *reinterpret_cast<uint8_t*>(&buf[2+i*5+1]) = *reinterpret_cast<uint8_t*>(pm.adrMap[val_num]);
            if(type == Type_e::UINT16) *reinterpret_cast<uint16_t*>(&buf[2+i*5+1]) = *reinterpret_cast<uint16_t*>(pm.adrMap[val_num]);
            if(type == Type_e::UINT32) *reinterpret_cast<uint32_t*>(&buf[2+i*5+1]) = *reinterpret_cast<uint32_t*>(pm.adrMap[val_num]);
            if(type == Type_e::INT8) *reinterpret_cast<int8_t*>(&buf[2+i*5+1]) = *reinterpret_cast<int8_t*>(pm.adrMap[val_num]);
            if(type == Type_e::INT16) *reinterpret_cast<int16_t*>(&buf[2+i*5+1]) = *reinterpret_cast<int16_t*>(pm.adrMap[val_num]);
            if(type == Type_e::INT32) *reinterpret_cast<int32_t*>(&buf[2+i*5+1]) = *reinterpret_cast<int32_t*>(pm.adrMap[val_num]);

        }
        else{
            buf[2+i*5] = 255;
        }
    }
}

void packWallSenAdjData(uint8_t *buf){
    uint8_t printfDataNum = 0;
    const uint8_t printfFieldNum = 20;

    UMouse &m = UMouse::getInstance();
//    FrontWallSensor &fws = FrontWallSensor::getInstance();
//    BackWallSensor &bws = BackWallSensor::getInstance();


    //header
    buf[0] = 0xff;
    buf[1] = 0xff;
    buf[2] = 0x48;
    buf[3] = 0x45;
    buf[4] = 0x41;
    buf[5] = 0x44;
    uint32_t elapsedTime = getElapsedMsec();
    set4ByteVal(buf, 8, elapsedTime);

    set2ByteVal(buf, 12, getAD_AN102());

    //パラメータマネージャのデータ
    static uint8_t count_paramMng = 0;
    packDataParamMng(count_paramMng, &buf[250]);
    count_paramMng++;
    if (count_paramMng == PARAM_MNG_PART_NUM) count_paramMng = 0;

    const uint8_t dataBuffNum = 30;
    //壁センサ値1kHz
    for (int i = 0; i < dataBuffNum; i++) {
/*
        set2ByteVal(buf, 16 + 2 * i, (int16_t) fws.left.at(i));
        set2ByteVal(buf, 76 + 2 * i, (int16_t) fws.right.at(i));
        set2ByteVal(buf, 136 + 2 * i, (int16_t) bws.left.at(i));
        set2ByteVal(buf, 302 + 2 * i, (int16_t) bws.right.at(i));
*/
    }

    //速度と角速度200Hz
    for(int i= 0; i<6; i++){
        set2ByteVal(buf, 196+ 2*i, (float)m.t_v_buff.at(i*5), 3000.0);
        set2ByteVal(buf, 208+ 2*i, (float)m.v_buff.at(i*5), 3000.0);
        set2ByteVal(buf, 220+ 2*i, (float)m.t_ang_v_buff.at(i*5), 20.0);
        set2ByteVal(buf, 232+ 2*i, (float)m.ang_v_buff.at(i*5), 20.0);
    }

    //printf Data
    uint16_t start_byte = PERIODIC_MSG_LEN - printfFieldNum;
    uint16_t end_byte = PERIODIC_MSG_LEN;
    for (int i = start_byte; i < end_byte; i++) {
        if (printfBuff.empty() == false) {
            buf[i] = printfBuff.front();
            printfBuff.pop();
            printfDataNum++;
        }
        else {
            buf[i] = 0;
        }
    }
    //printf Data Num
    buf[7] = printfDataNum;

    //check sum
    uint8_t sum = 0;
    for (int i = 7; i < PERIODIC_MSG_LEN; i++)
        sum += buf[i];
    buf[6] = sum;
}


void packPidPrmAdjData(uint8_t *buf){
    uint8_t printfDataNum = 0;
    const uint8_t printfFieldNum = 20;

    UMouse &m = UMouse::getInstance();

    //header
    buf[0] = 0xff;
    buf[1] = 0xff;
    buf[2] = 0x48;
    buf[3] = 0x45;
    buf[4] = 0x41;
    buf[5] = 0x44;
    uint32_t elapsedTime = getElapsedMsec();
    set4ByteVal(buf, 8, elapsedTime);

    set2ByteVal(buf, 12, getAD_AN102());

    //パラメータマネージャのデータ
    static uint8_t count_paramMng = 0;
    packDataParamMng(count_paramMng, &buf[250]);
    count_paramMng++;
    if(count_paramMng == PARAM_MNG_PART_NUM) count_paramMng = 0;

    const uint8_t dataBuffNum = 30;
    //速度
    for(int i= 0; i<dataBuffNum; i++){
        set2ByteVal(buf, 16+ 2*i, (float)m.t_v_buff.at(i), 10000.0);
        set2ByteVal(buf, 76+ 2*i, (float)m.v_buff.at(i), 10000.0);
        set2ByteVal(buf, 136+ 2*i, (float)m.t_ang_v_buff.at(i), 10.0);
        set2ByteVal(buf, 302+ 2*i, (float)m.ang_v_buff.at(i), 10.0);
    }


    //printf Data
    uint16_t start_byte = PERIODIC_MSG_LEN - printfFieldNum;
    uint16_t end_byte = PERIODIC_MSG_LEN;
    for (int i = start_byte; i < end_byte; i++) {
        if (printfBuff.empty() == false) {
            buf[i] = printfBuff.front();
            printfBuff.pop();
            printfDataNum++;
        } else {
            buf[i] = 0;
        }
    }
    //printf Data Num
    buf[7] = printfDataNum;

    //check sum
    uint8_t sum = 0;
    for (int i = 7; i < PERIODIC_MSG_LEN; i++)
        sum += buf[i];
    buf[6] = sum;
}

void packVelocityAdjData(uint8_t *buf){
    uint8_t printfDataNum = 0;
    const uint8_t printfFieldNum = 20;

    UMouse &m = UMouse::getInstance();

    //header
    buf[0] = 0xff;
    buf[1] = 0xff;
    buf[2] = 0x48;
    buf[3] = 0x45;
    buf[4] = 0x41;
    buf[5] = 0x44;
    uint32_t elapsedTime = getElapsedMsec();
    set4ByteVal(buf, 8, elapsedTime);

    set2ByteVal(buf, 12, getAD_AN102());

    //パラメータマネージャのデータ
    static uint8_t count_paramMng = 0;
    packDataParamMng(count_paramMng, &buf[250]);
    count_paramMng++;
    if(count_paramMng == PARAM_MNG_PART_NUM) count_paramMng = 0;

    const uint8_t dataBuffNum = 30;
    //速度
    for(int i= 0; i<dataBuffNum; i++){
        set2ByteVal(buf, 16+ 2*i, (float)m.t_v_buff.at(i), 3000.0);
        set2ByteVal(buf, 76+ 2*i, (float)m.v_buff.at(i), 3000.0);
        set2ByteVal(buf, 136+ 2*i, (float)m.v_acc_buff.at(i), 3000.0);
        set2ByteVal(buf, 302+ 2*i, (float)m.acc_y_buff.at(i), 20.0);
    }
/*
    for(int i= 0; i<27; i++){
        set2ByteVal(buf, 196+ 2*i, (float)m.v_comp_buff.at(i), 3000.0);
    }
    for(int i= 0; i<3; i++){
        set2ByteVal(buf, 362+ 2*i, (float)m.v_comp_buff.at(i+27), 3000.0);
    }
*/

    //printf Data
    uint16_t start_byte = PERIODIC_MSG_LEN - printfFieldNum;
    uint16_t end_byte = PERIODIC_MSG_LEN;
    for (int i = start_byte; i < end_byte; i++) {
        if (printfBuff.empty() == false) {
            buf[i] = printfBuff.front();
            printfBuff.pop();
            printfDataNum++;
        } else {
            buf[i] = 0;
        }
    }
    //printf Data Num
    buf[7] = printfDataNum;

    //check sum
    uint8_t sum = 0;
    for (int i = 7; i < PERIODIC_MSG_LEN; i++)
        sum += buf[i];
    buf[6] = sum;
}



void packData(uint8_t *buf) {
    uint8_t printfDataNum = 0;
    const uint8_t printfFieldNum = 60;

//    TactSw& tsw = TactSw::getInstance();
    UMouse &m = UMouse::getInstance();
    BatVoltageMonitor &bvm = BatVoltageMonitor::getInstance();
    WallSensor &ws = WallSensor::getInstance();
    WheelOdometry &wo = WheelOdometry::getInstance();
    ICM20602 &icm = ICM20602::getInstance();
    adis16470 &adis = adis16470::getInstance();

    PowerTransmission &pt = PowerTransmission::getInstance();
    FcLed &fcled = FcLed::getInstance();

    //header
    buf[0] = 0xff;
    buf[1] = 0xff;
    buf[2] = 0x48;
    buf[3] = 0x45;
    buf[4] = 0x41;
    buf[5] = 0x44;
    uint32_t elapsedTime = getElapsedMsec();
    set4ByteVal(buf, 8, elapsedTime);

    set2ByteVal(buf, 12, getAD_AN101());
    set2ByteVal(buf, 14, getAD_AN100());
    set2ByteVal(buf, 16, getAD_AN001());
    set2ByteVal(buf, 18, getAD_AN002());
    set2ByteVal(buf, 20, getAD_AN105());

    set2ByteVal(buf, 22, getDA());
    set2ByteVal(buf, 24, pt.getDuty_L(), 10000);
    set2ByteVal(buf, 26, pt.getDuty_R(), 10000);

    set2ByteVal(buf, 28, (float)(pt.getDuty_L() * bvm.bat_vol) , 5000.0);
    set2ByteVal(buf, 30, (float)(pt.getDuty_R() * bvm.bat_vol) , 5000.0);

    set2ByteVal(buf, 32, (uint16_t)getTimeuCountIntCMT0());
    set2ByteVal(buf, 34, (uint16_t)getTimeuCountIntCMT1());
    set2ByteVal(buf, 36, (uint16_t)getTimeuCount_sub());

    set2ByteVal(buf, 38, getCountMTU1());

    set2ByteVal(buf, 42, getCountMTU2());



    set2ByteVal(buf, 46, icm.omega_raw[0]);
    set2ByteVal(buf, 48, icm.omega_raw[1]);
    set2ByteVal(buf, 50, icm.omega_raw[2]);
    set2ByteVal(buf, 52, icm.acc_raw[0]);
    set2ByteVal(buf, 54, icm.acc_raw[1]);
    set2ByteVal(buf, 56, icm.acc_raw[2]);
    set2ByteVal(buf, 58, (int16_t)ws.ahead_l());
    set2ByteVal(buf, 60, (int16_t)ws.right());
    set2ByteVal(buf, 62, (int16_t)ws.left());
    set2ByteVal(buf, 64, (int16_t)ws.ahead_r());
    set2ByteVal(buf, 66, (uint16_t)(fcled.R.getState()));
    set2ByteVal(buf, 68, (uint16_t)(fcled.G.getState()));
    set2ByteVal(buf, 70, (uint16_t)(fcled.B.getState()));
    set2ByteVal(buf, 72, adis.omega_raw[2]);
    set2ByteVal(buf, 74, (uint16_t)0);
    set2ByteVal(buf, 76, adis.acc_raw[0]);
    set2ByteVal(buf, 78, adis.acc_raw[1]);
    

    set2ByteVal(buf, 80, m.start.x);
    set2ByteVal(buf, 82, m.start.y);
    set2ByteVal(buf, 84, m.goal.x);
    set2ByteVal(buf, 86, m.goal.y);
    set2ByteVal(buf, 88, m.coor.x);
    set2ByteVal(buf, 90, m.coor.y);

    set2ByteVal(buf, 92, (float)m.posEsti.getX(), 1000);
    set2ByteVal(buf, 94, (float)m.posEsti.getY(), 1000);
    set2ByteVal(buf, 96, (float)m.posEsti.getAng(), 100);
    set2ByteVal(buf, 98, (int16_t)m.direction);
    set2ByteVal(buf, 100, m.trajCommander.x, 1000);
    set2ByteVal(buf, 102, m.trajCommander.y, 1000);
    set2ByteVal(buf, 104, m.trajCommander.ang, 100);

    set2ByteVal(buf, 106, (float)wo.v, 10000);
    set2ByteVal(buf, 108, (float)m.posEsti.getV(), 10000);
    set2ByteVal(buf, 110, m.trajCommander.v, 10000);
    set2ByteVal(buf, 112, (float)m.posEsti.getVAcc(), 10000);

    set2ByteVal(buf, 114, (float)m.posEsti.getAngV(), 10);
    set2ByteVal(buf, 116, m.trajCommander.ang_v, 10);
    set2ByteVal(buf, 118, m.ctrlMixer.target_rot_v, 10);

    float a_x = - m.trajCommander.v * DEG2RAD(m.trajCommander.ang_v);
    set2ByteVal(buf, 120, a_x, 1000);
    set2ByteVal(buf, 122, m.trajCommander.a, 1000);
    set2ByteVal(buf, 124, icm.acc_f[0], 1000);
    set2ByteVal(buf, 126, icm.acc_f[1], 1000);
    set2ByteVal(buf, 128, m.ctrlMixer.target_rot_x, 100);

    set2ByteVal(buf, 130, m.ctrlMixer.v_pidf.getControlVal(), 10000);
    set2ByteVal(buf, 132, m.ctrlMixer.ang_v_pidf.getControlVal(), 10000);
    
    set2ByteVal(buf, 134, m.ctrlMixer.v_back_emf_FF, 10000);
    set2ByteVal(buf, 136, m.ctrlMixer.a_acc_FF, 10000);
    set2ByteVal(buf, 138, m.ctrlMixer.v_fric_FF, 10000);
    
    set2ByteVal(buf, 140, m.ctrlMixer.ang_v_back_emf_FF, 10000);
    set2ByteVal(buf, 142, m.ctrlMixer.ang_a_acc_FF, 10000);
    set2ByteVal(buf, 144, m.ctrlMixer.ang_v_fric_FF, 10000);

    set2ByteVal(buf, 146, m.ctrlMixer.wall_pidf.getControlVal(), 100);
    set2ByteVal(buf, 148, m.ctrlMixer.pos_pidf.getControlVal(), 100);

    set2ByteVal(buf, 150, (float)wo.getAng_v(), 10);
    set2ByteVal(buf, 152, (float)m.posEsti.getBeta(), 100);

    set2ByteVal(buf, 154, (float)m.ctrlMixer.ang_pidf.getControlVal(), 10);

    //迷路データ
    static uint8_t count = 0;
    packDataMaze(count, &buf[160]);
    count++;
    if (count == 4) count = 0;

    //パラメータマネージャのデータ
    static uint8_t count_paramMng = 0;
    packDataParamMng(count_paramMng, &buf[250]);
    count_paramMng++;
    if(count_paramMng == PARAM_MNG_PART_NUM) count_paramMng = 0;

    //printf Data
    uint16_t start_byte = PERIODIC_MSG_LEN - printfFieldNum;
    uint16_t end_byte = PERIODIC_MSG_LEN;
    for (int i = start_byte; i < end_byte; i++) {
        if (printfBuff.empty() == false) {
            buf[i] = printfBuff.front();
            printfBuff.pop();
            printfDataNum++;
        } else {
            buf[i] = 0;
        }
    }
    //printf Data Num
    buf[7] = printfDataNum;

    //check sum
    uint8_t sum = 0;
    for (int i = 7; i < PERIODIC_MSG_LEN; i++)
        sum += buf[i];
    buf[6] = sum;
}

uint8_t* getPointerOfPeriodicMsg() {
    return periodicMsg;
}


}//robot_object

inline void set2ByteVal(uint8_t *buf, uint16_t index, float val,float prop){
    int16_t int_val = (int16_t)(val * prop);
    buf[index]   = (0x0000ff00 & int_val) >> 8;
    buf[index+1] = (0x000000ff & int_val)   ;
}

inline void set2ByteVal(uint8_t *buf, uint16_t index, uint16_t val){
    buf[index]   = (0x0000ff00 & val) >> 8;
    buf[index+1] = (0x000000ff & val)   ;
}

inline void set2ByteVal(uint8_t *buf, uint16_t index, int16_t val){
    buf[index]   = (0x0000ff00 & val) >> 8;
    buf[index+1] = (0x000000ff & val)   ;
}

inline void set2ByteVal(uint8_t *buf, uint16_t index, uint32_t val){
    buf[index]   = (0x0000ff00 & val) >> 8;
    buf[index+1] = (0x000000ff & val)   ;
}

inline void set2ByteVal(uint8_t *buf, uint16_t index, int32_t val){
    buf[index]   = (0x0000ff00 & val) >> 8;
    buf[index+1] = (0x000000ff & val)   ;
}

inline void set4ByteVal(uint8_t *buf, uint16_t index, uint32_t val){
    buf[index] = ((0xff000000 & val) >> 24);
    buf[index+1] = ((0x00ff0000 & val) >> 16);
    buf[index+2] = ((0x0000ff00 & val) >> 8);
    buf[index+3] = ((0x000000ff & val));
}




