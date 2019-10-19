//#include "offboadCommand.h"
#include <stdint.h>
#include "trajectory.h"
#include "mouse.h"
#include "curveFactory.h"
#include "gamepad.h"
#include "parameterManager.h"
#include "sound.h"


static uint8_t calcCheckSum(uint8_t* buf, uint8_t start, uint8_t end){
    uint8_t sum = 0;
    for(uint8_t i=start;i<=end;i++) sum += buf[i];
    return sum;
}


static int16_t parseUint(uint8_t val1, uint8_t val0){
    uint16_t uint_val = val1 + val0 * 256;
    return uint_val;
}


static int16_t parseInt(uint8_t val1, uint8_t val0){
    int16_t int_val = val1 + val0 * 256;
    if(int_val > 32767) int_val = int_val - 65536;
    return int_val;
}


float parseFloat(uint8_t val1, uint8_t val0, float division_scaler){
    int16_t int_val = parseInt(val1, val0);
    return int_val / division_scaler;
}

namespace umouse {

// id:100 StopTrajectory挿入
void id_100_exec(uint8_t* buf){
    uint8_t sum = calcCheckSum(buf, 5, 15);
    if(sum != buf[4]) return;
    SED(); //音を鳴らす
    float stop_time =  parseFloat(buf[5], buf[6], 1000.0f);
    UMouse &m = UMouse::getInstance();
    auto traj = StopTrajectory::create(stop_time);
    m.trajCommander.push(std::move(traj));
}

// id:101 StraightTrajectory挿入
void id_101_exec(uint8_t* buf){
    uint8_t sum = calcCheckSum(buf, 5, 15);
    if(sum != buf[4]) return;
    SED(); //音を鳴らす
    float x = parseFloat(buf[5], buf[6], 1000.0f);
    float v_0 = parseFloat(buf[7], buf[8], 1000.0f);
    float v_max = parseFloat(buf[9], buf[10], 1000.0f);
    float v_end = parseFloat(buf[11], buf[12], 1000.0f);
    float a = parseFloat(buf[13], buf[14], 1000.0f);
    UMouse &m = UMouse::getInstance();
    auto traj = StraightTrajectory::createAsWallCenter(x, v_0, v_max, v_end, a, a);
    m.trajCommander.push(std::move(traj));
}

// id:102 SpinTurnTrajectory挿入
void id_102_exec(uint8_t* buf){
    uint8_t sum = calcCheckSum(buf, 5, 15);
    if(sum != buf[4]) return;
    SED(); //音を鳴らす
    float ang = parseFloat(buf[5], buf[6], 1.0f);
    float ang_v = parseFloat(buf[7], buf[8], 1.0f);
    float ang_a = parseFloat(buf[9], buf[10], 1.0f);
    UMouse &m = UMouse::getInstance();
    auto traj = SpinTurnTrajectory::create(ang, ang_v, ang_a);
    m.trajCommander.push(std::move(traj));
}

// id:102 CurveTrajectory挿入
void id_103_exec(uint8_t* buf){
    uint8_t sum = calcCheckSum(buf, 5, 15);
    if(sum != buf[4]) return;
    turn_type_e turn_type = (turn_type_e)parseFloat(buf[5], buf[6], 1.0f);
    turn_dir_e turn_dir;
    if( buf[6] == 1) turn_dir = turn_dir_e(1);
    else if(buf[6] == 128) turn_dir = (turn_dir_e)(-1);
    float v = parseFloat(buf[7], buf[8], 1000.0f);
    float a = parseFloat(buf[9], buf[10], 1000.0f);
    UMouse &m = UMouse::getInstance();
    auto traj0 = StraightTrajectory::create(CurveFactory::getPreDistWithOffset(turn_type, v), v, v, v, a, a);
    auto traj1 = CurveTrajectory::createAsNoStraght(v, turn_type, turn_dir);
    auto traj2 = StraightTrajectory::create(CurveFactory::getFolDist(turn_type), v, v, v, a, a);

    m.trajCommander.push(std::move(traj0));
    m.trajCommander.push(std::move(traj1));
    m.trajCommander.push(std::move(traj2));
}

// id:104 CurveTrajectory挿入
void id_104_exec(uint8_t* buf){
    uint8_t sum = calcCheckSum(buf, 5, 15);
    if(sum != buf[4]) return;
    float ang = parseFloat(buf[5], buf[6], 1.0f);    
    float ang_v = parseFloat(buf[7], buf[8], 1.0f);
    float v = parseFloat(buf[9], buf[10], 1000.0f);
    UMouse &m = UMouse::getInstance();
    auto traj = SteadyStateCircularTrajectory::create(ang, ang_v, v);

    m.trajCommander.push(std::move(traj));
}


//id:251 パラメータ書換
void id_251_exec(uint8_t* buf){
    ParameterManager &pm = ParameterManager::getInstance();
    pm.writeCommand(buf);
}

//id:254 ゲームパッド入力
void id_254_exec(uint8_t* buf){
    Gamepad &gamepad = Gamepad::getInstance();
    gamepad.updateCommand(buf);
}

//id:255 位置角度書換
    void id_255_exec(uint8_t* buf){
    uint8_t sum = calcCheckSum(buf, 5, 15);
    if(sum != buf[4]) return;
    SED(); //音を鳴らす
    float x = parseFloat(buf[5], buf[6], 1000.0f);
    float y = parseFloat(buf[7], buf[8], 1000.0f);
    float ang = parseFloat(buf[9], buf[10], 1.0f);
    UMouse &m = UMouse::getInstance();
    m.trajCommander.reset(x,y,ang);
    m.ctrlMixer.reset();
}





}
