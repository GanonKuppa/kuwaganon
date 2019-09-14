#pragma once

#include <myUtil.h>
#include "iodefine.h"
#include "stdint.h"
#include "parameterManager.h"
#include "ad.h"
#include <deque>
#include "communication.h"
#include "sound.h"
#include "timer.h"


namespace umouse {

class BatVoltageMonitor {

private:
    const uint8_t BUFF_SIZE = 10;
    const float alert_vol = 3.3;
    uint16_t count;
    std::deque<int16_t> buff;

    BatVoltageMonitor() {
        bat_vol = 4.2;
        average_bat_vol = 4.2;
        count = 0;

        for (uint8_t i = 0; i < BUFF_SIZE; i++) {
            buff.push_front(0);
        }

    }

    ~BatVoltageMonitor() {

    }

public:

    float bat_vol;
    float average_bat_vol; //0.5秒間隔で10回サンプルした場合の平均

    static BatVoltageMonitor& getInstance() {
         static BatVoltageMonitor instance;
         return instance;
    }

    void update(){
        int16_t ad = startAD_AN105();
        float bat_vol = ad2Voltage(ad);

        count ++;
        if(count > 2000) count = 0; //0.25msec割り込み2000回 = 0.5秒
        if(count == 0){
            buff.push_front(ad);
            buff.pop_back();

            uint32_t sum = 0;
            for(auto itr = buff.begin(); itr != buff.end(); ++itr) {
                   sum += *itr;
            }

            average_bat_vol = 30.0 / 20.0 * 3.2 * (float(sum) / float(BUFF_SIZE)) / 4095.0;

        }
    }

    float ad2Voltage(int16_t ad){
        return 30.0 / 20.0 * 3.2 * (float(ad) / 4095.0);
    }

    void lowVoltageCheck(){
        if(average_bat_vol < alert_vol){
            famima();
        }
    }

    void voltageSoundCount(){
        int16_t ad = startAD_AN105();
        float vol_f = ad2Voltage(ad);
        printfAsync("vol_f:%f \n", vol_f);
        uint8_t num_1V = (uint8_t)vol_f;
        uint8_t num_0_1V = uint8_t((vol_f - (float)num_1V) * 10.0);
        for (int i = 0; i < num_1V; i++) {
            SEA();
            if (i == 4)
                waitmsec(100);
            else
                waitmsec(50);
        }
        waitmsec(200);
        for (int i = 0; i < num_0_1V; i++) {
            SEB();
            waitmsec(100);
        }
        waitmsec(200);
    }


    void debug(){
        printfAsync("================\n");
        printfAsync("now:%f \n", bat_vol);
        printfAsync("ave %f \n", average_bat_vol);

    }

};

}

