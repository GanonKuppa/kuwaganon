#pragma once

#include <stdint.h>
#include "timer.h"
#include "communication.h"

namespace umouse{

class Gamepad{
public:
    volatile int8_t cross_x;
    volatile int8_t cross_y;
    volatile int8_t L3D_x;
    volatile int8_t L3D_y;
    volatile int8_t R3D_x;
    volatile int8_t R3D_y;
    volatile uint8_t RT;
    volatile uint8_t LT;
    volatile uint32_t A;
    volatile uint32_t B;
    volatile uint32_t Y;
    volatile uint32_t X;
    volatile uint32_t RB;
    volatile uint32_t LB;
    volatile uint32_t BACK;
    volatile uint32_t START;
    volatile uint8_t receiveCommand[16];

    static Gamepad& getInstance(){
        static Gamepad instance;
        return instance;
    }

    bool isConnecting(){
        if(getElapsedMsec() - preRecieveTime < 200){
            return true;
        }
        else{
            return false;
        }

    };

    void updateCommand(uint8_t *command){
        if(command[3] == 254 && command[4] == 253){
            preRecieveTime = getElapsedMsec();
            for(uint8_t i=0;i<16; i++){
                receiveCommand[i] = command[i];
            }
        }
    };

    //1msec毎に呼ぶこと
    void update(){
       volatile uint8_t *command = receiveCommand;
       if((command[6] & 0x01) == 1) A++;
       else A = 0;
       if(((command[6] & 0x02) >> 1) == 1) B++;
       else B = 0;
       if(((command[6] & 0x04) >> 2) == 1) X++;
       else X= 0;
       if(((command[6] & 0x08) >> 3) == 1) Y++;
       else Y = 0;
       if(((command[6] & 0x10) >> 4) == 1) RB++;
       else RB = 0;
       if(((command[6] & 0x20) >> 5) == 1) LB++;
       else LB = 0;
       if(((command[6] & 0x40) >> 6) == 1) BACK++;
       else BACK = 0;
       if(((command[6] & 0x80) >> 7) == 1) START++;
       else START = 0;
       RT = command[7];
       LT = command[8];
       cross_x = command[9]  - 128;
       cross_y = command[10] - 128;
       R3D_x = command[11] - 128;
       R3D_y = command[12] - 128;
       L3D_x = command[13] - 128;
       L3D_y = command[14] - 128;

       if(isConnecting() == false){
           A = 0;
           B = 0;
           X = 0;
           Y = 0;
           RB = 0;
           LB = 0;
           BACK = 0;
           START = 0;
           RT = 0;
           LT = 0;
           cross_x = 0;
           cross_y = 0;
           R3D_x = 0;
           R3D_y = 0;
           L3D_x = 0;
           L3D_y = 0;
       }

    };


    void debug() {
        printfAsync("A:%d, B:%d, X:%d, Y:%d, RT:%d, LT:%d  \n", A,B,X,Y,RT,LT);
    };

private:
    bool conStatus;
    uint64_t preRecieveTime;
    Gamepad(){
        cross_x = 0;
        cross_y = 0;
        L3D_x = 0;
        R3D_x = 0;
        R3D_y = 0;
        RT = 0;
        LT = 0;
        A  = 0;
        B  = 0;
        Y  = 0;
        X  = 0;
        RB = 0;
        LB = 0;
        BACK = 0;
        START = 0;
        conStatus = false;
        preRecieveTime = 0;
    };
    ~Gamepad(){};
};

}

