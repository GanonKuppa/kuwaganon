#pragma once

#include "baseActivity.h"
#include "communication.h"
#include "wallSensor.h"
#include "gamepad.h"

namespace umouse{

class SubModeSelectActivity : public BaseActivity{

    void onStart(){
        sub_mode_num = intent->int8_t_param["SUB_MODE_NUM"];
        printfAsync("sub mode select start. submodenum = %d\n",sub_mode_num);

        SEF();
        //LEDをチカチカ
        FcLed &fcled = FcLed::getInstance();
        fcled.turn(1, 1, 1);
        waitmsec(200);
        fcled.turn(0, 0, 0);
        waitmsec(200);
        fcled.turn(1, 1, 1);
        waitmsec(200);
        fcled.turn(0, 0, 0);

        PseudoDialL &dial_L = PseudoDialL::getInstance();
        PseudoDialR &dial_R = PseudoDialR::getInstance();
        dial_L.reset();
        dial_R.reset();
        dial_L.setEnable(1);
        dial_R.setEnable(1);
        dial_L.setDivisionNum(8);
        dial_R.setDivisionNum(8);
        dial_position_R_now = 0;
        dial_position_R_pre = 0;
        dial_position_L_now = 0;
        dial_position_L_pre = 0;
        turnFcled();

    }


    ELoopStatus loop(){
        Gamepad &gamepad = Gamepad::getInstance();
        PseudoDialL &dial_L = PseudoDialL::getInstance();
        PseudoDialR &dial_R = PseudoDialR::getInstance();
        WallSensor &ws = WallSensor::getInstance();
        dial_position_L_now = dial_L.getDialPosition();
        dial_position_R_now = dial_R.getDialPosition();

        if(dial_position_L_now != dial_position_L_pre){
            turnFcled();
            SE_CURSOR_MOVE(-1);
        }

        if(dial_position_R_now  == 4 || (gamepad.B > 50 && gamepad.B < 150) || ws.getAheadOnTime() > 0.5){
            SE_CONFIRM();
            waitmsec(500);

            return ELoopStatus::FINISH;
        }

        if(gamepad.cross_y == 1 ) {
            dial_L.setEnable(false);
            dial_R.setEnable(false);
            dial_L.incrementPosition();
            waitmsec(200);
        }
        if(gamepad.cross_y == -1 ) {
            dial_L.setEnable(false);
            dial_R.setEnable(false);
            dial_L.decrementPosition();
            waitmsec(200);
        }


        dial_position_L_pre = dial_position_L_now;
        dial_position_R_pre = dial_position_R_now;
        return ELoopStatus::CONTINUE;
    };


    void onFinish(){
        PseudoDialL &dial_L = PseudoDialL::getInstance();
        PseudoDialR &dial_R = PseudoDialR::getInstance();
        dial_L.setEnable(0);
        dial_R.setEnable(0);
        uint8_t mode = dial_L.getDialPosition() % sub_mode_num;
        EActivityColor color = modeNum2Color(mode);
        intent->uint8_t_param["SUB_MODE"] = mode;
        printfAsync("sub mode select finish. submode = %d\n",mode);
    }
private:
    uint8_t dial_position_R_now;
    uint8_t dial_position_R_pre;
    uint8_t dial_position_L_now;
    uint8_t dial_position_L_pre;
    uint8_t sub_mode_num;

    void turnFcled(){
        PseudoDialL &dial_L = PseudoDialL::getInstance();
        FcLed &fcled = FcLed::getInstance();
        uint8_t mode = dial_L.getDialPosition() % sub_mode_num;

        if (mode == 0) fcled.turn(0, 0, 0);      //BLACK
        else if (mode == 1) fcled.flash(1, 0, 0, 250, 250); //RED
        else if (mode == 2) fcled.flash(0, 1, 0, 250, 250); //GREEN
        else if (mode == 3) fcled.flash(1, 1, 0, 250, 250); //YELLOW
        else if (mode == 4) fcled.flash(0, 0, 1, 250, 250); //BLUE
        else if (mode == 5) fcled.flash(1, 0, 1, 250, 250); //MAGENTA
        else if (mode == 6) fcled.flash(0, 1, 1, 250, 250); //CYAN
        else if (mode == 7) fcled.flash(1, 1, 1, 250, 250); //WHITE
    }

    EActivityColor modeNum2Color(uint8_t mode){
        EActivityColor color;
        color = (EActivityColor)mode;
        return color;
    }

};



}
