#pragma once

#include "baseActivity.h"
#include "communication.h"
#include "wallSensor.h"
#include "gamepad.h"

namespace umouse{

class SubModeSelectActivity : public BaseActivity{

    void onStart(){
        sub_mode_num = intent->uint8_t_param["SUB_MODE_NUM"];
        
        if(intent->uint16_t_param.count("LED_ON_MSEC") == 1){
            led_on_msec = intent->uint16_t_param["LED_ON_MSEC"];
        }
        else{
            led_on_msec = 250;
        }

        if(intent->uint16_t_param.count("LED_OFF_MSEC") == 1){
            led_off_msec = intent->uint16_t_param["LED_OFF_MSEC"];
        }
        else{
            led_off_msec = 250;
        }

        if(intent->int8_t_param.count("NOTE_PITCH_OFSET") == 1){
            note_pitch_offset = intent->int8_t_param["NOTE_PITCH_OFSET"];
        }
        else{
            note_pitch_offset = 1;
        }

        printfAsync("sub mode select start. submodenum = %d\n",sub_mode_num);

        SEF(note_pitch_offset);
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
        dial_L.setDivisionNum(sub_mode_num);
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
            SE_CURSOR_MOVE(note_pitch_offset);
        }

        if(dial_position_R_now  == 4 || (gamepad.B > 50 && gamepad.B < 150) || ws.getAheadOnTime() > 0.5){
            SE_CONFIRM(note_pitch_offset);
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
        dial_L.setEnable(false);
        dial_R.setEnable(false);
        uint8_t mode = dial_L.getDialPosition() % sub_mode_num;
        intent->uint8_t_param["SUB_MODE"] = mode;
        printfAsync("sub mode select finish. submode = %d\n",mode);
    }
private:
    uint8_t dial_position_R_now;
    uint8_t dial_position_R_pre;
    uint8_t dial_position_L_now;
    uint8_t dial_position_L_pre;
    uint8_t sub_mode_num;
    uint16_t led_on_msec;
    uint16_t led_off_msec;
    int8_t note_pitch_offset;

    void turnFcled(){
        PseudoDialL &dial_L = PseudoDialL::getInstance();
        FcLed &fcled = FcLed::getInstance();
        uint8_t mode = dial_L.getDialPosition() % sub_mode_num;

        if (mode == 0) fcled.turn(0, 0, 0);      //BLACK
        else if (mode == 1) fcled.flash(1, 0, 0, led_on_msec, led_off_msec); //RED
        else if (mode == 2) fcled.flash(0, 1, 0, led_on_msec, led_off_msec); //GREEN
        else if (mode == 3) fcled.flash(1, 1, 0, led_on_msec, led_off_msec); //YELLOW
        else if (mode == 4) fcled.flash(0, 0, 1, led_on_msec, led_off_msec); //BLUE
        else if (mode == 5) fcled.flash(1, 0, 1, led_on_msec, led_off_msec); //MAGENTA
        else if (mode == 6) fcled.flash(0, 1, 1, led_on_msec, led_off_msec); //CYAN
        else if (mode == 7) fcled.flash(1, 1, 1, led_on_msec, led_off_msec); //WHITE
    }

};



}
