#pragma once

#include <stdint.h>

#include "baseActivity.h"
#include "pseudoDial.h"
#include "fcled.h"
#include "timer.h"
#include "wheelOdometry.h"
#include "sound.h"
#include "activityFactory.h"
#include "communication.h"
#include "gamepad.h"
#include "ICM20602.h"
#include "adis16470.h"
#include "wallsensor.h"
#include "mouse.h"

namespace umouse {

    class ModeSelectActivity : public BaseActivity {

    public:
        void onStart() {
            printfAsync("This is mode select activity.\n");
            RAMEN();
            //LEDをチカチカ
            FcLed &fcled = FcLed::getInstance();
            fcled.turn(1, 0, 0);
            waitmsec(200);
            fcled.turn(0, 1, 0);
            waitmsec(200);
            fcled.turn(0, 0, 1);
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
            was_gamepad_input = false;
            turnFcled();

        }

        ELoopStatus loop() {
            Gamepad &gamepad = Gamepad::getInstance();
            ICM20602 &icm = ICM20602::getInstance();
            adis16470 &adis = adis16470::getInstance();
            PseudoDialL &dial_L = PseudoDialL::getInstance();
            PseudoDialR &dial_R = PseudoDialR::getInstance();
            WallSensor &ws = WallSensor::getInstance();
            dial_position_L_now = dial_L.getDialPosition();
            dial_position_R_now = dial_R.getDialPosition();
            UMouse &m = UMouse::getInstance();



            if(dial_position_L_now != dial_position_L_pre) {
                turnFcled();
                SE_CURSOR_MOVE();
            }

            // タイヤ掃除用に前壁センサONのときは制御をかけない
            if(ws.isAhead()){
                dial_L.setEnable(false);
                dial_R.setEnable(false);
            }
            else if(!was_gamepad_input && !dial_L.getEnable() && !dial_R.getEnable()){
                PseudoDialL &dial_L = PseudoDialL::getInstance();
                PseudoDialR &dial_R = PseudoDialR::getInstance();
                dial_L.reset();
                dial_R.reset();
                dial_L.setEnable(true);
                dial_R.setEnable(true);
            }

            if(dial_position_R_now == 4 || (gamepad.B > 50 && gamepad.B < 150) && !ws.isAhead()) {
                SE_CONFIRM();
                waitmsec(500);

                return ELoopStatus::FINISH;
            }

            if(gamepad.X > 50 && gamepad.X < 150 ) {
                SEA();
                icm.calibOmegaOffset(800);
                icm.calibAccOffset(800);
                adis.calibOmegaOffset(1600);
                float x = 0.09f/2.0f;
                float y = 0.09f/2.0f;
                float ang = 90.0;
                m.posEsti.reset(x,y,ang);
                m.trajCommander.reset(x,y,ang);
                m.ctrlMixer.reset();

                SEF();
                waitmsec(100);
            }

            if(gamepad.Y > 50 && gamepad.Y < 150 ) {
                SEA();
                ws.setWallCenterVal();
                SEH();

                waitmsec(100);
            }

            if(gamepad.cross_y == 1 ) {
                dial_L.setEnable(false);
                dial_R.setEnable(false);
                dial_L.incrementPosition();
                was_gamepad_input = true;
                waitmsec(200);
            }
            if(gamepad.cross_y == -1 ) {
                dial_L.setEnable(false);
                dial_R.setEnable(false);
                dial_L.decrementPosition();
                was_gamepad_input = true;
                waitmsec(200);
            }


            dial_position_L_pre = dial_position_L_now;
            dial_position_R_pre = dial_position_R_now;
            return ELoopStatus::CONTINUE;
        };

        void onFinish() {
            PseudoDialL &dial_L = PseudoDialL::getInstance();
            PseudoDialR &dial_R = PseudoDialR::getInstance();
            dial_L.setEnable(0);
            dial_R.setEnable(0);
            uint8_t mode = dial_L.getDialPosition();
            EActivityColor color = modeNum2Color(mode);
            printfAsync("color num = %d\n",int(color));

            auto activity = ActivityFactory::create(color);
            activity->start();
        }
    private:
        uint8_t dial_position_R_now;
        uint8_t dial_position_R_pre;
        uint8_t dial_position_L_now;
        uint8_t dial_position_L_pre;
        bool was_gamepad_input;

        void turnFcled() {
            PseudoDialL &dial_L = PseudoDialL::getInstance();
            FcLed &fcled = FcLed::getInstance();
            uint8_t mode = dial_L.getDialPosition();

            if (mode == 0) fcled.turn(0, 0, 0); //BLACK
            else if (mode == 1) fcled.turn(1, 0, 0);//RED
            else if (mode == 2) fcled.turn(0, 1, 0);//GREEN
            else if (mode == 3) fcled.turn(1, 1, 0);//YELLOW
            else if (mode == 4) fcled.turn(0, 0, 1);//BLUE
            else if (mode == 5) fcled.turn(1, 0, 1);//MAGENTA
            else if (mode == 6) fcled.turn(0, 1, 1);//CYAN
            else if (mode == 7) fcled.turn(1, 1, 1);//WHITE
        }

        EActivityColor modeNum2Color(uint8_t mode) {
            EActivityColor color;
            color = (EActivityColor)mode;
            return color;
        }

    };

}
