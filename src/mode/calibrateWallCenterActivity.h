#pragma once

#include "baseActivity.h"
#include "communication.h"

#include "logger.h"
#include "wallsensor.h"
#include "fcLed.h"

namespace umouse {

    class CalibrateWallCenterActivity : public BaseActivity {
      protected:
        ELoopStatus loop() {

            return ELoopStatus::FINISH;
        };
        void onStart() {
            printfAsync("This is calibrate wall center activity.\n");
            
            Logger& logger = umouse::Logger::getInstance();
            WallSensor& ws = WallSensor::getInstance();
            FcLed& fcled = FcLed::getInstance();

            fcled.turn(0,1,0);
            while( ws.getAheadOnTime() < 0.5) {
                waitmsec(200);                    
                SEB();                
            }
            fcled.turn(0,1,1);
            logger.print();




        };
        void onFinish() {};
    };

}
