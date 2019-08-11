#pragma once

#include "baseActivity.h"
#include "communication.h"

namespace umouse{

class CalibrateWallCenterActivity : public BaseActivity{
protected:
    ELoopStatus loop(){

        return ELoopStatus::CONTINUE;
    };
    void onStart(){
        printfAsync("This is calibrate wall center activity.\n");
    };
    void onFinish(){};
};

}
