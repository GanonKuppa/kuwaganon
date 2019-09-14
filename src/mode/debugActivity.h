#pragma once

#include "baseActivity.h"
#include "communication.h"
#include "timer.h"
#include "mouse.h"
#include "trajectory.h"
#include "fcLed.h"
#include "sound.h"
#include "ICM20602.h"


namespace umouse{

enum class EDebagMode : uint8_t {
    BACK_MODE_SELECT = 0,
    SLOW_STRAIGHT_DEBUG = 1,
    SLOW_STRAIGHT_DEBUG_WALL = 2,
    FAST_STRAIGHT_DEBUG = 3,
    FAST_STRAIGHT_DEBUG_WALL = 4,
    SPIN_TURN_DEBUG = 5
};



class DebugActivity : public BaseActivity{
protected:

    void onStart(){
        printfAsync("■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■.\n");
        printfAsync("This is debug activity.\n");
/*
        Intent *intent = new Intent();
        intent->uint8_t_param["SUB_MODE_NUM"] = 6;
        auto activity = ActivityFactory::cteateSubModeSelect();
        activity->start(intent);
        printfAsync("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
*/
        UMouse &m = UMouse::getInstance();
        waitmsec(2000);
        ICM20602 &icm = ICM20602::getInstance();
        icm.calibOmegaOffset(800);
        icm.calibAccOffset(800);
/*
        std::unique_ptr<BaseState> state = std::unique_ptr<BaseState>(new Start2GoalState(intent));
        stateMachine.push(std::move(state));
        delete intent;
*/
        float x = 0.09f/2.0f;
        float y = 0.09f/2.0f - m.WALL2MOUSE_CENTER_DIST;
        float ang = 90.0;
        m.posEsti.reset(x,y,ang);
        m.trajCommander.reset(x,y,ang);
        m.ctrlMixer.reset();

        ParameterManager &pm = ParameterManager::getInstance();
        auto traj0 = StraightTrajectory::createAsWallCenter(0.09 * 15 + m.WALL2MOUSE_CENTER_DIST, 0.0, 0.15, 0.1, 1.0, 1.0);
        auto traj1 = StopTrajectory::create(1.0);
        m.trajCommander.push(std::move(traj0));
        m.trajCommander.push(std::move(traj1));



    };


    ELoopStatus loop(){
        FcLed &fcled = FcLed::getInstance();

        UMouse &m = UMouse::getInstance();

        if(m.trajCommander.empty() == true){
            SEB();
            return ELoopStatus::FINISH;        
        }
        else{
            return ELoopStatus::CONTINUE;
        }
    };
    void onFinish(){};
};

}
