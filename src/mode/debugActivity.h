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

        UMouse &m = UMouse::getInstance();
        waitmsec(2000);
        ICM20602 &icm = ICM20602::getInstance();
        icm.calibOmegaOffset(800);
        icm.calibAccOffset(800);

        std::unique_ptr<BaseState> state = std::unique_ptr<BaseState>(new Start2GoalState(intent));
        stateMachine.push(std::move(state));
        delete intent;
*/
    };


    ELoopStatus loop(){
        FcLed &fcled = FcLed::getInstance();

        UMouse &m = UMouse::getInstance();

        if(m.trajCommander.empty() == true){
            SEB();

            UMouse &m = UMouse::getInstance();
            ParameterManager &pm = ParameterManager::getInstance();
            //auto traj0 = StraightTrajectory::createAsWallCenter(0.09 , 0.0, 0.2, 0.2, 1.0, 1.0);
            //auto traj1 = StraightTrajectory::createAsWallCenter(0.09 , 0.2);
            //auto traj2 = StraightTrajectory::createAsWallCenter(0.09 , 0.2, 0.2, 0.0, 1.0, 1.0);


            //auto traj3 = StopTrajectory::create(3.0f);
            //auto traj4 = SpinTurnTrajectory::create(180.0f,360.0f, 720.0f);
            //auto traj5 = StopTrajectory::create(3.0f);
    /*
            auto traj4 = StraightTrajectory::createAsWallCenter(0.09  /2.0, 0.0, 0.13, 0.0, 1.0, 1.0);
            auto traj5 = StopTrajectory::create(0.2f);
            auto traj6 = SpinTurnTrajectory::create(-180.0f,360.0f, 720.0f);
            auto traj7 = StopTrajectory::create(0.2f);
    */
            //auto traj4 = StraightTrajectory::createAsWallCenter(0.09 * 1, 0.0, 0.2, 0.0, 1.0, 1.0);
            auto traj3 = StopTrajectory::create(1.2f);
            auto traj4 = StraightTrajectory::createAsWallCenter(0.09 * 1, 0.0, 0.2, 0.0, 1.0, 1.0);
            auto traj5 = SpinTurnTrajectory::create(90.0, 360.0f, 720.0f);
            auto traj6 = StopTrajectory::create(1.2f);
            auto traj7 = SpinTurnTrajectory::create(-90.0, 360.0f, 720.0f);

            //auto traj5 = StopTrajectory::create(1.0f);
            float x = 0.09f/2.0f;
            float y = 0.09f/2.0f - m.WALL2MOUSE_CENTER_DIST;
            float ang = 90.0;
            m.posEsti.reset(x,y,ang);
            m.trajCommander.reset(x,y,ang);
            m.ctrlMixer.reset();


            //auto traj6 = StraightTrajectory::createAsWallCenter(pm.test_run_x + m.WALL2MOUSE_CENTER_DIST, 0.0, pm.test_run_v, 0.1, pm.test_run_a, pm.test_run_a);
            //auto traj7 = StopTrajectory::create(3.0f);
            //auto traj8 = CompositeTrajectory::create(std::move(traj3), std::move(traj4),std::move(traj5), std::move(traj6), std::move(traj7));
            //m.trajCommander.push(std::move(traj7));
            //m.trajCommander.push(std::move(traj8));
            //m.trajCommander.push(std::move(traj6));
            //m.trajCommander.push(std::move(traj5));
            //m.trajCommander.push(std::move(traj3));
            //m.trajCommander.push(std::move(traj4));

        }

        return ELoopStatus::CONTINUE;
    };
    void onFinish(){};
};

}
