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
        waitmsec(3000);
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
        float v = 3.0;
        float v_slalom = 0.35;
        float a = 6.0;
        int8_t rot_times = -2;
        m.posEsti.reset(x,y,ang);
        m.trajCommander.reset(x,y,ang);
        m.ctrlMixer.reset();

        ParameterManager &pm = ParameterManager::getInstance();
        auto traj0 = StraightTrajectory::createAsWallCenter(0.09 * 15 + m.WALL2MOUSE_CENTER_DIST, 0.0, v, v_slalom, a ,a);

        auto traj1 = StraightTrajectory::create(CurveFactory::getPreDist(turn_type_e::TURN_90), v_slalom, v_slalom, v_slalom, a, a);
        auto traj2 = CurveTrajectory::createAsNoStraght(v_slalom, turn_type_e::TURN_90, (turn_dir_e)SIGN(rot_times));
        auto traj3 = StraightTrajectory::create(CurveFactory::getFolDist(turn_type_e::TURN_90), v_slalom, v_slalom, v_slalom, a, a);

        auto traj4 = StraightTrajectory::create(CurveFactory::getPreDist(turn_type_e::TURN_90), v_slalom, v_slalom, v_slalom, a, a);
        auto traj5 = CurveTrajectory::createAsNoStraght(v_slalom, turn_type_e::TURN_90, (turn_dir_e)SIGN(rot_times));
        auto traj6 = StraightTrajectory::create(CurveFactory::getFolDist(turn_type_e::TURN_90), v_slalom, v_slalom, v_slalom, a, a);

        auto traj7 = StraightTrajectory::createAsWallCenter(0.09 * 15 + m.WALL2MOUSE_CENTER_DIST, v_slalom, v, 0.1, a, a);

        auto traj8 = StopTrajectory::create(1.0);
        m.trajCommander.push(std::move(traj0));
        m.trajCommander.push(std::move(traj1));
        m.trajCommander.push(std::move(traj2));
        m.trajCommander.push(std::move(traj3));
        m.trajCommander.push(std::move(traj4));
        m.trajCommander.push(std::move(traj5));
        m.trajCommander.push(std::move(traj6));
        m.trajCommander.push(std::move(traj7));
        m.trajCommander.push(std::move(traj8));



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
