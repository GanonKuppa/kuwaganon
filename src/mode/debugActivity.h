#pragma once

#include "baseActivity.h"
#include "communication.h"
#include "timer.h"
#include "mouse.h"
#include "trajectory.h"
#include "fcLed.h"
#include "sound.h"
#include "ICM20602.h"
#include "powerTransmission.h"
#include "wheelOdometry.h"
#include "batVoltageMonitor.h"
//#include "adis16470.h"


namespace umouse {

    enum class EDebagMode : uint8_t {
        BACK_MODE_SELECT = 0,
        SLOW_STRAIGHT_DEBUG = 1,
        SLOW_STRAIGHT_DEBUG_WALL = 2,
        FAST_STRAIGHT_DEBUG = 3,
        FAST_STRAIGHT_DEBUG_WALL = 4,
        SPIN_TURN_DEBUG = 5
    };



    class DebugActivity : public BaseActivity {
      protected:

        void onStart() {
            printfAsync("■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■.\n");
            printfAsync("This is debug activity.\n");
            uint8_t mode;
            Intent* intent = new Intent();
            intent->uint8_t_param["SUB_MODE_NUM"] = 8;
            auto activity = ActivityFactory::cteateSubModeSelect();
            activity->start(intent);
            printfAsync("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
            mode = intent->uint8_t_param["SUB_MODE"];
            delete intent;
            if(mode == 0) return;


            UMouse& m = UMouse::getInstance();

            ICM20602& icm = ICM20602::getInstance();
            //adis16470& adis = adis16470::getInstance();
            icm.calibOmegaOffset(800);
            icm.calibAccOffset(800);
            //adis.calibOmegaOffset(1600);
            SEF();

            float x = 0.09f/2.0f;
            float y = 0.09f/2.0f - m.WALL2MOUSE_CENTER_DIST;
            float ang = 90.0;

            int8_t rot_times = -2;
            m.posEsti.reset(x,y,ang);
            m.trajCommander.reset(x,y,ang);
            m.ctrlMixer.reset();
            waitmsec(3000);
            ParameterManager& pm = ParameterManager::getInstance();



            if(mode == 1) {
                PowerTransmission& pt = PowerTransmission::getInstance();
                FcLed& fcled = FcLed::getInstance();
                WheelOdometry &wodo = WheelOdometry::getInstance();
                auto traj = StopTrajectory::createAsDirectDutySet(500.0);
                m.trajCommander.push(std::move(traj));
                for(int i=-50;i<50;i++){
                    fcled.turn(0,0,0);
                    pt.setDuty(i*0.01, i*0.01);
                    waitmsec(3000);
                    for(int j=0;j<50;j++){
                        float vol =  BatVoltageMonitor::getInstance().bat_vol * pt.getDuty_R();
                        printfSync("%d, %f, %f, %f\n",i , vol, wodo.getAveV_L(), wodo.getAveV_R());
                        waitmsec(1);
                    }
                    pt.setDuty(0.0, 0.0);
                    waitmsec(500);
                }

            } else if(mode == 2) {
                float v = pm.v_search_run;
                float a = pm.a_search_run;
                auto traj = StraightTrajectory::createAsWallCenter(0.09 * 15 + m.WALL2MOUSE_CENTER_DIST, 0.0, v, 0.1f, a, a);
                m.trajCommander.push(std::move(traj));
            } else if(mode == 3) {
                float v_slalom = pm.v_search_run;
                float a = pm.a_search_run;
                auto traj0 = StraightTrajectory::createAsWallCenter(0.045f + m.WALL2MOUSE_CENTER_DIST, 0.0, v_slalom, v_slalom, a, a);
                auto traj1 = StraightTrajectory::create(CurveFactory::getPreDistWithOffset(turn_type_e::TURN_90, v_slalom), v_slalom, v_slalom, v_slalom, a, a);
                auto traj2 = CurveTrajectory::createAsNoStraght(v_slalom, turn_type_e::TURN_90, (turn_dir_e)SIGN(rot_times));
                auto traj3 = StraightTrajectory::create(CurveFactory::getFolDist(turn_type_e::TURN_90), v_slalom, v_slalom, v_slalom, a, a);
                auto traj4 = StraightTrajectory::createAsWallCenter(0.045f, v_slalom, v_slalom, 0.1f, a, a);
                auto traj5 = StopTrajectory::create(1.0);

                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));
                m.trajCommander.push(std::move(traj2));
                m.trajCommander.push(std::move(traj3));
                m.trajCommander.push(std::move(traj4));
                m.trajCommander.push(std::move(traj5));
            } else if(mode == 4) {
                for(int i=0; i<20; i++) {
                    auto traj1 = SpinTurnTrajectory::create(90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj2 = SpinTurnTrajectory::create(90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj3 = StopTrajectory::create(0.5);
                    auto traj4 = SpinTurnTrajectory::create(90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj5 = SpinTurnTrajectory::create(90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj6 = StopTrajectory::create(0.5);
                    m.trajCommander.push(std::move(traj1));
                    m.trajCommander.push(std::move(traj2));
                    m.trajCommander.push(std::move(traj3));
                    m.trajCommander.push(std::move(traj4));
                    m.trajCommander.push(std::move(traj5));
                    m.trajCommander.push(std::move(traj6));
                }
            } else if(mode == 5) {
                for(int i=0; i<20; i++) {
                    auto traj1 = SpinTurnTrajectory::create(-90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj2 = SpinTurnTrajectory::create(-90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj3 = StopTrajectory::create(0.5);
                    auto traj4 = SpinTurnTrajectory::create(-90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj5 = SpinTurnTrajectory::create(-90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj6 = StopTrajectory::create(0.5);
                    m.trajCommander.push(std::move(traj1));
                    m.trajCommander.push(std::move(traj2));
                    m.trajCommander.push(std::move(traj3));
                    m.trajCommander.push(std::move(traj4));
                    m.trajCommander.push(std::move(traj5));
                    m.trajCommander.push(std::move(traj6));
                }



            } else if(mode == 6) {
                float v = pm.v_search_run;
                float a = pm.a_search_run;
                auto traj1 = StraightTrajectory::createAsWallCenter(0.09f * 8.0f, 0.0, v, 0.0f, a, a );
                auto traj2 = StopTrajectory::create(4.0);
                m.trajCommander.push(std::move(traj1));
                m.trajCommander.push(std::move(traj2));

            } else if(mode == 7) {
                SEF();
                icm.calibOmegaOffset(800);
                icm.calibAccOffset(800);
                //adis.calibOmegaOffset(1600);

                float v_slalom = pm.v_search_run;
                float a = pm.a_search_run;
                auto traj0 = StraightTrajectory::createAsWallCenter(0.045f, 0.0, v_slalom, v_slalom, a, a);
                m.trajCommander.push(std::move(traj0));

                for(int i=0; i<100; i++) {
                    straight_n_blocks(4.0f);
                    slalom90(turn_dir_e::CW);
                }

                auto traj1 = StraightTrajectory::createAsWallCenter(0.045f, v_slalom, v_slalom, 0.1f, a, a);
                m.trajCommander.push(std::move(traj1));

                auto traj2 = StopTrajectory::create(1.0);
                m.trajCommander.push(std::move(traj2));

            }


        };

        void slalom90(turn_dir_e dir) {
            ParameterManager& pm = ParameterManager::getInstance();
            float v_slalom = pm.v_search_run;
            float a = pm.a_search_run;
            UMouse& m = UMouse::getInstance();
            auto traj1 = StraightTrajectory::create(CurveFactory::getPreDistWithOffset(turn_type_e::TURN_90, v_slalom), v_slalom, v_slalom, v_slalom, a, a);
            auto traj2 = CurveTrajectory::createAsNoStraght(v_slalom, turn_type_e::TURN_90, dir);
            auto traj3 = StraightTrajectory::create(CurveFactory::getFolDist(turn_type_e::TURN_90), v_slalom, v_slalom, v_slalom, a, a);
            m.trajCommander.push(std::move(traj1));
            m.trajCommander.push(std::move(traj2));
            m.trajCommander.push(std::move(traj3));
        }

        void straight_n_blocks(float n_blocks) {
            ParameterManager& pm = ParameterManager::getInstance();
            float v_slalom = pm.v_search_run;
            float v_max = pm.test_run_v;
            float a = pm.test_run_a;
            UMouse& m = UMouse::getInstance();
            auto traj0 = StraightTrajectory::createAsWallCenter(0.09f * n_blocks, v_slalom, v_max, v_slalom, a, a);
            m.trajCommander.push(std::move(traj0));
        }


        ELoopStatus loop() {
            FcLed& fcled = FcLed::getInstance();

            UMouse& m = UMouse::getInstance();

            if(m.trajCommander.empty() == true) {
                SEB();
                return ELoopStatus::FINISH;
            } else {
                return ELoopStatus::CONTINUE;
            }
        };
        void onFinish() {};
    };

}
