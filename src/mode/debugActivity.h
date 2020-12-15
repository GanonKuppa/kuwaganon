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
#include "logger.h"
#include "wallsensor.h"
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
            auto activity = ActivityFactory::createSubModeSelect();
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
            float y = 0.09f/2.0f;
            float ang = 90.0;

            int8_t rot_times = -2;
            m.posEsti.reset(x,y,ang);
            m.trajCommander.reset(x,y,ang);
            m.ctrlMixer.reset();
            waitmsec(3000);
            ParameterManager& pm = ParameterManager::getInstance();



            if(mode == 1) {
                Logger& logger = umouse::Logger::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                FcLed& fcled = FcLed::getInstance();

                float v_ = 0.5;
                float a_ = 3.0;
                auto traj0 = StraightTrajectory::createAsWallCenter(0.045f * 10.0f, 0.0f, v_, 0.05f, a_, a_);
                auto traj1 = StopTrajectory::create(0.5);
                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));
                
                fcled.turn(0,0,1);
                logger.start();
                while(!m.trajCommander.empty()) {
                    waitmsec(1);                    
                }
                logger.end();
                
                fcled.turn(0,1,0);
                while( ws.getAheadOnTime() < 0.5) {
                    waitmsec(200);                    
                    SEB();                
                }
                fcled.turn(0,1,1);
                logger.print();

            } else if(mode == 2) {
                Logger& logger = umouse::Logger::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                FcLed& fcled = FcLed::getInstance();
                float v_slalom = pm.v_search_run;
                float a = pm.a_search_run;

                auto traj0 = StraightTrajectory::createAsWallCenter(0.045f, 0.0f, v_slalom, v_slalom, a, a);
                m.trajCommander.push(std::move(traj0));
                slalom90(turn_dir_e::CW);
                auto traj1 = StraightTrajectory::createAsWallCenter(0.045f, v_slalom, v_slalom, 0.05f, a, a);
                m.trajCommander.push(std::move(traj1));

                auto traj2 = StopTrajectory::create(1.0);
                m.trajCommander.push(std::move(traj2));




                fcled.turn(0,0,1);
                logger.start();
                while(!m.trajCommander.empty()) {
                    waitmsec(1);                    
                }
                logger.end();
                
                fcled.turn(0,1,0);
                while( ws.getAheadOnTime() < 0.5) {
                    waitmsec(200);                    
                    SEB();                
                }
                fcled.turn(0,1,1);
                logger.print();




            } else if(mode == 3) {
                Logger& logger = umouse::Logger::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                FcLed& fcled = FcLed::getInstance();

                auto traj0 = SpinTurnTrajectory::create(90.0f, pm.spin_ang_v, pm.spin_ang_a);
                auto traj1 = StopTrajectory::create(0.3);
                auto traj2 = SpinTurnTrajectory::create(180.0f, pm.spin_ang_v, pm.spin_ang_a);
                auto traj3 = StopTrajectory::create(0.3);
                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));
                m.trajCommander.push(std::move(traj2));
                m.trajCommander.push(std::move(traj3));


                fcled.turn(0,0,1);
                logger.start();
                while(!m.trajCommander.empty()) {
                    waitmsec(1);                    
                }
                logger.end();
                
                fcled.turn(0,1,0);
                while( ws.getAheadOnTime() < 0.5) {
                    waitmsec(200);                    
                    SEB();                
                }
                fcled.turn(0,1,1);
                logger.print();


            } else if(mode == 4) {
                Logger& logger = umouse::Logger::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                FcLed& fcled = FcLed::getInstance();
                PowerTransmission& pt = PowerTransmission::getInstance();
                UMouse& m = UMouse::getInstance();

                m.direct_duty_set_enable = true;
                pt.setDuty(0.1, 0.1);


                fcled.turn(0,0,1);
                logger.start();
                waitmsec(300);
                pt.setDuty(0.0, 0.0);
                waitmsec(300);
                m.direct_duty_set_enable = false;
                logger.end();
                
                fcled.turn(0,1,0);
                while( ws.getAheadOnTime() < 0.5) {
                    waitmsec(200);                    
                    SEB();                
                }
                fcled.turn(0,1,1);
                logger.print();



            } else if(mode == 5) {
                Logger& logger = umouse::Logger::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                FcLed& fcled = FcLed::getInstance();

                float v_ = pm.test_run_v;
                float a_ = pm.test_run_a;
                auto traj0 = StraightTrajectory::createAsWallCenter(0.09f * 3.0f, 0.0f, v_, 0.3, a_, a_);
                auto traj1 = StraightTrajectory::createAsWallCenter(0.045f, 0.3f, 0.3f, 0.3f, a_, a_);
                auto traj2 = StraightTrajectory::createAsWallCenter(0.045f, 0.3f, 0.3f, 0.05f, a_, a_);
                auto traj3 = StopTrajectory::create(0.5);
                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));
                m.trajCommander.push(std::move(traj2));
                m.trajCommander.push(std::move(traj3));

                fcled.turn(0,0,1);
                logger.start();
                while(!m.trajCommander.empty()) {
                    waitmsec(1);                    
                }
                logger.end();
                
                fcled.turn(0,1,0);
                while( ws.getAheadOnTime() < 0.5) {
                    waitmsec(200);                    
                    SEB();                
                }
                fcled.turn(0,1,1);
                logger.print();




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
