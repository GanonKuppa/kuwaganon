#pragma once

#include "baseActivity.h"
#include "communication.h"
#include "gamepad.h"
#include "powerTransmission.h"
#include "myUtil.h"
#include "mouse.h"
#include "sound.h"
#include "parameterManager.h"
#include "ICM20602.h"
//#include "adis16470.h"
//#include "activityFactory.h"

using namespace std;

namespace umouse {

    class RadioControlActivity : public BaseActivity {
      protected:
        ELoopStatus loop() {
            //std::unique_ptr<BaseActivity> activity = ActivityFactory::cteateModeSelect();
            Gamepad& gamepad = Gamepad::getInstance();
            PowerTransmission& pt = PowerTransmission::getInstance();
            ParameterManager& pm = ParameterManager::getInstance();
            UMouse& m = UMouse::getInstance();
            float limit = 0.0;
            if(gamepad.LB > 1) {
                limit = 0.2;
            } else limit = 0.1;

            //float l_duty = constrain( (gamepad.R3D_y/128.0 + gamepad.R3D_x/128.0) * limit ,-limit, limit);
            //float r_duty = constrain( (gamepad.R3D_y/128.0 - gamepad.R3D_x/128.0) * limit ,-limit, limit);
            float l_duty = constrain( (gamepad.R3D_y/128.0 ) * limit ,-limit, limit);
            float r_duty = constrain( (gamepad.R3D_y/128.0 ) * limit ,-limit, limit);


            if(m.trajCommander.empty() == true) {
                pt.setDuty(l_duty, r_duty);
            }

            if(gamepad.BACK == 1) {
                pt.debug_duty_l();
            }

            if(gamepad.BACK == 1) {
                pt.debug_duty_r();
            }


            if(gamepad.cross_x == 1) {
                m.direct_duty_set_enable = false;
                SE_CONFIRM();
                auto traj = SpinTurnTrajectory::create(90.0f, pm.spin_ang_v, pm.spin_ang_a);
                m.trajCommander.push(std::move(traj));
                waitmsec(1000);
                m.direct_duty_set_enable = true;
            }

            if(gamepad.cross_x == -1) {
                m.direct_duty_set_enable = false;
                SE_CONFIRM();
                auto traj = SpinTurnTrajectory::create(-90.0f, pm.spin_ang_v, pm.spin_ang_a);
                m.trajCommander.push(std::move(traj));
                waitmsec(1000);
                m.direct_duty_set_enable = true;
            }

            if(gamepad.cross_y == 1) {
                m.direct_duty_set_enable = false;
                float block = 1.0f;
                if(gamepad.LB > 1) {
                    block = 2.0f;
                } 
                if(gamepad.RB > 1) {
                    block = 15.0f;
                } 

                SE_CONFIRM();
                float v_slalom = pm.v_search_run;
                float a = pm.a_search_run;
                auto traj0 = StraightTrajectory::createAsWallCenter(0.09f*block, 0.0f, v_slalom, 0.0f, a, a);
                m.trajCommander.push(std::move(traj0));
                while(!m.trajCommander.empty()) waitmsec(100);
                m.direct_duty_set_enable = true;
            }

            if(gamepad.cross_y == -1) {
                m.direct_duty_set_enable = false;
                SE_CONFIRM();
                auto traj = SpinTurnTrajectory::create(180.0f, pm.spin_ang_v, pm.spin_ang_a);
                m.trajCommander.push(std::move(traj));
                waitmsec(1000);
                m.direct_duty_set_enable = true;
            }

            if(gamepad.X > 50 && gamepad.X < 150 ) {

                SEA();
                ICM20602& icm = ICM20602::getInstance();
                //adis16470& adis = adis16470::getInstance();
                icm.calibOmegaOffset(800);
                icm.calibAccOffset(800);
                //adis.calibOmegaOffset(1600);
                float x = 0.09f/2.0f;
                float y = 0.09f/2.0f - m.WALL2MOUSE_CENTER_DIST;
                float ang = 90.0;
                m.posEsti.reset(x,y,ang);
                m.trajCommander.reset(x,y,ang);
                m.ctrlMixer.reset();

                SEF();
                waitmsec(100);
            }


            if(gamepad.B > 2800 ) {
                SE_CONFIRM();
                return ELoopStatus::FINISH;
            } else {
                return ELoopStatus::CONTINUE;
            }


        };
        void onStart() {
            UMouse& m = UMouse::getInstance();
            m.direct_duty_set_enable = true;
            printfAsync("This is radio control activity.\n");
        };
        void onFinish() {
            UMouse& m = UMouse::getInstance();
            m.direct_duty_set_enable = false;
        };
    };

}
