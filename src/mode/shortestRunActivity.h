#pragma once

#include "baseActivity.h"
#include "communication.h"
#include "fcled.h"
#include "timer.h"
#include "activityFactory.h"
#include "pathCalculation.h"
#include "trajectory.h"
#include "trajectoryCommander.h"
#include "parameterManager.h"
#include "ICM20602.h"
#include "adis16470.h"
#include <vector>

namespace umouse {

    class ShortestRunActivity : public BaseActivity {
    protected:
        ELoopStatus loop() {
            waitmsec(100);
            UMouse &m = UMouse::getInstance();
            if(m.trajCommander.empty()== true) return ELoopStatus::FINISH;
            else return ELoopStatus::CONTINUE;
        };
        void onStart() {
            printfAsync("■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■.\n");
            printfAsync("This is search run activity.\n");
            uint8_t run_mode = 0;
            uint8_t param_mode = 0;
            {
                Intent *intent = new Intent();
                intent->uint8_t_param["SUB_MODE_NUM"] = 6;
                auto activity = ActivityFactory::cteateSubModeSelect();
                activity->start(intent);
                printfAsync("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
                run_mode = intent->uint8_t_param["SUB_MODE"];
                if(run_mode == 0) return;

                delete intent;
            }

            {
                Intent *intent = new Intent();
                intent->uint8_t_param["SUB_MODE_NUM"] = 8;
                intent->uint16_t_param["LED_ON_MSEC"] = 125;
                intent->uint16_t_param["LED_OFF_MSEC"] = 125;
                intent->int8_t_param["NOTE_PITCH_OFSET"] = 2;
                auto activity = ActivityFactory::cteateSubModeSelect();
                activity->start(intent);
                printfAsync("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
                param_mode = intent->uint8_t_param["SUB_MODE"];
                if(param_mode == 0) return;
                delete intent;
            }


            UMouse &m = UMouse::getInstance();
            ParameterManager &pm = ParameterManager::getInstance();

            waitmsec(1000);
            ICM20602 &icm = ICM20602::getInstance();
            adis16470 &adis = adis16470::getInstance();
            icm.calibOmegaOffset(800);
            icm.calibAccOffset(800);
            adis.calibOmegaOffset(800);

            float x = 0.09f/2.0f;
            float y = 0.09f/2.0f - m.WALL2MOUSE_CENTER_DIST;
            float ang = 90.0;
            m.posEsti.reset(x,y,ang);
            m.trajCommander.reset(x,y,ang);
            m.ctrlMixer.reset();
            m.goal.set(pm.goal_x, pm.goal_y);
            m.start.set(0, 0);
            m.coor.set(0, 0);


            TurnParameter turn_p;

            if (param_mode == 0) return;
            //                                  v   turn_v a
            else if(param_mode == 1) turn_p.set(0.36, 0.36, 4.0);
            else if(param_mode == 2) turn_p.set(0.6, 0.36, 4.0);
            //                                  v    v_d  90    l90   180  d90  45   135  a    a_diag
            else if(param_mode == 3) turn_p.set(1.0, 0.45, 0.36, 0.4, 0.4, 0.4, 0.4, 0.4, 5.0, 3.0);
            else if(param_mode == 4) turn_p.set(1.5, 0.45, 0.36, 0.4, 0.4, 0.4, 0.4, 0.4, 5.0, 3.0);
            else if(param_mode == 5) turn_p.set(2.0, 0.45, 0.36, 0.4, 0.4, 0.4, 0.4, 0.4, 6.0, 3.0);
            else if(param_mode == 6) turn_p.set(2.5, 0.45, 0.36 , 0.4, 0.4, 0.4, 0.4, 0.4, 6.0, 3.0);
            else if(param_mode == 7) turn_p.set(3.0, 0.45, 0.36 , 0.4, 0.4, 0.4, 0.4, 0.4, 6.0, 3.0);

            std::vector<Path> path_vec;
            makeMinStepPath(pm.goal_x, pm.goal_y, m.maze, path_vec);
            printfAsync("--- makeMinStepPath ----\n");
            printPath(path_vec);


            if(run_mode == 0 ){
                 return;
            }
            else if(run_mode == 1 ){
                translatePathSpin(path_vec);
                HF_playPathSpin(turn_p, path_vec, m.trajCommander);
            }
            else if(run_mode == 2){
                translatePathDiagonal(path_vec);
                HF_playPathSpinDiagonal(turn_p, path_vec, m.trajCommander);
            }
            else if(run_mode == 3){
                translatePath90Deg(path_vec);
                HF_playPath(turn_p, path_vec, m.trajCommander);
            }
            else if(run_mode == 4){
                translatePathLong(path_vec);
                HF_playPath(turn_p, path_vec, m.trajCommander);
            }
            else if(run_mode == 5){
                translatePathDiagonal(path_vec);
                HF_playPath(turn_p, path_vec, m.trajCommander);
            }
            // ゴール区画に微妙に入り切れないことを防ぐための処理
            auto traj0 = StraightTrajectory::create(0.09f, 0.1f);
            m.trajCommander.push(std::move(traj0));    

        };
        void onFinish() { };
    };

}
