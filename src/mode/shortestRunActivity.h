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
#include "wallsensor.h"
#include "logger.h"

namespace umouse {

    class ShortestRunActivity : public BaseActivity {
      protected:
        bool pre_in_read_wall_area;
        Coor2D<uint16_t> pre_read_wall_coor;

        ELoopStatus loop() {
            UMouse& m = UMouse::getInstance();

            waitmsec(1);

            
            if(m.trajCommander.empty()== true) return ELoopStatus::FINISH;
            else return ELoopStatus::CONTINUE;
        }

        void onStart() {
            printfAsync("■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■.\n");
            printfAsync("This is search run activity.\n");
            uint8_t run_mode = 0;
            uint8_t param_mode = 0;
            {
                Intent* intent = new Intent();
                intent->uint8_t_param["SUB_MODE_NUM"] = 6;
                auto activity = ActivityFactory::createSubModeSelect();
                activity->start(intent);
                printfAsync("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
                run_mode = intent->uint8_t_param["SUB_MODE"];
                delete intent;
                if(run_mode == 0) return;

            }

            {
                Intent* intent = new Intent();
                intent->uint8_t_param["SUB_MODE_NUM"] = 6;
                intent->uint16_t_param["LED_ON_MSEC"] = 125;
                intent->uint16_t_param["LED_OFF_MSEC"] = 125;
                intent->int8_t_param["NOTE_PITCH_OFSET"] = 2;
                auto activity = ActivityFactory::createSubModeSelect();
                activity->start(intent);
                printfAsync("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
                param_mode = intent->uint8_t_param["SUB_MODE"];
                delete intent;
                if(param_mode == 0) return;
            }


            UMouse& m = UMouse::getInstance();
            m.setMode(ERunMode::SHORTEST_RUN);
            ParameterManager& pm = ParameterManager::getInstance();

            waitmsec(1000);
            ICM20602& icm = ICM20602::getInstance();
            //adis16470& adis = adis16470::getInstance();
            icm.calibOmegaOffset(800);
            icm.calibAccOffset(800);
            //adis.calibOmegaOffset(800);

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
            else if(param_mode == 1) setParamFromFlash(0, turn_p);
            else if(param_mode == 2) setParamFromFlash(1, turn_p);
            else if(param_mode == 3) setParamFromFlash(2, turn_p);
            else if(param_mode == 4) setParamFromFlash(3, turn_p);
            else if(param_mode == 5) setParamFromFlash(4, turn_p);
            else turn_p.set(0.2, 0.2, 2.0);

            std::vector<Path> path_vec;

            if(run_mode == 0 ) {
                return;
            } else if(run_mode == 1 ) {
                makeMinStepPath(pm.goal_x, pm.goal_y, m.maze, path_vec);
                translatePathSpin(path_vec);
                HF_playPathSpin(turn_p, path_vec, m.trajCommander);
            } else if(run_mode == 2) {
                makeMinStepPath(pm.goal_x, pm.goal_y, m.maze, path_vec);
                translatePathDiagonal(path_vec);
                HF_playPathSpinDiagonal(turn_p, path_vec, m.trajCommander);
            } else if(run_mode == 3) {
                makeMinStepPath(pm.goal_x, pm.goal_y, m.maze, path_vec);
                translatePath90Deg(path_vec);
                HF_playPath(turn_p, path_vec, m.trajCommander);
            } else if(run_mode == 4) {
                makeMinStepPath(pm.goal_x, pm.goal_y, m.maze, path_vec);
                translatePathLong(path_vec);
                HF_playPath(turn_p, path_vec, m.trajCommander);
            } else if(run_mode == 5) {
                makeFastestDiagonalPath(500, turn_p, pm.goal_x, pm.goal_y, m.maze, path_vec);
                HF_playPath(turn_p, path_vec, m.trajCommander);
            }
            printfAsync("--- makeMinStepPath ----\n");
            printPath(path_vec);


            // ゴール区画に微妙に入り切れないことを防ぐための処理
            //auto traj0 = StraightTrajectory::create(0.09f, 0.1f);
            //m.trajCommander.push(std::move(traj0));

            pre_read_wall_coor.set(255, 255);
            pre_in_read_wall_area = false;

            Logger& logger = umouse::Logger::getInstance();
            logger.start(4);


        }
        void onFinish() {
            Logger& logger = umouse::Logger::getInstance();
            logger.end();
        }

      private:
        void setParamFromFlash(uint8_t num, TurnParameter& turn_p) {
            ParameterManager& pm = ParameterManager::getInstance();
            float v, v_d, v_90, v_l90, v_180, v_d90, v_45, v_135, a, a_diag;
            if(num == 0) {
                v = pm.shortest_0_v;
                v_d = pm.shortest_0_v_d;
                v_90 = pm.shortest_0_v_90;
                v_l90 = pm.shortest_0_v_l90;
                v_180 = pm.shortest_0_v_180;
                v_d90 = pm.shortest_0_v_d90;
                v_45 = pm.shortest_0_v_45;
                v_135 = pm.shortest_0_v_135;
                a = pm.shortest_0_a;
                a_diag = pm.shortest_0_a_diag;
            } else if(num == 1) {
                v = pm.shortest_1_v;
                v_d = pm.shortest_1_v_d;
                v_90 = pm.shortest_1_v_90;
                v_l90 = pm.shortest_1_v_l90;
                v_180 = pm.shortest_1_v_180;
                v_d90 = pm.shortest_1_v_d90;
                v_45 = pm.shortest_1_v_45;
                v_135 = pm.shortest_1_v_135;
                a = pm.shortest_1_a;
                a_diag = pm.shortest_1_a_diag;
            } else if(num == 2) {
                v = pm.shortest_2_v;
                v_d = pm.shortest_2_v_d;
                v_90 = pm.shortest_2_v_90;
                v_l90 = pm.shortest_2_v_l90;
                v_180 = pm.shortest_2_v_180;
                v_d90 = pm.shortest_2_v_d90;
                v_45 = pm.shortest_2_v_45;
                v_135 = pm.shortest_2_v_135;
                a = pm.shortest_2_a;
                a_diag = pm.shortest_2_a_diag;
            } else if(num == 3) {
                v = pm.shortest_3_v;
                v_d = pm.shortest_3_v_d;
                v_90 = pm.shortest_3_v_90;
                v_l90 = pm.shortest_3_v_l90;
                v_180 = pm.shortest_3_v_180;
                v_d90 = pm.shortest_3_v_d90;
                v_45 = pm.shortest_3_v_45;
                v_135 = pm.shortest_3_v_135;
                a = pm.shortest_3_a;
                a_diag = pm.shortest_3_a_diag;
            } else if(num == 4) {
                v = pm.shortest_4_v;
                v_d = pm.shortest_4_v_d;
                v_90 = pm.shortest_4_v_90;
                v_l90 = pm.shortest_4_v_l90;
                v_180 = pm.shortest_4_v_180;
                v_d90 = pm.shortest_4_v_d90;
                v_45 = pm.shortest_4_v_45;
                v_135 = pm.shortest_4_v_135;
                a = pm.shortest_4_a;
                a_diag = pm.shortest_4_a_diag;
            } else {
                v = 0.35f;
                v_d = 0.35f;
                v_90 = 0.35f;
                v_l90 = 0.35f;
                v_180 = 0.35f;
                v_d90 = 0.35f;
                v_45 = 0.35f;
                v_135 = 0.35f;
                a = 1.0f;
                a_diag = 1.0f;
            }
            turn_p.set(v, v_d, v_90, v_l90, v_180, v_d90, v_45, v_135, a, a_diag);
        }

    };

}
