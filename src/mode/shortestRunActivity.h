#pragma once

#include "baseActivity.h"
#include "communication.h"
#include "fcled.h"
#include "timer.h"
#include "activityFactory.h"
#include "pathCalculation.h"
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

            Intent *intent = new Intent();
            intent->int8_t_param["SUB_MODE_NUM"] = 8;
            auto activity = ActivityFactory::cteateSubModeSelect();
            activity->start(intent);
            printfAsync("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
            uint8_t mode = intent->uint8_t_param["SUB_MODE"];
            delete intent;
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

            if (mode == 0) return;
            else if(mode == 1) turn_p.set(0.6, 0.2, 3.0);
            else if(mode == 2) turn_p.set(0.6, 0.2 ,3.0);
            else if(mode == 3) turn_p.set(0.6, 0.3 ,3.0);
            else if(mode == 4) turn_p.set(2.0, 0.35, 3.0);
            else if(mode == 5) turn_p.set(2.5, 0.35, 6.0);
            else if(mode == 6) turn_p.set(3.0, 0.35, 8.0);
            else if(mode == 7) turn_p.set(3.0, 0.35 ,10.0);



            std::vector<Path> path_vec;
            makeMinStepPath(pm.goal_x, pm.goal_y, m.maze, path_vec);
            printfAsync("--- makeMinStepPath ----\n");
            printPath(path_vec);
            printfAsync("--- jointStraightPath ----\n");
            jointStraightPath(path_vec);

            if(mode == 1 ) HF_playPathPivot(turn_p, path_vec);
            else if(mode == 2 || mode == 3 || mode == 4 || mode == 5 || mode == 6 || mode == 7) HF_playPath(turn_p, path_vec);


        };
        void onFinish() {};
    };

}
