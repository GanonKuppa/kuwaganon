#pragma once

#include "baseActivity.h"
#include "communication.h"
#include "timer.h"
#include "mouse.h"
#include "sound.h"

namespace umouse {
    class DeleteMazeActivity : public BaseActivity {
      protected:
        ELoopStatus loop() {

            return ELoopStatus::FINISH;
        };
        void onStart() {
            printfAsync("■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■.\n");
            printfAsync("This is delete maze activity.\n");
            printfAsync("RG mode\n");


            Intent* intent = new Intent();
            intent->uint8_t_param["SUB_MODE_NUM"] = 2;
            auto activity = ActivityFactory::createSubModeSelect();
            activity->start(intent);
            printfAsync("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
            uint8_t sub_mode_result = intent->uint8_t_param["SUB_MODE"];
            delete intent;
            if(sub_mode_result == 0) return;
            else if(sub_mode_result == 1){
                printfAsync("迷路データ消去\n");

                UMouse& m = UMouse::getInstance();
                m.maze.init();
                m.maze.writeMazeData2Flash();
                SEC();
                waitmsec(200);
                SEC();
                waitmsec(200);
                SEC();
                waitmsec(1000);
            }


        };
        void onFinish() {};
    };

}
