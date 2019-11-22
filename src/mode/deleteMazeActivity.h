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
        };
        void onFinish() {};
    };

}
