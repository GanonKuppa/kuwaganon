#pragma once

#include "baseActivity.h"
#include "communication.h"

namespace umouse {

    class FullAutonomousRunActivity : public BaseActivity {
      protected:
        ELoopStatus loop() {

            return ELoopStatus::CONTINUE;
        };
        void onStart() {
            printfAsync("This is full autonomous activity.\n");
        };
        void onFinish() {};
    };

}
