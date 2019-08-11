#pragma once

#include "intent.h"
#include "timer.h"



namespace umouse{

class BaseActivity{
public:
    void start(Intent* intent_){
        lower_limit_loop_msec = 1;
        intent = intent_;
        onStart();
        while(1){
            startTimeuCount_sub();
            uint32_t start_msec = getElapsedMsec();
            if(loop() == ELoopStatus::FINISH) break;
            uint32_t end_msec = getElapsedMsec();
            uint32_t elapsed_msec = end_msec - start_msec;
            int32_t wait_msec = lower_limit_loop_msec - elapsed_msec;
            //if(wait_msec > 0) waitmsec(wait_msec);
            endTimeuCount_sub();
        };
        onFinish();
    };

    void start(){
        start(nullptr);
    };

    virtual ~BaseActivity(){};

protected:
    enum class ELoopStatus{
        CONTINUE = 1,
        FINISH = 0
    };

    virtual ELoopStatus loop() = 0;
    virtual void onStart() = 0;
    virtual void onFinish() = 0;


    uint32_t lower_limit_loop_msec;
    Intent* intent;

};

}
