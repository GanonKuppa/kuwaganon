#pragma once

#include <stdint.h>
#include "communication.h"

#include "timeInterrupt.h"

#include "wallsensor.h"
#include "wheelOdometry.h"
#include "powerTransmission.h"
#include "mouse.h"


#define FULL_PARAM 0


namespace umouse{
    class Logger{
      private:
        
        uint32_t _data_num;
        uint32_t _start_time_ms;
        #if FULL_PARAM        
        const uint32_t _max_data_num = 1200;
        #else
        const uint32_t _max_data_num = 1200;
        #endif

        bool _logging;
      public:
        static Logger& getInstance() {
            static Logger instance;
            return instance;
        }

        void printHeadder();
        void print();
        void start();
        void end();
        void update();

    };




}


