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
        #if FULL_PARAM        
        const uint32_t _max_data_num = 1000;
        #else
        const uint32_t _max_data_num = 2000;
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


