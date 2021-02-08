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
        uint8_t  _skip_mod;
        #if FULL_PARAM        
        const uint32_t _max_data_num = 1200;
        #else
        const uint32_t _max_data_num = 3000;
        #endif

        bool _logging;
      public:
        static Logger& getInstance() {
            static Logger instance;
            return instance;
        }

        void printHeadder();
        void print();
        void start(uint8_t skip_mod = 0);
        void end();
        void update();

    };




}


