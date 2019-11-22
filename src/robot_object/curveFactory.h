#pragma once

#include "arcLengthParameterizedCurve.h"

namespace umouse {

    enum class turn_type_e {
        STRAIGHT = 0,
        TURN_90,
        TURN_L_90,
        TURN_180,
        TURN_S2D_45,
        TURN_S2D_135,
        TURN_D_90,
        TURN_D2S_45,
        TURN_D2S_135,
        D_STRAIGHT,
        STOP,
        CIRCULAR
    };

    enum class turn_dir_e {
        CW = -1, NO_TURN = 0, CCW = 1
    };


    class CurveFactory {
      public:
        static ArcLengthParameterizedCurve* create(turn_type_e turn_type_);
        static float getPreDist(turn_type_e turn_type_);
        static float getFolDist(turn_type_e turn_type_);
        static float getPreDistWithOffset(turn_type_e turn_type_, float v);
    };

}


