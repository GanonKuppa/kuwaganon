#pragma once

#include "curveFactory.h"

namespace umouse {

    class TurnParameter {
      public:
        float v_straight;
        float v_d_straight;
        float v_turn_90;
        float v_turn_l_90;
        float v_turn_180;
        float v_turn_d_90;
        float v_turn_45;
        float v_turn_135;
        float a_straight;
        float a_d_straight;

        TurnParameter() {
            v_straight = 0.0;
            v_d_straight = 0.0;
            v_turn_90 = 0.0;
            v_turn_l_90 = 0.0;
            v_turn_180 = 0.0;
            v_turn_d_90 = 0.0;
            v_turn_45 = 0.0;
            v_turn_135 = 0.0;
            a_straight = 0.0;
            a_d_straight = 0.0;
        }

        TurnParameter(float v, float a) {
            v_straight = v;
            v_d_straight = v;
            v_turn_90 = v;
            v_turn_l_90 = v;
            v_turn_180 = v;
            v_turn_d_90 = v;
            v_turn_45 = v;
            v_turn_135 = v;
            a_straight = a;
            a_d_straight = a;
        }

        TurnParameter(float v, float v_turn, float a) {
            v_straight = v;
            v_d_straight = v;
            v_turn_90 = v_turn;
            v_turn_l_90 = v_turn;
            v_turn_180 = v_turn;
            v_turn_d_90 = v_turn;
            v_turn_45 = v_turn;
            v_turn_135 = v_turn;
            a_straight = a;
            a_d_straight = a;
        }

        void set(float v_straight_, float v_d_straight_, float v_turn_90_, float v_turn_l_90_, float v_turn_180_,
                 float v_turn_d_90_, float v_turn_45_, float v_turn_135_, float a_straight_, float a_d_straight_) {
            v_straight = v_straight_;
            v_d_straight = v_d_straight_;
            v_turn_90 = v_turn_90_;
            v_turn_l_90 = v_turn_l_90_;
            v_turn_180 = v_turn_180_;
            v_turn_d_90 = v_turn_d_90_;
            v_turn_45 = v_turn_45_;
            v_turn_135 = v_turn_135_;
            a_straight = a_straight_;
            a_d_straight = a_d_straight_;
        }

        void set(float v, float a) {
            v_straight = v;
            v_d_straight = v;
            v_turn_90 = v;
            v_turn_l_90 = v;
            v_turn_180 = v;
            v_turn_d_90 = v;
            v_turn_45 = v;
            v_turn_135 = v;
            a_straight = a;
            a_d_straight = a;
        }

        void set(float v, float v_turn, float a) {
            v_straight = v;
            v_d_straight = v;
            v_turn_90 = v_turn;
            v_turn_l_90 = v_turn;
            v_turn_180 = v_turn;
            v_turn_d_90 = v_turn;
            v_turn_45 = v_turn;
            v_turn_135 = v_turn;
            a_straight = a;
            a_d_straight = a;
        }

        TurnParameter(float v_straight_, float v_d_straight_, float v_turn_90_, float v_turn_l_90_, float v_turn_180_,
                      float v_turn_d_90_, float v_turn_45_, float v_turn_135_, float a_straight_, float a_d_straight_) {
            v_straight = v_straight_;
            v_d_straight = v_d_straight_;
            v_turn_90 = v_turn_90_;
            v_turn_l_90 = v_turn_l_90_;
            v_turn_180 = v_turn_180_;
            v_turn_d_90 = v_turn_d_90_;
            v_turn_45 = v_turn_45_;
            v_turn_135 = v_turn_135_;
            a_straight = a_straight_;
            a_d_straight = a_d_straight_;
        }

        float getTurnV(turn_type_e turn_type) {
            switch (turn_type) {
                case turn_type_e::STRAIGHT:
                    return v_straight;
                case turn_type_e::TURN_90:
                    return v_turn_90;
                case turn_type_e::TURN_L_90:
                    return v_turn_l_90;
                case turn_type_e::TURN_180:
                    return v_turn_180;
                case turn_type_e::TURN_S2D_45:
                    return v_turn_45;
                case turn_type_e::TURN_S2D_135:
                    return v_turn_135;
                case turn_type_e::TURN_D2S_45:
                    return v_turn_45;
                case turn_type_e::TURN_D2S_135:
                    return v_turn_135;
                case turn_type_e::TURN_D_90:
                    return v_turn_d_90;
                case turn_type_e::D_STRAIGHT:
                    return v_d_straight;
                default:
                    return 0.0;
            }
        }
    };

}