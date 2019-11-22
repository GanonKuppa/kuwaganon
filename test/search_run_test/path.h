#pragma once
#include "curveFactory.h"
#include "communication.h"
#include <string>

namespace umouse {

    class Path {
      public:
        turn_type_e turn_type;
        uint8_t block_num;
        turn_dir_e turn_dir;

        Path() {
            turn_type = turn_type_e::STRAIGHT;
            block_num = 0;
            turn_dir =  turn_dir_e::NO_TURN;
        }

        Path(turn_type_e turn_type_, uint8_t block_num_, turn_dir_e turn_dir_) {
            turn_type = turn_type_;
            turn_dir = turn_dir_;
            block_num = block_num_;
        }

        Path(const Path& obj) {
            this->block_num = obj.block_num;
            this->turn_dir = obj.turn_dir;
            this->turn_type = obj.turn_type;
        }

        bool operator==(const Path& r) const {
            return (turn_type == r.turn_type) && (block_num == r.block_num) && (turn_dir == r.turn_dir);
        }

        bool operator!=(const Path& r) const {
            return !(*this == r);
        }


        bool isStraightEnd() {
            if(turn_type == turn_type_e::STRAIGHT     ||
                    turn_type == turn_type_e::TURN_180     ||
                    turn_type == turn_type_e::TURN_D2S_45  ||
                    turn_type == turn_type_e::TURN_D2S_135 ||
                    turn_type == turn_type_e::TURN_L_90    ||
                    turn_type == turn_type_e::TURN_90       ) {
                return true;
            } else {
                return false;
            }
        }

        bool isStraightStart() {
            if(turn_type == turn_type_e::STRAIGHT     ||
                    turn_type == turn_type_e::TURN_180     ||
                    turn_type == turn_type_e::TURN_S2D_45  ||
                    turn_type == turn_type_e::TURN_S2D_135 ||
                    turn_type == turn_type_e::TURN_L_90    ||
                    turn_type == turn_type_e::TURN_90       ) {
                return true;
            } else {
                return false;
            }

        }

        bool isDiagonalEnd() {
            if(turn_type == turn_type_e::D_STRAIGHT   ||
                    turn_type == turn_type_e::TURN_S2D_45  ||
                    turn_type == turn_type_e::TURN_S2D_135 ||
                    turn_type == turn_type_e::TURN_D_90    ) {
                return true;
            } else {
                return false;
            }
        }

        bool isDiagonalStart() {
            if(turn_type == turn_type_e::D_STRAIGHT   ||
                    turn_type == turn_type_e::TURN_D2S_45  ||
                    turn_type == turn_type_e::TURN_D2S_135 ||
                    turn_type == turn_type_e::TURN_D_90    ) {
                return true;
            } else {
                return false;
            }
        }

        ~Path() {

        }

        void print() {
            std::string str;
            if(turn_type == turn_type_e::STRAIGHT) str = "STRAIGHT";
            else if(turn_type == turn_type_e::TURN_90) str = "TURN_90";
            else if(turn_type == turn_type_e::TURN_L_90) str = "TURN_L_90";
            else if(turn_type == turn_type_e::TURN_180) str = "TURN_180";
            else if(turn_type == turn_type_e::TURN_S2D_45) str = "TURN_S2D_45";
            else if(turn_type == turn_type_e::TURN_S2D_135) str = "TURN_S2D_135";
            else if(turn_type == turn_type_e::TURN_D_90) str = "TURN_D_90";
            else if(turn_type == turn_type_e::TURN_D2S_45) str = "TURN_D2S_45";
            else if(turn_type == turn_type_e::TURN_D2S_135) str = "TURN_D2S_135";
            else if(turn_type == turn_type_e::D_STRAIGHT) str = "D_STRAIGHT";

            printfAsync("turn_type %s, block_num %d, turn_dir %d\n", str.c_str(), block_num, turn_dir);
        }

    };

}