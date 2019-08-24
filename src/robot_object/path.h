#pragma once

#include "curveFactory.h"
#include "communication.h"

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

    Path(const Path &obj) {
        this->block_num = obj.block_num;
        this->turn_dir = obj.turn_dir;
        this->turn_type = obj.turn_type;
    }

    bool isStraightEnd(){
        if(turn_type == turn_type_e::STRAIGHT     ||
           turn_type == turn_type_e::TURN_180     ||
           turn_type == turn_type_e::TURN_D2S_45  ||
           turn_type == turn_type_e::TURN_D2S_135 ||
           turn_type == turn_type_e::TURN_L_90    ||
           turn_type == turn_type_e::TURN_90       )
        {
            return true;
        }
        else{
            return false;
        }
    }

    bool isStraightStart(){
        if(turn_type == turn_type_e::STRAIGHT     ||
           turn_type == turn_type_e::TURN_180     ||
           turn_type == turn_type_e::TURN_S2D_45  ||
           turn_type == turn_type_e::TURN_S2D_135 ||
           turn_type == turn_type_e::TURN_L_90    ||
           turn_type == turn_type_e::TURN_90       )
        {
            return true;
        }
        else{
            return false;
        }

    }

    bool isDiagonalEnd(){
        if(turn_type == turn_type_e::D_STRAIGHT   ||
           turn_type == turn_type_e::TURN_S2D_45  ||
           turn_type == turn_type_e::TURN_S2D_135 ||
           turn_type == turn_type_e::TURN_D_90    )
        {
            return true;
        }
        else{
            return false;
        }
    }

    bool isDiagonalStart(){
        if(turn_type == turn_type_e::D_STRAIGHT   ||
           turn_type == turn_type_e::TURN_D2S_45  ||
           turn_type == turn_type_e::TURN_D2S_135 ||
           turn_type == turn_type_e::TURN_D_90    )
        {
            return true;
        }
        else{
            return false;
        }
    }

    ~Path() {

    }


    void print() {
        printfAsync("turn_type %d, block_num %d, turn_dir %d\n", turn_type, block_num, turn_dir);
    }

};

}
