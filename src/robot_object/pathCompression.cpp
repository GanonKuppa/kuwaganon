#include "path.h"
#include <stdint.h>
#include <vector>

namespace umouse {
    void compress_l_90(std::vector<Path>& path_vec, uint16_t start_index=0) {
        for (int i=start_index; i<path_vec.size()-4; i++) {
            if(path_vec[i+0].isStraightEnd() &&

                    path_vec[i+1].turn_type   == turn_type_e::STRAIGHT &&
                    path_vec[i+2].turn_type == turn_type_e::TURN_90    &&
                    path_vec[i+3].turn_type == turn_type_e::STRAIGHT   &&

                    path_vec[i+4].isStraightStart()
              ) {
                path_vec[i+1].turn_type = turn_type_e::TURN_L_90;
                path_vec[i+1].turn_dir = path_vec[i+2].turn_dir;
                path_vec.erase(path_vec.begin()+i+2);
                path_vec.erase(path_vec.begin()+i+2);
            }
        }
    }

    void compress_180(std::vector<Path>& path_vec, uint16_t start_index=0) {
        for (int i=start_index; i<path_vec.size()-5; i++) {
            if(path_vec[i+0].isStraightEnd() &&

                    path_vec[i+1].turn_type == turn_type_e::STRAIGHT &&
                    path_vec[i+2].turn_type == turn_type_e::TURN_90  &&
                    path_vec[i+3].turn_type == turn_type_e::TURN_90  &&
                    path_vec[i+4].turn_type == turn_type_e::STRAIGHT &&
                    path_vec[i+2].turn_dir == path_vec[i+3].turn_dir &&

                    path_vec[i+5].isStraightStart()
              ) {
                path_vec[i+1].turn_type = turn_type_e::TURN_180;
                path_vec[i+1].turn_dir = path_vec[i+2].turn_dir;
                path_vec.erase(path_vec.begin()+i+2);
                path_vec.erase(path_vec.begin()+i+2);
                path_vec.erase(path_vec.begin()+i+2);
            }
        }

    }

    void compress_s2d_135(std::vector<Path>& path_vec, uint16_t start_index=0) {
        for (int i=start_index; i<path_vec.size()-4; i++) {
            if(path_vec[i+0].isStraightEnd() &&

                    path_vec[i+1].turn_type == turn_type_e::STRAIGHT &&
                    path_vec[i+2].turn_type == turn_type_e::TURN_90  &&
                    path_vec[i+3].turn_type == turn_type_e::TURN_90  &&
                    path_vec[i+2].turn_dir == path_vec[i+3].turn_dir &&

                    path_vec[i+3].turn_dir != path_vec[i+4].turn_dir &&
                    (path_vec[i+4].isDiagonalStart() ||
                     path_vec[i+4].turn_type == turn_type_e::TURN_90)
              ) {
                path_vec[i+1].turn_type = turn_type_e::TURN_S2D_135;
                path_vec[i+1].turn_dir = path_vec[i+2].turn_dir;
                path_vec.erase(path_vec.begin()+i+2);
                path_vec.erase(path_vec.begin()+i+2);
            }
        }
    }

    void compress_d2s_135(std::vector<Path>& path_vec, uint16_t start_index=0) {
        for (int i=start_index; i<path_vec.size()-4; i++) {
            if( path_vec[i+0].turn_dir != path_vec[i+1].turn_dir &&
                    (path_vec[i+0].isDiagonalEnd() ||
                     path_vec[i+0].turn_type == turn_type_e::TURN_90) &&

                    path_vec[i+1].turn_type == turn_type_e::TURN_90   &&
                    path_vec[i+2].turn_type == turn_type_e::TURN_90   &&
                    path_vec[i+3].turn_type == turn_type_e::STRAIGHT  &&
                    path_vec[i+1].turn_dir == path_vec[i+2].turn_dir  &&

                    path_vec[i+4].isStraightStart()
              ) {
                path_vec[i+1].turn_type = turn_type_e::TURN_D2S_135;
                path_vec.erase(path_vec.begin()+i+2);
                path_vec.erase(path_vec.begin()+i+2);
            }
        }
    }

    void compress_s2d_45(std::vector<Path>& path_vec, uint16_t start_index=0) {
        for (int i=start_index; i<path_vec.size()-3; i++) {
            if( path_vec[i+0].isStraightEnd() &&

                    path_vec[i+1].turn_type == turn_type_e::STRAIGHT &&
                    path_vec[i+2].turn_type == turn_type_e::TURN_90  &&

                    path_vec[i+2].turn_dir != path_vec[i+3].turn_dir &&
                    (path_vec[i+3].isDiagonalStart() ||
                     path_vec[i+3].turn_type == turn_type_e::TURN_90 )
              ) {
                path_vec[i+1].turn_type = turn_type_e::TURN_S2D_45;
                path_vec[i+1].turn_dir = path_vec[i+2].turn_dir;
                path_vec.erase(path_vec.begin()+i+2);
            }
        }
    }

    void compress_d2s_45(std::vector<Path>& path_vec, uint16_t start_index=0) {
        for (int i=start_index; i<path_vec.size()-3; i++) {
            if( path_vec[i+0].turn_dir != path_vec[i+1].turn_dir &&
                    (path_vec[i+0].isDiagonalEnd() ||
                     path_vec[i+0].turn_type == turn_type_e::TURN_90) &&

                    path_vec[i+1].turn_type == turn_type_e::TURN_90 &&
                    path_vec[i+2].turn_type == turn_type_e::STRAIGHT &&

                    path_vec[i+3].isStraightStart()
              ) {
                path_vec[i+1].turn_type = turn_type_e::TURN_D2S_45;
                path_vec.erase(path_vec.begin()+i+2);
            }
        }
    }

    void compress_d_90(std::vector<Path>& path_vec, uint16_t start_index=0) {
        for (int i=start_index; i<path_vec.size()-3; i++) {
            if(path_vec[i+0].turn_dir != path_vec[i+1].turn_dir &&
                    (path_vec[i+0].isDiagonalEnd() ||
                     path_vec[i+0].turn_type == turn_type_e::TURN_90) &&


                    path_vec[i+1].turn_type == turn_type_e::TURN_90 &&
                    path_vec[i+2].turn_type == turn_type_e::TURN_90 &&
                    path_vec[i+1].turn_dir == path_vec[i+2].turn_dir &&

                    path_vec[i+2].turn_dir != path_vec[i+3].turn_dir &&
                    (path_vec[i+3].isDiagonalStart() ||
                     path_vec[i+3].turn_type == turn_type_e::TURN_90 )

              ) {
                path_vec[i+1].turn_type = turn_type_e::TURN_D_90;
                path_vec[i+1].turn_dir = path_vec[i+2].turn_dir;
                path_vec.erase(path_vec.begin()+i+2);
            }
        }
    }


    void compress_straight(std::vector<Path>& path_vec, uint16_t start_num=0) {
        for (uint16_t i = start_num; i < (uint16_t)path_vec.size(); i++) {
            //printfAsync("-------%d turntype %d \n", i, path_vec[i].turn_type);
            //waitmsec(3000);
            if (i == (uint16_t)path_vec.size()) {
                break;
            } else if (path_vec[i].turn_type == turn_type_e::STRAIGHT) {
                if (i + 1 == (uint16_t)path_vec.size())
                    break;
                while (path_vec[i + 1].turn_type == turn_type_e::STRAIGHT) {
                    path_vec.erase(path_vec.begin() + i + 1);
                    printfAsync("          -- i=%d -- size=%d --block_num=%d \n", i, path_vec.size(), path_vec[i].block_num);
                    path_vec[i].block_num++;
                    if (i + 1 == (uint16_t)path_vec.size())
                        break;
                }
            }
        }
    }

    void compress_d_straight(std::vector<Path>& path_vec, uint16_t start_index=0) {
        for (int i=start_index; i<path_vec.size()-2; i++) {
            if(path_vec[i+0].isDiagonalEnd() &&

                    path_vec[i+1].turn_type == turn_type_e::TURN_90 &&

                    path_vec[i+2].isDiagonalStart()

              ) {
                path_vec[i+1].turn_type = turn_type_e::D_STRAIGHT;
                path_vec[i+1].turn_dir = turn_dir_e::NO_TURN;
            }
        }


        for (uint16_t i = start_index; i < (uint16_t)path_vec.size(); i++) {
            //printfAsync("-------%d turntype %d \n", i, path_vec[i].turn_type);
            //waitmsec(3000);
            if (i == (uint16_t)path_vec.size()) {
                break;
            } else if (path_vec[i].turn_type == turn_type_e::TURN_90) {
                if (i + 1 == (uint16_t)path_vec.size()) break;
                while (path_vec[i + 1].turn_type == turn_type_e::TURN_90 &&
                        path_vec[i].turn_dir != path_vec[i+1].turn_dir) {

                    printfAsync("          -- i=%d -- size=%d --block_num=%d \n", i, path_vec.size(), path_vec[i].block_num);
                    path_vec[i].block_num++;
                    path_vec[i].turn_type = turn_type_e::D_STRAIGHT;
                    path_vec[i].turn_dir = path_vec[i+1].turn_dir;
                    path_vec.erase(path_vec.begin() + i + 1);
                    if (i + 1 == (uint16_t)path_vec.size()) break;
                }

            }
        }

    }

}