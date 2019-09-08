/*
 * parameterManager.cpp
 *
 *  Created on: 2017/08/27
 *      Author: ryota
 */

#include <stdint.h>
#include <string>
#include <map>
#include <typeinfo>

#include "parameterManager.h"
#include "dataFlash.h"
#include "communication.h"
#include "timer.h"
#include "sound.h"


namespace umouse {

void ParameterManager::init() {
    registration<float>(0, mass);
    registration<float>(1, dia_tire);
    registration<float>(2, tread);
    registration<float>(3, inertia);
    registration<float>(4, K_T_left);
    registration<float>(5, K_T_right);
    registration<float>(6, circuit_res_left);
    registration<float>(7, circuit_res_right);
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    registration<float>(20, trans_friction_coef);
    registration<float>(21, trans_friction_offset);
    registration<float>(22, trans_v_P);
    registration<float>(23, trans_v_I);
    registration<float>(24, trans_v_D);
    registration<float>(25, trans_v_F);
    registration<float>(26, trans_v_spin_P);
    registration<float>(27, trans_v_spin_I);
    registration<float>(28, trans_v_spin_D);
    registration<float>(29, trans_v_spin_F);
    registration<float>(30, trans_v_slalom_P);
    registration<float>(31, trans_v_slalom_I);
    registration<float>(32, trans_v_slalom_D);
    registration<float>(33, trans_v_slalom_F);
    registration<float>(34, trans_x_P);
    registration<float>(35, trans_x_I);
    registration<float>(36, trans_x_D);
    registration<float>(37, trans_x_F);
    ;
    ;
    registration<float>(40, rot_friction_coef);
    registration<float>(41, rot_friction_offset);
    registration<float>(42, rot_v_P);
    registration<float>(43, rot_v_I);
    registration<float >(44, rot_v_D);
    registration<float>(45, rot_v_F);
    registration<float>(46, rot_v_spin_P);
    registration<float>(47, rot_v_spin_I);
    registration<float>(48, rot_v_spin_D);
    registration<float>(49, rot_v_spin_F);
    registration<float>(50, rot_v_slalom_P);
    registration<float>(51, rot_v_slalom_I);
    registration<float>(52, rot_v_slalom_D);
    registration<float>(53, rot_v_slalom_F);
    registration<float>(54, rot_x_spin_P);
    registration<float>(55, rot_x_spin_I);
    registration<float>(56, rot_x_spin_D);
    registration<float>(57, rot_x_spin_F);
    ;
    ;
    registration<float>(60, wall_P);
    registration<float>(61, wall_I);
    registration<float>(62, wall_D);
    registration<float>(63, wall_F);
    registration<float>(64, wall_diag_P);
    registration<float>(65, wall_diag_I);
    registration<float>(66, wall_diag_D);
    registration<float>(67, wall_diag_F);
    ;
    ;
    registration<float >(70, pos_P);
    registration<float>(71, pos_I);
    registration<float>(72, pos_D);
    registration<float>(73, pos_F);
    registration<float>(74, rot_x_slalom_P);
    registration<float>(75, rot_x_slalom_I);
    registration<float>(76, rot_x_slalom_D);
    registration<float>(77, rot_x_slalom_F);
    ;
    ;
    registration<uint8_t>(80, trans_v_PIDF_satuation_enable);
    registration<uint8_t>(81, rot_v_PIDF_satuation_enable);
    registration<uint8_t>(82, pos_PIDF_satuation_enable);
    registration<float>(83, trans_v_satuation_offset_duty);
    registration<float>(84, trans_v_satuation_FF_multiplier);
    registration<float>(85, rot_v_satuation_offset_duty);
    registration<float>(86, rot_v_satuation_FF_multiplier);
    registration<float>(87, pos_satuation_xy_dd);
    registration<float>(88, pos_slalom_P);
    registration<float>(89, pos_slalom_I);
    registration<float>(90, pos_slalom_D);
    registration<float>(91, pos_slalom_F);
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    registration<uint8_t>(100, trans_v_FF_enable);
    registration<uint8_t>(101, trans_v_PIDF_enable);
    registration<uint8_t>(102, trans_x_PIDF_enable);
    registration<uint8_t>(103, rot_v_FF_enable);
    registration<uint8_t>(104, rot_v_PIDF_enable);
    registration<uint8_t>(105, rot_x_spin_PIDF_enable);
    registration<uint8_t>(106, pos_PIDF_enable);
    registration<uint8_t>(107, wall_PIDF_enable);
    registration<uint8_t>(108, wall_diag_PIDF_enable);
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    registration<uint16_t>(120, wall_center_r);
    registration<uint16_t>(121, wall_center_l);
    registration<uint16_t>(122, collision_thr_ahead);
    registration<uint16_t>(123, wall_threshold_right);
    registration<uint16_t>(124, wall_threshold_left);
    registration<uint16_t>(125, wall_threshold_ahead_r);
    registration<uint16_t>(126, wall_threshold_ahead_l);
    registration<uint16_t>(127, wall_ctrl_threshold_right);
    registration<uint16_t>(128, wall_ctrl_threshold_left);
    registration<uint16_t>(129, wall_ctrl_threshold_delta_right);
    registration<uint16_t>(130, wall_ctrl_threshold_delta_left);
    registration<uint16_t>(131, wall_ctrl_add_val_right);
    registration<uint16_t>(132, wall_ctrl_add_val_left);

    registration<float>(133, diag_r_corner_read_offset);
    registration<float>(134, diag_l_corner_read_offset);

    registration<uint16_t>(135, wall_corner_threshold_on_r);
    registration<uint16_t>(136, wall_corner_threshold_off_r);
    registration<uint16_t>(137, wall_corner_threshold_on_l);
    registration<uint16_t>(138, wall_corner_threshold_off_l);
    registration<float>(139, wall_corner_read_offset_r);
    registration<float>(140, wall_corner_read_offset_l);
    registration<uint16_t>(141, wall_contact_threshold_right);
    registration<uint16_t>(142, wall_contact_threshold_left);
    registration<uint16_t>(143, wall_contact_threshold_ahead_r);
    registration<uint16_t>(144, wall_contact_collision_threshold_ahead_l);
    registration<float>(145, wall_contact_offset);
    registration<uint16_t>(146, wall_diagonal_ahead_l_threshold); //146;
    registration<uint16_t>(147, wall_diagonal_ahead_r_threshold); //147;
    registration<float>(148, wall_diagonal_avoid_add_ang); //148;

    ;
    ;
    registration<int16_t>(150, gyro_x_ref);
    registration<int16_t>(151, gyro_y_ref);
    registration<int16_t>(152, gyro_z_ref);
    registration<int16_t>(153, acc_x_ref);
    registration<int16_t>(154, acc_y_ref);
    registration<int16_t>(155, acc_z_ref);
    registration<float>(156, duty_limit);
    registration<uint8_t>(157, silent_flag);
    registration<uint8_t>(158, send_data_mode);
    registration<float>(159, test_run_v);
    registration<float>(160, test_run_a);
    registration<float>(161, test_run_x);
    registration<uint8_t>(162, test_run_wall_flag);
    registration<float>(163, v_search_run);
    registration<float>(164, a_search_run);
    registration<uint8_t>(165, goal_x);
    registration<uint8_t>(166, goal_y);
    registration<uint16_t>(167, search_limit_time_sec);
    ;
    ;
    ;
    ;
    registration<float>(172, spin_ang_v);
    registration<float>(173, spin_ang_a);
    registration<float>(174, gyro_scaler_right);
    registration<float>(175, gyro_scaler_left);
    registration<float>(176, v_comp_gain);
    registration<float>(177, gyro2_scaler_right);
    registration<float>(178, gyro2_scaler_left);
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    ;
    registration<float>(190, gyro2_x_ref);
    registration<float>(191, gyro2_y_ref);
    registration<float>(192, gyro2_z_ref);
    registration<float>(193, acc2_x_ref);
    registration<float>(194, acc2_x_ref);
    registration<float>(195, acc2_x_ref);
    ;
    ;
    ;
    ;
    ;

}

void ParameterManager::setStrkey(uint16_t val_num, std::string key) {
    strkeyMap[key] = val_num;
}

//プログラム中の変数にデータフラッシュの保存域を割り当て
//登録時に変数にデータフラッシュに保存されている値を代入
//登録を行った変数はデータフラッシュの保存域を更新する(write関数)際に値を共に変更
template<typename T>
void ParameterManager::registration(uint16_t val_num, T& r_val) {
    uint16_t index = val_num * 64;
    //uint8_t len = sizeof(T);
    T* adr = &r_val;
    adrMap[val_num] = reinterpret_cast<uint32_t>(adr);
    r_val = read<T>(val_num);

    if (typeid(float) == typeid(r_val)) typeMap[val_num] = Type_e::FLOAT;
    if (typeid(uint8_t) == typeid(r_val)) typeMap[val_num] = Type_e::UINT8;
    if (typeid(uint16_t) == typeid(r_val)) typeMap[val_num] = Type_e::UINT16;
    if (typeid(uint32_t) == typeid(r_val)) typeMap[val_num] = Type_e::UINT32;
    if (typeid(int8_t) == typeid(r_val)) typeMap[val_num] = Type_e::INT8;
    if (typeid(int16_t) == typeid(r_val)) typeMap[val_num] = Type_e::INT16;
    if (typeid(int32_t) == typeid(r_val)) typeMap[val_num] = Type_e::INT32;

}

template<typename T>
bool ParameterManager::write(uint16_t val_num, T val) {
    uint16_t index = val_num * 64;
    bool rtn;
    while (1) {
        if (eraseCheckDataFlash(index, 64) == false) {
            eraseDataFlash(index);
        };
        rtn = writeDataFlash(index, &val, sizeof(T));
        printfAsync("write error!\n");
        if (read<T>(val_num) == val) break;
    }

    //val_numに変数が登録されている場合はその変数を書き換え
    if (adrMap.find(val_num) != adrMap.end()) {
        *reinterpret_cast<T*>(adrMap[val_num]) = val;
        printfAsync("write: %f %f \n", val, *reinterpret_cast<T*>(adrMap[val_num]));
    }
    return rtn;
}

template<typename T>
T ParameterManager::read(uint16_t val_num) {
    T val;
    uint16_t index = val_num * 64;
    readDataFlash(index, &val, sizeof(T));

    return val;
}


template<typename T>
bool ParameterManager::write(std::string key, T val) {
    return write(strkeyMap[key], val);
}

template<typename T>
T ParameterManager::read(std::string key) {
    return read<T>(strkeyMap[key]);
}


bool ParameterManager::writeCommand(uint8_t *command) {
    uint8_t chk_sum = 0;
    for (uint8_t i = 5; i < 16; i++)
        chk_sum += command[i];
    if (chk_sum == command[4]) {
        SED(); //音を鳴らす
        switch (command[5]) {
            case 0:
                write<float>(command[10], *((float*) &command[6]));
                printfAsync("%d %f %f \n", command[10], *((float*) &command[6]), read<float>(command[10]));
                break;
            case 1:
                write<uint8_t>(command[10], command[6]);
                printfAsync("%d \n", *((uint8_t*) &command[6]));
                break;
            case 2:
                write<uint16_t>(command[10], *((uint16_t*) &command[6]));
                printfAsync("%d \n", *((uint16_t*) &command[6]));
                break;
            case 3:
                write<uint32_t>(command[10], *((uint32_t*) &command[6]));
                printfAsync("%d \n", *((uint32_t*) &command[6]));
                break;
            case 4:
                write<int8_t>(command[10], *((int8_t*) &command[6]));
                printfAsync("%d \n", *((int8_t*) &command[6]));
                break;
            case 5:
                write<int16_t>(command[10], *((int16_t*) &command[6]));
                printfAsync("%d | %d %d  \n", command[10], *((int16_t*) &command[6]), read<int16_t>(command[10]));

                break;
            case 6:
                write<int32_t>(command[10], *((int32_t*) &command[6]));
                printfAsync("%d \n", *((int32_t*) &command[6]));
                break;

        }
    }

}

//テンプレートクラスの実体化
template void ParameterManager::registration(uint16_t val_num, float& r_val);
template void ParameterManager::registration(uint16_t val_num, uint8_t& r_val);
template void ParameterManager::registration(uint16_t val_num, uint16_t& r_val);
template void ParameterManager::registration(uint16_t val_num, uint32_t& r_val);
template void ParameterManager::registration(uint16_t val_num, int8_t& r_val);
template void ParameterManager::registration(uint16_t val_num, int16_t& r_val);
template void ParameterManager::registration(uint16_t val_num, int32_t& r_val);

template bool ParameterManager::write(uint16_t val_num, float val);
template bool ParameterManager::write(uint16_t val_num, uint8_t val);
template bool ParameterManager::write(uint16_t val_num, uint16_t val);
template bool ParameterManager::write(uint16_t val_num, uint32_t val);
template bool ParameterManager::write(uint16_t val_num, int8_t val);
template bool ParameterManager::write(uint16_t val_num, int16_t val);
template bool ParameterManager::write(uint16_t val_num, int32_t val);

template float ParameterManager::read<float>(uint16_t val_num);
template uint8_t ParameterManager::read<uint8_t>(uint16_t val_num);
template uint16_t ParameterManager::read<uint16_t>(uint16_t val_num);
template uint32_t ParameterManager::read<uint32_t>(uint16_t val_num);
template int8_t ParameterManager::read<int8_t>(uint16_t val_num);
template int16_t ParameterManager::read<int16_t>(uint16_t val_num);
template int32_t ParameterManager::read<int32_t>(uint16_t val_num);

template bool ParameterManager::write(std::string key, float val);
template bool ParameterManager::write(std::string key, uint8_t val);
template bool ParameterManager::write(std::string key, uint16_t val);
template bool ParameterManager::write(std::string key, uint32_t val);
template bool ParameterManager::write(std::string key, int8_t val);
template bool ParameterManager::write(std::string key, int16_t val);
template bool ParameterManager::write(std::string key, int32_t val);

template float ParameterManager::read<float>(std::string key);
template uint8_t ParameterManager::read<uint8_t>(std::string key);
template uint16_t ParameterManager::read<uint16_t>(std::string key);
template uint32_t ParameterManager::read<uint32_t>(std::string key);
template int8_t ParameterManager::read<int8_t>(std::string key);
template int16_t ParameterManager::read<int16_t>(std::string key);
template int32_t ParameterManager::read<int32_t>(std::string key);

}

