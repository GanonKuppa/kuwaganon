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
        registration<float>(8, duty_coef_left_p);
        registration<float>(9, duty_offset_left_p);
        registration<float>(10, duty_coef_right_p);
        registration<float>(11, duty_offset_right_p);
        registration<float>(12, duty_coef_left_m);
        registration<float>(13, duty_offset_left_m);
        registration<float>(14, duty_coef_right_m);
        registration<float>(15, duty_offset_right_m);
        registration<float>(16, ff_v_coef);
        registration<float>(17, ff_v_offset);
        registration<float>(18, ff_a_coef);
        registration<float>(19, ff_a_offset);
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
        registration<float>(34, ff_rot_v_coef);
        registration<float>(35, ff_rot_v_offset);
        registration<float>(36, ff_rot_a_coef);
        registration<float>(37, ff_rot_a_offset);
        registration<float>(38, rot_friction_coef);
        registration<float>(39, rot_friction_offset);
        registration<float>(40, rot_v_P);
        registration<float>(41, rot_v_I);
        registration<float>(42, rot_v_D);
        registration<float>(43, rot_v_F);
        registration<float>(44, rot_v_spin_P);
        registration<float>(45, rot_v_spin_I);
        registration<float>(46, rot_v_spin_D);
        registration<float>(47, rot_v_spin_F);
        registration<float>(48, rot_v_slalom_P);
        registration<float>(49, rot_v_slalom_I);
        registration<float>(50, rot_v_slalom_D);
        registration<float>(51, rot_v_slalom_F);
        registration<float>(52, rot_x_P);
        registration<float>(53, rot_x_I);
        registration<float>(54, rot_x_D);
        registration<float>(55, rot_x_F);
        registration<float>(56, rot_x_spin_P);
        registration<float>(57, rot_x_spin_I);
        registration<float>(58, rot_x_spin_D);
        registration<float>(59, rot_x_spin_F);
        registration<float>(60, rot_x_slalom_P);
        registration<float>(61, rot_x_slalom_I);
        registration<float>(62, rot_x_slalom_D);
        registration<float>(63, rot_x_slalom_F);
        registration<float>(64, wall_P);
        registration<float>(65, wall_I);
        registration<float>(66, wall_D);
        registration<float>(67, wall_F);
        registration<float>(68, wall_diag_P);
        registration<float>(69, wall_diag_I);
        registration<float >(70, wall_diag_D);
        registration<float>(71, wall_diag_F);
        registration<float>(72, pos_P);
        registration<float>(73, pos_I);
        registration<float>(74, pos_D);
        registration<float>(75, pos_F);
        ;//76
        ;//77
        ;//78
        ;//79
        registration<uint8_t>(80, trans_v_PIDF_saturation_enable);
        registration<uint8_t>(81, rot_v_PIDF_saturation_enable);
        ;//82
        registration<float>(83, trans_v_saturation_offset_duty);
        registration<float>(84, trans_v_saturation_FF_multiplier);
        registration<float>(85, rot_v_saturation_offset_duty);
        registration<float>(86, rot_v_saturation_FF_multiplier);
        ;//87
        ;//88
        ;//89
        registration<float>(90, trans_v_PIDF_integral_saturation);
        registration<float>(91, rot_v_PIDF_integral_saturation);
        registration<float>(92, rot_x_PIDF_integral_saturation);
        registration<float>(93, pos_PIDF_integral_saturation);
        registration<float>(94, wall_PIDF_integral_saturation);
        registration<float>(95, target_rot_x_saturation);
        registration<float>(96, rot_x_wall_abs_center_offset);
        ;//97
        ;//98
        ;//99
        registration<uint8_t>(100, trans_v_FF_enable);
        registration<uint8_t>(101, trans_v_PIDF_enable);
        registration<uint8_t>(102, rot_v_FF_enable);
        registration<uint8_t>(103, rot_v_PIDF_enable);
        registration<uint8_t>(104, rot_x_PIDF_enable);
        registration<uint8_t>(105, pos_PIDF_enable);
        registration<uint8_t>(106, wall_PIDF_enable);
        registration<uint8_t>(107, wall_diag_PIDF_enable);
        ;//108
        ;//109
        ;//110
        ;//111
        ;//112
        ;//113
        ;//114
        ;//115
        ;//116
        ;//117
        ;//118
        ;//119
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
        registration<float>(146, wall_diagonal_ahead_l_threshold);
        registration<float>(147, wall_diagonal_ahead_r_threshold);
        registration<float>(148, wall_diagonal_avoid_add_ang);
        ;//149
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
        ;//168
        ;//169
        ;//170
        ;//171
        registration<float>(172, spin_ang_v);
        registration<float>(173, spin_ang_a);
        registration<float>(174, gyro_scaler_right);
        registration<float>(175, gyro_scaler_left);
        registration<float>(176, v_comp_gain);
        registration<float>(177, gyro2_scaler_right);
        registration<float>(178, gyro2_scaler_left);
        registration<float>(179, acc_x_scaler);
        registration<float>(180, acc_y_scaler);
        ;//181
        ;//182
        ;//183
        ;//184
        ;//185
        ;//186
        ;//187
        ;//188
        ;//189
        registration<float>(190, gyro2_x_ref);
        registration<float>(191, gyro2_y_ref);
        registration<float>(192, gyro2_z_ref);
        registration<int16_t>(193, acc2_x_ref);
        registration<int16_t>(194, acc2_y_ref);
        registration<int16_t>(195, acc2_z_ref);
        ;//196
        ;//197
        ;//198
        ;//199
        registration<float>(200, shortest_0_v);
        registration<float>(201, shortest_0_v_d);
        registration<float>(202, shortest_0_v_90);
        registration<float>(203, shortest_0_v_l90);
        registration<float>(204, shortest_0_v_180);
        registration<float>(205, shortest_0_v_d90);
        registration<float>(206, shortest_0_v_45);
        registration<float>(207, shortest_0_v_135);
        registration<float>(208, shortest_0_a);
        registration<float>(209, shortest_0_a_diag);
        ;//210
        ;//211
        ;//212
        ;//213
        ;//214
        ;//215
        ;//216
        ;//217
        ;//218
        ;//219
        registration<float>(220, shortest_1_v);
        registration<float>(221, shortest_1_v_d);
        registration<float>(222, shortest_1_v_90);
        registration<float>(223, shortest_1_v_l90);
        registration<float>(224, shortest_1_v_180);
        registration<float>(225, shortest_1_v_d90);
        registration<float>(226, shortest_1_v_45);
        registration<float>(227, shortest_1_v_135);
        registration<float>(228, shortest_1_a);
        registration<float>(229, shortest_1_a_diag);
        ;//230
        ;//231
        ;//232
        ;//233
        ;//234
        ;//235
        ;//236
        ;//237
        ;//238
        ;//239
        registration<float>(240, shortest_2_v);
        registration<float>(241, shortest_2_v_d);
        registration<float>(242, shortest_2_v_90);
        registration<float>(243, shortest_2_v_l90);
        registration<float>(244, shortest_2_v_180);
        registration<float>(245, shortest_2_v_d90);
        registration<float>(246, shortest_2_v_45);
        registration<float>(247, shortest_2_v_135);
        registration<float>(248, shortest_2_a);
        registration<float>(249, shortest_2_a_diag);
        ;//250
        ;//251
        ;//252
        ;//253
        ;//254
        ;//255
        ;//256
        ;//257
        ;//258
        ;//259
        registration<float>(260, shortest_3_v);
        registration<float>(261, shortest_3_v_d);
        registration<float>(262, shortest_3_v_90);
        registration<float>(263, shortest_3_v_l90);
        registration<float>(264, shortest_3_v_180);
        registration<float>(265, shortest_3_v_d90);
        registration<float>(266, shortest_3_v_45);
        registration<float>(267, shortest_3_v_135);
        registration<float>(268, shortest_3_a);
        registration<float>(269, shortest_3_a_diag);
        ;//270
        ;//271
        ;//272
        ;//273
        ;//274
        ;//275
        ;//276
        ;//277
        ;//278
        ;//279
        registration<float>(280, shortest_4_v);
        registration<float>(281, shortest_4_v_d);
        registration<float>(282, shortest_4_v_90);
        registration<float>(283, shortest_4_v_l90);
        registration<float>(284, shortest_4_v_180);
        registration<float>(285, shortest_4_v_d90);
        registration<float>(286, shortest_4_v_45);
        registration<float>(287, shortest_4_v_135);
        registration<float>(288, shortest_4_a);
        registration<float>(289, shortest_4_a_diag);
        ;//290
        ;//291
        ;//292
        ;//293
        ;//294
        ;//295
        ;//296
        ;//297
        ;//298
        ;//299
        registration<float>(300, turn_90_v_35_d_pre_offset);
        registration<float>(301, turn_90_v_40_d_pre_offset);
        registration<float>(302, turn_90_v_45_d_pre_offset);
        registration<float>(303, turn_90_v_50_d_pre_offset);
        registration<float>(304, turn_90_v_55_d_pre_offset);
        registration<float>(305, turn_90_v_60_d_pre_offset);
        registration<float>(306, turn_90_v_65_d_pre_offset);
        registration<float>(307, turn_90_v_70_d_pre_offset);
        registration<float>(308, turn_90_v_75_d_pre_offset);
        registration<float>(309, turn_90_v_80_d_pre_offset);
        registration<float>(310, turn_l90_v_35_d_pre_offset);
        registration<float>(311, turn_l90_v_40_d_pre_offset);
        registration<float>(312, turn_l90_v_45_d_pre_offset);
        registration<float>(313, turn_l90_v_50_d_pre_offset);
        registration<float>(314, turn_l90_v_55_d_pre_offset);
        registration<float>(315, turn_l90_v_60_d_pre_offset);
        registration<float>(316, turn_l90_v_65_d_pre_offset);
        registration<float>(317, turn_l90_v_70_d_pre_offset);
        registration<float>(318, turn_l90_v_75_d_pre_offset);
        registration<float>(319, turn_l90_v_80_d_pre_offset);
        registration<float>(320, turn_180_v_35_d_pre_offset);
        registration<float>(321, turn_180_v_40_d_pre_offset);
        registration<float>(322, turn_180_v_45_d_pre_offset);
        registration<float>(323, turn_180_v_50_d_pre_offset);
        registration<float>(324, turn_180_v_55_d_pre_offset);
        registration<float>(325, turn_180_v_60_d_pre_offset);
        registration<float>(326, turn_180_v_65_d_pre_offset);
        registration<float>(327, turn_180_v_70_d_pre_offset);
        registration<float>(328, turn_180_v_75_d_pre_offset);
        registration<float>(329, turn_180_v_80_d_pre_offset);
        registration<float>(330, turn_d90_v_35_d_pre_offset);
        registration<float>(331, turn_d90_v_40_d_pre_offset);
        registration<float>(332, turn_d90_v_45_d_pre_offset);
        registration<float>(333, turn_d90_v_50_d_pre_offset);
        registration<float>(334, turn_d90_v_55_d_pre_offset);
        registration<float>(335, turn_d90_v_60_d_pre_offset);
        registration<float>(336, turn_d90_v_65_d_pre_offset);
        registration<float>(337, turn_d90_v_70_d_pre_offset);
        registration<float>(338, turn_d90_v_75_d_pre_offset);
        registration<float>(339, turn_d90_v_80_d_pre_offset);
        registration<float>(340, turn_s2d45_v_35_d_pre_offset);
        registration<float>(341, turn_s2d45_v_40_d_pre_offset);
        registration<float>(342, turn_s2d45_v_45_d_pre_offset);
        registration<float>(343, turn_s2d45_v_50_d_pre_offset);
        registration<float>(344, turn_s2d45_v_55_d_pre_offset);
        registration<float>(345, turn_s2d45_v_60_d_pre_offset);
        registration<float>(346, turn_s2d45_v_65_d_pre_offset);
        registration<float>(347, turn_s2d45_v_70_d_pre_offset);
        registration<float>(348, turn_s2d45_v_75_d_pre_offset);
        registration<float>(349, turn_s2d45_v_80_d_pre_offset);
        registration<float>(350, turn_s2d135_v_35_d_pre_offset);
        registration<float>(351, turn_s2d135_v_40_d_pre_offset);
        registration<float>(352, turn_s2d135_v_45_d_pre_offset);
        registration<float>(353, turn_s2d135_v_50_d_pre_offset);
        registration<float>(354, turn_s2d135_v_55_d_pre_offset);
        registration<float>(355, turn_s2d135_v_60_d_pre_offset);
        registration<float>(356, turn_s2d135_v_65_d_pre_offset);
        registration<float>(357, turn_s2d135_v_70_d_pre_offset);
        registration<float>(358, turn_s2d135_v_75_d_pre_offset);
        registration<float>(359, turn_s2d135_v_80_d_pre_offset);
        registration<float>(360, turn_d2s45_v_35_d_pre_offset);
        registration<float>(361, turn_d2s45_v_40_d_pre_offset);
        registration<float>(362, turn_d2s45_v_45_d_pre_offset);
        registration<float>(363, turn_d2s45_v_50_d_pre_offset);
        registration<float>(364, turn_d2s45_v_55_d_pre_offset);
        registration<float>(365, turn_d2s45_v_60_d_pre_offset);
        registration<float>(366, turn_d2s45_v_65_d_pre_offset);
        registration<float>(367, turn_d2s45_v_70_d_pre_offset);
        registration<float>(368, turn_d2s45_v_75_d_pre_offset);
        registration<float>(369, turn_d2s45_v_80_d_pre_offset);
        registration<float>(370, turn_d2s135_v_35_d_pre_offset);
        registration<float>(371, turn_d2s135_v_40_d_pre_offset);
        registration<float>(372, turn_d2s135_v_45_d_pre_offset);
        registration<float>(373, turn_d2s135_v_50_d_pre_offset);
        registration<float>(374, turn_d2s135_v_55_d_pre_offset);
        registration<float>(375, turn_d2s135_v_60_d_pre_offset);
        registration<float>(376, turn_d2s135_v_65_d_pre_offset);
        registration<float>(377, turn_d2s135_v_70_d_pre_offset);
        registration<float>(378, turn_d2s135_v_75_d_pre_offset);
        registration<float>(379, turn_d2s135_v_80_d_pre_offset);
        ;//380
        ;//381
        ;//382
        ;//383
        ;//384
        ;//385
        ;//386
        ;//387
        ;//388
        ;//389
        ;//390
        ;//391
        ;//392
        ;//393
        ;//394
        ;//395
        ;//396
        ;//397
        ;//398
        ;//399
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
            printfAsync("%d | write f: %f %f", val_num, val, *reinterpret_cast<T*>(adrMap[val_num]));
            printfAsync("|| write d: %d %d \n", val, *reinterpret_cast<T*>(adrMap[val_num]));
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


    bool ParameterManager::writeCommand(uint8_t* command) {
        uint8_t chk_sum = 0;
        uint16_t param_num = 0;
        for (uint8_t i = 5; i < 16; i++)
            chk_sum += command[i];
        if (chk_sum == command[4]) {
            SED(); //音を鳴らす
            param_num = command[10] + (command[11] << 8);
            switch (command[5]) {
                case 0:
                    write<float>(param_num, *((float*) &command[6]));
                    printfAsync("%d %f %f \n", command[10], *((float*) &command[6]), read<float>(command[10]));
                    break;
                case 1:
                    write<uint8_t>(param_num, command[6]);
                    printfAsync("%d \n", *((uint8_t*) &command[6]));
                    break;
                case 2:
                    write<uint16_t>(param_num, *((uint16_t*) &command[6]));
                    printfAsync("%d \n", *((uint16_t*) &command[6]));
                    break;
                case 3:
                    write<uint32_t>(param_num, *((uint32_t*) &command[6]));
                    printfAsync("%d \n", *((uint32_t*) &command[6]));
                    break;
                case 4:
                    write<int8_t>(param_num, *((int8_t*) &command[6]));
                    printfAsync("%d \n", *((int8_t*) &command[6]));
                    break;
                case 5:
                    write<int16_t>(param_num, *((int16_t*) &command[6]));
                    printfAsync("%d | %d %d  \n", command[10], *((int16_t*) &command[6]), read<int16_t>(command[10]));
                    break;
                case 6:
                    write<int32_t>(param_num, *((int32_t*) &command[6]));
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

