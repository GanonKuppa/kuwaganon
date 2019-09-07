/*
 * parameterManager.h
 *
 *  Created on: 2017/08/27
 *      Author: ryota
 */

#pragma once

#include<stdint.h>
#include<string>
#include<map>

namespace umouse {

    enum struct Type_e {
        FLOAT =0,
        UINT8 ,
        UINT16,
        UINT32,
        INT8,
        INT16,
        INT32
    };

    class ParameterManager {
    public:
        std::map<uint16_t, uint32_t> adrMap;
        std::map<uint16_t, Type_e> typeMap;
        std::map<std::string,uint16_t> strkeyMap;

        static ParameterManager& getInstance() {
            static ParameterManager instance;
            return instance;
        }

        void init();

        template<typename T>
        bool write(uint16_t val_num, T val);

        template<typename T>
        T read(uint16_t val_num);

        template<typename T>
        bool write(std::string key, T val);

        template<typename T>
        T read(std::string key);

        template<typename T>
        void registration(uint16_t val_num, T& r_val);
        void setStrkey(uint16_t val_num, std::string key);

        bool writeCommand(uint8_t *commnd);

        //----管理下においた変数たち-----
        float mass; //0
        float dia_tire; //1
        float tread; //2
        float inertia; //3
        float K_T_left; //4
        float K_T_right; //5
        float circuit_res_left; //6
        float circuit_res_right; //7
         ; //8
         ; //9
         ; //10
         ; //11
         ; //12
         ; //13
         ; //14
         ; //15
         ; //16
         ; //17
         ; //18
         ; //19
        float trans_friction_coef; //20
        float trans_friction_offset; //21
        float trans_v_P; //22
        float trans_v_I; //23
        float trans_v_D; //24
        float trans_v_F; //25
        float trans_v_spin_P; //26
        float trans_v_spin_I; //27
        float trans_v_spin_D; //28
        float trans_v_spin_F; //29
        float trans_v_slalom_P; //30
        float trans_v_slalom_I; //31
        float trans_v_slalom_D; //32
        float trans_v_slalom_F; //33
        float trans_x_P; //34
        float trans_x_I; //35
        float trans_x_D; //36
        float trans_x_F; //37
         ; //38
         ; //39
        float rot_friction_coef; //40
        float rot_friction_offset; //41
        float rot_v_P; //42
        float rot_v_I; //43
        float  rot_v_D; //44
        float rot_v_F; //45
        float rot_v_spin_P; //46
        float rot_v_spin_I; //47
        float rot_v_spin_D; //48
        float rot_v_spin_F; //49
        float rot_v_slalom_P; //50
        float rot_v_slalom_I; //51
        float rot_v_slalom_D; //52
        float rot_v_slalom_F; //53
        float rot_x_spin_P; //54
        float rot_x_spin_I; //55
        float rot_x_spin_D; //56
        float rot_x_spin_F; //57
         ; //58
         ; //59
        float wall_P; //60
        float wall_I; //61
        float wall_D; //62
        float wall_F; //63
        float wall_diag_P; //64
        float wall_diag_I; //65
        float wall_diag_D; //66
        float wall_diag_F; //67
         ; //68
         ; //69
        float pos_P; //70
        float pos_I; //71
        float pos_D; //72
        float pos_F; //73
        float rot_x_slalom_P; //74
        float rot_x_slalom_I; //75
        float rot_x_slalom_D; //76
        float rot_x_slalom_F; //77

        ; //78
        ; //79
        uint8_t trans_v_PIDF_satuation_enable; //80
        uint8_t rot_v_PIDF_satuation_enable; //81
        uint8_t pos_PIDF_satuation_enable; //82
        float trans_v_satuation_offset_duty; //83
        float trans_v_satuation_FF_multiplier; //84
        float rot_v_satuation_offset_duty; //85
        float rot_v_satuation_FF_multiplier; //86
        float pos_satuation_xy_dd; //87
        float pos_slalom_P; //88
        float pos_slalom_I; //89
        float pos_slalom_D; //90
        float pos_slalom_F; //91
         ; //92
         ; //93
         ; //94
         ; //95
         ; //96
         ; //97
         ; //98
         ; //99
        uint8_t trans_v_FF_enable; //100
        uint8_t trans_v_PIDF_enable; //101
        uint8_t trans_x_PIDF_enable; //102
        uint8_t rot_v_FF_enable; //103
        uint8_t rot_v_PIDF_enable; //104
        uint8_t rot_x_spin_PIDF_enable; //105
        uint8_t pos_PIDF_enable; //106
        uint8_t wall_PIDF_enable; //107
        uint8_t wall_diag_PIDF_enable; //108
         ; //109
         ; //110
         ; //111
         ; //112
         ; //113
         ; //114
         ; //115
         ; //116
         ; //117
         ; //118
         ; //119
        uint16_t wall_center_r; //120
        uint16_t wall_center_l; //121
        uint16_t collision_thr_ahead; //122
        uint16_t wall_threshold_right; //123
        uint16_t wall_threshold_left; //124
        uint16_t wall_threshold_ahead_r; //125
        uint16_t wall_threshold_ahead_l; //126
        uint16_t wall_ctrl_threshold_right; //127
        uint16_t wall_ctrl_threshold_left; //128
        uint16_t wall_ctrl_threshold_delta_right; //129
        uint16_t wall_ctrl_threshold_delta_left; //130
        uint16_t wall_ctrl_add_val_right; //131
        uint16_t wall_ctrl_add_val_left; //132
         ; //133
         ; //134
         uint16_t wall_corner_threshold_on_r; //135
         uint16_t wall_corner_threshold_off_r; //136
         uint16_t wall_corner_threshold_on_l; //137
         uint16_t wall_corner_threshold_off_l; //138
         float wall_corner_read_offset_r; //139
         float wall_corner_read_offset_l; //140

         uint16_t wall_contact_threshold_right; //141
         uint16_t wall_contact_threshold_left; //142
         uint16_t wall_contact_threshold_ahead_r; //143
         uint16_t wall_contact_collision_threshold_ahead_l; //144
         float wall_contact_offset; //145
         uint16_t wall_diagonal_ahead_l_threshold; //146;
         uint16_t wall_diagonal_ahead_r_threshold; //147;
         float wall_diagonal_avoid_add_ang; //148;



         int16_t gyro_x_ref; //150
        int16_t gyro_y_ref; //151
        int16_t gyro_z_ref; //152
        int16_t acc_x_ref; //153
        int16_t acc_y_ref; //154
        int16_t acc_z_ref; //155
        float duty_limit; //156
        uint8_t silent_flag; //157
        uint8_t send_data_mode; //158
        float test_run_v; //159
        float test_run_a; //160
        float test_run_x; //161
        uint8_t test_run_wall_flag; //162
        float v_search_run; //163
        float a_search_run; //164
        uint8_t goal_x; //165
        uint8_t goal_y; //166
        uint16_t search_limit_time_sec; //167
         ; //168
         ; //169
         ; //170
         ; //171
        float spin_ang_v; //172
        float spin_ang_a; //173
        float gyro_scaler_right; //174
        float gyro_scaler_left; //175
        float v_comp_gain; //176
        float gyro2_scaler_right; //177
        float gyro2_scaler_left; //178
         ; //179
         ; //180
         ; //181
         ; //182
         ; //183
         ; //184
         ; //185
         ; //186
         ; //187
         ; //188
         ; //189
         float gyro2_x_ref; //190
         float gyro2_y_ref; //191
         float gyro2_z_ref; //192
         float acc2_x_ref; //193
         float acc2_y_ref; //194
         float acc2_z_ref; //195
         ; //196
         ; //197
         ; //198
         ; //199
         ; //200
        //----管理下においた変数たち-----

    private:

        ParameterManager() {};
        ~ParameterManager() {};
        ParameterManager(ParameterManager&) {};
    };

}

