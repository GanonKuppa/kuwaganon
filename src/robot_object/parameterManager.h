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
        ; //34
        ; //35
        ; //36
        ; //37
        float rot_friction_coef; //38
        float rot_friction_offset; //39
        float rot_v_P; //40
        float rot_v_I; //41
        float  rot_v_D; //42
        float rot_v_F; //43
        float rot_v_spin_P; //44
        float rot_v_spin_I; //45
        float rot_v_spin_D; //46
        float rot_v_spin_F; //47
        float rot_v_slalom_P; //48
        float rot_v_slalom_I; //49
        float rot_v_slalom_D; //50
        float rot_v_slalom_F; //51
        float rot_x_P; //52
        float rot_x_I; //53
        float rot_x_D; //54
        float rot_x_F; //55
        float rot_x_spin_P; //56
        float rot_x_spin_I; //57
        float rot_x_spin_D; //58
        float rot_x_spin_F; //59
        float rot_x_slalom_P; //60
        float rot_x_slalom_I; //61
        float rot_x_slalom_D; //62
        float rot_x_slalom_F; //63
        float wall_P; //64
        float wall_I; //65
        float wall_D; //66
        float wall_F; //67
        float wall_diag_P; //68
        float wall_diag_I; //69
        float  wall_diag_D; //70
        float wall_diag_F; //71
        float pos_P; //72
        float pos_I; //73
        float pos_D; //74
        float pos_F; //75
        ; //76
        ; //77
        ; //78
        ; //79
        uint8_t trans_v_PIDF_saturation_enable; //80
        uint8_t rot_v_PIDF_saturation_enable; //81
        ; //82
        float trans_v_saturation_offset_duty; //83
        float trans_v_saturation_FF_multiplier; //84
        float rot_v_saturation_offset_duty; //85
        float rot_v_saturation_FF_multiplier; //86
        ; //87
        ; //88
        ; //89
        float trans_v_PIDF_integral_saturation; //90
        float rot_v_PIDF_integral_saturation; //91
        float rot_x_PIDF_integral_saturation; //92
        float pos_PIDF_integral_saturation; //93
        float wall_PIDF_integral_saturation; //94
        float target_rot_x_saturation; //95
        float rot_x_wall_abs_center_offset; //96
        ; //97
        ; //98
        ; //99
        uint8_t trans_v_FF_enable; //100
        uint8_t trans_v_PIDF_enable; //101
        uint8_t rot_v_FF_enable; //102
        uint8_t rot_v_PIDF_enable; //103
        uint8_t rot_x_PIDF_enable; //104
        uint8_t pos_PIDF_enable; //105
        uint8_t wall_PIDF_enable; //106
        uint8_t wall_diag_PIDF_enable; //107
        ; //108
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
        float diag_r_corner_read_offset; //133
        float diag_l_corner_read_offset; //134
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
        float wall_diagonal_ahead_l_threshold; //146
        float wall_diagonal_ahead_r_threshold; //147
        float wall_diagonal_avoid_add_ang; //148
        ; //149
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
        int16_t gyro2_x_ref; //190
        int16_t gyro2_y_ref; //191
        int16_t gyro2_z_ref; //192
        int16_t acc2_x_ref; //193
        int16_t acc2_y_ref; //194
        int16_t acc2_z_ref; //195
        ; //196
        ; //197
        ; //198
        ; //199
        float shortest_0_v; //200
        float shortest_0_v_d; //201
        float shortest_0_v_90; //202
        float shortest_0_v_l90; //203
        float shortest_0_v_180; //204
        float shortest_0_v_d90; //205
        float shortest_0_v_45; //206
        float shortest_0_v_135; //207
        float shortest_0_a; //208
        float shortest_0_a_diag; //209
        ; //210
        ; //211
        ; //212
        ; //213
        ; //214
        ; //215
        ; //216
        ; //217
        ; //218
        ; //219
        float shortest_1_v; //220
        float shortest_1_v_d; //221
        float shortest_1_v_90; //222
        float shortest_1_v_l90; //223
        float shortest_1_v_180; //224
        float shortest_1_v_d90; //225
        float shortest_1_v_45; //226
        float shortest_1_v_135; //227
        float shortest_1_a; //228
        float shortest_1_a_diag; //229
        ; //230
        ; //231
        ; //232
        ; //233
        ; //234
        ; //235
        ; //236
        ; //237
        ; //238
        ; //239
        float shortest_2_v; //240
        float shortest_2_v_d; //241
        float shortest_2_v_90; //242
        float shortest_2_v_l90; //243
        float shortest_2_v_180; //244
        float shortest_2_v_d90; //245
        float shortest_2_v_45; //246
        float shortest_2_v_135; //247
        float shortest_2_a; //248
        float shortest_2_a_diag; //249
        ; //250
        ; //251
        ; //252
        ; //253
        ; //254
        ; //255
        ; //256
        ; //257
        ; //258
        ; //259
        float shortest_3_v; //260
        float shortest_3_v_d; //261
        float shortest_3_v_90; //262
        float shortest_3_v_l90; //263
        float shortest_3_v_180; //264
        float shortest_3_v_d90; //265
        float shortest_3_v_45; //266
        float shortest_3_v_135; //267
        float shortest_3_a; //268
        float shortest_3_a_diag; //269
        ; //270
        ; //271
        ; //272
        ; //273
        ; //274
        ; //275
        ; //276
        ; //277
        ; //278
        ; //279
        float shortest_4_v; //280
        float shortest_4_v_d; //281
        float shortest_4_v_90; //282
        float shortest_4_v_l90; //283
        float shortest_4_v_180; //284
        float shortest_4_v_d90; //285
        float shortest_4_v_45; //286
        float shortest_4_v_135; //287
        float shortest_4_a; //288
        float shortest_4_a_diag; //289
        ; //290
        ; //291
        ; //292
        ; //293
        ; //294
        ; //295
        ; //296
        ; //297
        ; //298
        ; //299
        float turn_90_v_35_d_pre_offset; //300
        float turn_90_v_40_d_pre_offset; //301
        float turn_90_v_45_d_pre_offset; //302
        float turn_90_v_50_d_pre_offset; //303
        float turn_90_v_55_d_pre_offset; //304
        float turn_90_v_60_d_pre_offset; //305
        float turn_90_v_65_d_pre_offset; //306
        float turn_90_v_70_d_pre_offset; //307
        float turn_90_v_75_d_pre_offset; //308
        float turn_90_v_80_d_pre_offset; //309
        float turn_l90_v_35_d_pre_offset; //310
        float turn_l90_v_40_d_pre_offset; //311
        float turn_l90_v_45_d_pre_offset; //312
        float turn_l90_v_50_d_pre_offset; //313
        float turn_l90_v_55_d_pre_offset; //314
        float turn_l90_v_60_d_pre_offset; //315
        float turn_l90_v_65_d_pre_offset; //316
        float turn_l90_v_70_d_pre_offset; //317
        float turn_l90_v_75_d_pre_offset; //318
        float turn_l90_v_80_d_pre_offset; //319
        float turn_180_v_35_d_pre_offset; //320
        float turn_180_v_40_d_pre_offset; //321
        float turn_180_v_45_d_pre_offset; //322
        float turn_180_v_50_d_pre_offset; //323
        float turn_180_v_55_d_pre_offset; //324
        float turn_180_v_60_d_pre_offset; //325
        float turn_180_v_65_d_pre_offset; //326
        float turn_180_v_70_d_pre_offset; //327
        float turn_180_v_75_d_pre_offset; //328
        float turn_180_v_80_d_pre_offset; //329
        float turn_d90_v_35_d_pre_offset; //330
        float turn_d90_v_40_d_pre_offset; //331
        float turn_d90_v_45_d_pre_offset; //332
        float turn_d90_v_50_d_pre_offset; //333
        float turn_d90_v_55_d_pre_offset; //334
        float turn_d90_v_60_d_pre_offset; //335
        float turn_d90_v_65_d_pre_offset; //336
        float turn_d90_v_70_d_pre_offset; //337
        float turn_d90_v_75_d_pre_offset; //338
        float turn_d90_v_80_d_pre_offset; //339
        float turn_45_v_35_d_pre_offset; //340
        float turn_45_v_40_d_pre_offset; //341
        float turn_45_v_45_d_pre_offset; //342
        float turn_45_v_50_d_pre_offset; //343
        float turn_45_v_55_d_pre_offset; //344
        float turn_45_v_60_d_pre_offset; //345
        float turn_45_v_65_d_pre_offset; //346
        float turn_45_v_70_d_pre_offset; //347
        float turn_45_v_75_d_pre_offset; //348
        float turn_45_v_80_d_pre_offset; //349
        float turn_135_v_35_d_pre_offset; //350
        float turn_135_v_40_d_pre_offset; //351
        float turn_135_v_45_d_pre_offset; //352
        float turn_135_v_50_d_pre_offset; //353
        float turn_135_v_55_d_pre_offset; //354
        float turn_135_v_60_d_pre_offset; //355
        float turn_135_v_65_d_pre_offset; //356
        float turn_135_v_70_d_pre_offset; //357
        float turn_135_v_75_d_pre_offset; //358
        float turn_135_v_80_d_pre_offset; //359
        ; //360
        ; //361
        ; //362
        ; //363
        ; //364
        ; //365
        ; //366
        ; //367
        ; //368
        ; //369
        ; //370
        ; //371
        ; //372
        ; //373
        ; //374
        ; //375
        ; //376
        ; //377
        ; //378
        ; //379
        ; //380
        ; //381
        ; //382
        ; //383
        ; //384
        ; //385
        ; //386
        ; //387
        ; //388
        ; //389
        ; //390
        ; //391
        ; //392
        ; //393
        ; //394
        ; //395
        ; //396
        ; //397
        ; //398
        ; //399
        //----管理下においた変数たち-----

    private:

        ParameterManager() {};
        ~ParameterManager() {};
        ParameterManager(ParameterManager&) {};
    };

}

