#pragma once

#include <stdint.h>
#include <math.h>
#include <myUtil.h>

#include "iodefine.h"
#include "phaseCounting.h"
#include "parameterManager.h"
#include "timer.h"
#include <deque>

#include "communication.h"






const uint16_t BLACKMAN_LPF_330Hz_SIZE = 31;
const float BLACKMANG_LPF_330Hz_DELAY = 0.015; // 15msec = サンプリング周波数が1msecのときの値
const float BLACKMAN_LPF_330Hz[] = {
    0.0000000000f,
    -0.0000624097f,
    0.0003974281f,
    -0.0002652724f, 
    -0.0016257931f,
    0.0039354990f, 
    -0.0013305556f, 
    -0.0088749263f,
    0.0166586791f, 
    -0.0033896433f, 
    -0.0324472815f, 
    0.0537294900f, 
    -0.0056578080f, 
    -0.1250131762f, 
    0.2739601272f, 
    0.6600000000f, 
    0.2739601272f, 
    -0.1250131762f, 
    -0.0056578080f, 
    0.0537294900f, 
    -0.0324472815f, 
    -0.0033896433f, 
    0.0166586791f, 
    -0.0088749263f, 
    -0.0013305556f, 
    0.0039354990f, 
    -0.0016257931f, 
    -0.0002652724f, 
    0.0003974281f, 
    -0.0000624097f, 
    0.0000000000f, 
};


const uint16_t BLACKMAN_LPF_125Hz_SIZE = 31;
const float BLACKMANG_LPF_125Hz_DELAY = 0.015; // 15msec = サンプリング周波数が1msecのときの値
const float BLACKMAN_LPF_125Hz[] = {
    0.00000000f,
    -0.00009117f, 
    -0.00029014f, 
    0.00000000f, 
    0.00157704f, 
    0.00413803f, 
    0.00502102f, 
    0.00000000f, 
    -0.01266914f, 
    -0.02704505f, 
    -0.02835996f, 
    0.00000000f, 
    0.06371462f, 
    0.14806235f, 
    0.22106309f, 
    0.25000000f, 
    0.22106309f, 
    0.14806235f, 
    0.06371462f, 
    0.00000000f, 
    -0.02835996f, 
    -0.02704505f, 
    -0.01266914f, 
    0.00000000f, 
    0.00502102f, 
    0.00413803f, 
    0.00157704f, 
    0.00000000f, 
    -0.00029014f, 
    -0.00009117f, 
    0.00000000f, 
};


const uint16_t BLACKMAN_LPF_50Hz_SIZE = 127;
const float BLACKMANG_LPF_50Hz_DELAY = 0.063; // 15msec = サンプリング周波数が1msecのときの値
const float BLACKMAN_LPF_50Hz[] = {
0.00000000f,
0.00000068f, 
0.00000145f, 
0.00000000f, 
-0.00000605f, 
-0.00001845f, 
-0.00003756f, 
-0.00006183f, 
-0.00008751f, 
-0.00010873f, 
-0.00011804f, 
-0.00010741f, 
-0.00006964f, 
0.00000000f, 
0.00010212f, 
0.00023174f, 
0.00037744f, 
0.00052129f, 
0.00063983f, 
0.00070625f, 
0.00069365f, 
0.00057911f, 
0.00034834f, 
0.00000000f, 
-0.00045067f, 
-0.00096989f, 
-0.00150555f, 
-0.00199013f, 
-0.00234640f, 
-0.00249576f, 
-0.00236861f, 
-0.00191561f, 
-0.00111869f, 
0.00000000f, 
0.00137249f, 
0.00288431f, 
0.00437955f, 
0.00567221f, 
0.00656320f, 
0.00686223f, 
0.00641230f, 
0.00511462f, 
0.00295089f, 
0.00000000f, 
-0.00355346f, 
-0.00742096f, 
-0.01122318f, 
-0.01451450f, 
-0.01681746f, 
-0.01766464f, 
-0.01664426f,
-0.01344527f, 
-0.00789715f, 
0.00000000f, 
0.01005888f, 
0.02190441f, 
0.03499045f, 
0.04863295f, 
0.06205746f, 
0.07445694f, 
0.08505490f, 
0.09316811f, 
0.09826292f, 
0.10000000f, 
0.09826292f, 
0.09316811f, 
0.08505490f, 
0.07445694f, 
0.06205746f, 
0.04863295f, 
0.03499045f, 
0.02190441f, 
0.01005888f, 
0.00000000f, 
-0.00789715f, 
-0.01344527f, 
-0.01664426f, 
-0.01766464f, 
-0.01681746f, 
-0.01451450f, 
-0.01122318f, 
-0.00742096f, 
-0.00355346f, 
0.00000000f, 
0.00295089f, 
0.00511462f, 
0.00641230f, 
0.00686223f, 
0.00656320f, 
0.00567221f, 
0.00437955f, 
0.00288431f, 
0.00137249f, 
0.00000000f, 
-0.00111869f, 
-0.00191561f, 
-0.00236861f, 
-0.00249576f, 
-0.00234640f, 
-0.00199013f, 
-0.00150555f, 
-0.00096989f, 
-0.00045067f, 
0.00000000f, 
0.00034834f, 
0.00057911f, 
0.00069365f, 
0.00070625f, 
0.00063983f, 
0.00052129f, 
0.00037744f, 
0.00023174f, 
0.00010212f, 
0.00000000f, 
-0.00006964f, 
-0.00010741f, 
-0.00011804f, 
-0.00010873f, 
-0.00008751f, 
-0.00006183f, 
-0.00003756f, 
-0.00001845f, 
-0.00000605f, 
0.00000000f, 
0.00000145f, 
0.00000068f, 
0.00000000f, 
};



namespace umouse {

    class WheelOdometry {
      private:
        uint32_t R_ENC_pre;
        uint32_t L_ENC_pre;
        uint32_t R_ENC_now;
        uint32_t L_ENC_now;

        const float BIG_R = 500000.0;
        const uint32_t AVERAGE_NUM = 40;
        const double DELTA_T = 0.001;
        const double TIRE_GEAR_NUM = 41.0;
        const double ENC_GEAR_NUM = 41.0;
        const double GEAR_RATIO = TIRE_GEAR_NUM / ENC_GEAR_NUM;
        const double ENC_RESOLUTION = 4096.0;
        const double ENC_R_DIR = -1.0; // 機体が進む方向にタイヤを回した場合に位相係数カウント値が増えるか減るか
        const double ENC_L_DIR = 1.0; // 増えるなら 1.0, 減るなら -1.0
        const double PI = 3.14159265358979323846264;

        WheelOdometry() {
            v = 0.0;
            a = 0.0;
            kappa = 0.0;
            ang_v = 0.0;
            r = BIG_R;
            v_R = 0.0;
            v_L = 0.0;

            R_ENC_pre = 0;
            L_ENC_pre = 0;
            R_ENC_now = 0;
            L_ENC_now = 0;

            ab_pos_x = 0.0;
            ab_pos_y = 0.0;
            ab_ang = 0.0;

            rpm_R = 0.0;
            rpm_L = 0.0;

            tire_ang_R = 0.0;
            tire_ang_L = 0.0;
            v_ave = 0.0;


            for (uint8_t i = 0; i < AVERAGE_NUM; i++) {
                v_R_list.push_front(0);
                v_L_list.push_front(0);
                v_list.push_front(0);
            }

        }

        ~WheelOdometry() {
        }


      public:
        double v;
        double a;
        double v_R;
        double v_L;
        double v_R_no_lerp;
        double v_L_no_lerp;

        float rpm_R;
        float rpm_L;
        float ang_v;
        float kappa;
        float r;
        float ab_pos_x;
        float ab_pos_y;
        float ab_ang;
        float tire_ang_R;
        float tire_ang_L;
        double v_ave;
        double v_R_ave;
        double v_L_ave;
        std::deque<float> v_list;
        std::deque<float> v_R_list;
        std::deque<float> v_L_list;


        const uint16_t ENC_L_OUT_TABLE_SIZE = 83;
        float ENC_L_OUT_COUNT_TABLE[83] = {
            0.0f,50.0f,100.0f,150.0f,200.0f,250.0f,300.0f,350.0f,400.0f,450.0f,
            500.0f,550.0f,600.0f,650.0f,700.0f,750.0f,800.0f,850.0f,900.0f,950.0f,
            1000.0f,1050.0f,1100.0f,1150.0f,1200.0f,1250.0f,1300.0f,1350.0f,1400.0f,1450.0f,
            1500.0f,1550.0f,1600.0f,1650.0f,1700.0f,1750.0f,1800.0f,1850.0f,1900.0f,1950.0f,
            2000.0f,2050.0f,2100.0f,2150.0f,2200.0f,2250.0f,2300.0f,2350.0f,2400.0f,2450.0f,
            2500.0f,2550.0f,2600.0f,2650.0f,2700.0f,2750.0f,2800.0f,2850.0f,2900.0f,2950.0f,
            3000.0f,3050.0f,3100.0f,3150.0f,3200.0f,3250.0f,3300.0f,3350.0f,3400.0f,3450.0f,
            3500.0f,3550.0f,3600.0f,3650.0f,3700.0f,3750.0f,3800.0f,3850.0f,3900.0f,3950.0f,
            4000.0f,4050.0f,4100.0f
        };

        const uint16_t ENC_R_OUT_TABLE_SIZE = 83;
        float ENC_R_OUT_COUNT_TABLE[83] = {
            0.0f,50.0f,100.0f,150.0f,200.0f,250.0f,300.0f,350.0f,400.0f,450.0f,
            500.0f,550.0f,600.0f,650.0f,700.0f,750.0f,800.0f,850.0f,900.0f,950.0f,
            1000.0f,1050.0f,1100.0f,1150.0f,1200.0f,1250.0f,1300.0f,1350.0f,1400.0f,1450.0f,
            1500.0f,1550.0f,1600.0f,1650.0f,1700.0f,1750.0f,1800.0f,1850.0f,1900.0f,1950.0f,
            2000.0f,2050.0f,2100.0f,2150.0f,2200.0f,2250.0f,2300.0f,2350.0f,2400.0f,2450.0f,
            2500.0f,2550.0f,2600.0f,2650.0f,2700.0f,2750.0f,2800.0f,2850.0f,2900.0f,2950.0f,
            3000.0f,3050.0f,3100.0f,3150.0f,3200.0f,3250.0f,3300.0f,3350.0f,3400.0f,3450.0f,
            3500.0f,3550.0f,3600.0f,3650.0f,3700.0f,3750.0f,3800.0f,3850.0f,3900.0f,3950.0f,
            4000.0f,4050.0f,4100.0f
        };





        void update() {
            R_ENC_pre = R_ENC_now;
            L_ENC_pre = L_ENC_now;
            R_ENC_now = (getCountMTU1() + 4096) % 4096;
            L_ENC_now = (getCountMTU2() + 4096) % 4096;
            MTU1.TCNT = R_ENC_now;
            MTU2.TCNT = L_ENC_now;

            int32_t count_diff_R = (int32_t) (R_ENC_now - R_ENC_pre);
            int32_t count_diff_L = (int32_t) (L_ENC_now - L_ENC_pre);
            float count_diff_R_f = enc_r_lerp(R_ENC_now) - enc_r_lerp(R_ENC_pre);
            float count_diff_L_f = enc_l_lerp(L_ENC_now) - enc_l_lerp(L_ENC_pre);

            //オーバーフロー対策
            if (count_diff_R > 2048){ count_diff_R -= 4096;}
            if (count_diff_R < -2048){ count_diff_R += 4096;}
            if (count_diff_L > 2048){ count_diff_L -= 4096;}
            if (count_diff_L < -2048){ count_diff_L += 4096;}

            if (count_diff_R_f > 2048.0f){ count_diff_R_f -= 4096.0f;}
            if (count_diff_R_f < -2048.0f){ count_diff_R_f += 4096.0f;}
            if (count_diff_L_f > 2048.0f){ count_diff_L_f -= 4096.0f;}
            if (count_diff_L_f < -2048.0f){ count_diff_L_f += 4096.0f;}


            ParameterManager& pm = ParameterManager::getInstance();
            //エンコーダより計測された速度

            v_R = (ENC_R_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (double)count_diff_R_f / DELTA_T;

            v_L = (ENC_L_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (double)count_diff_L_f / DELTA_T;

            v_R_no_lerp = (ENC_R_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (double)count_diff_R / DELTA_T;

            v_L_no_lerp = (ENC_L_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (double)count_diff_L / DELTA_T;


            double v_pre = v;
            v = (v_R + v_L) * 0.5;
            a = (v - v_pre) / DELTA_T;
            if( fabs(v_R + v_L) > 0.001)kappa = 2.0 * (v_R - v_L) / (pm.tread * (v_R + v_L));
            float ang_v_rad = (v_R - v_L) / pm.tread; //v * kappa;
            ang_v = RAD2DEG(ang_v_rad);

            if (kappa != 0.0)
                r = 1 / kappa;
            else
                r = BIG_R;

            ab_ang += ang_v * DELTA_T;
            ab_ang = fmodf(ab_ang + 360.0f, 360.0f);
            ab_pos_x += v * cosf(DEG2RAD(ab_ang)) * DELTA_T;
            ab_pos_y += v * sinf(DEG2RAD(ab_ang)) * DELTA_T;

            rpm_R = v_R * 60.0 /(PI * pm.dia_tire);
            rpm_L = v_L * 60.0 /(PI * pm.dia_tire);
            tire_ang_R += ENC_R_DIR * count_diff_R_f / ENC_RESOLUTION / GEAR_RATIO * 360.0;
            tire_ang_L += ENC_L_DIR * count_diff_L_f / ENC_RESOLUTION / GEAR_RATIO * 360.0;
            tire_ang_R = fmod(tire_ang_R + 360.0, 360.0);
            tire_ang_L = fmod(tire_ang_L + 360.0, 360.0);

            v_R_list.push_front(count_diff_R_f);
            v_R_list.pop_back();
            v_L_list.push_front(count_diff_L_f);
            v_L_list.pop_back();
            
        }

        float getLpfV(){
            ParameterManager& pm = ParameterManager::getInstance();
            float v_lpf = 0.0f;
            for(uint8_t i=0; i<BLACKMAN_LPF_50Hz_SIZE; i++){
                v_lpf += (SIGN(ENC_R_DIR) * v_R_list[i] + SIGN(ENC_L_DIR) * v_L_list[i]) * BLACKMAN_LPF_50Hz[i];
            }

            v_lpf = (PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (double)v_lpf  * 0.5 / DELTA_T;

            return v_lpf;
        }

        float getLpfDelay(){
            return BLACKMANG_LPF_50Hz_DELAY; 
        }


        double getAveV() {
            ParameterManager& pm = ParameterManager::getInstance();
            int32_t sum = 0;
            for (uint8_t i = 0; i < AVERAGE_NUM; i++) {
                sum += SIGN(ENC_R_DIR) * v_R_list[i] + SIGN(ENC_L_DIR) * v_L_list[i];
            }
            v_ave = (PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (double)sum  * 0.5 / DELTA_T  / AVERAGE_NUM;
            return v_ave;
        }

        double getAveV_R() {
            ParameterManager& pm = ParameterManager::getInstance();
            float sum  = 0;
            for (uint8_t i = 0; i < AVERAGE_NUM; i++) {
                sum += v_R_list[i];
            }
            v_R_ave = (ENC_R_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (double)sum / DELTA_T / AVERAGE_NUM;
            return v_R_ave;
        }

        double getAveV_L() {
            ParameterManager& pm = ParameterManager::getInstance();
            float sum;
            for (uint8_t i = 0; i < AVERAGE_NUM; i++) {
                sum += v_L_list[i];
            }
            v_L_ave = (ENC_L_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (double)sum / DELTA_T / AVERAGE_NUM;
            return v_L_ave;
        }


        float getV() {
            return (float)v;
        };

        float getA() {
            return (float)a;
        }

        double getV_double() {
            return v;
        }

        float getAng_v() {
            return ang_v;
        };

        float getKappa() {
            return -kappa;
        };

        float getV_R() {
            return (float)v_R;
        };

        float getV_L() {
            return (float)v_L;
        };

        float getV_L_COR() {
            

            float x =  (float)(L_ENC_now + L_ENC_pre)*0.5f / 4096.0f;
            float factor =  1.0f - 0.1f * sinf((2.0f*PI)*(x-0.1f));
            return (float)v_L_no_lerp * factor;
        }

        float getTireAng_L() {
            return tire_ang_L;
        }

        float getTireAng_R() {
            return tire_ang_R;
        }

        void resetTireAng() {
            tire_ang_R = 0.0;
            tire_ang_L = 0.0;
        }


        static WheelOdometry& getInstance() {
            static WheelOdometry instance;
            return instance;
        }

        uint16_t getAbsCount_R(){
            return (getCountMTU1() + 4096) % 4096;
        }

        uint16_t getAbsCount_L(){
            return (getCountMTU2() + 4096) % 4096;
        }

        float enc_l_lerp(uint16_t in){
            const uint16_t ENC_COUNT_PER_ROUND = 4096;
            uint16_t in_mod_round = in % ENC_COUNT_PER_ROUND;
            
            uint16_t index = (in_mod_round) / 50;
            float mod_index = (float)(in_mod_round - 50.0f * index) / 50.0f;      
            return ENC_L_OUT_COUNT_TABLE[index] + (ENC_L_OUT_COUNT_TABLE[index+1] - ENC_L_OUT_COUNT_TABLE[index]) * mod_index;
        }

        float enc_r_lerp(uint16_t in){
            const uint16_t ENC_COUNT_PER_ROUND = 4096;
            uint16_t in_mod_round = in % ENC_COUNT_PER_ROUND;
            
            uint16_t index = (in_mod_round) / 50;
            float mod_index = (float)(in_mod_round - 50.0f * index) / 50.0f;      
            return ENC_R_OUT_COUNT_TABLE[index] + (ENC_R_OUT_COUNT_TABLE[index+1] - ENC_R_OUT_COUNT_TABLE[index]) * mod_index;
        }




    };

} /* namespace robot_object */

