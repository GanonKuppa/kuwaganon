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



namespace umouse {

    class WheelOdometry {
      private:
        uint32_t R_ENC_pre;
        uint32_t L_ENC_pre;
        uint32_t R_ENC_now;
        uint32_t L_ENC_now;

        const float BIG_R = 500000.0;
        const uint32_t AVERAGE_NUM = 30;
        const float DELAY_TIME = (AVERAGE_NUM / 2.0) * DELTA_T;
        const float DELTA_T = 0.001;
        const float TIRE_GEAR_NUM = 41.0;
        const float ENC_GEAR_NUM = 41.0;
        const float GEAR_RATIO = TIRE_GEAR_NUM / ENC_GEAR_NUM;
        const float ENC_RESOLUTION = 4096.0;
        const float ENC_R_DIR = -1.0; // 機体が進む方向にタイヤを回した場合に位相係数カウント値が増えるか減るか
        const float ENC_L_DIR = 1.0; // 増えるなら 1.0, 減るなら -1.0
        const float PI = 3.14159265358979323846264;

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
                v_R_list.push_front(0.0f);
                v_L_list.push_front(0.0f);                
            }

        }

        ~WheelOdometry() {
        }


      public:
        float v;
        float a;
        float v_R;
        float v_L;
        float v_R_no_lerp;
        float v_L_no_lerp;

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
        float v_ave;
        float v_R_ave;
        float v_L_ave;        
        std::deque<float> v_R_list;
        std::deque<float> v_L_list;
        int32_t count_diff_R;
        int32_t count_diff_L;

        int32_t getCountDiffR(){
            int32_t diff = count_diff_R;
            if (diff > 2048){ diff -= 4096;}
            if (diff < -2048){ diff += 4096;}
            return diff;
        }

        int32_t getCountDiffL(){
            int32_t diff = count_diff_L;
            if (diff > 2048){ diff -= 4096;}
            if (diff < -2048){ diff += 4096;}
            return diff;
        }


        void update() {
            R_ENC_pre = R_ENC_now;
            L_ENC_pre = L_ENC_now;
            R_ENC_now = (getCountMTU1() + 4096) % 4096;
            L_ENC_now = (getCountMTU2() + 4096) % 4096;
            MTU1.TCNT = R_ENC_now;
            MTU2.TCNT = L_ENC_now;

            count_diff_R = (int32_t) (R_ENC_now - R_ENC_pre);
            count_diff_L = (int32_t) (L_ENC_now - L_ENC_pre);

            //オーバーフロー対策
            if (count_diff_R > 2048){ count_diff_R -= 4096;}
            if (count_diff_R < -2048){ count_diff_R += 4096;}
            if (count_diff_L > 2048){ count_diff_L -= 4096;}
            if (count_diff_L < -2048){ count_diff_L += 4096;}


            ParameterManager& pm = ParameterManager::getInstance();
            //エンコーダより計測された速度

            v_R = (ENC_R_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (float)count_diff_R / DELTA_T;

            v_L = (ENC_L_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (float)count_diff_L / DELTA_T;

            float v_pre = v;
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
            tire_ang_R += ENC_R_DIR * count_diff_R / ENC_RESOLUTION / GEAR_RATIO * 360.0;
            tire_ang_L += ENC_L_DIR * count_diff_L / ENC_RESOLUTION / GEAR_RATIO * 360.0;
            tire_ang_R = fmod(tire_ang_R + 360.0, 360.0);
            tire_ang_L = fmod(tire_ang_L + 360.0, 360.0);

            v_R_list.push_front(count_diff_R);
            v_R_list.pop_back();
            v_L_list.push_front(count_diff_L);
            v_L_list.pop_back();
            
        }



        float getAveV() {
            ParameterManager& pm = ParameterManager::getInstance();
            int32_t sum = 0;
            for (uint8_t i = 0; i < AVERAGE_NUM; i++) {
                sum += SIGN(ENC_R_DIR) * v_R_list[i] + SIGN(ENC_L_DIR) * v_L_list[i];
            }
            v_ave = (PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (float)sum  * 0.5 / DELTA_T  / AVERAGE_NUM;
            return v_ave;
        }

        float getAveV_R() {
            ParameterManager& pm = ParameterManager::getInstance();
            float sum  = 0;
            for (uint8_t i = 0; i < AVERAGE_NUM; i++) {
                sum += v_R_list[i];
            }
            v_R_ave = (ENC_R_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (float)sum / DELTA_T / AVERAGE_NUM;
            return v_R_ave;
        }

        float getAveV_L() {
            ParameterManager& pm = ParameterManager::getInstance();
            float sum;
            for (uint8_t i = 0; i < AVERAGE_NUM; i++) {
                sum += v_L_list[i];
            }
            v_L_ave = (ENC_L_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (float)sum / DELTA_T / AVERAGE_NUM;
            return v_L_ave;
        }


        float getV() {
            return v;
        };

        float getA() {
            return a;
        }

        float getAng_v() {
            return ang_v;
        };

        float getKappa() {
            return -kappa;
        };

        float getV_R() {
            return v_R;
        };

        float getV_L() {
            return v_L;
        };


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

        float getDelay(){
            return DELAY_TIME;
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

    };

} /* namespace robot_object */

