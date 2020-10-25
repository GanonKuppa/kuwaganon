#pragma once

#include <stdint.h>
#include <math.h>
#include <myUtil.h>

#include "iodefine.h"
#include "phaseCounting.h"
#include "parameterManager.h"
#include "timer.h"
#include <deque>



const uint16_t ENC_L_OUT_TABLE_SIZE = 83;
const float ENC_L_OUT_COUNT_TABLE[] = {
    0.0f,52.27881548f,104.4387755f,156.3566224f,207.8914904f,
    257.6347217f,307.4615921f,357.3544933f,406.4110039f,454.8864431f,
    502.7411924f,550.930498f,598.5695466f,646.1733787f,693.6803656f,
    740.1132509f,786.3084252f,832.543218f,878.7824128f,925.2108961f,
    970.5696797f,1016.377473f,1062.017988f,1107.191886f,1152.119268f,
    1196.113414f,1240.895528f,1285.673241f,1330.569809f,1375.708489f,
    1420.961624f,1466.382036f,1511.855273f,1558.112076f,1604.685827f,
    1651.594135f,1698.630102f,1746.414418f,1794.198735f,1842.83705f,
    1891.757097f,1940.958875f,1990.266303f,2040.12839f,2089.730755f,
    2139.381543f,2189.256836f,2239.523912f,2289.804194f,2340.379414f,
    2390.981046f,2441.613493f,2493.201186f,2545.594455f,2597.952507f,
    2650.579085f,2703.26289f,2755.717787f,2809.03989f,2862.256344f,
    2915.754528f,2968.830116f,3022.610033f,3076.319516f,3130.015794f,
    3183.615226f,3237.311503f,3291.447986f,3346.187551f,3401.076786f,
    3455.732711f,3510.054081f,3564.956522f,3618.850892f,3673.379158f,
    3727.229507f,3781.396805f,3835.308783f,3889.317606f,3942.864214f,
    3995.754915f,4048.271442f,4100.0f
};

const uint16_t ENC_R_OUT_TABLE_SIZE = 83;
const float ENC_R_OUT_COUNT_TABLE[] = {
    0.0f,50.20826355f,101.6434289f,153.190522f,205.172412f,
    256.7238101f,309.0630083f,360.9028359f,412.8502865f,464.7934322f,
    516.3146958f,567.7498611f,619.9383872f,672.6478083f,725.8092459f,
    778.7640474f,831.1979538f,884.1527553f,937.3314126f,991.1170633f,
    1044.274196f,1098.408545f,1152.788275f,1206.397424f,1260.028097f,
    1313.129266f,1366.437071f,1419.262725f,1471.718156f,1524.203721f,
    1576.913142f,1629.403013f,1682.784001f,1736.001403f,1789.825798f,
    1842.961406f,1895.554594f,1948.091819f,2000.27604f,2051.577753f,
    2102.784757f,2153.358939f,2203.657606f,2253.792686f,2303.41548f,
    2352.620698f,2401.335155f,2450.170149f,2498.40676f,2545.984719f,
    2592.279811f,2637.980825f,2683.475203f,2729.387158f,2774.881536f,
    2820.29412f,2865.97361f,2911.622965f,2958.193572f,3003.489924f,
    3049.160803f,3094.930696f,3141.277448f,3186.987072f,3233.518935f,
    3279.245778f,3325.209392f,3371.676682f,3418.669171f,3465.562648f,
    3512.619711f,3558.695253f,3605.847024f,3654.040586f,3702.600066f,
    3751.2069f,3799.292838f,3848.795095f,3899.115287f,3949.211622f,
    3999.200335f,4049.180438f,4100.0f
};





namespace umouse {

    class WheelOdometry {
      private:
        uint32_t R_ENC_pre;
        uint32_t L_ENC_pre;
        uint32_t R_ENC_now;
        uint32_t L_ENC_now;

        const float BIG_R = 500000.0;
        const uint32_t AVERAGE_NUM = 20;
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

        void updateZ() {
            if(PORTB.PIDR.BIT.B3 == 1) MTU2.TCNT = 0;
            if(PORTA.PIDR.BIT.B2 == 1)  MTU1.TCNT = 0;
            MTU2.TCNT = (MTU2.TCNT + 4096) % 4096;
            MTU1.TCNT = (MTU1.TCNT + 4096) % 4096;
        }



        void update() {
            R_ENC_pre = R_ENC_now;
            L_ENC_pre = L_ENC_now;
            R_ENC_now = (getCountMTU1() + 4096) % 4096;
            L_ENC_now = (getCountMTU2() + 4096) % 4096;
            int32_t count_diff_R = (int32_t) (R_ENC_now - R_ENC_pre);
            int32_t count_diff_L = (int32_t) (L_ENC_now - L_ENC_pre);
            float count_diff_R_f = enc_r_lerp(R_ENC_now) - enc_r_lerp(R_ENC_pre);
            float count_diff_L_f = enc_l_lerp(L_ENC_now) - enc_l_lerp(L_ENC_pre);

            //オーバーフロー対策
            if (count_diff_R > 2048)
                count_diff_R -= 4096;
            if (count_diff_R < -2048)
                count_diff_R += 4096;
            if (count_diff_L > 2048)
                count_diff_L -= 4096;
            if (count_diff_L < -2048)
                count_diff_L += 4096;

            if (count_diff_R_f > 2048)
                count_diff_R_f -= 4096;
            if (count_diff_R_f < -2048)
                count_diff_R_f += 4096;
            if (count_diff_L_f > 2048)
                count_diff_L_f -= 4096;
            if (count_diff_L_f < -2048)
                count_diff_L_f += 4096;


            ParameterManager& pm = ParameterManager::getInstance();
            //エンコーダより計測された速度

            v_R = (ENC_R_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (double)count_diff_R_f / DELTA_T;

            v_L = (ENC_L_DIR * PI * pm.dia_tire / GEAR_RATIO / ENC_RESOLUTION) *
                  (double)count_diff_L_f / DELTA_T;

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

