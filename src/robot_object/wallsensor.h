#pragma once

#include <stdint.h>

#include "myUtil.h"
#include "iodefine.h"
#include "parameterManager.h"
#include "ad.h"
#include <deque>
#include "communication.h"
#include "timer.h"
#include "wheelOdometry.h"
#include "pwm.h"
#include <math.h>

#include "ICM20602.h"

constexpr int L_WALL_DIST_TABLE_SIZE = 54;
constexpr float L_WALL_DIST_TABLE_IN[L_WALL_DIST_TABLE_SIZE] = {
    50.0f,100.0f,150.0f,200.0f,250.0f,300.0f,350.0f,400.0f,450.0f,500.0f,
    550.0f,600.0f,650.0f,700.0f,750.0f,800.0f,850.0f,900.0f,950.0f,
    1000.0f,1050.0f,1100.0f,1150.0f,1200.0f,1250.0f,1300.0f,1350.0f,1400.0f,1450.0f,
    1500.0f,1550.0f,1600.0f,1650.0f,1700.0f,1750.0f,1800.0f,1850.0f,1900.0f,1950.0f,
    2000.0f,2050.0f,2100.0f,2150.0f,2200.0f,2250.0f,2300.0f,2350.0f,2400.0f,2450.0f,
    2500.0f,2550.0f,2600.0f,2650.0f,2700.0f
};

constexpr float L_WALL_DIST_TABLE_OUT[L_WALL_DIST_TABLE_SIZE] = {
    0.09099136f,0.069727587f,0.059674061f,0.053432946f,0.049045106f,
    0.045728829f,0.04310052f,0.0409462f,0.039135513f,0.037583754f,
    0.036233089f,0.035042458f,0.033981763f,0.033028359f,0.032164819f,
    0.031377482f,0.030655455f,0.029989934f,0.029373708f,0.028800806f,
    0.028266234f,0.027765778f,0.027295853f,0.026853385f,0.026435723f,
    0.026040564f,0.025665898f,0.025309961f,0.024971195f,0.024648222f,
    0.024339815f,0.024044877f,0.023762427f,0.023491581f,0.023231541f,
    0.022981586f,0.022741059f,0.022509366f,0.02228596f,0.022070345f,
    0.021862064f,0.021660697f,0.021465859f,0.021277193f,0.02109437f,
    0.020917084f,0.020745054f,0.020578017f,0.020415728f,0.020257958f,
    0.020104497f,0.019955144f,0.019809715f,0.019668034f
};

constexpr int R_WALL_DIST_TABLE_SIZE = 54;
constexpr float R_WALL_DIST_TABLE_IN[R_WALL_DIST_TABLE_SIZE] = {
    50.0f,100.0f,150.0f,200.0f,250.0f,300.0f,350.0f,400.0f,450.0f,500.0f,
    550.0f,600.0f,650.0f,700.0f,750.0f,800.0f,850.0f,900.0f,950.0f,
    1000.0f,1050.0f,1100.0f,1150.0f,1200.0f,1250.0f,1300.0f,1350.0f,1400.0f,1450.0f,
    1500.0f,1550.0f,1600.0f,1650.0f,1700.0f,1750.0f,1800.0f,1850.0f,1900.0f,1950.0f,
    2000.0f,2050.0f,2100.0f,2150.0f,2200.0f,2250.0f,2300.0f,2350.0f,2400.0f,2450.0f,
    2500.0f,2550.0f,2600.0f,2650.0f,2700.0f
};

constexpr float R_WALL_DIST_TABLE_OUT[R_WALL_DIST_TABLE_SIZE] = {
0.089668631f,0.068003235f,0.057845444f,0.05157255f,0.047179302f,
0.043869046f,0.041252134f,0.039111785f,0.037316231f,0.035780017f,
0.034444894f,0.033269573f,0.032223829f,0.031284949f,0.030435478f,
0.029661743f,0.028952857f,0.028300024f,0.027696051f,0.027134984f,
0.026611848f,0.026122449f,0.025663219f,0.025231104f,0.024823469f,
0.024438029f,0.024072788f,0.023725998f,0.023396115f,0.023081773f,
0.022781757f,0.022494985f,0.022220482f,0.021957377f,0.02170488f,
0.021462279f,0.021228927f,0.021004236f,0.020787667f,0.020578732f,
0.020376978f,0.020181994f,0.019993399f,0.019810841f,0.019633998f,
0.019462569f,0.019296276f,0.01913486f,0.018978081f,0.018825716f,
0.018677556f,0.018533405f,0.01839308f,0.018256412f
};


constexpr int LA_WALL_DIST_TABLE_SIZE = 31;
constexpr float LA_WALL_DIST_TABLE_IN[LA_WALL_DIST_TABLE_SIZE] = {
    25.0f,50.0f,100.0f,150.0f,200.0f,
    250.0f,300.0f,350.0f,400.0f,450.0f,
    500.0f,550.0f,600.0f,650.0f,700.0f,
    750.0f,800.0f,850.0f,900.0f,950.0f,
    1000.0f,1050.0f,1100.0f,1150.0f,1200.0f,
    1250.0f,1300.0f,1350.0f,1400.0f,1450.0f,
    1500.0f
};

constexpr float LA_WALL_DIST_TABLE_OUT[LA_WALL_DIST_TABLE_SIZE] = {
    0.119702211f,0.097235147f,0.078984956f,0.069941447f,0.064160167f,
    0.060007036f,0.056814046f,0.054247331f,0.052117862f,0.05030903f,
    0.048744237f,0.047370679f,0.046150544f,0.0450559f,0.044065579f,
    0.043163187f,0.042335793f,0.041573027f,0.040866463f,0.040209167f,
    0.039595368f,0.03902022f,0.038479615f,0.037970044f,0.037488488f,
    0.037032333f,0.036599299f,0.036187392f,0.035794853f,0.035420127f,
    0.035061833f
};

constexpr int RA_WALL_DIST_TABLE_SIZE = 31;
constexpr float RA_WALL_DIST_TABLE_IN[RA_WALL_DIST_TABLE_SIZE]{
    25.0f,50.0f,100.0f,150.0f,200.0f,
    250.0f,300.0f,350.0f,400.0f,450.0f,
    500.0f,550.0f,600.0f,650.0f,700.0f,
    750.0f,800.0f,850.0f,900.0f,950.0f,
    1000.0f,1050.0f,1100.0f,1150.0f,1200.0f,
    1250.0f,1300.0f,1350.0f,1400.0f,1450.0f,
    1500.0f
};

constexpr float RA_WALL_DIST_TABLE_OUT[RA_WALL_DIST_TABLE_SIZE] = {
    0.134209416,0.108409102,0.087568621,0.077288059,0.070734497,
    0.066036339,0.062430263,0.059535438,0.057136552,0.055100946,
    0.053341564,0.051798454,0.050428718,0.04920069,0.048090391,
    0.047079261,0.046152666,0.045298882,0.044508383,0.043773335,
    0.043087223,0.042444576,0.04184076,0.041271816,0.04073434,
    0.040225386,0.039742388,0.039283096,0.038845532,0.038427945,
    0.038028781
};



namespace umouse {

    class WallSensor {

      public:

        int16_t ahead_l_on;
        int16_t ahead_r_on;
        int16_t left_on;
        int16_t right_on;

        int16_t ahead_l_off;
        int16_t ahead_r_off;
        int16_t left_off;
        int16_t right_off;

        std::deque<int16_t> ahead_l_q;
        std::deque<int16_t> ahead_r_q;
        std::deque<int16_t> left_q;
        std::deque<int16_t> right_q;

        void turnOffAllLed() {
            PORTE.PODR.BIT.B0 = 0; //SLED_OUT1
            PORTD.PODR.BIT.B7 = 0;//SLED_OUT2
            PORTD.PODR.BIT.B0 = 0;//SLED_OUT3
            PORT0.PODR.BIT.B7 = 0;//SLED_OUT4
        }

        static WallSensor& getInstance() {
            static WallSensor instance;
            return instance;
        }

        void setEnable(bool en) {
            enable = en;
            if (enable == false)
                turnOffAllLed();
        }

        void update() {
            if(enable == true) {
                updateAllOffVal();

                turnLed(true, false, true, false);
                waitusec_sub(LED_ON_USEC);
                updateOnVal(true, false, true, false);

                turnLed(false, true, false, true);
                waitusec_sub(LED_ON_USEC);
                updateOnVal(false, true, false, true);
                /*
                 turnLed(false, false, true, false);
                 waitusec_sub(LED_ON_USEC);
                 updateOnVal(true, false, true, false);

                 turnLed(false, false, false, true);
                 waitusec_sub(LED_ON_USEC);
                 updateOnVal(false, false, false, true);
                 */
                turnOffAllLed();

                modulateVal();
                if(isContactWall() == true) contact_wall_time += DELTA_T;
                else contact_wall_time = 0.0f;
            } else {
                turnOffAllLed();
            }

            float right_err = ABS(right() - center_r());
            float left_err = ABS(left() - center_l());
            
            if(isLeft() && isRight()){
                if(right_err < 30 && left_err < 30) in_wall_center_time += DELTA_T;
            }
/*            else if(isLeft()){
                if(left_err < 10) in_wall_center_time += DELTA_T;
            }
            else if(isRight()){
                if(right_err < 10) in_wall_center_time += DELTA_T;
            }*/
            else {
                in_wall_center_time = 0.0f;
            }
            

            if(isAhead() == true) ahead_on_time += DELTA_T;
            else ahead_on_time = 0.0f;

        }

        void debug() {
            printfAsync("=============================\n");
            printfAsync("ON : al, r, l, ar: %d, %d, %d, %d\n", ahead_l_on, right_on, left_on, ahead_r_on);
            printfAsync("OFF: al, r, l, ar: %d, %d, %d, %d\n", ahead_l_off, right_off, left_off, ahead_r_off);
            printfAsync("MOD: al, r, l, ar: %d, %d, %d, %d\n", ahead_l_q.at(0), right_q.at(0), left_q.at(0), ahead_r_q.at(0));
            printfAsync("=============================\n");
        }

        bool isRight() {
            if(dist_r() < 0.065) return true;
            else return false;
        }

        bool isLeft() {
            if(dist_l() < 0.065) return true;
            else return false;
        }

        bool isAhead() {
            return (isAhead_l() || isAhead_r() );
        }

        bool isAhead_l() {
            if(ahead_dist_l() < 0.115) return true;
            else return false;

        }

        bool isAhead_r() {
            if(ahead_dist_r() < 0.115) return true;
            else return false;
        }

        bool isAheadCloseWall() {return 0;}

        bool isRight_for_ctrl() {
            ParameterManager& pm = ParameterManager::getInstance();
            int16_t d_RS = right_q.at(0) - right_q.at(2);
            int16_t threshold = pm.wall_ctrl_threshold_right;
            int16_t threshold_delta = pm.wall_ctrl_threshold_delta_right;
            int16_t threshold_add_val = pm.wall_ctrl_add_val_right;

            int16_t threshold_ctrl = threshold;
            if (ABS(d_RS) > threshold_delta) threshold_ctrl += threshold_add_val;
            if (right_q.at(0) > threshold_ctrl ) return true;
            else return false;

        }
        bool isLeft_for_ctrl() {
            ParameterManager& pm = ParameterManager::getInstance();
            int16_t d_LS = left_q.at(0) - left_q.at(2);
            int16_t threshold = pm.wall_ctrl_threshold_left;
            int16_t threshold_delta = pm.wall_ctrl_threshold_delta_left;
            int16_t threshold_add_val = pm.wall_ctrl_add_val_left;

            int16_t threshold_ctrl = threshold;
            if (ABS(d_LS) > threshold_delta) threshold_ctrl += threshold_add_val;
            if (left_q.at(0) > threshold_ctrl ) return true;
            else return false;

        }

        int16_t right() {return right_q.at(0);}
        int16_t left() {return left_q.at(0);}
        int16_t ahead_l() {return ahead_l_q.at(0);}
        int16_t ahead_r() {return ahead_r_q.at(0);}

        int16_t center_r() {
            ParameterManager& pm = ParameterManager::getInstance();
            return pm.wall_center_r;
        }
        int16_t center_l() {
            ParameterManager& pm = ParameterManager::getInstance();
            return pm.wall_center_l;
        }

        bool isContactWall() {
            ParameterManager& pm = ParameterManager::getInstance();
            float r_th = pm.wall_contact_threshold_right;
            float l_th = pm.wall_contact_threshold_left;
            float r_a_th = pm.wall_contact_threshold_ahead_r;
            float l_a_th = pm.wall_contact_collision_threshold_ahead_l;

            if(right() > r_th && left() > l_th && ahead_l() > l_a_th && ahead_r() > r_a_th) return true;
            else return false;
        }

        float getContactWallTime() {
            return contact_wall_time;
        }

        float getAheadOnTime() {
            return ahead_on_time;
        }

        bool isOnWallCenter() {
            if(in_wall_center_time > 0.15) {
                return true;
            } else return false;

        }

        float getOnWallCenterTime() {
            return in_wall_center_time;
        }

        void setWallCenterVal() {
            ParameterManager& pm = ParameterManager::getInstance();
            pm.wall_center_r = right();
            pm.wall_center_l = left();
            pm.write<uint16_t>(120, pm.wall_center_r);
            pm.write<uint16_t>(121, pm.wall_center_l);
            printfAsync("----------\n");
            printfAsync("wall sensor l r: %d, %d\n",pm.wall_center_l, pm.wall_center_r);
            printfAsync("----------\n");
        };

        bool isCornerL() {
            ParameterManager& pm = ParameterManager::getInstance();
            int16_t th_on = pm.wall_corner_threshold_on_l;
            int16_t th_off = pm.wall_corner_threshold_off_l;

            if(left_q.at(0) < th_off) {
                for(int i=1; i<BUFF_SIZE; i++) {
                    if(left_q.at(i)> th_on) {
                        return true;
                    }
                }
            }
            return false;

        }

        bool isCornerR() {
            ParameterManager& pm = ParameterManager::getInstance();
            int16_t th_on = pm.wall_corner_threshold_on_r;
            int16_t th_off = pm.wall_corner_threshold_off_r;

            if(right_q.at(0) < th_off) {
                for(int i=1; i<BUFF_SIZE; i++) {
                    if(right_q.at(i)> th_on) {
                        return true;
                    }
                }
            }
            return false;
        }

        float calcAheadWallDist() {
            float ahead_dist_l = linearInterpolation( ahead_l(), LA_WALL_DIST_TABLE_IN, LA_WALL_DIST_TABLE_OUT, LA_WALL_DIST_TABLE_SIZE);
            float ahead_dist_r = linearInterpolation( ahead_r(), RA_WALL_DIST_TABLE_IN, RA_WALL_DIST_TABLE_OUT, RA_WALL_DIST_TABLE_SIZE);
            if (ahead_dist_l > 0.0115 && ahead_dist_r > 0.0115) return -1.0f;
            else return ahead_dist_l + ahead_dist_r;
        }



        float calcAheadAngle(){
            const float lr_dist = 0.03323;
            float ahead_dist_l = linearInterpolation( ahead_l(), LA_WALL_DIST_TABLE_IN, LA_WALL_DIST_TABLE_OUT, LA_WALL_DIST_TABLE_SIZE);
            float ahead_dist_r = linearInterpolation( ahead_r(), RA_WALL_DIST_TABLE_IN, RA_WALL_DIST_TABLE_OUT, RA_WALL_DIST_TABLE_SIZE);
            return  atan2f(ahead_dist_l - ahead_dist_r ,lr_dist) * 180.0f/3.14159265f;
        }

        float ahead_dist_l() {
            float dist = linearInterpolation( ahead_l(), LA_WALL_DIST_TABLE_IN, LA_WALL_DIST_TABLE_OUT, LA_WALL_DIST_TABLE_SIZE);
            return dist;
        }
        
        float ahead_dist_r() {
            float dist = linearInterpolation( ahead_r(), RA_WALL_DIST_TABLE_IN, RA_WALL_DIST_TABLE_OUT, RA_WALL_DIST_TABLE_SIZE);
            return dist;            
        }

        float dist_l() {
            float dist = linearInterpolation( left(), L_WALL_DIST_TABLE_IN, L_WALL_DIST_TABLE_OUT, L_WALL_DIST_TABLE_SIZE);
            return dist;
        }
        
        float dist_r() {
            float dist = linearInterpolation( right(), R_WALL_DIST_TABLE_IN, R_WALL_DIST_TABLE_OUT, R_WALL_DIST_TABLE_SIZE);
            return dist;            
        }

        float center_dist_l() {
            float dist = linearInterpolation( left(), L_WALL_DIST_TABLE_IN, L_WALL_DIST_TABLE_OUT, L_WALL_DIST_TABLE_SIZE);
            return 0.045f - dist;
        }
        
        float center_dist_r() {
            float dist = linearInterpolation( right(), R_WALL_DIST_TABLE_IN, R_WALL_DIST_TABLE_OUT, R_WALL_DIST_TABLE_SIZE);
            return 0.045f -dist;            
        }



      private:
        const uint8_t BUFF_SIZE = 30;
        const uint16_t LED_ON_USEC = 70;
        bool enable;
        float in_wall_center_time;
        float contact_wall_time;
        float ahead_on_time;
        const float DELTA_T = 0.0005;



        WallSensor() {

            enable = true;
            in_wall_center_time = 0.0f;
            contact_wall_time = 0.0f;
            ahead_on_time = 0.0f;
            ahead_l_on = 0;
            ahead_r_on = 0;
            left_on = 0;
            right_on = 0;

            ahead_l_off = 0;
            ahead_r_off = 0;
            left_off = 0;
            right_off = 0;


            for (uint8_t i = 0; i < BUFF_SIZE; i++) {
                ahead_l_q.push_front(0);
                ahead_r_q.push_front(0);
                right_q.push_front(0);
                left_q.push_front(0);
            }

        }
        ~WallSensor() {}

        void turnOnAllLed() {
            PORTE.PODR.BIT.B0 = 1; //SLED_OUT1
            PORTD.PODR.BIT.B7 = 1;//SLED_OUT2
            PORTD.PODR.BIT.B0 = 1;//SLED_OUT3
            PORT0.PODR.BIT.B7 = 1;//SLED_OUT4
        }

        void updateAllOffVal() {
            ahead_l_off = startAD_AN101(); //AD_SLED1
            ahead_l_off = startAD_AN101(); //AD_SLED1
            ahead_l_off = startAD_AN101(); //AD_SLED1
            ahead_l_off = startAD_AN101(); //AD_SLED1
            left_off = startAD_AN100();//AD_SLED2
            left_off = startAD_AN100();//AD_SLED2
            right_off = startAD_AN001();//AD_SLED3
            right_off = startAD_AN001();//AD_SLED3
            ahead_r_off = startAD_AN002();//AD_SLED4
            ahead_r_off = startAD_AN002();//AD_SLED4
        }

        void updateAllOnVal() {
            ahead_l_on = startAD_AN101(); //AD_SLED1
            left_on = startAD_AN100();//AD_SLED2
            right_on = startAD_AN001();//AD_SLED3
            ahead_r_on = startAD_AN002();//AD_SLED4
        }

        void updateOnVal(bool sled1, bool sled2, bool sled3, bool sled4) {
            if(sled1 == true) {
                ahead_l_on = startAD_AN101(); //AD_SLED1
                ahead_l_on = startAD_AN101(); //AD_SLED1
            }
            if(sled2 == true) {
                left_on = startAD_AN100();//AD_SLED2
                left_on = startAD_AN100();//AD_SLED2
            }
            if(sled3 == true) {
                right_on = startAD_AN001();//AD_SLED3
                right_on = startAD_AN001();//AD_SLED3
            }
            if(sled4 == true) {
                ahead_r_on = startAD_AN002();//AD_SLED4
                ahead_r_on = startAD_AN002();//AD_SLED4
            }
        }

        void turnLed(bool sled1, bool sled2, bool sled3, bool sled4) {
            if(sled1 == true) PORTE.PODR.BIT.B0 = 1; //SLED_OUT1
            else PORTE.PODR.BIT.B0 = 0;
            if(sled2 == true) PORTD.PODR.BIT.B7 = 1;//SLED_OUT2
            else PORTD.PODR.BIT.B7 = 0;
            if(sled3 == true) PORTD.PODR.BIT.B0 = 1;//SLED_OUT3
            else PORTD.PODR.BIT.B0 = 0;
            if(sled4 == true) PORT0.PODR.BIT.B7 = 1;//SLED_OUT4
            else PORT0.PODR.BIT.B7 = 0;
        };

        void modulateVal() {
            int16_t ahl_mod = ahead_l_on - ahead_l_off;
            int16_t ahr_mod = ahead_r_on - ahead_r_off;
            int16_t l_mod = left_on - left_off;
            int16_t r_mod = right_on - right_off;

            ahead_l_q.push_front(0.2 * ahl_mod + 0.8*ahead_l_q.at(0));
            ahead_r_q.push_front(0.2 * ahr_mod + 0.8*ahead_r_q.at(0));
            left_q.push_front(0.2 * l_mod + 0.8*left_q.at(0));
            right_q.push_front(0.2 * r_mod + 0.8*right_q.at(0));

            ahead_l_q.pop_back();
            ahead_r_q.pop_back();
            left_q.pop_back();
            right_q.pop_back();
        }

        bool isCorner() {
            return (isCornerL() || isCornerR());
        }


        float linearInterpolation(float in, const float* table_in, const float* table_out, int table_size){
            if(table_size == 1){
                return table_out[0];
            }
            if(in < table_in[0]){
                return table_out[0];
            } 
            if(in > table_in[table_size - 1]){
                return table_out[table_size-1];
            } 
            
            int head = 0;
            int tail = table_size;
            int index = 0;
            
            while(head <= tail){
                index = (head + tail) / 2;
                if( table_in[index] < in && table_in[index+1] > in  ){
                    break;
                } 
                else if(table_in[index] < in){
                    head = index + 1;
                }
                else{
                    tail = index - 1;
                }                
            }

            float rasio = (in - table_in[index]) /(table_in[index+1] - table_in[index]);
            float out = table_out[index] + (table_out[index+1] - table_out[index]) * rasio;
            return out;
        }
    };

} //end namespace

