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

#include "ICM20602.h"

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

                turnLed(true, true, false, false);
                waitusec_sub(LED_ON_USEC);
                updateOnVal(true, true, false, false);

                turnLed(false, false, true, true);
                waitusec_sub(LED_ON_USEC);
                updateOnVal(false, false, true, true);
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
            if(right_err < 50 && left_err < 50) in_wall_center_time += DELTA_T;
            else in_wall_center_time = 0.0f;


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
            ParameterManager& pm = ParameterManager::getInstance();
            int16_t threshold = pm.wall_threshold_right;
            if (right_q.at(0) > threshold) return true;
            else return false;
        }

        bool isLeft() {
            ParameterManager& pm = ParameterManager::getInstance();
            int16_t threshold = pm.wall_threshold_left;
            if (left_q.at(0) > threshold) return true;
            else return false;
        }

        bool isAhead() {
            ParameterManager& pm = ParameterManager::getInstance();
            int16_t threshold_ar = pm.wall_threshold_ahead_r;
            int16_t threshold_al = pm.wall_threshold_ahead_l;
            if (ahead_r_q.at(0) > threshold_ar ||
                    ahead_l_q.at(0) > threshold_al ) {
                return true;
            }
            //else if( 0.09 < calcAheadWallDist() && calcAheadWallDist() < 0.135 ) return true;
            else return false;
        }

        bool isAhead_l() {
            ParameterManager& pm = ParameterManager::getInstance();
            int16_t threshold_al = pm.wall_threshold_ahead_l;
            if (ahead_l_q.at(0) > threshold_al ) return true;
            else return false;

        }

        bool isAhead_r() {
            ParameterManager& pm = ParameterManager::getInstance();
            int16_t threshold_ar = pm.wall_threshold_ahead_r;
            if (ahead_r_q.at(0) > threshold_ar ) return true;
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
            if(in_wall_center_time > 0.05) {
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
            float dist = 0.0;
            if(isAhead() && !isLeft()) {
                dist = hornerMethod(ahead_l(), WALL_LA_POLY6_CONSTANT, 7);
            } else if(isAhead() && !isRight()) {
                dist = hornerMethod(ahead_r(), WALL_RA_POLY6_CONSTANT, 7);
            } else {
                dist = (hornerMethod(ahead_l(), WALL_LA_POLY6_CONSTANT, 7) +
                        hornerMethod(ahead_r(), WALL_RA_POLY6_CONSTANT, 7)) * 0.5;
            }
            return constrainL(dist, 0.045);
        }


      private:
        const uint8_t BUFF_SIZE = 30;
        const uint16_t LED_ON_USEC = 20;
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
            right_off = startAD_AN100();//AD_SLED2
            right_off = startAD_AN100();//AD_SLED2
            left_off = startAD_AN001();//AD_SLED3
            left_off = startAD_AN001();//AD_SLED3
            ahead_r_off = startAD_AN002();//AD_SLED4
            ahead_r_off = startAD_AN002();//AD_SLED4
        }

        void updateAllOnVal() {
            ahead_l_on = startAD_AN101(); //AD_SLED1
            right_on = startAD_AN100();//AD_SLED2
            left_on = startAD_AN001();//AD_SLED3
            ahead_r_on = startAD_AN002();//AD_SLED4
        }

        void updateOnVal(bool sled1, bool sled2, bool sled3, bool sled4) {
            if(sled1 == true) {
                ahead_l_on = startAD_AN101(); //AD_SLED1
                ahead_l_on = startAD_AN101(); //AD_SLED1
            }
            if(sled2 == true) {
                right_on = startAD_AN100();//AD_SLED2
                right_on = startAD_AN100();//AD_SLED2
            }
            if(sled3 == true) {
                left_on = startAD_AN001();//AD_SLED3
                left_on = startAD_AN001();//AD_SLED3
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

            ahead_l_q.push_front(0.5 * (ahl_mod + ahead_l_q.at(0)) );
            ahead_r_q.push_front(0.5 * (ahr_mod + ahead_r_q.at(0)));
            left_q.push_front(0.5 * (l_mod + left_q.at(0)));
            right_q.push_front(0.5 * (r_mod + right_q.at(0)));

            ahead_l_q.pop_back();
            ahead_r_q.pop_back();
            left_q.pop_back();
            right_q.pop_back();
        }

        bool isCorner() {
            return (isCornerL() || isCornerR());
        }

        //y = 2E-21x6 - 3E-17x5 + 2E-13x4 - 4E-10x3 + 5E-07x2 - 0.0003x + 0.1612
        const float WALL_LA_POLY6_CONSTANT[7] = {
            0.000000000000000000002496979705f,
            -0.000000000000000031343123812308f,
            0.000000000000152350727274190000f,
            -0.000000000364670883847464000000f,
            0.000000455240592245447000000000f,
            -0.000300062017225004000000000000f,
            0.161200042128944000000000000000f
        };


        // ra
        //y = 3E-21x6 - 4E-17x5 + 2E-13x4 - 4E-10x3 + 5E-07x2 - 0.0003x + 0.1679
        const float WALL_RA_POLY6_CONSTANT[7] = {
            0.000000000000000000003120871021f,
            -0.000000000000000037347282693124f,
            0.000000000000173869092712112000f,
            -0.000000000401311698062999000000f,
            0.000000488601588712184000000000f,
            -0.000320169565262498000000000000f,
            0.167931370815903000000000000000f
        };

        float hornerMethod(float x, const float a[], int n) {
            float f = a[0];
            for (int i = 1; i < n; i++)
                f = f*x + a[i];
            return f;
        }




    };

} //end namespace

