#pragma once
#include <math.h>
#include "ICM20602.h"
#include "adis16470.h"
#include "sound.h"
#include "trajectory.h"
#include "communication.h"
#include "parameterManager.h"
#include "wallSensor.h"
#include "fcled.h"

namespace umouse {


    class PositionEstimator {
      public:

        PositionEstimator() {
            x = 0.09 / 2.0;
            y = 0.09 / 2.0;
            ang = 90.0;
            ang_v = 0.0;
            v = 0.0;
            x_d = 0.0;
            y_d = 0.0;

            contact_wall_cool_down_time = 0.0f;
        }

        PositionEstimator(float x_, float y_, float ang_) {
            x = (double)x_;
            y = (double)y_;
            ang = (double)ang_;
            ang_v = 0.0;
            v = 0.0;
            x_d = 0.0;
            y_d = 0.0;

            contact_wall_cool_down_time = 0.0f;
        }

        void reset(float x_, float y_, float ang_) {
            x = (double)x_;
            y = (double)y_;
            ang = (double)ang_;

            ang_v = 0.0;
            v = 0.0;

            ang_v_1 = 0.0;
            ang_v_2 = 0.0;

            x_d = 0.0;
            y_d = 0.0;

            x_d_1 = 0.0;
            x_d_2 = 0.0;

            y_d_1 = 0.0;
            y_d_2 = 0.0;

        }


        void update(double v_, double ang_v_, double a_y, double a_x, EMotionType motion_type, WallSensor& ws) {
            ICM20602& icm = ICM20602::getInstance();
            umouse::adis16470& adis = umouse::adis16470::getInstance();
            ParameterManager& pm = ParameterManager::getInstance();

            double ang_v_rad = deg2rad(ang_v);
            double sin_val;
            double cos_val;
            double beta_dot;

            // 角度算出
            alpha = (ang_v_ - ang_v) / DELTA_T;
            ang_v = ang_v_;
            ang += calcAdamsBashforthDelta(ang_v, ang_v_1, ang_v_2);
            ang = fmod(ang + 360.0, 360.0);

            double ang_rad = deg2rad(ang);
            sincos(ang_rad, &sin_val, &cos_val);

            if(motion_type == EMotionType::STRAIGHT ||
                    motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                    motion_type == EMotionType::DIAGONAL ||
                    motion_type == EMotionType::DIAGONAL_CENTER ||
                    motion_type == EMotionType::CURVE) {
                // 並進速度算出
                double gain = (double)pm.v_comp_gain;
                //a_y = (double)adis.acc_f[2];
                //if(fabs(a_y) < 0.5) gain = 0.0;
                v = (gain)*(v + a_y * DELTA_T) + (1.0 - gain)*(v_);

                // 加速度積分速度算出
                v_acc += a_y * DELTA_T;
                if(fabs(v_) < 0.05 || fabs(a_y) < 0.25) v_acc = v_;

                // グローバル座標系速度算出
                x_d = v * cos_val;
                y_d = v * sin_val;

                // グローバル座標系位置算出
                if(fabs(ang_v) > 10.0) {
                    x += x_d * sin(ang_v_rad * DELTA_T * 0.5) / (ang_v_rad * 0.5);
                    y += y_d * sin(ang_v_rad * DELTA_T * 0.5) / (ang_v_rad * 0.5);

                } else {
                    x += calcAdamsBashforthDelta(x_d, x_d_1, x_d_2);
                    y += calcAdamsBashforthDelta(y_d, y_d_1, y_d_2);
                }
            } else if(motion_type == EMotionType::CURVE) {
                // 並進速度算出
                v = v_;

                // 加速度積分速度算出
                v_acc += a_y * DELTA_T;
                if(fabs(v_) < 0.005 || fabs(a_y) < 0.25) v_acc = v_;

                // グローバル座標系速度算出

                double x_dd =   a_x * sin_val + a_y * cos_val;
                double y_dd = - a_x * cos_val + a_y * sin_val;

                if (ABS(v_) < 0.005) {
                    x_d = 0.0;
                    y_d = 0.0;
                } else if(fabs(v_acc - v_) > 0.3) {
                    x_d += x_dd * DELTA_T;
                    y_d += y_dd * DELTA_T;
                    v_acc = sqrt(x_d * x_d + y_d * y_d);
                } else {
                    x_d = v * cos_val;
                    y_d = v * sin_val;
                }

                // グローバル座標系位置算出
                if(fabs(ang_v) > 10.0 && fabs(v_acc - v_) <= 0.1) {
                    x += x_d * sin(ang_v_rad * DELTA_T * 0.5) / (ang_v_rad * 0.5);
                    y += y_d * sin(ang_v_rad * DELTA_T * 0.5) / (ang_v_rad * 0.5);

                } else {
                    x += calcAdamsBashforthDelta(x_d, x_d_1, x_d_2);
                    y += calcAdamsBashforthDelta(y_d, y_d_1, y_d_2);
                }


            } else if(motion_type == EMotionType::STOP ||
                      motion_type == EMotionType::SPINTURN ||
                      motion_type == EMotionType::DIRECT_DUTY_SET) {
                // 並進速度算出
                v = v_;
                // 加速度積分速度算出 グローバル座標系速度算出

                double x_dd =   a_x * sin_val + a_y * cos_val;
                double y_dd = - a_x * cos_val + a_y * sin_val;

                if (ABS(v_) < 0.001) {
                    v_acc = 0.0;
                    x_d = 0.0;
                    y_d = 0.0;
                } else if(motion_type == EMotionType::SPINTURN) {
                    x_d += x_dd * DELTA_T;
                    y_d += y_dd * DELTA_T;
                    v_acc = sqrt(x_d * x_d + y_d * y_d);
                } else if(motion_type == EMotionType::STOP) {
                    x_d = v * cos_val;
                    y_d = v * sin_val;

                    v_acc += a_y * DELTA_T;
                    if(fabs(v_) < 0.05 || fabs(a_y) < 0.25) v_acc = v_;
                }

                // グローバル座標系位置算出
                x += calcAdamsBashforthDelta(x_d, x_d_1, x_d_2);
                y += calcAdamsBashforthDelta(y_d, y_d_1, y_d_2);

            }

            // スリップ角算出
            if(v > 0.2) beta_dot =  -a_x / v - ang_v_rad;
            else beta_dot = 0.0;

            if(fabs(ang_v) < 50.0 || !(motion_type == EMotionType::CURVE )) {
                beta = 0.0;
            } else {
                beta += beta_dot * DELTA_T;
            }


            // 次回計算用の変数保持
            ang_v_2 = ang_v_1;
            ang_v_1 = ang_v;

            x_d_2 = x_d_1;
            x_d_1 = x_d;

            y_d_2 = y_d_1;
            y_d_1 = y_d;

            onWallCenterCorrection(ws);
            cornerLCorrection(ws);
            cornerRCorrection(ws);
            diagCornerRCorrection(ws);
            diagCornerLCorrection(ws);
            //aheadWallCorrection(ws, motion_type);
            //contactWallCorrection(motion_type, ws);
        }

        float calcDiagWallCenterOffset() {
            float ang_ = getAng();
            float x_ = getX();
            float y_ = getY();

            float fmod_x = fmodf(x_, 0.09f);
            float fmod_y = fmodf(y_, 0.09f);

            if(ang_ >= 0.0f && ang_ < 90.0f) { //45deg
                if(fmod_y > fmod_x) {
                    return (fmod_x - fmod_y + 0.045f) / SQRT_2;
                } else {
                    return (fmod_x - fmod_y - 0.045f) / SQRT_2;
                }
            } else if(ang_ >= 90.0f && ang_ < 180.0f) { // 135deg
                if(fmod_y > - fmod_x + 0.09f) {
                    return - (fmod_x + fmod_y - 0.135f) / SQRT_2;
                } else {
                    return - (fmod_x + fmod_y - 0.045f) / SQRT_2;
                }
            } else if(ang_ >= 180.0f && ang_ < 270.0f) { //225deg
                if(fmod_y > fmod_x) {
                    return - (fmod_x - fmod_y + 0.045f) / SQRT_2;
                } else {
                    return - (fmod_x - fmod_y - 0.045f) / SQRT_2;
                }
            } else if(ang_ >= 270.0f && ang_ < 360.0f) { // 315deg
                if(fmod_y > - fmod_x + 0.09f) {
                    return (fmod_x + fmod_y - 0.135f) / SQRT_2;
                } else {
                    return (fmod_x + fmod_y - 0.135f) / SQRT_2;
                }
            } else {
                return 0.0f;
            }
        }

        float calcAheadWallDist(WallSensor& ws) {
            int16_t ahead_l = ws.ahead_l();
            int16_t ahead_r = ws.ahead_r();
            float dist = 0.0;
            if(ws.isAhead() && !ws.isLeft()) {
                dist = hornerMethod(ahead_l, WALL_LA_POLY6_CONSTANT, 7);
            } else if(ws.isAhead() && !ws.isRight()) {
                dist = hornerMethod(ahead_r, WALL_RA_POLY6_CONSTANT, 7);
            } else {
                dist = (hornerMethod(ahead_l, WALL_LA_POLY6_CONSTANT, 7) +
                        hornerMethod(ahead_r, WALL_RA_POLY6_CONSTANT, 7)) * 0.5;
            }
            return constrainL(dist, 0.045);
        }


        void aheadWallCorrection(WallSensor& ws, uint8_t x_next, uint8_t y_next) {
            float ang_ = getAng();
            float x_ = getX();
            float y_ = getY();
            float wall_dist = calcAheadWallDist(ws);
            bool good_posture = (ang >= 355.0 || ang < 5.0)  ||
                                (ang_ >= 85.0 && ang_ < 95.0) ||
                                (ang_ >= 175.0 && ang_ < 185.0) ||
                                (ang_ >= 265.0 && ang_ < 275.0);

            if( !good_posture ||
                    !(ws.isAhead()) ||
                    (wall_dist > 0.135) ||
                    (ws.isLeft() && ws.isRight())
              ) {
                return;
            }

            if(ang_ >= 315.0 || ang_ < 45.0) {
                float nearest_wall_x = (x_next + 1) * 0.09;
                x = nearest_wall_x - wall_dist;                
            } else if(ang_ >= 45.0 && ang_ < 135.0) {
                float nearest_wall_y = (y_next + 1) * 0.09;
                y = nearest_wall_y - wall_dist;
            } else if(ang_ >= 135.0 && ang_ < 225.0) {
                float nearest_wall_x = (x_next) * 0.09;
                x = nearest_wall_x + wall_dist;
            } else if(ang_ >= 225.0 && ang_ < 315.0) {
                float nearest_wall_y = (y_next) * 0.09;
                y = nearest_wall_y + wall_dist;
            }
        }


        float calcWallCenterOffset() {
            float ang_ = getAng();
            float x_ = getX();
            float y_ = getY();

            if(ang_ >= 315.0f || ang_ < 45.0f) {
                return ((uint8_t)(y_ / 0.09f) * 0.09f + 0.09f/2.0f) - y_ ;
            } else if(ang_ >= 45.0f && ang_ < 135.0f) {
                return x_ - ((uint8_t)(x_ / 0.09f) * 0.09f + 0.09f/2.0f);
            } else if(ang_ >= 135.0f && ang_ < 225.0f) {
                return y_ - ((uint8_t)(y_ / 0.09f) * 0.09f + 0.09f/2.0f);
            } else if(ang_ >= 225.0f && ang_ < 315.0f) {
                return ((uint8_t)(x_ / 0.09f) * 0.09f + 0.09f/2.0f) - x_;
            }
            return 0.0f;
        }

        float calcDiffAngle() {
            float target_;
            float ang_ = getAng();
            if(ang_ >= 315.0f || ang_ < 45.0f) target_ = 0.0f;
            else if(ang_ >= 45.0f && ang_ < 135.0f) target_ = 90.0f;
            else if(ang_ >= 135.0f && ang_ < 225.0f) target_ = 180.0f;
            else if(ang_ >= 225.0f && ang_ < 315.0f) target_ = 270.0f;

            float diff = target_ - ang_;
            while(diff > 180.0f) diff -= 360.0f;
            while(diff < -180.0f) diff += 360.0f;
            return diff;
        }

        float getV() {return (float)v;}
        float getVAcc() {return (float)v_acc;}
        float getAng() {return (float)ang;}
        float getAngV() {return (float)ang_v;}
        float getX() {return (float)x;}
        float getY() {return (float)y;}
        float getBeta() {return (float)RAD2DEG(beta);}

        void setV(double v_) {
            v = v_;
        }

        void setAng(double ang_) {
            ang = ang_;
        }

        void setX(double x_) {
            x = x_;
        }

        void setY(double y_) {
            y = y_;
        }

        void setAngV(double ang_v_) {
            ang_v = ang_v_;
        }

      private:

        double v;
        double v_acc;
        double alpha;
        double ang;
        double beta;
        double x;
        double y;
        double x_d_;
        double y_d_;


        float contact_wall_cool_down_time;
        float corner_r_cool_down_time;
        float corner_l_cool_down_time;
        float diag_corner_r_cool_down_time;
        float diag_corner_l_cool_down_time;

        const double DELTA_T = 0.0005;
        const float SQRT_2 = 1.4142356f;
        const double PI = 3.14159265358979323846264338327950288;

        double ang_v;
        double ang_v_1;
        double ang_v_2;

        double x_d;
        double y_d;

        double x_d_1;
        double y_d_1;

        double x_d_2;
        double y_d_2;

        void onWallCenterCorrection(WallSensor& ws) {
            if(ws.isOnWallCenter() == true && ws.isAhead() == false && v > 0.1f && fabsf(ang_v) < 50.0f) {

                if(ang >= 315.0 || ang < 45.0) {
                    if(ws.getOnWallCenterTime() > 0.15) ang = 0.0f;
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09/2.0;
                } else if(ang >= 45.0 && ang < 135.0) {
                    if(ws.getOnWallCenterTime() > 0.15) ang = 90.0f;
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09/2.0;
                } else if(ang >= 135.0 && ang < 225.0) {
                    if(ws.getOnWallCenterTime() > 0.15) ang = 180.0f;
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09/2.0;
                } else if(ang >= 225.0 && ang < 315.0) {
                    if(ws.getOnWallCenterTime() > 0.15) ang = 270.0f;
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09/2.0;
                }
                if(ws.getOnWallCenterTime() > 0.1 && ws.getOnWallCenterTime() < 0.11)SE_POSITION_CHANGE();
            }
        }

        void cornerLCorrection(WallSensor& ws) {
            ParameterManager& pm = ParameterManager::getInstance();

            if( corner_l_cool_down_time == 0.0f &&
                    ws.isCornerL() == true &&
                    fabs(ang_v) < 50.0 &&
                    v > 0.1 &&
                    fabs(calcWallCenterOffset()) < 0.005
              ) {
                bool done = true;
                if(ang >= 350.0 || ang < 10.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09 - (double)pm.wall_corner_read_offset_l;
                } else if(ang >= 80.0 && ang < 100.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09 - (double)pm.wall_corner_read_offset_l;
                } else if(ang >= 170.0 && ang < 190.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 + (double)pm.wall_corner_read_offset_l;
                } else if(ang >= 260.0 && ang < 280.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + (double)pm.wall_corner_read_offset_l;
                } else {
                    done = false;
                }

                if(done == true) {
                    printfAsync("msg_flag:%f,%f,isCornerL\n", x, y);
                    SE_CORNER_L();
                    corner_l_cool_down_time = 0.1;
                }

            }
            corner_l_cool_down_time -= (float)DELTA_T;
            if(corner_l_cool_down_time < 0.0f)corner_l_cool_down_time = 0.0f;

        }

        void cornerRCorrection(WallSensor& ws) {
            ParameterManager& pm = ParameterManager::getInstance();
            if(corner_r_cool_down_time == 0.0f &&
                    ws.isCornerR() == true &&
                    fabs(ang_v) < 50.0 &&
                    v > 0.1 &&
                    fabs(calcWallCenterOffset()) < 0.005
              ) {

                bool done = true;

                if(ang >= 350.0 || ang < 10.0) {
                    x = (uint8_t)(x / 0.09f) * 0.09f + 0.09f - (double)pm.wall_corner_read_offset_r;
                } else if(ang >= 80.0 && ang < 100.0) {
                    y = (uint8_t)(y / 0.09f) * 0.09f + 0.09f - (double)pm.wall_corner_read_offset_r;
                } else if(ang >= 170.0 && ang < 190.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 + (double)pm.wall_corner_read_offset_r;
                } else if(ang >= 260.0 && ang < 280.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + (double)pm.wall_corner_read_offset_r;
                } else {
                    done = false;
                }

                if(done == true) {
                    printfAsync("msg_flag:%f,%f,isCornerR\n", getX(), getY());
                    SE_CORNER_R();
                    corner_r_cool_down_time = 0.1f;
                }

            }
            corner_r_cool_down_time -= (float)DELTA_T;
            if(corner_r_cool_down_time < 0.0f) corner_r_cool_down_time = 0.0f;
        }

        void contactWallCorrection(EMotionType motion_type, WallSensor& ws) {
            ParameterManager& pm = ParameterManager::getInstance();
            if(ws.getContactWallTime() > 0.1f &&
                    contact_wall_cool_down_time == 0.0f &&
                    (motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                     motion_type == EMotionType::STRAIGHT)
              ) {
                SE_CONTACT_WALL();
                printfAsync("<<<<<<contact wall (x_e, y_e, ang_e)=(%f, %f, %f)\n", x, y, ang);

                if(ang >= 315.0 || ang < 45.0) {
                    ang = 0.0;
                    x = (uint8_t)(x / 0.09) * 0.09 + (double)pm.wall_contact_offset;
                } else if(ang >= 45.0 && ang < 135.0) {
                    ang = 90.0;
                    y = (uint8_t)(y / 0.09) * 0.09 + (double)pm.wall_contact_offset;
                } else if(ang >= 135.0 && ang < 225.0) {
                    ang = 180.0f;
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09 - (double)pm.wall_contact_offset;
                } else if(ang >= 225.0 && ang < 315.0) {
                    ang = 270.0;
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09 - (double)pm.wall_contact_offset;
                }
                printfAsync(">>>>>contact wall (x_e, y_e, ang_e)=(%f, %f, %f)\n", getX(), getY(), getAng());
                contact_wall_cool_down_time = 0.1f;
            }
            contact_wall_cool_down_time -= (float)DELTA_T;
            if(contact_wall_cool_down_time < 0.0f) contact_wall_cool_down_time = 0.0f;

        }






        double calcAdamsBashforthDelta(double s_0, double s_1, double s_2 ) {
            return s_0 * DELTA_T; // 単純なオイラー法

            if(s_1 == 0.0 && s_2 == 0.0) return s_0 * DELTA_T;
            else return (23.0 * s_0 - 16.0 * s_1 + 5.0 * s_2) / 12.0 * DELTA_T;
        }

        void diagCornerRCorrection(WallSensor& ws) {
            ParameterManager& pm = ParameterManager::getInstance();
            if(pm.diag_r_corner_read_offset < 0.0f) return;
            if(diag_corner_r_cool_down_time == 0.0f &&
                    ws.isCornerR() == true &&
                    fabs(ang_v) < 50.0 &&
                    v > 0.1 ) {

                bool done = true;


                if( ang >= 40.0  && ang < 50.0 ) {
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09 - (double)pm.diag_r_corner_read_offset;
                } else if(ang >= 130.0 && ang < 140.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 +  (double)pm.diag_r_corner_read_offset;
                } else if(ang >= 220.0 && ang < 230.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + (double)pm.diag_r_corner_read_offset;
                } else if(ang >= 310.0 && ang < 320.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09 - (double)pm.diag_r_corner_read_offset;
                } else {
                    done = false;
                }

                if(done == true) {
                    printfAsync("msg_flag:%f,%f,isDiagCornerR\n", getX(), getY());
                    SE_CORNER_R();
                    diag_corner_r_cool_down_time = 0.1f;
                }

            }
            diag_corner_r_cool_down_time -= (float)DELTA_T;
            if(diag_corner_r_cool_down_time < 0.0f) diag_corner_r_cool_down_time = 0.0f;
        }


        void diagCornerLCorrection(WallSensor& ws) {
            ParameterManager& pm = ParameterManager::getInstance();
            if(pm.diag_l_corner_read_offset < 0.0f) return;
            if(diag_corner_l_cool_down_time == 0.0f &&
                    ws.isCornerL() == true &&
                    fabs(ang_v) < 50.0 &&
                    v > 0.1 ) {

                bool done = true;

                if( ang >= 40.0  && ang < 50.0 ) {
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09 - (double)pm.diag_l_corner_read_offset;
                } else if(ang >= 130.0 && ang < 140.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09 - (double)pm.diag_l_corner_read_offset;
                } else if(ang >= 220.0 && ang < 230.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 + (double)pm.diag_l_corner_read_offset;
                } else if(ang >= 310.0 && ang < 320.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + (double)pm.diag_l_corner_read_offset;
                } else {
                    done = false;
                }

                if(done == true) {
                    printfAsync("msg_flag:%f,%f,isDiagCornerL\n", getX(), getY());
                    SE_CORNER_L();
                    diag_corner_l_cool_down_time = 0.1f;
                }

            }
            diag_corner_l_cool_down_time -= (float)DELTA_T;
            if(diag_corner_l_cool_down_time < 0.0f) diag_corner_l_cool_down_time = 0.0f;
        }

        double deg2rad(double deg) {
            return PI * deg/180.0;
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


}
