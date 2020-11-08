#pragma once
#include <math.h>
#include "ICM20602.h"
#include <cmath>
//#include "adis16470.h"
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
            x_pre = x;
            y_pre = y;
            ang = 90.0;
            ang_v = 0.0;
            v = 0.0;
            x_d = 0.0;
            y_d = 0.0;

            contact_wall_cool_down_time = 0.0f;
            on_wall_center_pre = false;
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
            on_wall_center_pre = false;
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

            on_wall_center_pre = false;
        }



        void update(double v_, double ang_v_, double a_y, double a_x, EMotionType motion_type, WallSensor& ws) {
            
            
            
            if(!__isfinite(x)){
                printfAsync("x nan! %f %f \n", x, x_pre);
                x = x_pre;
            }

            if(!__isfinite(y)){
                printfAsync("y NAN? %f %f \n", y, y_pre);
                y = y_pre;
            }



            ICM20602& icm = ICM20602::getInstance();            
            ParameterManager& pm = ParameterManager::getInstance();

            double ang_v_rad = deg2rad(ang_v);
            double sin_val;
            double cos_val;
            double beta_dot;

            // 角度算出
            alpha = (ang_v_ - ang_v) / DELTA_T;
            ang_v = ang_v_;
            ang += calcEulerDelta(ang_v);
            ang = fmod(ang + 360.0, 360.0);

            double ang_rad = deg2rad(ang);
            sin_val = sin(ang_rad);
            cos_val = cos(ang_rad);

            if(motion_type == EMotionType::STRAIGHT ||
                    motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                    motion_type == EMotionType::DIAGONAL ||
                    motion_type == EMotionType::DIAGONAL_CENTER ||
                    motion_type == EMotionType::CURVE ||
                    motion_type == EMotionType::STOP ||
                    motion_type == EMotionType::SPINTURN ||
                    motion_type == EMotionType::DIRECT_DUTY_SET
                    
                    ) {
                // 並進速度算出
                double gain = (double)pm.v_comp_gain;
                //a_y = (double)adis.acc_f[2];
                //if(fabs(a_y) < 0.5) gain = 0.0;
                //v = (gain)*(v + a_y * DELTA_T) + (1.0 - gain)*(v_);
                v = v_;

                // 加速度積分速度算出
                v_acc += a_y * DELTA_T;
                if(fabs(v_) < 0.05 || fabs(a_y) < 0.25) v_acc = v_;

                // グローバル座標系速度算出
                x_d = v * cos_val;
                y_d = v * sin_val;
                                
                x += calcEulerDelta(x_d);
                y += calcEulerDelta(y_d);

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
                    x += calcEulerDelta(x_d);
                    y += calcEulerDelta(y_d);
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
                x += calcEulerDelta(x_d);
                y += calcEulerDelta(y_d);

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

            if(__isfinite(x)){
                x_pre = x;
            }

            if(__isfinite(y)){
                y_pre = y;
            }



            //onWallCenterCorrection(ws, motion_type);
            //cornerLCorrection(ws);
            //cornerRCorrection(ws);
            //diagCornerRCorrection(ws);
            //diagCornerLCorrection(ws);
            //nearWallCorrection(motion_type, ws);            
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

        void sideWallCorrection(WallSensor& ws, uint8_t x_next, uint8_t y_next) {
            float ang_ = getAng();
            float x_ = getX();
            float y_ = getY();
            float wall_dist_r = ws.dist_r();
            float wall_dist_l = ws.dist_l();
            float ok_ang = 3.0f;
            bool good_posture = (ang >= 360.0 - ok_ang || ang < ok_ang)  ||
                                (ang_ >= 90.0f - ok_ang && ang_ < 90.0f + ok_ang) ||
                                (ang_ >= 180.0f - ok_ang && ang_ < 180.0f + ok_ang) ||
                                (ang_ >= 270.0f - ok_ang && ang_ < 270.0f + ok_ang);

            bool is_right = ws.isRight();
            bool is_left = ws.isLeft();
            
            if( !good_posture || (!is_right && !is_left) ){
                return;
            }

            if(ang_ >= 315.0 || ang_ < 45.0) {
                float right_wall_y = y_next * 0.09f;
                float left_wall_y =  (y_next + 1) * 0.09f;
                float y_r = right_wall_y + wall_dist_r;
                float y_l = left_wall_y - wall_dist_l;                                
                if(is_left && is_right) y = (y_r + y_l) * 0.5;
                else if(is_left) y = y_l;
                else if(is_right) y = y_r;

            } else if(ang_ >= 45.0 && ang_ < 135.0) {
                float right_wall_x = (x_next + 1) * 0.09f;
                float left_wall_x = x_next * 0.09f;
                float x_r = right_wall_x - wall_dist_r;
                float x_l = left_wall_x + wall_dist_l;
                if(is_left && is_right) x = (x_r + x_l) * 0.5;
                else if(is_left) x = x_l;
                else if(is_right) x = x_r;

            } else if(ang_ >= 135.0 && ang_ < 225.0) {
                float right_wall_y = (y_next + 1) * 0.09f;
                float left_wall_y = y_next * 0.09f;
                float y_r = right_wall_y - wall_dist_r;
                float y_l = left_wall_y + wall_dist_l;                                
                if(is_left && is_right) y = (y_r + y_l) * 0.5;
                else if(is_left) y = y_l;
                else if(is_right) y = y_r;

            } else if(ang_ >= 225.0 && ang_ < 315.0) {
                float right_wall_x = x_next * 0.09f;
                float left_wall_x = (x_next + 1) * 0.09f;
                float x_r = right_wall_x + wall_dist_r;
                float x_l = left_wall_x - wall_dist_l;
                if(is_left && is_right) x = (x_r + x_l) * 0.5;
                else if(is_left) x = x_l;
                else if(is_right) x = x_r;
            }
        }




        void aheadWallCorrection(WallSensor& ws, uint8_t x_next, uint8_t y_next) {
            float ang_ = getAng();
            float x_ = getX();
            float y_ = getY();
            float wall_dist = ws.calcAheadWallDist();
            float ok_ang = 3.0f;
            bool good_posture = (ang >= 360.0 - ok_ang || ang < ok_ang)  ||
                                (ang_ >= 90.0f - ok_ang && ang_ < 90.0f + ok_ang) ||
                                (ang_ >= 180.0f - ok_ang && ang_ < 180.0f + ok_ang) ||
                                (ang_ >= 270.0f - ok_ang && ang_ < 270.0f + ok_ang);

            if( !good_posture || !ws.isAhead() ) {
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
        bool on_wall_center_pre;

        double v;
        double v_acc;
        double alpha;
        double ang;
        double beta;
        double x;
        double y;
        double x_pre;
        double y_pre;
        double x_d_;
        double y_d_;


        float contact_wall_cool_down_time;
        float corner_r_cool_down_time;
        float corner_l_cool_down_time;
        float diag_corner_r_cool_down_time;
        float diag_corner_l_cool_down_time;

        const double DELTA_T = 0.001;
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

        bool __isfinite(float x){
            union { float f; uint32_t i; } a;
            a.f = x;
            if( ((a.i >> 23) & 0xff) == 255) return false;
            else return true;
        }

        bool __isfinite(double x){
            union { double f; uint64_t i; } a;
            a.f = x;
            if( ((a.i >> 52) & 0x007ff) == 2047) return false;
            else return true;
        }



        void onWallCenterCorrection(WallSensor& ws, EMotionType motion_type) {
            if(motion_type == EMotionType::STRAIGHT_WALL_CENTER && ws.isOnWallCenter() && ws.isAhead() == false && v > 0.1f && fabsf(ang_v) < 50.0f) {
                if(ang >= 315.0 || ang < 45.0) {
                    if(ws.getOnWallCenterTime() > 0.175) ang = 0.0f;
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09/2.0;
                    ang = 0.0;
                } else if(ang >= 45.0 && ang < 135.0) {
                    if(ws.getOnWallCenterTime() > 0.175) ang = 90.0f;
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09/2.0;
                    ang = 90.0;
                } else if(ang >= 135.0 && ang < 225.0) {
                    if(ws.getOnWallCenterTime() > 0.175) ang = 180.0f;
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09/2.0;
                    ang = 180.0;
                } else if(ang >= 225.0 && ang < 315.0) {
                    if(ws.getOnWallCenterTime() > 0.175) ang = 270.0f;
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09/2.0;
                    ang = 270.0;
                }
                if(!on_wall_center_pre) SE_POSITION_CHANGE();
                on_wall_center_pre = true;
            }
            else{
                on_wall_center_pre = false;
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


        void nearWallCorrection(EMotionType motion_type, WallSensor& ws) {
            ParameterManager& pm = ParameterManager::getInstance();
            if(     v > 0.0f && v < 0.15f &&
                    (motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                     motion_type == EMotionType::STRAIGHT) &&
                     ws.ahead_l() + ws.ahead_r() > 950
                    
              ) {
                SE_CONTACT_WALL();
                printfAsync("<<<<<<near wall (x_e, y_e, ang_e)=(%f, %f, %f)\n", x, y, ang);

                if(ang >= 315.0 || ang < 45.0) {
                    ang = 0.0;
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09 - 0.045;
                } else if(ang >= 45.0 && ang < 135.0) {
                    ang = 90.0;
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09 - 0.045;
                } else if(ang >= 135.0 && ang < 225.0) {
                    ang = 180.0f;
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.045 ;
                } else if(ang >= 225.0 && ang < 315.0) {
                    ang = 270.0;
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.045;
                }
                printfAsync(">>>>>near wall (x_e, y_e, ang_e)=(%f, %f, %f)\n", getX(), getY(), getAng());
            }

        }


        double calcCircularArcDelta(double ang_v_rad, double x_d, double y_d){
            return x_d * sin(ang_v_rad * DELTA_T * 0.5) / (ang_v_rad * 0.5);
        }

        double calcEulerDelta(double s_0){
            return s_0 * DELTA_T;
        }

        double calcAdamsBashforthDelta(double s_0, double s_1, double s_2 ) {
            if(s_1 == 0.0 && s_2 == 0.0) return s_0 * DELTA_T;
            else if(s_2 == 0.0) return s_0 * DELTA_T;
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

    };


}
