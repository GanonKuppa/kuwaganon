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
#include <deque>
#include <float.h>

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
            v_for_int = 0.0;

            for (uint8_t i = 0; i < ACC_Y_AVERAGE_NUM; i++) {
                acc_y_list.push_front(0.0f);
            }


            contact_wall_cool_down_time = 0.0f;            
            on_wall_center_dist = 0.0f;
            after_curve_time = 0.0f;
        }

        PositionEstimator(float x_, float y_, float ang_) {
            x = (float)x_;
            y = (float)y_;
            ang = (float)ang_;
            ang_v = 0.0;
            v = 0.0;
            x_d = 0.0;
            y_d = 0.0;

            contact_wall_cool_down_time = 0.0f;            
            on_wall_center_dist = 0.0f;
            after_curve_time = 0.0f;
        }

        void reset(float x_, float y_, float ang_) {
            x = (float)x_;
            y = (float)y_;
            ang = (float)ang_;

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
            
            on_wall_center_dist = 0.0f;
            after_curve_time = 0.0f;
        }



        void update(float v_enc, float v_ave, float ang_v_, float a_y, float a_x, EMotionType motion_type, WallSensor& ws, float a_setp) {
            acc_y_list.push_front(a_y);
            acc_y_list.pop_back();
            
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

            float ang_v_rad = deg2rad(ang_v);
            float sin_val;
            float cos_val;
            float beta_dot;

            // 角度算出
            alpha = (ang_v_ - ang_v) / DELTA_T;
            ang_v = ang_v_;
            ang += calcEulerDelta(ang_v);
            ang = fmodf(ang + 360.0, 360.0);

            float ang_rad = deg2rad(ang + beta);
            sin_val = sinf(ang_rad);
            cos_val = cosf(ang_rad);

            // 並進速度算出                
            float acc_y_int = 0.0f;
            for (uint8_t i=0; i< ACC_Y_AVERAGE_NUM; i++){
                acc_y_int += acc_y_list[i];
            }
            
            v = v_ave + acc_y_int * DELTA_T;
            
            if(!(motion_type == EMotionType::CURVE ) ){
                v_for_int = v_enc;
            }
            else{
                v_for_int = v;
            }

            // 加速度積分速度算出
            v_acc += a_y * DELTA_T;
            if(ABS(v) < 0.01f) v_acc = v;

            // グローバル座標系速度算出
            x_d = v_for_int * cos_val;
            y_d = v_for_int * sin_val;
                            
            x += calcEulerDelta(x_d);
            y += calcEulerDelta(y_d);
                
            // スリップ角算出
            after_curve_time -= DELTA_T;   
            if(after_curve_time < 0.0f) after_curve_time = 0.0f;
            if(motion_type == EMotionType::CURVE) after_curve_time = afrer_curve_beta_dot_time;

            if(v > 0.1) beta_dot =  -a_x / v - ang_v_rad;
            else beta_dot = 0.0;

            if(motion_type != EMotionType::CURVE && (after_curve_time < FLT_EPSILON) ) {
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



            // 壁中心判定
            if(motion_type == EMotionType::STRAIGHT_WALL_CENTER && 
               ws.isOnWallCenter() && 
               !ws.isAhead() && 
               v > 0.1f && 
               fabsf(ang_v) < 50.0f
            ){
                on_wall_center_dist += v_enc * DELTA_T;
            }
            else{
                on_wall_center_dist = 0.0f;
            }

            // 壁切れ位置無補正でどれくらい走ったか判定
            if( (motion_type == EMotionType::STRAIGHT_WALL_CENTER || 
                 motion_type == EMotionType::STRAIGHT) &&
                 v > 0.1f
            ){
                corner_l_cool_down_dist += v_enc * DELTA_T;
                corner_r_cool_down_dist += v_enc * DELTA_T;
            }
            else{
                corner_l_cool_down_dist = 0.0f;
                corner_r_cool_down_dist = 0.0f;
            }

            onWallCenterCorrection();
            cornerLCorrection(ws);
            cornerRCorrection(ws);
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
            float ok_ang = 2.0f;
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




        bool aheadWallCorrection(WallSensor& ws, uint8_t x_next, uint8_t y_next) {
            float ang_ = getAng();
            float x_ = getX();
            float y_ = getY();
            float wall_dist = ws.calcAheadWallDist();
            float ok_ang = 3.0f;
            bool good_posture = (ang >= 360.0 - ok_ang || ang < ok_ang)  ||
                                (ang_ >= 90.0f - ok_ang && ang_ < 90.0f + ok_ang) ||
                                (ang_ >= 180.0f - ok_ang && ang_ < 180.0f + ok_ang) ||
                                (ang_ >= 270.0f - ok_ang && ang_ < 270.0f + ok_ang);

            if( !good_posture || !ws.isAhead() || wall_dist ) {
                return false;
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

            return true;
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

        void setV(float v_) {
            v = v_;
        }

        void setAng(float ang_) {
            ang = ang_;
        }

        void setX(float x_) {
            x = x_;
        }

        void setY(float y_) {
            y = y_;
        }

        void setAngV(float ang_v_) {
            ang_v = ang_v_;
        }

      private:        
        float on_wall_center_dist;

        float v;
        float v_acc;
        float alpha;
        float ang;
        float beta;
        float x;
        float y;
        float x_pre;
        float y_pre;
        float x_d_;
        float y_d_;

        std::deque<float> acc_y_list;
        const uint8_t ACC_Y_AVERAGE_NUM = 15;

        float contact_wall_cool_down_time;
        float corner_r_cool_down_dist;
        float corner_l_cool_down_dist;
        float diag_corner_r_cool_down_time;
        float diag_corner_l_cool_down_time;
        const float afrer_curve_beta_dot_time = 0.050f;
        float after_curve_time;

        const float DELTA_T = 0.001;
        const float SQRT_2 = 1.4142356f;
        const float PI = 3.14159265358979323846264338327950288;

        float ang_v;
        float ang_v_1;
        float ang_v_2;

        float x_d;
        float y_d;

        float x_d_1;
        float y_d_1;

        float x_d_2;
        float y_d_2;

        float v_for_int;

        bool __isfinite(float x){
            union { float f; uint32_t i; } a;
            a.f = x;
            if( ((a.i >> 23) & 0xff) == 255) return false;
            else return true;
        }

        bool __isfinite(double x){
            union { float f; uint64_t i; } a;
            a.f = x;
            if( ((a.i >> 52) & 0x007ff) == 2047) return false;
            else return true;
        }



        void onWallCenterCorrection() {
            if(on_wall_center_dist > 0.06f) {
                if(ang >= 315.0 || ang < 45.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09/2.0;
                    ang = 0.0;
                } else if(ang >= 45.0 && ang < 135.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09/2.0;
                    ang = 90.0;
                } else if(ang >= 135.0 && ang < 225.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09/2.0;
                    ang = 180.0;
                } else if(ang >= 225.0 && ang < 315.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09/2.0;
                    ang = 270.0;
                }
                SE_POSITION_CHANGE();
                on_wall_center_dist = 0.0f;
            }
        }

        void cornerLCorrection(WallSensor& ws) {
            ParameterManager& pm = ParameterManager::getInstance();
            float ok_ang = 3.0f;
            if( corner_l_cool_down_dist > 0.07f &&
                    ws.isCornerL()&&
                    fabs(ang_v) < 50.0f &&
                    v > 0.1 &&
                    fabs(calcWallCenterOffset()) < 0.005f
              ) {
                bool done = true;
                if(ang >= 360.0 - ok_ang || ang < ok_ang) {
                    x = (uint8_t)((x-0.045) / 0.09) * 0.09 + 0.09 - (float)pm.wall_corner_read_offset_l;
                } else if(ang >= 90.0 - ok_ang && ang < 90.0 + ok_ang) {
                    y = (uint8_t)((y-0.045) / 0.09) * 0.09 + 0.09 - (float)pm.wall_corner_read_offset_l;
                } else if(ang >= 180.0 - ok_ang && ang < 180.0 + ok_ang) {
                    x = (uint8_t)((x+0.045) / 0.09) * 0.09 + (float)pm.wall_corner_read_offset_l;
                } else if(ang >= 270.0 - ok_ang && ang < 270.0 + ok_ang) {
                    y = (uint8_t)((y+0.045) / 0.09) * 0.09 + (float)pm.wall_corner_read_offset_l;
                } else {
                    done = false;
                }

                if(done) {
                    printfAsync("msg_flag:%f,%f,isCornerL\n", x, y);
                    SE_CORNER_L();
                    corner_l_cool_down_dist = 0.0f;
                }
            }
        }

        void cornerRCorrection(WallSensor& ws) {
            ParameterManager& pm = ParameterManager::getInstance();
            float ok_ang = 3.0f;

            if(corner_r_cool_down_dist > 0.07f &&
                    ws.isCornerR() == true &&
                    fabs(ang_v) < 50.0 &&
                    v > 0.1 &&
                    fabs(calcWallCenterOffset()) < 0.005
              ) {

                bool done = true;

                if(ang >= 360.0f - ok_ang || ang < ok_ang) {
                    x = (uint8_t)((x-0.045f) / 0.09f) * 0.09f + 0.09f - (float)pm.wall_corner_read_offset_r;
                } else if(ang >= 90.0f - ok_ang && ang < 90.0f + ok_ang) {
                    y = (uint8_t)((y-0.045f) / 0.09f) * 0.09f + 0.09f - (float)pm.wall_corner_read_offset_r;
                } else if(ang >= 180.0 - ok_ang && ang < 180.0 + ok_ang) {
                    x = (uint8_t)((x+0.045f) / 0.09) * 0.09 + (float)pm.wall_corner_read_offset_r;
                } else if(ang >= 270.0 - ok_ang && ang < 270.0 + ok_ang) {
                    y = (uint8_t)((y+0.045f) / 0.09) * 0.09 + (float)pm.wall_corner_read_offset_r;
                } else {
                    done = false;
                }

                if(done) {
                    printfAsync("msg_flag:%f,%f,isCornerR\n", getX(), getY());
                    SE_CORNER_R();
                    corner_r_cool_down_dist = 0.0f;
                }

            }
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
                    x = (uint8_t)(x / 0.09) * 0.09 + (float)pm.wall_contact_offset;
                } else if(ang >= 45.0 && ang < 135.0) {
                    ang = 90.0;
                    y = (uint8_t)(y / 0.09) * 0.09 + (float)pm.wall_contact_offset;
                } else if(ang >= 135.0 && ang < 225.0) {
                    ang = 180.0f;
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09 - (float)pm.wall_contact_offset;
                } else if(ang >= 225.0 && ang < 315.0) {
                    ang = 270.0;
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09 - (float)pm.wall_contact_offset;
                }
                printfAsync(">>>>>contact wall (x_e, y_e, ang_e)=(%f, %f, %f)\n", getX(), getY(), getAng());
                contact_wall_cool_down_time = 0.1f;
            }
            contact_wall_cool_down_time -= (float)DELTA_T;
            if(contact_wall_cool_down_time < 0.0f) contact_wall_cool_down_time = 0.0f;

        }


        void nearWallCorrection(EMotionType motion_type, WallSensor& ws) {
            float ang_ = getAng();
            float x_ = getX();
            float y_ = getY();
            uint8_t x_int = (uint8_t)(x_ / 0.09f);
            uint8_t y_int = (uint8_t)(y_ / 0.09f);
            float wall_dist = ws.calcAheadWallDist();
            float ok_ang = 3.0f;
            bool good_posture = (ang >= 360.0 - ok_ang || ang < ok_ang)  ||
                                (ang_ >= 90.0f - ok_ang && ang_ < 90.0f + ok_ang) ||
                                (ang_ >= 180.0f - ok_ang && ang_ < 180.0f + ok_ang) ||
                                (ang_ >= 270.0f - ok_ang && ang_ < 270.0f + ok_ang);

            if( !good_posture || 
                !ws.isAhead() || 
                 wall_dist > 0.06 ||
                !(motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                  motion_type == EMotionType::STRAIGHT)
            ){
                return;
            }

            if(ang_ >= 315.0 || ang_ < 45.0) {
                float nearest_wall_x = (x_int + 1) * 0.09;
                x = nearest_wall_x - wall_dist;                
            } else if(ang_ >= 45.0 && ang_ < 135.0) {
                float nearest_wall_y = (y_int + 1) * 0.09;
                y = nearest_wall_y - wall_dist;
            } else if(ang_ >= 135.0 && ang_ < 225.0) {
                float nearest_wall_x = (x_int) * 0.09;
                x = nearest_wall_x + wall_dist;
            } else if(ang_ >= 225.0 && ang_ < 315.0) {
                float nearest_wall_y = (y_int) * 0.09;
                y = nearest_wall_y + wall_dist;
            }
            SE_POSITION_CHANGE();
        }


        float calcCircularArcDelta(float ang_v_rad, float x_d, float y_d){
            return x_d * sin(ang_v_rad * DELTA_T * 0.5) / (ang_v_rad * 0.5);
        }

        float calcEulerDelta(float s_0){
            return s_0 * DELTA_T;
        }

        float calcAdamsBashforthDelta(float s_0, float s_1, float s_2 ) {
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
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09 - (float)pm.diag_r_corner_read_offset;
                } else if(ang >= 130.0 && ang < 140.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 +  (float)pm.diag_r_corner_read_offset;
                } else if(ang >= 220.0 && ang < 230.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + (float)pm.diag_r_corner_read_offset;
                } else if(ang >= 310.0 && ang < 320.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09 - (float)pm.diag_r_corner_read_offset;
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
            if(diag_corner_l_cool_down_time > 0.07f &&
                    ws.isCornerL() &&
                    fabs(ang_v) < 50.0f &&
                    v > 0.1f ) {

                bool done = true;

                if( ang >= 40.0  && ang < 50.0 ) {
                    x = (uint8_t)(x / 0.09) * 0.09 + 0.09 - (float)pm.diag_l_corner_read_offset;
                } else if(ang >= 130.0 && ang < 140.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + 0.09 - (float)pm.diag_l_corner_read_offset;
                } else if(ang >= 220.0 && ang < 230.0) {
                    x = (uint8_t)(x / 0.09) * 0.09 + (float)pm.diag_l_corner_read_offset;
                } else if(ang >= 310.0 && ang < 320.0) {
                    y = (uint8_t)(y / 0.09) * 0.09 + (float)pm.diag_l_corner_read_offset;
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

        float deg2rad(float deg) {
            return PI * deg/180.0;
        }

    };


}
