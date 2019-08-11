#pragma once
#include <math.h>
#include "ICM20602.h"
#include "fcled.h"
#include "sound.h"
#include "trajectory.h"
#include "communication.h"

namespace umouse
{


    class PositionEstimator {
    public:
        float x;
        float y;
        float v;
        float ang;
        float ang_v;
        float contact_wall_cool_down_time;
        const float DELTA_T = 0.0005;

        PositionEstimator() {
            x = 0.09f /2.0f;
            y = 0.09f / 2.0f;
            ang = 90.0f;
            ang_v = 0.0f;
            v = 0.0f;
            contact_wall_cool_down_time = 0.0f;
        }

        PositionEstimator(float x_, float y_, float ang_) {
            x = x_;
            y = y_;
            ang = ang_;
            ang_v = 0.0f;
            v = 0.0f;
            contact_wall_cool_down_time = 0.0f;
        }

        void reset(float x_, float y_, float ang_) {
            x = x_;
            y = y_;
            ang = ang_;
            ang_v = 0.0f;
            v = 0.0f;
        }

        void update(float v_, float ang_v_, EMotionType motion_type) {
            ICM20602 &icm = ICM20602::getInstance();
            WallSensor &ws = WallSensor::getInstance();
            ParameterManager &pm = ParameterManager::getInstance();
            ang_v = ang_v_;
            v = v_;

            ang += ang_v * DELTA_T;
            ang = fmod(ang + 360.0f, 360.0f);

            float ang_rad = DEG2RAD(ang);

            float x_d = v * cosf(ang_rad);
            float y_d = v * sinf(ang_rad);

            x += DELTA_T * x_d;
            y += DELTA_T * y_d;

            FcLed &fcled = FcLed::getInstance();
            if(ws.isOnWallCenter() == true && ws.isAhead() == false && v > 0.1 && ABS(ang_v) < 100.0f) {
                //fcled.turn(1,1,1);

                if(ang >= 315.0f || ang < 45.0f) {
                    ang = 0.0f;
                    y = (uint8_t)(y / 0.09f) * 0.09f + 0.09f/2.0f;
                    SE_POSITION_CHANGE();
                }
                if(ang >= 45.0f && ang < 135.0f) {
                    ang = 90.0f;
                    x = (uint8_t)(x / 0.09f) * 0.09f + 0.09f/2.0f;
                    SE_POSITION_CHANGE();
                }
                if(ang >= 135.0f && ang < 225.0f) {
                    ang = 180.0f;
                    y = (uint8_t)(y / 0.09f) * 0.09f + 0.09f/2.0f;
                    SE_POSITION_CHANGE();
                }
                if(ang >= 225.0f && ang < 315.0f) {
                    ang = 270.0f;
                    x = (uint8_t)(x / 0.09f) * 0.09f + 0.09f/2.0f;
                    SE_POSITION_CHANGE();
                }
            }

            if(ws.isCornerL() == true && ABS(ang_v) < 50.0f && v > 0.1 ) {
                if(ang >= 350.0f || ang < 10.0f) {
                    x = (uint8_t)(x / 0.09f) * 0.09f + 0.09f - pm.wall_corner_read_offset_l;
                    SE_CORNER_L();
                }
                if(ang >= 80.0f && ang < 100.0f) {
                    y = (uint8_t)(y / 0.09f) * 0.09f + 0.09f - pm.wall_corner_read_offset_l;
                    SE_CORNER_L();
                }
                if(ang >= 170.0f && ang < 190.0f) {
                    x = (uint8_t)(x / 0.09f) * 0.09f + pm.wall_corner_read_offset_l;
                    SE_CORNER_L();
                }
                if(ang >= 260.0f && ang < 280.0f) {
                    y = (uint8_t)(y / 0.09f) * 0.09f + pm.wall_corner_read_offset_l;
                    SE_CORNER_L();
                }

            }

            if(ws.isCornerR() == true && ABS(ang_v) < 50.0f && v > 0.1 ) {
                if(ang >= 350.0f || ang < 10.0f) {
                    x = (uint8_t)(x / 0.09f) * 0.09f + 0.09f - pm.wall_corner_read_offset_r;
                    SE_CORNER_R();
                }
                if(ang >= 80.0f && ang < 100.0f) {
                    y = (uint8_t)(y / 0.09f) * 0.09f + 0.09f - pm.wall_corner_read_offset_r;
                    SE_CORNER_R();
                }
                if(ang >= 170.0f && ang < 190.0f) {
                    x = (uint8_t)(x / 0.09f) * 0.09f + pm.wall_corner_read_offset_r;
                    SE_CORNER_R();
                }
                if(ang >= 260.0f && ang < 280.0f) {
                    y = (uint8_t)(y / 0.09f) * 0.09f + pm.wall_corner_read_offset_r;
                    SE_CORNER_R();
                }
            }


            if(ws.getContactWallTime() > 0.1 &&
               contact_wall_cool_down_time == 0.0f &&
               (motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                motion_type == EMotionType::STRAIGHT)
            ) {
                SE_CONTACT_WALL();
                printfAsync("<<<<<<contact wall (x_e, y_e, ang_e)=(%f, %f, %f)\n", x, y, ang);

                if(ang >= 315.0f || ang < 45.0f) {
                    ang = 0.0f;
                    x = (uint8_t)(x / 0.09f) * 0.09f + pm.wall_contact_offset;
                }
                if(ang >= 45.0f && ang < 135.0f) {
                    ang = 90.0f;
                    y = (uint8_t)(y / 0.09f) * 0.09f + pm.wall_contact_offset;
                }
                if(ang >= 135.0f && ang < 225.0f) {
                    ang = 180.0f;
                    x = (uint8_t)(x / 0.09f) * 0.09f + 0.09f - pm.wall_contact_offset;
                }
                if(ang >= 225.0f && ang < 315.0f) {
                    ang = 270.0f;
                    y = (uint8_t)(y / 0.09f) * 0.09f + 0.09f - pm.wall_contact_offset;
                }
                printfAsync(">>>>>contact wall (x_e, y_e, ang_e)=(%f, %f, %f)\n", x, y, ang);
                contact_wall_cool_down_time = 0.1;
            }
            contact_wall_cool_down_time -= DELTA_T;
            if(contact_wall_cool_down_time < 0.0f)contact_wall_cool_down_time = 0.0f;

        }

        float calcWallCenterOffset() {
            if(ang >= 315.0f || ang < 45.0f) {
                return (uint8_t)(y / 0.09f) * 0.09f + 0.09f/2.0f - y ;
            }
            if(ang >= 45.0f && ang < 135.0f) {
                return (uint8_t)(x / 0.09f) * 0.09f + 0.09f/2.0f - x;
            }
            if(ang >= 135.0f && ang < 225.0f) {
                return (uint8_t)(y / 0.09f) * 0.09f + 0.09f/2.0f - y;
            }
            if(ang >= 225.0f && ang < 315.0f) {
                return (uint8_t)(x / 0.09f) * 0.09f + 0.09f/2.0f - x;
            }
            return 0.0f;
        }

        float calcDiffAngle() {
            float target_;
            if(ang >= 315.0f || ang < 45.0f) target_ = 0.0f;
            if(ang >= 45.0f && ang < 135.0f) target_ = 90.0f;
            if(ang >= 135.0f && ang < 225.0f) target_ = 180.0f;
            if(ang >= 225.0f && ang < 315.0f) target_ = 270.0f;

            float observed_val_  = ang;
            float diff = target_ - observed_val_;
            while(diff > 180.0f) diff -= 360.0f;
            while(diff < -180.0f) diff += 360.0f;
            return diff;
        };

    };

}
