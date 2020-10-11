#pragma once

#include <stdint.h>
#include <math.h>
#include <myUtil.h>
#include <deque>
#include <cmath>

#include "iodefine.h"
#include "pwm.h"
#include "phaseCounting.h"

#include "maze.h"
#include "math.h"
#include "parameterManager.h"
#include "timer.h"
#include "wheelOdometry.h"
#include "ICM20602.h"
#include "adis16470.h"
#include "wallsensor.h"
#include "controlMixer.h"
#include <Eigen/Core>
#include "positionEstimator.h"
#include "trajectoryCommander.h"
#include "powerTransmission.h"
#include "pseudoDial.h"
#include "sound.h"

//#include <Core>
//#include <Geometry>
//using namespace Eigen;
//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

namespace umouse {

    template<typename T>
    class Coor2D {
      public:
        T x;
        T y;

        Coor2D() {}

        bool operator==(const Coor2D& coor) {
            if (coor.x == x && coor.y == y) return true;
            else return false;
        }

        bool operator!=(const Coor2D& coor) {
            if (coor.x != x || coor.y != y) return true;
            else return false;
        }


        void set(T x_, T y_) {
            x = x_;
            y = y_;
        }

        Coor2D(const Coor2D& coor) {
            x = coor.x;
            y = coor.y;
        }

    };


    class UMouse {
      public:
        static constexpr float WALL2MOUSE_CENTER_DIST = 0.0183;
        static constexpr float READ_WALL_OFFSET = 0.01;
        static constexpr float DELTA_T = 0.0005;

        // -32768 から 32767
        direction_e direction;
        Coor2D<uint16_t> coor;
        Coor2D<uint16_t> start;
        Coor2D<uint16_t> goal;

        Maze maze;
        bool error_run_flag;
        bool direct_duty_set_enable;

        ControlMixer ctrlMixer;
        TrajectoryCommander trajCommander;
        PositionEstimator posEsti;
        float running_sec;
        float ang_no_calib_sec;

        static UMouse& getInstance() {
            static UMouse instance;
            return instance;
        }

        void update() {
            ICM20602& icm = ICM20602::getInstance();
            WheelOdometry& wo = WheelOdometry::getInstance();
            PowerTransmission& pt = PowerTransmission::getInstance();
            WallSensor& ws = WallSensor::getInstance();
            PseudoDialL& pdl = PseudoDialL::getInstance();
            PseudoDialR& pdr = PseudoDialR::getInstance();


            posEsti.update(wo.getAveV(), (double)icm.omega_f[2], (double)icm.acc_f[1], (double)icm.acc_f[0], trajCommander.getMotionType(), ws);
            trajCommander.update(posEsti);

            if(pdl.getEnable() || pdr.getEnable()){
                float duty_l = pdl.getDuty();
                float duty_r = pdr.getDuty();
                pt.setRawDuty(duty_l, duty_r);
            }
            else{
                if(!trajCommander.empty() && trajCommander.getMotionType() == EMotionType::DIRECT_DUTY_SET) {
                    direct_duty_set_enable = true;
                } 
                else {
                    direct_duty_set_enable = false;
                }

                if(direct_duty_set_enable == false) {
                    if(!trajCommander.empty()) {
                        auto traj = trajCommander.getTraj();
                        ctrlMixer.update(traj, posEsti, isRWallControllable(), isLWallControllable());
                        auto duty = ctrlMixer.getDuty();
                        pt.setDuty(duty(0), duty(1));
                    } else {
                        ctrlMixer.reset();
                        pt.setDuty(0.0, 0.0);
                    }
                }

                float x0 = trajCommander.x;
                float x1 = posEsti.getX();
                float y0 = trajCommander.y;
                float y1 = posEsti.getY();

                float esti_target_diff = sqrtf((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));

                if( !trajCommander.empty() &&
                    (icm.isUpsideDown()      ||
                     esti_target_diff > 0.045 ||
                    (ctrlMixer.isOutOfControl() && (ABS(pt.getDuty_R()) > 0.01f || ABS(pt.getDuty_L()) > 0.01f ))
                    )
                ) {                    
                    printOutOfControl();
                    trajCommander.clear();
                    pt.setDuty(0.0f, 0.0f);

                    SEH();
                }
            }

            coor.x = (uint8_t)(posEsti.getX() / 0.09f);
            coor.y = (uint8_t)(posEsti.getY() / 0.09f);

            direction = getDirection();
            running_sec += DELTA_T;
            ang_no_calib_sec += DELTA_T;
        }

        uint8_t getCoorX(){return coor.x;}
        uint8_t getCoorY(){return coor.y;}
        
        direction_e getDirection() {
            if(posEsti.getAng() >= 315.0f || posEsti.getAng() <  45.0f) return direction_e::E;
            if(posEsti.getAng() >=  45.0f && posEsti.getAng() < 135.0f) return direction_e::N;
            if(posEsti.getAng() >= 135.0f && posEsti.getAng() < 225.0f) return direction_e::W;
            if(posEsti.getAng() >= 225.0f && posEsti.getAng() < 315.0f) return direction_e::S;
            return direction_e::E;
        }


        bool inReadWallArea(float read_wall_offset = READ_WALL_OFFSET) {
            float fmod_x = fmodf(posEsti.getX(), 0.09f);
            float fmod_y = fmodf(posEsti.getY(), 0.09f);

            if(getDirection() == direction_e::E) {
                if(fmod_x < 0.089f && fmod_x >= 0.09f - read_wall_offset) return true;
                else return false;
            }
            if(getDirection() == direction_e::N) {
                if(fmod_y < 0.089f && fmod_y >= 0.09f - read_wall_offset) return true;
                else return false;
            }
            if(getDirection() == direction_e::W) {
                if(fmod_x > 0.001f && fmod_x <= read_wall_offset) return true;
                else return false;
            }
            if(getDirection() == direction_e::S) {
                if(fmod_y > 0.001f && fmod_y <= read_wall_offset) return true;
                else return false;
            }
            return false;
        }



        float calcDistNextSection() {
            float fmod_x = fmodf(posEsti.getX(), 0.09f);
            float fmod_y = fmodf(posEsti.getY(), 0.09f);
            if(getDirection() == direction_e::E) {
                return 0.09f - fmod_x;
            }
            if(getDirection() == direction_e::N) {
                return 0.09f - fmod_y;
            }
            if(getDirection() == direction_e::W) {
                return fmod_x - 0.0f;
            }
            if(getDirection() == direction_e::S) {
                return fmod_y - 0.0f;
            }
            return 0.0f;
        }


        void printPos() {
            printfAsync("pos = (%f, %f), ang=%f, coor = (%d, %d) dir = %d , \n",
                        posEsti.getX(), posEsti.getY(), posEsti.getAng(), coor.x, coor.y, direction);
        }

        bool isRWallControllable() {
            // マウスと同じ角度の座標系における区画の入口からの距離
            float fmod_x = fmodf(posEsti.getX(), 0.09f);
            float fmod_y = fmodf(posEsti.getY(), 0.09f);
            float dist = 0.0f;
            if(getDirection() == direction_e::E) {
                dist = fmod_x;
            }
            if(getDirection() == direction_e::N) {
                dist = fmod_y;
            }
            if(getDirection() == direction_e::W) {
                dist = 0.09f - fmod_x;
            }
            if(getDirection() == direction_e::S) {
                dist = 0.09f - fmod_y;
            }

            uint8_t x_ = coor.x;
            uint8_t y_ = coor.y;
            if(dist > 0.045f) {
                if (direction == direction_e::E) x_++;
                else if (direction == direction_e::N) y_++;
                else if (direction == direction_e::W) x_--;
                else if (direction == direction_e::S) y_--;
            }

            return maze.existRWall(x_, y_, direction);
        }

        bool isLWallControllable() {
            // マウスと同じ角度の座標系における区画の入口からの距離
            float fmod_x = fmodf(posEsti.getX(), 0.09f);
            float fmod_y = fmodf(posEsti.getY(), 0.09f);
            float dist = 0.0f;
            if(getDirection() == direction_e::E) {
                dist = fmod_x;
            }
            if(getDirection() == direction_e::N) {
                dist = fmod_y;
            }
            if(getDirection() == direction_e::W) {
                dist = 0.09f - fmod_x;
            }
            if(getDirection() == direction_e::S) {
                dist = 0.09f - fmod_y;
            }

            uint8_t x_ = coor.x;
            uint8_t y_ = coor.y;
            if(dist > 0.045f) {
                if (direction == direction_e::E) x_++;
                else if (direction == direction_e::N) y_++;
                else if (direction == direction_e::W) x_--;
                else if (direction == direction_e::S) y_--;
            }

            return maze.existLWall(x_, y_, direction);
        }


      private:
        UMouse() {

            direction = N;
            direct_duty_set_enable = false;
            maze.readMazeDataFromFlash();
            running_sec = 0.0f;
            ang_no_calib_sec = 0.0f;
        }
        ;
        ~UMouse() {
        }

        void printOutOfControl(){
            ICM20602& icm = ICM20602::getInstance();
            WheelOdometry& wo = WheelOdometry::getInstance();
            WallSensor& ws = WallSensor::getInstance();

            printfAsync("◇◇◇◇ OutOfControl!\n");
            printfAsync("is upside down: %d\n", icm.isUpsideDown());
            printfAsync("is out of control: %d\n", ctrlMixer.isOutOfControl());
            printfAsync("ang_pidf.e_k0: %f\n", ctrlMixer.ang_pidf.e_k0);
            printfAsync("motion_type: %d\n", trajCommander.getTraj().motion_type);
            printfAsync("(ang_v_t, wodo, gyro)=(%f, %f, %f)\n",trajCommander.ang_v, wo.ang_v, icm.omega_f[2]);
            printfAsync("(x_t, y_t, ang_t)=(%f, %f, %f)\n", trajCommander.x, trajCommander.y, trajCommander.ang);
            printfAsync("(x_e, y_e  ang_e)=(%f, %f, %f)\n", posEsti.getX(), posEsti.getY(), posEsti.getAng());
            printfAsync("-------------------------\n");
        }
    };

}

