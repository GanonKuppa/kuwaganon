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
        static constexpr float WALL2MOUSE_CENTER_DIST = 0.01765;
        static constexpr float READ_WALL_OFFSET = 0.0125;
        static constexpr float DELTA_T = 0.0005;

        // -32768 から 32767




        float t_a;//event側で更新     x 1000
        float t_v;//event側で更新     x 3000
        float t_x;//event側で更新     x 3000
        float accum_x;//event側で更新     x 3000

        float t_ang_a;//event側で更新    x 100
        float t_ang_v;//event側で更新    x 100
        float t_ang;//event側で更新    x 100
        float accum_ang;//event側で更新    x 100
        float gyro_ang_v;// -2000deg/secから+2000deg/sec
        float v_acc;
        float v_side;

        float wall_P;//event側で更新  -10.0から10.0   x 3000
        float wall_I;//event側で更新  -10.0から10.0   x 3000
        float wall_D;//event側で更新  -10.0から10.0   x 3000

        float v_P;//event側で更新  -10.0から10.0   x 3000
        float v_I;//event側で更新  -10.0から10.0   x 3000
        float v_D;//event側で更新  -10.0から10.0   x 3000

        float ang_v_P;//event側で更新  -10.0から10.0   x 3000
        float ang_v_I;//event側で更新  -10.0から10.0   x 3000
        float ang_v_D;//event側で更新  -10.0から10.0   x 3000

        float ang_P;//event側で更新  -10.0から10.0   x 3000
        float ang_I;//event側で更新  -10.0から10.0   x 3000
        float ang_D;//event側で更新  -10.0から10.0   x 3000

        float v_FF;//event側で更新  -1.0から1.0     x 3000
        float ang_FF;//event側で更新  -1.0から1.0     x 3000

        float duty_L;
        float duty_R;

        direction_e direction;
        Coor2D<uint16_t> coor;
        Coor2D<uint16_t> start;
        Coor2D<uint16_t> goal;

        float ang;
        float ang_v;
        float ang_a;

        Coor2D<float> v_g;
        Coor2D<float> x_g;
        Coor2D<float> a_g;

        const uint16_t buff_size = 30;
        std::deque<float> t_ang_v_buff;
        std::deque<float> ang_v_buff;
        std::deque<float> t_v_buff;
        std::deque<float> v_buff;
        std::deque<float> v_acc_buff;
        std::deque<float> acc_x_buff;
        std::deque<float> acc_y_buff;

        Maze maze;
        bool error_run_flag;
        bool direct_duty_set_enable;

        ControlMixer ctrlMixer;
        TrajectoryCommander trajCommander;
        PositionEstimator posEsti;
        float running_sec;
        float ang_no_calib_sec;

        void initBuff() {
            for (int i = 0; i < buff_size; i++) {
                t_ang_v_buff.push_front(0.0);
                ang_v_buff.push_front(0.0);
                t_v_buff.push_front(0.0);
                v_buff.push_front(0.0);
                v_acc_buff.push_front(0.0);
                acc_x_buff.push_front(0.0);
                acc_y_buff.push_front(0.0);
            }
        }

        void updateBuff() {
            t_ang_v_buff.push_front(trajCommander.ang_v);
            ang_v_buff.push_front(posEsti.getAngV());
            t_v_buff.push_front(trajCommander.v);
            v_buff.push_front(posEsti.getV());
            v_acc_buff.push_front(v_acc);
            acc_y_buff.push_front(0.0f);

            t_ang_v_buff.pop_back();
            ang_v_buff.pop_back();
            t_v_buff.pop_back();
            v_buff.pop_back();
            v_acc_buff.pop_back();
            acc_y_buff.pop_back();
        }


        static UMouse& getInstance() {
            static UMouse instance;
            return instance;
        }

        void update() {
            ICM20602& imu = ICM20602::getInstance();
            umouse::adis16470& adis = umouse::adis16470::getInstance();
            WheelOdometry& wo = WheelOdometry::getInstance();
            PowerTransmission& pt = PowerTransmission::getInstance();
            WallSensor& ws = WallSensor::getInstance();
            PseudoDialL& pdl = PseudoDialL::getInstance();
            PseudoDialR& pdr = PseudoDialR::getInstance();


            //posEsti.update(wo.getV_double(), (double)adis.omega_f[2], (double)adis.originOffsetCompAy(), (double)adis.originOffsetCompAx(), trajCommander.getMotionType(), ws);
            posEsti.update(wo.getV_double(), (double)adis.omega_f[2], (double)imu.acc_f[1], (double)imu.acc_f[0], trajCommander.getMotionType(), ws);

            trajCommander.update(posEsti);

            if(!trajCommander.empty() && trajCommander.getMotionType() == EMotionType::DIRECT_DUTY_SET) {
                direct_duty_set_enable = true;
            } else {
                direct_duty_set_enable = false;
            }

            if(direct_duty_set_enable == false) {
                if(trajCommander.empty() == false) {
                    auto traj = trajCommander.getTraj();
                    ctrlMixer.update(traj, posEsti, isRWallControllable(), isLWallControllable());
                    auto duty = ctrlMixer.getDuty();

                    if( ws.getContactWallTime() > 0.1f &&
                            (trajCommander.getMotionType() == EMotionType::STRAIGHT ||
                             trajCommander.getMotionType() == EMotionType::STRAIGHT_WALL_CENTER)
                      ) {
                        pt.setDuty(0.35, 0.35);
                        ctrlMixer.reset();
                        //trajCommander.clear();
                    } else {
                        pt.setDuty(duty(0), duty(1));
                    }
                } else {
                    ctrlMixer.reset();
                    if(pdl.getEnable() == false && pdr.getEnable() == false ) pt.setDuty(0.0, 0.0);
                }
            }
            if( imu.isUpsideDown() == true ||
                    (ctrlMixer.isOutOfControl() == true && (ABS(pt.getDuty_R()) > 0.1f || ABS(pt.getDuty_L()) > 0.1f )) ||
                    (ABS(ctrlMixer.ang_pidf.e_k0) > 100.0f )

              ) {
                printfAsync("◇◇◇◇ OutOfControl!\n");
                printfAsync("is upside down: %d\n", imu.isUpsideDown());
                printfAsync("is out of control: %d\n", ctrlMixer.isOutOfControl());
                printfAsync("ang_pidf.e_k0: %f\n", ctrlMixer.ang_pidf.e_k0);
                printfAsync("motion_type: %d\n", trajCommander.getTraj().motion_type);
                printfAsync("(ang_v_t, wodo, gyro)=(%f, %f, %f)\n",trajCommander.ang_v, wo.ang_v, imu.omega_f[2]);
                printfAsync("(x_t, y_t, ang_t)=(%f, %f, %f)\n", trajCommander.x, trajCommander.y, trajCommander.ang);
                printfAsync("(x_e, y_e  ang_e)=(%f, %f, %f)\n", posEsti.getX(), posEsti.getY(), posEsti.getAng());
                printfAsync("-------------------------\n");

                trajCommander.clear();
                pt.setDuty(0.0f, 0.0f);

                SEH();
            }

            if(direct_duty_set_enable == false && trajCommander.empty()
                    && pdl.getEnable() == false && pdr.getEnable() == false) {
                pt.setDuty(0.0f, 0.0f);
            }

            updateBuff();

            coor.x = (uint8_t)(posEsti.getX() / 0.09f);
            coor.y = (uint8_t)(posEsti.getY() / 0.09f);

            direction = getDirection();
            running_sec += DELTA_T;
            ang_no_calib_sec += DELTA_T;
        }






        /*
        uint8_t getCoorX(){return (uint8_t)(posEsti.x / 0.09f); }
        uint8_t getCoorY(){return (uint8_t)(posEsti.y / 0.09f); }
        */
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
            initBuff();
            maze.readMazeDataFromFlash();
            running_sec = 0.0f;
            ang_no_calib_sec = 0.0f;
        }
        ;
        ~UMouse() {
        }

    };

}

