#pragma once

#include "trajectory.h"
#include "myUtil.h"
#include "sound.h"
#include <deque>
#include <string>
#include "communication.h"
#include "wallsensor.h"
#include <functional>

namespace umouse {

    class TrajectoryCommander {

      public:
        float x;
        float y;
        float v;
        float a;
        float ang;
        float ang_v;
        float ang_a;
        float x_dd;
        float y_dd;


        TrajectoryCommander() {
            motion_type = EMotionType::STOP;
            turn_type = turn_type_e::STOP;
            traj_hash = 0xBEAF;
            x = 0.09f/2.0f;
            y = 0.09f/2.0f;
            ang = 90.0f;
            ang_v = 0.0f;
            ang_a = 0.0f;
            v = 0.0f;
            a = 0.0f;
            x_dd = 0.0f;
            y_dd = 0.0f;
            lock_guard = false;

        }

        EMotionType getMotionType() {            
            lock_guard = true;
            if(trajQueue.empty()){
                return EMotionType::STOP;
            }
            EMotionType type = motion_type;
            lock_guard = false;
            return type;
        }

        turn_type_e getTurnType() {
            lock_guard = true;
            if(trajQueue.empty()){
                return turn_type_e::STOP;
            }

            turn_type_e type = turn_type;
            lock_guard = false;
            return type;
        }

        uint16_t getTrajHash() {
            lock_guard = true;
            uint16_t hash = traj_hash;
            lock_guard = false;
            return hash;
        }


        bool isStraight() {
            if(trajQueue.empty()){
                return false;
            }
            
            if(motion_type == EMotionType::STRAIGHT ||
                    motion_type == EMotionType::STRAIGHT_WALL_CENTER){
                return true;
            }
            else{
                return false;
            }
        }

        void update(float x_esti, float y_esti, float ang_esti) {
            if(lock_guard) return;

            if(trajQueue.empty() == false) {

                trajQueue.front()->update();
                motion_type = trajQueue.front()->motion_type;
                turn_type = trajQueue.front()->turn_type;
                traj_hash = trajQueue.front()->hash;
                x = trajQueue.front()->x;
                y = trajQueue.front()->y;
                ang = trajQueue.front()->ang;
                ang_v = trajQueue.front()->ang_v;
                ang_a = trajQueue.front()->ang_a;
                v = trajQueue.front()->v;
                a = trajQueue.front()->a;
                x_dd = trajQueue.front()->x_dd;
                y_dd = trajQueue.front()->y_dd;


                float dist = residualDist(x_esti, y_esti);

                // 距離を超過
                if((motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                        motion_type == EMotionType::STRAIGHT ||
                        motion_type == EMotionType::DIAGONAL ||
                        motion_type == EMotionType::DIAGONAL_CENTER
                   ) &&
                        dist < 0.0) {
                    printfAsync("■over e=(%f, %f)", x_esti, y_esti);
                    printfAsync(" t=(%f, %f)\n", x, y);

                    x = trajQueue.front()->getEndX();
                    y = trajQueue.front()->getEndY();
                    ang = trajQueue.front()->getEndAng();                    
                    trajQueue.front()->forceEnd();
                }

                
                if(trajQueue.front()->isEnd() == true) {
                    x = trajQueue.front()->getEndX();
                    y = trajQueue.front()->getEndY();
                    ang = trajQueue.front()->getEndAng();
                                        
                    printfAsync("●end e=(%f, %f)", x_esti, y_esti);
                    printfAsync(" t=(%f, %f)\n", x, y);

                    trajQueue.pop_front();

                    WallSensor& ws = WallSensor::getInstance();

                    if((motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                            motion_type == EMotionType::STRAIGHT ||
                            motion_type == EMotionType::DIAGONAL ||
                            motion_type == EMotionType::DIAGONAL_CENTER
                       ) &&                            
                            dist > 0.0001 
                      ) {                        
                        printfAsync("★less (%f, %f) %f \n", x_esti, y_esti, dist);
                        residualCompensation(dist);
                    }

                    if(trajQueue.empty() == false) {
                        trajQueue.front()->setInitPos(x, y, ang);
                        motion_type = trajQueue.front()->motion_type;
                    }
                }
            }


        }

        void residualCompensation(float dist) {            
            float v_0 = constrainL(v, 0.025f);
            float a_0 = 0.0f;
            std::unique_ptr<BaseTrajectory> traj = nullptr;

            if(motion_type == EMotionType::STRAIGHT) {
                traj = StraightTrajectory::create(ABS(dist), v_0, v_0, v_0, a_0, a_0);
            } else if(motion_type == EMotionType::STRAIGHT_WALL_CENTER) {
                traj = StraightTrajectory::createAsWallCenter(ABS(dist), v_0, v_0, v_0, a_0, a_0);
            } else if(motion_type == EMotionType::DIAGONAL) {
                traj = StraightTrajectory::createAsDiagonal(ABS(dist), v_0, v_0, v_0, a_0, a_0);
            } else if(motion_type == EMotionType::DIAGONAL_CENTER) {
                traj = StraightTrajectory::createAsDiagonalCenter(ABS(dist), v_0, v_0, v_0, a_0, a_0);
            } else {
                printfAsync("EEEEEE residual compensation Error!\n");
                return;
            }

            float rad = DEG2RAD(ang);

            x -= cosf(rad) * dist;
            y -= sinf(rad) * dist;

            traj->setInitPos(x, y, ang);            
            trajQueue.push_front(std::move(traj));
            //SE_Im7();
        }

        float residualDist(float x_esti, float y_esti) {
            float dist = 0.0f;
            if(!trajQueue.empty()) {

                float rad = DEG2RAD(ang);
                float x1 = trajQueue.front()->getEndX();
                float y1 = trajQueue.front()->getEndY();
                dist = (x1 - x_esti) * cosf(rad) + (y1 - y_esti) * sinf(rad);
            }

            return dist;
        }

        void push(std::unique_ptr<BaseTrajectory>&& traj) {
            lock_guard = true;
            if(trajQueue.empty()) {
                traj->setInitPos(x, y, ang);
            }
            trajQueue.push_back(std::move(traj));
            lock_guard = false;
        }

        void clear() {
            lock_guard = true;
            while(!trajQueue.empty()) trajQueue.pop_front();
            lock_guard = false;
        }

        bool empty() {
            lock_guard = true;
            bool is_empty = trajQueue.empty();
            lock_guard = false;
            return is_empty;
            
        }

        uint16_t size() {
            lock_guard = true;
            uint16_t size = trajQueue.size();
            lock_guard = false;
            return size;
        }

        bool isEnd() {
            lock_guard = true;
            bool rtn = trajQueue.front()->isEnd();
            lock_guard = false;
            if(trajQueue.empty() == false) return rtn;
            else return false;
        }

        const BaseTrajectory& getTraj() {
            return *trajQueue.front();
        }

        void reset(float x_, float y_, float ang_) {
            lock_guard = true;
            x = x_;
            y = y_;
            ang = ang_;
            lock_guard = false;
        }
      private:
        EMotionType motion_type;
        turn_type_e turn_type;
        uint16_t traj_hash;
        std::deque< std::unique_ptr<BaseTrajectory> > trajQueue;        
        bool lock_guard;

    };

}

