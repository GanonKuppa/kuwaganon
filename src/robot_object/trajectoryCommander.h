#pragma once

#include "trajectory.h"
#include "myUtil.h"
#include "sound.h"
#include <deque>
#include <string>
#include "communication.h"
#include "wallsensor.h"
#include "positionEstimator.h"
#include <functional>

#define DEBUG 0

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

        }

        EMotionType getMotionType() {
            return motion_type;
        }

        turn_type_e getTurnType() {
            return turn_type;
        }

        uint16_t getTrajHash() {
            return traj_hash;
        }



        bool isStraight() {
            if(motion_type == EMotionType::STRAIGHT ||
                    motion_type == EMotionType::STRAIGHT_WALL_CENTER)
                return true;
            else
                return false;
        }

        void update(PositionEstimator& esti) {


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


                float dist = residualDist(esti);

                // 距離を超過
                if((motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                        motion_type == EMotionType::STRAIGHT ||
                        motion_type == EMotionType::DIAGONAL ||
                        motion_type == EMotionType::DIAGONAL_CENTER
                   ) &&
                        dist < 0.0) {
                    auto s = trajQueue.front()->getMotionTypeString().c_str();
                    uint16_t hash = trajQueue.front()->hash;
#if DEBUG                    
                                    printfAsync(">    ------- exceeded traj end! --------\n");
                                    printfAsync(">    motion_type: %s %04x\n", s, hash);
                                    printfAsync(">    (x_t, y_t, ang_t)=(%f, %f, %f) queue num:%d | dist:%f \n", x, y, ang ,trajQueue.size(), trajQueue.front()->target_dist);
                                    printfAsync(">    (x_e, y_e, ang_e)=(%f, %f, %f)\n", esti.getX(), esti.getY(), esti.getAng());
                                    printfAsync(">    cumulative_dist overwrited!\n");
#endif                    
                    x = trajQueue.front()->getEndX();
                    y = trajQueue.front()->getEndY();
                    ang = trajQueue.front()->getEndAng();
                    //trajQueue.front()->cumulative_dist = trajQueue.front()->target_dist;
                    trajQueue.front()->forceEnd();
                }

                
                if(trajQueue.front()->isEnd() == true) {
                    x = trajQueue.front()->getEndX();
                    y = trajQueue.front()->getEndY();
                    ang = trajQueue.front()->getEndAng();

                    auto s = trajQueue.front()->getMotionTypeString().c_str();
                    uint16_t hash = trajQueue.front()->hash;
#if DEBUG                    
                                    printfAsync(">    ------- traj end --------\n");
                                    printfAsync(">    motion_type: %s %04x\n", s, hash);
                                    printfAsync(">    (x_t, y_t, ang_t)=(%f, %f, %f) queue num:%d | dist:%f \n", x, y, ang ,trajQueue.size(), trajQueue.front()->target_dist);
                                    printfAsync(">    (x_e, y_e, ang_e)=(%f, %f, %f)\n", esti.getX(), esti.getY(), esti.getAng());
                                    printfAsync(">    residualDist: %f\n", dist);
                                    printfAsync(">    -------------------------\n");
#endif                    
                    trajQueue.pop_front();

                    WallSensor& ws = WallSensor::getInstance();

                    if((motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                            motion_type == EMotionType::STRAIGHT ||
                            motion_type == EMotionType::DIAGONAL ||
                            motion_type == EMotionType::DIAGONAL_CENTER
                       ) &&
                            //ws.isContactWall() == false &&
                            dist > 0.0001 
                      ) {
                        //printfAsync(">    push_front residual straight %f\n", dist);
                        printfAsync("★pf (%f, %f, %f) %f \n", esti.getX(), esti.getY(), esti.getAng(), dist);

                        residualCompensation(esti, dist);
                    }

                    if(trajQueue.empty() == false) {
                        trajQueue.front()->setInitPos(x, y, ang);
                        motion_type = trajQueue.front()->motion_type;

                        auto s = trajQueue.front()->getMotionTypeString().c_str();
                        uint16_t hash = trajQueue.front()->hash;
#if DEBUG                        
                                            printfAsync("★    ------- traj begin --------\n");
                                            printfAsync("★    motion_type: %s %04x\n", s, hash);
                                            printfAsync("★    (x_t, y_t, ang_t)=(%f, %f, %f) queue num:%d | dist:%f \n", x, y, ang ,trajQueue.size(), trajQueue.front()->target_dist);
                                            printfAsync("★    (x_e, y_e, ang_e)=(%f, %f, %f)\n", esti.getX(), esti.getY(), esti.getAng());
                                            printfAsync("★    residualDist: %f\n", dist);
                                            printfAsync("★    -------------------------\n");
#endif

                    }
                }
            }


        }

        void residualCompensation(const PositionEstimator& esti, float dist) {
            //printfAsync(">    <<<<< %f %f %f\n", x, y, ang);
            float v_0 = constrainL(v, 0.05f);
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
            //printfAsync(">    >>>>> %f %f %f\n", x, y, ang);
            trajQueue.push_front(std::move(traj));
            //SE_Im7();
        }

        float residualDist(PositionEstimator& esti) {
            float dist = 0.0f;
            if(!trajQueue.empty()) {

                float rad = DEG2RAD(ang);
                float x1 = trajQueue.front()->getEndX();
                float y1 = trajQueue.front()->getEndY();
                dist = (x1 - esti.getX()) * cosf(rad) + (y1 - esti.getY()) * sinf(rad);

            }

            return dist;
        }

        void push(std::unique_ptr<BaseTrajectory>&& traj) {
            if(trajQueue.empty()) {
                traj->setInitPos(x, y, ang);

                
                    auto s = trajQueue.front()->getMotionTypeString().c_str(); // null参照
                    uint16_t hash = trajQueue.front()->hash;
#if DEBUG
                    printfAsync("★    ------- traj begin --------\n");
                    printfAsync("★    motion_type: %s %04x\n", s, hash);
                    printfAsync("★    (x_t, y_t, ang_t)=(%f, %f, %f) queue num:%d | dist:%f \n", x, y, ang ,trajQueue.size(), trajQueue.front()->target_dist);
                    printfAsync("★    -------------------------\n");
#endif                
            }
            trajQueue.push_back(std::move(traj));
        }

        uint16_t getPushedNum() {
            return trajQueue.size();
        }

        void clear() {
            while(!trajQueue.empty()) trajQueue.pop_front();
        }

        bool empty() {
            return trajQueue.empty();
        }

        uint16_t size() {
            return trajQueue.size();
        }

        bool isEnd() {
            if(trajQueue.empty() == false) return trajQueue.front()->isEnd();
            else return false;
        }

        const BaseTrajectory& getTraj() {
            return *trajQueue.front();
        }
        const BaseTrajectory& getPrevious() {
            return *trajPrevious;
        }

        void reset(float x_, float y_, float ang_) {
            x = x_;
            y = y_;
            ang = ang_;
        }
      private:
        EMotionType motion_type;
        turn_type_e turn_type;
        uint16_t traj_hash;
        std::deque< std::unique_ptr<BaseTrajectory> > trajQueue;
        std::unique_ptr<BaseTrajectory> trajPrevious;

    };

}

