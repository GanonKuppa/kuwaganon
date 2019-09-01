#pragma once

#include "trajectory.h"
#include "myUtil.h"
//#include "sound.h"
#include <deque>
#include <string>
#include "communication.h"
#include "sendData2WebApp.h"
//#include "wallsensor.h"

namespace umouse{

class TrajectoryCommander{

public:
    float x;
    float y;
    float v;
    float ang;
    float ang_v;


    TrajectoryCommander(){
        motion_type = EMotionType::STOP;
        x = 0.09f/2.0f;
        y = 0.09f/2.0f - 0.01765;
        ang = 90.0f;
        ang_v = 0.0f;
        v = 0.0f;
    }

    EMotionType getMotionType() {
        return motion_type;
    }

    bool isStraight(){
        if(motion_type == EMotionType::STRAIGHT ||
           motion_type == EMotionType::STRAIGHT_WALL_CENTER)
            return true;
        else
            return false;
    }

    void update(){
        

        if(trajQueue.empty() == false){
           
            trajQueue.front()->update();
            motion_type = trajQueue.front()->motion_type;
            x = trajQueue.front()->x;
            y = trajQueue.front()->y;
            ang = trajQueue.front()->ang;
            ang_v = trajQueue.front()->ang_v;
            v = trajQueue.front()->v;
/*
            float dist = residualDist(esti);

            if((motion_type == EMotionType::STRAIGHT_WALL_CENTER || 
                    motion_type == EMotionType::STRAIGHT) &&
                    dist < 0.0){
                auto s = trajQueue.front()->getMotionTypeString().c_str();
                uint16_t hash = trajQueue.front()->hash;

                printfAsync(">    ------- exceeded traj end! --------\n");
                printfAsync(">    motion_type: %s %04x\n", s, hash);
                printfAsync(">    (x_t, y_t, ang_t)=(%f, %f, %f) queue num:%d | dist:%f \n", x, y, ang ,trajQueue.size(), trajQueue.front()->target_dist);
                printfAsync(">    (x_e, y_e, ang_e)=(%f, %f, %f)\n", esti.x, esti.y, esti.ang);
                printfAsync(">    cumulative_dist overwrited!\n");

                x = trajQueue.front()->getEndX();
                y = trajQueue.front()->getEndY();
                ang = trajQueue.front()->getEndAng();
                trajQueue.front()->cumulative_dist = trajQueue.front()->target_dist;
            }
*/

/*
            if(motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                    motion_type == EMotionType::STRAIGHT){
                WallSensor &ws = WallSensor::getInstance();
                if(ws.isContactWall() == true)trajQueue.front()->forceEnd();
            }
*/

            if(trajQueue.front()->isEnd() == true){
                x = trajQueue.front()->x;
                y = trajQueue.front()->y;
                ang = trajQueue.front()->ang;


                auto s = trajQueue.front()->getMotionTypeString().c_str();
                uint16_t hash = trajQueue.front()->hash;

                printfAsync(">    ------- traj end --------\n");
                printfAsync(">    motion_type: %s %04x\n", s, hash);
                printfAsync(">    (x_t, y_t, ang_t)=(%f, %f, %f) queue num:%d | dist:%f \n", x, y, ang ,trajQueue.size(), trajQueue.front()->target_dist);
                //printfAsync(">    (x_e, y_e, ang_e)=(%f, %f, %f)\n", esti.x, esti.y, esti.ang);
                //printfAsync(">    residualDist: %f\n", dist);
                printfAsync(">    -------------------------\n");
                sendNeedle(x, y);

                trajQueue.pop_front();
/*
                WallSensor &ws = WallSensor::getInstance();

                if((motion_type == EMotionType::STRAIGHT_WALL_CENTER || 
                        motion_type == EMotionType::STRAIGHT) &&
                        ws.isContactWall() == false &&
                        dist > 0.0
                        ){
                    residualCompensation(esti, dist);
                    printfAsync(">    push_front residual straight\n");
                }
*/
                if(trajQueue.empty() == false){
                    trajQueue.front()->setInitPos(x, y, ang);
                    motion_type = trajQueue.front()->motion_type;

                    auto s = trajQueue.front()->getMotionTypeString().c_str();
                    uint16_t hash = trajQueue.front()->hash;
/*
                    printfAsync("★    ------- traj begin --------\n");
                    printfAsync("★    motion_type: %s %04x\n", s, hash);
                    printfAsync("★    (x_t, y_t, ang_t)=(%f, %f, %f) queue num:%d | dist:%f \n", x, y, ang ,trajQueue.size(), trajQueue.front()->target_dist);
                    printfAsync("★    (x_e, y_e, ang_e)=(%f, %f, %f)\n", esti.x, esti.y, esti.ang);
                    printfAsync("★    residualDist: %f\n", dist);
                    printfAsync("★    -------------------------\n");
*/

                }
            }
        }


    }
/*
    void residualCompensation(const PositionEstimator& esti, float dist){
        printfAsync(">    <<<<< %f %f %f\n", x, y, ang);
        float v_0 = constrainL(v, 0.1);
        float a_0 = 0.0f;
        std::unique_ptr<BaseTrajectory> traj = nullptr;

        if(motion_type == EMotionType::STRAIGHT){
            traj = StraightTrajectory::create(ABS(dist), v_0, v_0, v_0, a_0, a_0);
        }
        else if(motion_type == EMotionType::STRAIGHT_WALL_CENTER){
            traj = StraightTrajectory::createAsWallCenter(ABS(dist), v_0, v_0, v_0, a_0, a_0);
        } 
        else{
            printfAsync("EEEEEE residual compensation Error!\n");
            return;
        }
        if(esti.ang >= 315.0f || esti.ang <  45.0f) x -= dist;        
        if(esti.ang >=  45.0f && esti.ang < 135.0f) y -= dist;
        if(esti.ang >= 135.0f && esti.ang < 225.0f) x += dist;
        if(esti.ang >= 225.0f && esti.ang < 315.0f) y += dist;
        traj->setInitPos(x, y, ang);
        printfAsync(">    >>>>> %f %f %f\n", x, y, ang);
        trajQueue.push_front(std::move(traj));
        //SE_Im7();
    }

    float residualDist(const PositionEstimator& esti){
        float dist = 0.0f;
        if(trajQueue.empty() == false){
            if(esti.ang >= 315.0f || esti.ang <  45.0f) dist = trajQueue.front()->getEndX() - esti.x;
            if(esti.ang >=  45.0f && esti.ang < 135.0f) dist = trajQueue.front()->getEndY() - esti.y;
            if(esti.ang >= 135.0f && esti.ang < 225.0f) dist = esti.x - trajQueue.front()->getEndX();
            if(esti.ang >= 225.0f && esti.ang < 315.0f) dist = esti.y - trajQueue.front()->getEndY();
        }
        return dist;
    }
*/
    void push(std::unique_ptr<BaseTrajectory> &&traj){
        if(trajQueue.empty() == true){
            traj->setInitPos(x, y, ang);

            auto s = trajQueue.front()->getMotionTypeString().c_str();
            uint16_t hash = trajQueue.front()->hash;
/*
            printfAsync("★    ------- traj begin --------\n");
            printfAsync("★    motion_type: %s %04x\n", s, hash);
            printfAsync("★    (x_t, y_t, ang_t)=(%f, %f, %f) queue num:%d | dist:%f \n", x, y, ang ,trajQueue.size(), trajQueue.front()->target_dist);
            printfAsync("★    -------------------------\n");
*/
        }
        trajQueue.push_back(std::move(traj));
    }

    uint16_t getPushedNum() {
        return trajQueue.size();
    }

    void clear(){
        while(!trajQueue.empty()) trajQueue.pop_front();
    }

    bool empty(){
        return trajQueue.empty();
    }

    uint16_t size(){
        return trajQueue.size();
    }

    bool isEnd(){
        if(trajQueue.empty() == false) return trajQueue.front()->isEnd();
        else return false;
    }

    const BaseTrajectory& getTraj(){
        return *trajQueue.front();
    }
    const BaseTrajectory& getPrevious(){
        return *trajPrevious;
    }

    void reset(float x_, float y_, float ang_){
        x = x_;
        y = y_;
        ang = ang_;
    }
private:
    EMotionType motion_type;
    std::deque< std::unique_ptr<BaseTrajectory> > trajQueue;
    std::unique_ptr<BaseTrajectory> trajPrevious;

};

}

