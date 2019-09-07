#pragma once

#include "baseActivity.h"
#include "fcled.h"
#include "stateMachine.h"
#include "timer.h"
#include "trajectory.h"
#include "curveFactory.h"
#include "communication.h"
#include "sound.h"
#include "wallSensor.h"
#include "parameterManager.h"
#include "powerTransmission.h"
#include "adis16470.h"

namespace umouse {

    enum class ESearchMode : uint8_t {
        BACK_MODE_SELECT = 0,
        SPIN_TURN_SERCH = 1,
        SLALOM_SERCH = 2,
        SPIN_TURN_SERCH_BOTHWAYS = 3,
        SLALOM_SERCH_BOTHWAYS = 4
    };

    class SearchRunActivity : public BaseActivity {

    protected:

        ELoopStatus loop() {
            stateMachine.update();
            if(stateMachine.empty() == true) return ELoopStatus::FINISH;
            else return ELoopStatus::CONTINUE;
        }
        void onStart() {
            printfAsync("■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■.\n");
            printfAsync("This is search run activity.\n");

            Intent *intent = new Intent();
            intent->uint8_t_param["SUB_MODE_NUM"] = 5;
            auto activity = ActivityFactory::cteateSubModeSelect();
            activity->start(intent);
            printfAsync("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
            if(intent->uint8_t_param["SUB_MODE"] == (uint8_t)ESearchMode::BACK_MODE_SELECT) return;

            UMouse &m = UMouse::getInstance();
            waitmsec(1000);
            ICM20602 &icm = ICM20602::getInstance();
            adis16470 &adis = adis16470::getInstance();
            icm.calibOmegaOffset(200);
            icm.calibAccOffset(200);
            adis.calibOmegaOffset(800);

            std::unique_ptr<BaseState> state = std::unique_ptr<BaseState>(new Start2GoalState(intent));
            stateMachine.push(std::move(state));
            delete intent;

        }
        void onFinish() {

        };

    private:
        StateMachine stateMachine;
        class Start2GoalState : public BaseState {
        public:
            float a;
            float v;
            ESearchMode mode;
            bool pre_in_read_wall_area;
            Coor2D<uint16_t> pre_read_wall_coor;
            uint32_t ahead_comp_timestamp;

            Start2GoalState(Intent* intent_) : BaseState(intent_) {
                mode = (ESearchMode)(intent_->uint8_t_param["SUB_MODE"]);
            }

            void onStart() {
                ICM20602 &icm = ICM20602::getInstance();
                ParameterManager &pm = ParameterManager::getInstance();

                UMouse &m = UMouse::getInstance();
                float x = 0.09f/2.0f;
                float y = 0.09f/2.0f - m.WALL2MOUSE_CENTER_DIST;
                a = pm.a_search_run;
                v = pm.v_search_run;
                pre_in_read_wall_area = false;
                float ang = 90.0f;
                m.posEsti.reset(x,y,ang);
                m.trajCommander.reset(x,y,ang);
                m.ctrlMixer.reset();
                m.goal.set(pm.goal_x, pm.goal_y);
                m.start.set(0, 0);
                m.coor.set(0, 0); 
                m.maze.updateStartSectionWall();

                float dist = 0.09f/2.0f + m.WALL2MOUSE_CENTER_DIST;
                auto traj = StraightTrajectory::createAsWallCenter(dist, 0.0f, v, v, a, a);
                m.trajCommander.push(std::move(traj));
                ahead_comp_timestamp = getElapsedMsec();
                pre_read_wall_coor.set(255, 255);

            }
            void update() {
                UMouse &m = UMouse::getInstance();
                WallSensor &ws = WallSensor::getInstance();
                FcLed &fcled = FcLed::getInstance();
                bool in_read_wall_area = m.inReadWallArea();
                ParameterManager &pm = ParameterManager::getInstance();

                if(m.maze.isExistPath(m.goal.x, m.goal.y) == false) {
                    SE_Im7();
                    waitmsec(500);
                    SE_Im7();
                    waitmsec(500);
                    SE_Im7();
                    waitmsec(10000);
                }
/*
                if(m.trajCommander.empty() == true){
                    if(ws.isAhead() == true){
                        SE_I7();
                        printfAsync("msg_flag:%f,%f,壁あるんだけど\n",m.posEsti.x, m.posEsti.y);
                        float ang_crash;
                        Wall wall = m.maze.readWall(m.coor.x, m.coor.y);
                        if(m.getDirection() == direction_e::E){
                            wall.E = true;
                            ang_crash = 0.0f;
                        }
                        else if(m.getDirection() == direction_e::N){
                            wall.N = true;
                            ang_crash = 90.0f;
                        }
                        else if(m.getDirection() == direction_e::W){
                            wall.W = true;
                            ang_crash = 180.0f;
                        }
                        else if(m.getDirection() == direction_e::S){
                            wall.S = true;
                            ang_crash = 270.0f;
                        }

                        m.maze.writeWall(m.coor.x, m.coor.y, wall);
                        m.trajCommander.reset(0.045 + 0.09 * m.coor.x, 0.045 + 0.09 * m.coor.y ,ang_crash);
                        m.maze.makeSearchMap(m.goal.x, m.goal.y);
                        direction_e dest_dir_next = m.maze.getMinDirection(m.coor.x, m.coor.y, m.direction);
                        int8_t rot_times = m.maze.calcRotTimes(dest_dir_next, m.direction);

                        auto traj0 = StopTrajectory::create(2.0);
                        auto traj1 = SpinTurnTrajectory::create(rot_times * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                        auto traj2 = StopTrajectory::create(0.8);
                        auto traj3 = StraightTrajectory::createAsWallCenter(0.09/2.0, 0.0, v, v, a, a);

                        m.trajCommander.push(std::move(traj0));
                        m.trajCommander.push(std::move(traj1));
                        m.trajCommander.push(std::move(traj2));
                        m.trajCommander.push(std::move(traj3));

                    }

                }
*/

                if(in_read_wall_area) fcled.turn(0,1,0);
                else fcled.turn(0,0,1);

                if(m.trajCommander.getMotionType() == EMotionType::STOP_DIRECT_DUTY_SET) {
                    PowerTransmission &pt = PowerTransmission::getInstance();
                    if(ws.right() < 2700 && ws.left() <2700) {
                        pt.setDuty(0.5f,0.5f);
                    }
                    else {
                        if(ws.getContactWallTime() > 0.2f) pt.setDuty(0.5f, 0.5f);
                        else pt.setDuty(0.5f,0.5f);
                    }
                    if(m.getDirection() == direction_e::E || m.getDirection() == direction_e::W) m.posEsti.setX((double)m.trajCommander.x);
                    else m.posEsti.setY(m.trajCommander.y);
                    //m.posEsti.ang = m.trajCommander.ang;

                }

                if(in_read_wall_area && pre_in_read_wall_area == false && pre_read_wall_coor != m.coor) {
                    printfAsync("＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝\n");
                    pre_read_wall_coor = m.coor;
                    //SE_I7();
                    m.maze.makeSearchMap(m.goal.x, m.goal.y);
                    uint8_t x_next = m.coor.x;
                    uint8_t y_next = m.coor.y;
                    if (m.direction == direction_e::E) x_next++;
                    else if (m.direction == direction_e::N) y_next++;
                    else if (m.direction == direction_e::W) x_next--;
                    else if (m.direction == direction_e::S) y_next--;

                    printfAsync("msg_flag:%f,%f,壁|| 左 %d 前 (%d %d) 右 %d\n", m.posEsti.getX(), m.posEsti.getY(), ws.left(), ws.ahead_l(), ws.ahead_r(), ws.right());

                    m.maze.updateWall(x_next, y_next, m.direction, ws);
                    direction_e dest_dir_next = m.maze.getMinDirection(x_next, y_next, m.direction);
                    int8_t rot_times = m.maze.calcRotTimes(dest_dir_next, m.direction);

                    printfAsync("XXXXX read wall. rot_times=%d\n",rot_times);
                    //printfAsync("msg_flag:%f,%f,★next(%d %d) now(%d %d) rt %d\n", m.posEsti.getX(), m.posEsti.getY(), x_next, y_next, m.coor.x, m.coor.y, rot_times);

                    if(x_next == m.goal.x && y_next == m.goal.y) {
                        printfAsync("OOOOO Goal!\n");
                        m.maze.writeMazeData2Flash();
                        auto traj0 = StraightTrajectory::createAsWallCenter(0.03, v, v, 0.1f, a, a);
                        auto traj1 = StraightTrajectory::create(0.015f, 0.1f, 0.1f, 0.1f, a, a);
                        auto traj2 = StopTrajectory::create(0.5f);
                        m.trajCommander.push(std::move(traj0));
                        m.trajCommander.push(std::move(traj1));
                        m.trajCommander.push(std::move(traj2));

                        if(mode == ESearchMode::SLALOM_SERCH_BOTHWAYS || mode == ESearchMode::SPIN_TURN_SERCH_BOTHWAYS) {
                            Intent *intent = new Intent();
                            intent->uint8_t_param["SUB_MODE"] = (uint8_t)mode;

                            std::unique_ptr<BaseState> state = std::unique_ptr<BaseState>(new Goal2StartState(intent));
                            nextState = std::move(state);
                            delete intent;
                        }
                    }
                    else {

                        if(rot_times == 0) {
                            auto traj0 = StraightTrajectory::createAsWallCenter(0.09f, v, v, v, a, a);
                            m.trajCommander.push(std::move(traj0));
                        }
                        else if (ABS(rot_times) == 4 ||
                                ABS(m.posEsti.calcDiffAngle()) > 5.0f ||
                                ABS(m.posEsti.calcWallCenterOffset()) > 0.0075f ||
                                mode == ESearchMode::SPIN_TURN_SERCH ||
                                mode == ESearchMode::SPIN_TURN_SERCH_BOTHWAYS
                        ) {
                            printfAsync("XXXXX ahead %d | elapsed %d | timestamp %d\n",ws.isAhead(), (getElapsedMsec() - ahead_comp_timestamp), ahead_comp_timestamp  );
                            std::unique_ptr<BaseTrajectory> traj0;
                            std::unique_ptr<BaseTrajectory> traj1;
                            std::unique_ptr<BaseTrajectory> traj2;

                            if (ws.isAhead() == true) {
                                traj0 = StraightTrajectory::create(0.03f, v, v, 0.1f, a, a);
                                traj1 = StraightTrajectory::create(0.015f, 0.1f, 0.1f, 0.1f, a, a);
                                traj2 = StopTrajectory::createAsDirectDutySet(0.2);
                                SEG();
                                ahead_comp_timestamp = getElapsedMsec();
                            }
                            else {
                                traj0 = StraightTrajectory::createAsWallCenter(0.03f, v, v, 0.1f, a, a);
                                traj1 = StraightTrajectory::create(0.015f, 0.1f, 0.1f, 0.1f, a, a);
                                traj2 = StopTrajectory::createAsDirectDutySet(0.0f);
                            }
                            if(ABS(rot_times == 4))rot_times *= SIGN(m.posEsti.calcWallCenterOffset());
                            auto traj3 = SpinTurnTrajectory::create(rot_times * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                            auto traj4 = StopTrajectory::create(0.1f);
                            auto traj5 = StraightTrajectory::createAsWallCenter(0.045f, 0.0f, v, v, a, a);
                            m.trajCommander.push(std::move(traj0));
                            m.trajCommander.push(std::move(traj1));
                            m.trajCommander.push(std::move(traj2));
                            m.trajCommander.push(std::move(traj3));
                            m.trajCommander.push(std::move(traj4));
                            m.trajCommander.push(std::move(traj5));
                        }
                        else if(ABS(rot_times) == 2) {
                            //auto traj0 = CurveTrajectory::create(v, turn_type_e::TURN_90, (turn_dir_e)SIGN(rot_times));
                            auto traj0 = StraightTrajectory::create(CurveFactory::getPreDist(turn_type_e::TURN_90), v, v, v, a, a);
                            auto traj1 = CurveTrajectory::createAsNoStraght(v, turn_type_e::TURN_90, (turn_dir_e)SIGN(rot_times));
                            auto traj2 = StraightTrajectory::create(CurveFactory::getFolDist(turn_type_e::TURN_90), v, v, v, a, a);

                            m.trajCommander.push(std::move(traj0));
                            m.trajCommander.push(std::move(traj1));
                            m.trajCommander.push(std::move(traj2));
                        }
                    }
                }
                pre_in_read_wall_area = in_read_wall_area;
            }
            bool isEnd() {
                UMouse &m = UMouse::getInstance();
                ParameterManager &pm = ParameterManager::getInstance();
                if(m.goal.x == m.coor.x && m.goal.y == m.coor.y){
                    return true;
                } 
                else if(getElapsedSec() > pm.search_limit_time_sec){
                    printfAsync("search limit time up!\n");
                    m.maze.writeMazeData2Flash();
                    return true;
                }
                return false;
            }
        };

        class Goal2StartState : public BaseState {
        public:
            float a;
            float v;
            ESearchMode mode;
            bool pre_in_read_wall_area;
            Coor2D<uint16_t> pre_read_wall_coor;


            Goal2StartState(Intent* intent_) : BaseState(intent_) {
                mode = (ESearchMode)(intent_->uint8_t_param["SUB_MODE"]);
                printfAsync("＊＊＊＊＊＊＊＊＊＊ %d\n", mode);
            }

            void onStart() {
                ICM20602 &icm = ICM20602::getInstance();
                ParameterManager &pm = ParameterManager::getInstance();

                UMouse &m = UMouse::getInstance();
                a = pm.a_search_run;
                v = pm.v_search_run;
                pre_in_read_wall_area = false;
                //icm.calibOmegaOffset(800);
                //icm.calibAccOffset(800);

                direction_e dest_dir_next = m.maze.getMinDirection(m.goal.x, m.goal.y, m.direction);
                int8_t rot_times = m.maze.calcRotTimes(dest_dir_next, m.direction);

                auto traj0 = SpinTurnTrajectory::create(rot_times * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                auto traj1 = StopTrajectory::create(1.0f);
                auto traj2 = StraightTrajectory::createAsWallCenter(0.09f/2.0f, 0.0f, v, v, a, a);
                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));
                m.trajCommander.push(std::move(traj2));
                pre_read_wall_coor.set(255, 255);

            }
            void update() {
                UMouse &m = UMouse::getInstance();
                WallSensor &ws = WallSensor::getInstance();
                FcLed &fcled = FcLed::getInstance();
                bool in_read_wall_area = m.inReadWallArea();
                ParameterManager &pm = ParameterManager::getInstance();

                if(in_read_wall_area) fcled.turn(0,1,0);
                else fcled.turn(0,0,1);

                if(m.trajCommander.getMotionType() == EMotionType::STOP_DIRECT_DUTY_SET) {
                    PowerTransmission &pt = PowerTransmission::getInstance();
                    if(ws.right() < 2700 && ws.left() <2700) {
                        pt.setDuty(0.5f,0.5f);
                    }
                    else {
                        if(ws.getContactWallTime() > 0.2) pt.setDuty(0.5f, 0.5f);
                        else pt.setDuty(0.5f,0.5f);
                    }

                    if(m.getDirection() == direction_e::E || m.getDirection() == direction_e::W) m.posEsti.setX(m.trajCommander.x);
                    else m.posEsti.setY(m.trajCommander.y);
                    //m.posEsti.ang = m.trajCommander.ang;
                }

                if(in_read_wall_area && pre_in_read_wall_area == false && pre_read_wall_coor != m.coor) {
                    printfAsync("＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝\n");
                    pre_read_wall_coor = m.coor;
                    //SE_I7();
                    m.maze.makeSearchMap(m.start.x, m.start.y);
                    uint8_t x_next = m.coor.x;
                    uint8_t y_next = m.coor.y;
                    if (m.direction == direction_e::E) x_next++;
                    else if (m.direction == direction_e::N) y_next++;
                    else if (m.direction == direction_e::W) x_next--;
                    else if (m.direction == direction_e::S) y_next--;

                    m.maze.updateWall(x_next, y_next, m.direction, ws);
                    direction_e dest_dir_next = m.maze.getMinDirection(x_next, y_next, m.direction);
                    int8_t rot_times = m.maze.calcRotTimes(dest_dir_next, m.direction);

                    printfAsync("XXXXX read wall. rot_times=%d\n",rot_times);

                    if(x_next == m.start.x && y_next == m.start.y) {
                        m.maze.writeMazeData2Flash();
                        printfAsync("OOOOO Start!\n");
                        auto traj0 = StraightTrajectory::createAsWallCenter(0.09f/2.0f, v, v, 0.0f, a, a);
                        auto traj1 = StopTrajectory::create(1.0f);
                        m.trajCommander.push(std::move(traj0));
                    }
                    else {

                        if(rot_times==0) {
                            auto traj0 = StraightTrajectory::createAsWallCenter(0.09f, v, v, v, a, a);
                            m.trajCommander.push(std::move(traj0));
                        }
                        else if (ABS(rot_times) == 4 ||
                                ABS(m.posEsti.calcDiffAngle()) > 5.0f ||
                                ABS(m.posEsti.calcWallCenterOffset()) > 0.0075f ||
                                mode == ESearchMode::SPIN_TURN_SERCH ||
                                mode == ESearchMode::SPIN_TURN_SERCH_BOTHWAYS
                        ) {
                            std::unique_ptr<BaseTrajectory> traj0;
                            std::unique_ptr<BaseTrajectory> traj1;
                            std::unique_ptr<BaseTrajectory> traj2;
                            if (ws.isAhead() == true ) {
                                traj0 = StraightTrajectory::create(0.03f, v, v, 0.1f, a, a);
                                traj1 = StraightTrajectory::create(0.015f, 0.1f, 0.1f, 0.1f, a, a);
                                traj2 = StopTrajectory::createAsDirectDutySet(0.2f);
                                SEG();
                            }

                            else {
                                traj0 = StraightTrajectory::createAsWallCenter(0.03f, v, v, 0.1f, a, a);
                                traj1 = StraightTrajectory::create(0.015f, 0.1f, 0.1f, 0.1f, a, a);
                                traj2 = StopTrajectory::createAsDirectDutySet(0.0f);
                            }
                            if(ABS(rot_times==4))rot_times *= SIGN(m.posEsti.calcWallCenterOffset());
                            auto traj3 = SpinTurnTrajectory::create(rot_times * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                            auto traj4 = StopTrajectory::create(0.05f);
                            auto traj5 = StraightTrajectory::createAsWallCenter(0.045f, 0.0f, v, v, a, a);
                            m.trajCommander.push(std::move(traj0));
                            m.trajCommander.push(std::move(traj1));
                            m.trajCommander.push(std::move(traj2));
                            m.trajCommander.push(std::move(traj3));
                            m.trajCommander.push(std::move(traj4));
                            m.trajCommander.push(std::move(traj5));
                        }
                        else if(ABS(rot_times) == 2) {
                            //auto traj0 = CurveTrajectory::create(v, turn_type_e::TURN_90, (turn_dir_e)SIGN(rot_times));
                            //m.trajCommander.push(std::move(traj0));

                            auto traj0 = StraightTrajectory::create(CurveFactory::getPreDist(turn_type_e::TURN_90), v, v, v, a, a);
                            auto traj1 = CurveTrajectory::createAsNoStraght(v, turn_type_e::TURN_90, (turn_dir_e)SIGN(rot_times));
                            auto traj2 = StraightTrajectory::create(CurveFactory::getFolDist(turn_type_e::TURN_90), v, v, v, a, a);

                            m.trajCommander.push(std::move(traj0));
                            m.trajCommander.push(std::move(traj1));
                            m.trajCommander.push(std::move(traj2));

                        }
                    }
                }
                pre_in_read_wall_area = in_read_wall_area;
            }
            bool isEnd() {
                UMouse &m = UMouse::getInstance();
                ParameterManager &pm = ParameterManager::getInstance();
                if(m.start.x == m.coor.x && m.start.y == m.coor.y){
                    return true;
                } 
                else if(getElapsedSec() > pm.search_limit_time_sec){
                    printfAsync("search limit time up!\n");
                    m.maze.writeMazeData2Flash();
                    return true;
                }
                return false;
            }
        };

    };

};
