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
#include "fcled.h"

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

            Intent* intent = new Intent();
            intent->uint8_t_param["SUB_MODE_NUM"] = 5;
            auto activity = ActivityFactory::cteateSubModeSelect();
            activity->start(intent);
            printfAsync("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
            if(intent->uint8_t_param["SUB_MODE"] == (uint8_t)ESearchMode::BACK_MODE_SELECT) return;

            UMouse& m = UMouse::getInstance();
            waitmsec(1000);
            ICM20602& icm = ICM20602::getInstance();
            adis16470& adis = adis16470::getInstance();
            icm.calibOmegaOffset(200);
            icm.calibAccOffset(200);
            adis.calibOmegaOffset(800);

            std::unique_ptr<BaseState> state = std::unique_ptr<BaseState>(new Start2GoalState(intent));
            m.ang_no_calib_sec = 0.0f;
            m.running_sec = 0.0f;
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
            Coor2D<uint16_t> desti_coor;
            const float ANG_CALIB_TIME = 60.0f;


            Start2GoalState(Intent* intent_) : BaseState(intent_) {
                mode = (ESearchMode)(intent_->uint8_t_param["SUB_MODE"]);
            }

            virtual void onStart() {
                ICM20602& icm = ICM20602::getInstance();
                ParameterManager& pm = ParameterManager::getInstance();

                UMouse& m = UMouse::getInstance();
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
                desti_coor = m.goal;

                float dist = 0.09f/2.0f + m.WALL2MOUSE_CENTER_DIST;
                auto traj = StraightTrajectory::createAsWallCenter(dist, 0.0f, v, v, a, a);
                m.trajCommander.push(std::move(traj));
                pre_read_wall_coor.set(255, 255);

            }
            virtual void update() {
                UMouse& m = UMouse::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                FcLed& fcled = FcLed::getInstance();
                bool in_read_wall_area = m.inReadWallArea();
                ParameterManager& pm = ParameterManager::getInstance();
                if(m.running_sec > 25.0) v = pm.v_search_run * 0.95;
                else if(m.running_sec > 35.0) v = pm.v_search_run * 0.90;
                else if(m.running_sec > 45.0) v = pm.v_search_run * 0.85;

                if(in_read_wall_area) fcled.turn(0,1,0);
                else fcled.turn(0,0,1);
/*
                if(m.trajCommander.empty()) {
                    SE_I7();
                    waitmsec(200);
                    // 前壁を見る
                    m.maze.writeAheadWall(m.coor.x, m.coor.y, m.getDirection(), ws.isAhead());
                    headBudWall(0.3);
                    // 左壁を見る
                    auto traj0 = SpinTurnTrajectory::create(90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    m.trajCommander.push(std::move(traj0));
                    while(!m.trajCommander.empty()) {waitmsec(1);}
                    m.maze.writeAheadWall(m.coor.x, m.coor.y, m.getDirection(), ws.isAhead());
                    headBudWall(0.3);
                    // 後壁を見る
                    auto traj1 = SpinTurnTrajectory::create(90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    m.trajCommander.push(std::move(traj1));
                    while(!m.trajCommander.empty()) {waitmsec(1);}
                    m.maze.writeAheadWall(m.coor.x, m.coor.y, m.getDirection(), ws.isAhead());
                    headBudWall(0.3);
                    // 右壁を見る
                    auto traj2 = SpinTurnTrajectory::create(90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    m.trajCommander.push(std::move(traj2));
                    while(!m.trajCommander.empty()) {waitmsec(1);}
                    m.maze.writeAheadWall(m.coor.x, m.coor.y, m.getDirection(), ws.isAhead());
                    headBudWall(0.3);

                    // 最初の向きに戻る
                    auto traj3 = SpinTurnTrajectory::create(90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    m.trajCommander.push(std::move(traj3));
                    while(!m.trajCommander.empty()) {waitmsec(1);}

                    m.maze.makeSearchMap(desti_coor.x, desti_coor.y);
                    bool able_goal = m.maze.isExistPath(m.coor.x, m.coor.y);
                    if(!able_goal) {
                        for(int i=0; i<3; i++) {
                            SE_Im7();
                            waitmsec(500);
                        }
                        return; //以降の処理を行わず次のループへ
                    }

                    direction_e dest_dir_next = m.maze.getMinDirection(m.coor.x, m.coor.y, m.direction);
                    int8_t rot_times = m.maze.calcRotTimes(dest_dir_next, m.direction);
                    auto traj4 = SpinTurnTrajectory::create(rot_times * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj5 = StraightTrajectory::createAsWallCenter(0.045f, 0.0f, v, v, a, a);
                    m.trajCommander.push(std::move(traj4));
                    m.trajCommander.push(std::move(traj5));

                }
*/

                if(in_read_wall_area && pre_in_read_wall_area == false && pre_read_wall_coor != m.coor) {
                    printfAsync("＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝\n");
                    pre_read_wall_coor = m.coor;
                    uint8_t x_next = m.coor.x;
                    uint8_t y_next = m.coor.y;
                    if (m.direction == direction_e::E) x_next++;
                    else if (m.direction == direction_e::N) y_next++;
                    else if (m.direction == direction_e::W) x_next--;
                    else if (m.direction == direction_e::S) y_next--;

                    //printfAsync("msg_flag:%f,%f,壁|| 左 %d 前 (%d %d) 右 %d\n", m.posEsti.getX(), m.posEsti.getY(), ws.left(), ws.ahead_l(), ws.ahead_r(), ws.right());
                    
                    if (ws.isAhead()){
                        printfAsync("pre:%f, %f\n", m.posEsti.getX(), m.posEsti.getY());
                        m.posEsti.aheadWallCorrection(ws, x_next, y_next);
                        printfAsync("fol:%f, %f\n", m.posEsti.getX(), m.posEsti.getY());
                        SE_I7();
                    }

                    m.maze.updateWall(x_next, y_next, m.direction, ws);
                    m.maze.makeSearchMap(desti_coor.x, desti_coor.y);
                    bool able_goal = m.maze.isExistPath(x_next, y_next);
/*                    
                    if(!able_goal) {
                        auto traj0 = StraightTrajectory::createAsWallCenter(0.045f, v, v, 0.1, a, a);
                        m.trajCommander.push(std::move(traj0));

                        for(int i=0; i<3; i++) {
                            SE_Im7();
                            waitmsec(500);
                        }
                        return; //以降の処理を行わず次のループへ
                    }
*/
                    direction_e dest_dir_next = m.maze.getSearchDirection(x_next, y_next, m.direction);
                    int8_t rot_times = m.maze.calcRotTimes(dest_dir_next, m.direction);

                    printfAsync("XXXXX read wall. rot_times=%d\n",rot_times);
                    //printfAsync("msg_flag:%f,%f,★next(%d %d) now(%d %d) rt %d\n", m.posEsti.getX(), m.posEsti.getY(), x_next, y_next, m.coor.x, m.coor.y, rot_times);

                    if(x_next == desti_coor.x && y_next == desti_coor.y) {
                        destiMove();
                    } else {

                        if(rot_times == 0) {
                            auto traj0 = StraightTrajectory::createAsWallCenter(0.09f, v, v, v, a, a);
                            m.trajCommander.push(std::move(traj0));
                        } else if (ABS(rot_times) == 4) {
                            spin180(rot_times);
                        }

                        else if (ABS(m.posEsti.calcDiffAngle()) > 7.5f ||
                                 ABS(m.posEsti.calcWallCenterOffset()) > 0.01f ||
                                 mode == ESearchMode::SPIN_TURN_SERCH ||
                                 mode == ESearchMode::SPIN_TURN_SERCH_BOTHWAYS
                                ) {
                            spin90(rot_times);
                        }

                        else if(ABS(rot_times) == 2) {
                            slalom90(rot_times, x_next, y_next);
                        }
                    }
                }
                pre_in_read_wall_area = in_read_wall_area;
            }
            virtual bool isEnd() {
                UMouse& m = UMouse::getInstance();
                ParameterManager& pm = ParameterManager::getInstance();
                if(desti_coor.x == m.coor.x && desti_coor.y == m.coor.y) {
                    return true;
                } else if(getElapsedSec() > pm.search_limit_time_sec) {
                    printfAsync("search limit time up!\n");
                    m.maze.writeMazeData2Flash();
                    return true;
                }
                return false;
            }

            void slalom90(int8_t rot_times, uint8_t x_next, uint8_t y_next) {
                auto traj0 = StraightTrajectory::create(CurveFactory::getPreDistWithOffset(turn_type_e::TURN_90, v), v, v, v, a, a);
                auto traj1 = CurveTrajectory::createAsNoStraght(v, turn_type_e::TURN_90, (turn_dir_e)SIGN(rot_times));
                auto traj2 = StraightTrajectory::create(CurveFactory::getFolDist(turn_type_e::TURN_90), v, v, v, a, a);
                WallSensor &ws = WallSensor::getInstance();
                UMouse& m = UMouse::getInstance();
/*
                std::function< void(void) > on_start_func = [&m, &ws, &x_next, &y_next]() {
                    printfAsync("in on_start_func\n");
                    if (ws.isAhead()){
                        printfAsync("pre:%f, %f\n", m.posEsti.getX(), m.posEsti.getY());
                        m.posEsti.aheadWallCorrection(ws, x_next, y_next);
                        printfAsync("fol:%f, %f\n", m.posEsti.getX(), m.posEsti.getY());
                        SE_I7();
                    }
                };
                traj2->setOnStartFunc(on_start_func);
*/
                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));
                m.trajCommander.push(std::move(traj2));
            }

            void spin90(int8_t rot_times) {
                UMouse& m = UMouse::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                ParameterManager& pm = ParameterManager::getInstance();

                if (ws.isAhead() == true) {
                    std::function< void(void) > update_func = [&m, &ws]() {
                        if(m.maze.existAWall(m.coor.x, m.coor.y, m.direction)) {
                            if(m.posEsti.getV() > 0.05) {
                                PowerTransmission::getInstance().setDuty(0.35, 0.35);
                            } else {
                                PowerTransmission::getInstance().setDuty(0.45, 0.45);
                            }
                        }
                        m.posEsti.setX(m.trajCommander.x);
                        m.posEsti.setY(m.trajCommander.y);
                        m.ctrlMixer.reset();
                    };

                    auto traj0 = StraightTrajectory::create(0.035f, v, v, 0.1f, a, a);
                    auto traj1 = StraightTrajectory::create(0.01f, 0.1f, 0.1f, 0.1f, a, a);
                    auto traj2 = StopTrajectory::create(0.1f);
                    auto traj3 = UpdateInjectionTrajectory::create(0.35f, update_func);
                    auto traj4 = SpinTurnTrajectory::create(- rot_times * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj5 = UpdateInjectionTrajectory::create(0.35f, update_func);
                    auto traj6 = SpinTurnTrajectory::create(180.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj7 = StraightTrajectory::createAsWallCenter(0.045f, 0.0f, v, v, a, a);
                    m.trajCommander.push(std::move(traj0));
                    m.trajCommander.push(std::move(traj1));
                    m.trajCommander.push(std::move(traj2));
                    m.trajCommander.push(std::move(traj3));
                    m.trajCommander.push(std::move(traj4));
                    m.trajCommander.push(std::move(traj5));
                    m.trajCommander.push(std::move(traj6));
                    m.trajCommander.push(std::move(traj7));
                    SEG();
                } else {
                    auto traj0 = StraightTrajectory::createAsWallCenter(0.035f, v, v, 0.1f, a, a);
                    auto traj1 = StraightTrajectory::create(0.01f, 0.1f, 0.1f, 0.1f, a, a);
                    auto traj2 = StopTrajectory::create(0.2f);
                    auto traj3 = SpinTurnTrajectory::create(rot_times * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj4 = StraightTrajectory::createAsWallCenter(0.045f, 0.0f, v, v, a, a);
                    m.trajCommander.push(std::move(traj0));
                    m.trajCommander.push(std::move(traj1));
                    m.trajCommander.push(std::move(traj2));
                    m.trajCommander.push(std::move(traj3));
                    m.trajCommander.push(std::move(traj4));
                }

            }

            void headBudWall(float time) {
                UMouse& m = UMouse::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                std::function< void(void) > update_func = [&m, &ws]() {
                    if(m.maze.existAWall(m.coor.x, m.coor.y, m.direction)) {
                        if(m.posEsti.getV() > 0.05) {
                            PowerTransmission::getInstance().setDuty(0.35, 0.35);
                        } else {
                            PowerTransmission::getInstance().setDuty(0.45, 0.45);
                        }
                    }
                    m.posEsti.setX(m.trajCommander.x);
                    m.posEsti.setY(m.trajCommander.y);
                    //m.posEsti.setAng(m.trajCommander.ang);
                    m.ctrlMixer.reset();
                };
                auto traj = UpdateInjectionTrajectory::create(time, update_func);
                m.trajCommander.push(std::move(traj));
            }

            void spin180(int8_t rot_times) {
                UMouse& m = UMouse::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                ParameterManager& pm = ParameterManager::getInstance();
                rot_times *= SIGN(m.posEsti.calcWallCenterOffset());

                if (ws.isAhead()) {
                    std::function< void(void) > update_func = [&m, &ws]() {
                        if(m.maze.existAWall(m.coor.x, m.coor.y, m.direction)) {
                            if(m.posEsti.getV() > 0.05) {
                                PowerTransmission::getInstance().setDuty(0.35, 0.35);
                            } else {
                                PowerTransmission::getInstance().setDuty(0.45, 0.45);
                            }
                        }
                        m.posEsti.setX(m.trajCommander.x);
                        m.posEsti.setY(m.trajCommander.y);
                        m.ctrlMixer.reset();
                    };

                    std::function< void(void) > update_func2 = [&m, &ws]() {
                        if(m.maze.existAWall(m.coor.x, m.coor.y, m.direction)) {
                            PowerTransmission::getInstance().setDuty(0.6, 0.6);
                        }
                        m.posEsti.setX(m.trajCommander.x);
                        m.posEsti.setY(m.trajCommander.y);
                        //m.posEsti.setAng(m.trajCommander.ang);
                        m.ctrlMixer.reset();
                    };


                    auto traj0 = StraightTrajectory::create(0.035f, v, v, 0.1f, a, a);
                    auto traj1 = StraightTrajectory::create(0.01f, 0.1f, 0.1f, 0.1f, a, a);
                    auto traj2 = StopTrajectory::create(0.05f);
                    std::unique_ptr<BaseTrajectory> traj3;
                    if(m.ang_no_calib_sec < ANG_CALIB_TIME ) {
                        traj3 = UpdateInjectionTrajectory::create(0.35f, update_func);
                        SEG();
                    } else {
                        traj3 = UpdateInjectionTrajectory::create(1.0f, update_func2);
                        m.ang_no_calib_sec = 0.0f;
                        SE_Im7();
                    }
                    auto traj4 = SpinTurnTrajectory::create(rot_times/2 * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj5 = UpdateInjectionTrajectory::create(0.35f, update_func);
                    auto traj6 = SpinTurnTrajectory::create(rot_times/2 * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj7 = StraightTrajectory::createAsWallCenter(0.045f, 0.0f, v, v, a, a);
                    m.trajCommander.push(std::move(traj0));
                    m.trajCommander.push(std::move(traj1));
                    m.trajCommander.push(std::move(traj2));
                    m.trajCommander.push(std::move(traj3));
                    m.trajCommander.push(std::move(traj4));
                    m.trajCommander.push(std::move(traj5));
                    m.trajCommander.push(std::move(traj6));
                    m.trajCommander.push(std::move(traj7));

                    m.maze.writeMazeData2Flash();
                } else {
                    auto traj0 = StraightTrajectory::createAsWallCenter(0.035f, v, v, 0.1f, a, a);
                    auto traj1 = StraightTrajectory::create(0.01f, 0.1f, 0.1f, 0.1f, a, a);
                    auto traj2 = StopTrajectory::create(0.4f);
                    auto traj3 = SpinTurnTrajectory::create(rot_times/2 * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj4 = SpinTurnTrajectory::create(rot_times/2 * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj5 = StraightTrajectory::createAsWallCenter(0.045f, 0.0f, v, v, a, a);
                    m.trajCommander.push(std::move(traj0));
                    m.trajCommander.push(std::move(traj1));
                    m.trajCommander.push(std::move(traj2));
                    m.trajCommander.push(std::move(traj3));
                    m.trajCommander.push(std::move(traj4));
                    m.trajCommander.push(std::move(traj5));
                }



            }

            virtual void destiMove() {
                printfAsync("OOOOO Goal!\n");
                UMouse& m = UMouse::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                m.maze.writeMazeData2Flash();
                auto traj0 = StraightTrajectory::createAsWallCenter(0.045f, v, v, 0.1f, a, a);
                auto traj1 = StopTrajectory::create(0.5f);
                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));

                if (ws.isAhead() == true) {
                    std::function< void(void) > update_func = [&m, &ws]() {
                        if(m.posEsti.getV() > 0.05) {
                            PowerTransmission::getInstance().setDuty(0.35, 0.35);
                        } else {
                            PowerTransmission::getInstance().setDuty(0.45, 0.45);
                        }


                        m.posEsti.setX(m.trajCommander.x);
                        m.posEsti.setY(m.trajCommander.y);
                        m.ctrlMixer.reset();
                    };

                    auto traj2 = UpdateInjectionTrajectory::create(0.35f, update_func);
                    m.trajCommander.push(std::move(traj2));
                    SEG();
                }


                if(mode == ESearchMode::SLALOM_SERCH_BOTHWAYS || mode == ESearchMode::SPIN_TURN_SERCH_BOTHWAYS) {
                    Intent* intent = new Intent();
                    intent->uint8_t_param["SUB_MODE"] = (uint8_t)mode;

                    std::unique_ptr<BaseState> state = std::unique_ptr<BaseState>(new Goal2StartState(intent));
                    nextState = std::move(state);
                    delete intent;
                }

            }
        };

        class Goal2StartState : public Start2GoalState {
          public:
            Goal2StartState(Intent* intent_) : Start2GoalState(intent_) {

            }

            void onStart() {
                ICM20602& icm = ICM20602::getInstance();
                ParameterManager& pm = ParameterManager::getInstance();

                UMouse& m = UMouse::getInstance();
                desti_coor = m.start;
                a = pm.a_search_run;
                v = pm.v_search_run;
                pre_in_read_wall_area = false;

                direction_e dest_dir_next = m.maze.getSearchDirection(m.goal.x, m.goal.y, m.direction);
                int8_t rot_times = m.maze.calcRotTimes(dest_dir_next, m.direction);

                auto traj0 = SpinTurnTrajectory::create(rot_times * 45.0f, pm.spin_ang_v, pm.spin_ang_a);
                auto traj1 = StopTrajectory::create(1.0f);
                auto traj2 = StraightTrajectory::createAsWallCenter(0.09f/2.0f, 0.0f, v, v, a, a);
                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));
                m.trajCommander.push(std::move(traj2));
                pre_read_wall_coor.set(255, 255);

            }

            virtual void destiMove() {
                printfAsync("OOOOO return to start!\n");
                UMouse& m = UMouse::getInstance();
                WallSensor& ws = WallSensor::getInstance();
                m.maze.writeMazeData2Flash();
                auto traj0 = StraightTrajectory::createAsWallCenter(0.045f, v, v, 0.1f, a, a);
                auto traj1 = StopTrajectory::create(0.5f);
                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));

                if (ws.isAhead() == true) {
                    std::function< void(void) > update_func = [&m, &ws]() {
                        if(ws.getContactWallTime() > 0.1f) {
                            PowerTransmission::getInstance().setDuty(0.4, 0.4);
                        } else {
                            PowerTransmission::getInstance().setDuty(0.5, 0.5);
                        }
                        m.posEsti.setX(m.trajCommander.x);
                        m.posEsti.setY(m.trajCommander.y);
                        m.ctrlMixer.reset();
                    };

                    auto traj2 = UpdateInjectionTrajectory::create(0.35f, update_func);
                    m.trajCommander.push(std::move(traj2));
                    SEG();
                }

            }



        };
    };

};
