#include <chrono>
#include <iostream>
#include <memory>
#include <time.h>
#include "sendData2WebApp.h"
#include "maze.h"
#include "communication.h"
#include "pathCalculation.h"
#include "pathCompression.h"
#include "trajectoryCommander.h"
#include "turnParameter.h"
#include "mazeArchive.h"
#include <windows.h>

using namespace umouse;
#define MAZE_SET maze_archive::AllJapan2017Final_HF

void slalom90(TrajectoryCommander& trajCommander, int8_t rot_times, float v, float a) {    
    auto traj1 = CurveTrajectory::create(v, turn_type_e::TURN_90, (turn_dir_e)SIGN(rot_times));
    trajCommander.push(std::move(traj1));
}

void spin90(TrajectoryCommander& trajCommander, int8_t rot_times) {
    auto traj1 = SpinTurnTrajectory::create(rot_times * 45.0f, 3000.0, 3000.0);
    trajCommander.push(std::move(traj1));
}

void spin180(TrajectoryCommander& trajCommander, float v, float a) {
    auto traj0 = StraightTrajectory::create(0.045, 0, v, 0.0, a, a);
    auto traj1 = SpinTurnTrajectory::create(180.0f, 3000.0, 3000.0);
    auto traj2 = StraightTrajectory::create(0.045, 0, v, v, a, a);

    trajCommander.push(std::move(traj0));
    trajCommander.push(std::move(traj1));
    trajCommander.push(std::move(traj2));
}


void straight_one_block(TrajectoryCommander& trajCommander, float v) {
    auto traj1 = StraightTrajectory::create(0.09, v);
    trajCommander.push(std::move(traj1));
}

void straight_half_block(TrajectoryCommander& trajCommander, float v, float a) {
    auto traj1 = StraightTrajectory::create(0.045, 0.0, v, v, a, a);
    trajCommander.push(std::move(traj1));
}

direction_e getDirection(float ang) {
    if(ang >= 315.0f || ang <  45.0f) return direction_e::E;
    if(ang >=  45.0f && ang < 135.0f) return direction_e::N;
    if(ang >= 135.0f && ang < 225.0f) return direction_e::W;
    if(ang >= 225.0f && ang < 315.0f) return direction_e::S;
    return direction_e::E;
}

Wall readWall(uint16_t x, uint16_t y) {
    MAZE_SET md;
    Wall wall;
    //壁情報の配列番号に変換
    int8_t v_left = x-1;
    int8_t v_right = x;
    int8_t h_up = y;
    int8_t h_down = y-1;
    //壁番号が範囲外の場合は外周の壁
    wall.E = v_right != 31 ? ((md.walls_vertical[v_right] >>y )& 1) : 1;
    wall.N = h_up != 31 ? ((md.walls_horizontal[h_up] >>x )& 1) : 1;
    wall.W = v_left != -1 ? ((md.walls_vertical[v_left] >>y )& 1) : 1;
    wall.S = h_down != -1 ? ((md.walls_horizontal[h_down]>>x )& 1) : 1;
    /*
    wall.EF = isReached(x, y) || isReached(x+1, y );
    wall.NF = isReached(x, y) || isReached(x, y+1);
    wall.WF = isReached(x, y) || isReached(x-1, y );
    wall.SF = isReached(x, y) || isReached(x, y-1);
    */
    return wall;
};



int main(int argc, const char* argv[]) {
    srand((unsigned int)time(NULL));
    std::cout << "=========== 1st run ============" << std::endl;
    initWebAppConnection();
    sendReload();
    Sleep(4000);

    Maze maze;
    TrajectoryCommander trajCommander;
    trajCommander.reset(0.045f, 0.045f, 90.0f);
    //----------- 迷路に壁をセット
    MAZE_SET md;

    for(int i=0; i<32; i++) maze.reached[i]=0x00000000;

    sendMazeWall((uint32_t*)md.walls_vertical, (uint32_t*)md.walls_horizontal);
    Sleep(4000);
    maze.init();
    maze.writeWall(0, 0, readWall(0, 0));
    maze.writeReached(0, 0, true);


    long tick_count = 0;
    double real_time = 0.0;
    float v = 0.3;
    float a = 5.0;
    int coor_x = 0;
    int coor_y = 0;
    
    int coor_x_pre = 0;
    int coor_y_pre = 0;

    straight_half_block(trajCommander, v, a);

    while(1) {
        if(real_time> 360.0) break;
        if(real_time > 30.0 &&(coor_x == 0 && coor_y == 0)) break;
        trajCommander.update();
        real_time += 0.0005;
        if(tick_count % 60 == 0) {
            sendRobotPos(trajCommander.x, trajCommander.y, trajCommander.ang, trajCommander.v);
            int trajQueue_empty = trajCommander.empty();
            //printf("real time  %f\n", real_time);
            Sleep(5);
        }
        coor_x = (int)(trajCommander.x/0.09);
        coor_y = (int)(trajCommander.y/0.09);
        if(coor_x_pre != coor_x || coor_y_pre != coor_y) {
            sendNumberSqure(maze.p_map[coor_x][coor_y], (int)(trajCommander.x/0.09), (int)(trajCommander.y/0.09));
        }
        coor_x_pre = (int)(trajCommander.x/0.09);
        coor_y_pre = (int)(trajCommander.y/0.09);
        
        if(trajCommander.empty()) {
            uint8_t watch_x = (int)((trajCommander.x + 0.045 * cos(trajCommander.ang*3.14158265/180.0))/0.09);
            uint8_t watch_y = (int)((trajCommander.y + 0.045 * sin(trajCommander.ang*3.14158265/180.0))/0.09);            
            //printf("ang: %f\n",trajCommander.ang);
            direction_e watch_dir = getDirection(trajCommander.ang);
            maze.writeWall(watch_x, watch_y, readWall(watch_x, watch_y));
            maze.writeReached(watch_x, watch_y, true);
            sendMazeWall(maze.walls_vertical, maze.walls_horizontal);

            int8_t rot_times = 0;
            if(!maze.isReached(md.goal_x, md.goal_y)) {
                maze.makeSearchMap(md.goal_x, md.goal_y);
                rot_times = maze.calcRotTimes(maze.getSearchDirection2(watch_x, watch_y, watch_dir), watch_dir);
            }
            else{
                maze.makeSearchMap(0, 0);
                rot_times = maze.calcRotTimes(maze.getSearchDirection2(watch_x, watch_y, watch_dir), watch_dir);
            }

            
            
            if(rot_times == 0) {
                straight_one_block(trajCommander, v);
            }
            else if(rot_times == 4) {
                spin180(trajCommander, v, a);
            }
            else if(rot_times == 2 || rot_times == -2) {
                slalom90(trajCommander, rot_times, v, a);
            }
        }
        
        tick_count++;


    }
    printf("1st run end time:%f\n", real_time);
    
    
#ifdef HOGE
    
    std::cout << "=========== 2nd run ============" << std::endl;

    trajCommander.reset(0.045f, 0.045f, 90.0f);
    float goal_time = real_time;
    maze.writeReached(md.goal_x, md.goal_y,false);
    while(1) {    
        if(real_time - goal_time > 30.0 &&(coor_x == 0 && coor_y == 0)) break;
        trajCommander.update();
        real_time += 0.0005;
        if(tick_count % 60 == 0) {
            sendRobotPos(trajCommander.x, trajCommander.y, trajCommander.ang, trajCommander.v);
            int trajQueue_empty = trajCommander.empty();
            //printf("real time  %f\n", real_time);
            Sleep(5);
        }
        coor_x = (int)(trajCommander.x/0.09);
        coor_y = (int)(trajCommander.y/0.09);
        if(coor_x_pre != coor_x || coor_y_pre != coor_y) {
            sendNumberSqure(maze.p_map[coor_x][coor_y], (int)(trajCommander.x/0.09), (int)(trajCommander.y/0.09));
        }
        coor_x_pre = (int)(trajCommander.x/0.09);
        coor_y_pre = (int)(trajCommander.y/0.09);
        
        if(trajCommander.empty()) {
            uint8_t watch_x = (int)((trajCommander.x + 0.045 * cos(trajCommander.ang*3.14158265/180.0))/0.09);
            uint8_t watch_y = (int)((trajCommander.y + 0.045 * sin(trajCommander.ang*3.14158265/180.0))/0.09);            
            //printf("ang: %f\n",trajCommander.ang);
            direction_e watch_dir = getDirection(trajCommander.ang);
            maze.writeWall(watch_x, watch_y, readWall(watch_x, watch_y));
            maze.writeReached(watch_x, watch_y, true);
            sendMazeWall(maze.walls_vertical, maze.walls_horizontal);

            if(!maze.isReached(md.goal_x, md.goal_y))maze.makeSearchMap(md.goal_x, md.goal_y);
            else maze.makeSearchMap(0, 0);
            int8_t rot_times = maze.calcRotTimes(maze.getSearchDirection(watch_x, watch_y, watch_dir), watch_dir);

            if(rot_times == 0) {
                straight_one_block(trajCommander, v);
            }
            else if(rot_times == 4) {
                spin180(trajCommander, v, a);
            }
            else if(rot_times == 2 || rot_times == -2) {
                slalom90(trajCommander, rot_times, v, a);
            }
        }
        
        tick_count++;


    }
    printf("2nd run end time:%f\n", real_time);
    sendMazeWall((uint32_t*)md.walls_vertical, (uint32_t*)md.walls_horizontal);
#endif
    std::cout << "=========== 3rd run ============" << std::endl;
    //----------- パスを計算
    std::vector<Path> path_vec;
    trajCommander.clear();
    trajCommander.reset(0.045f, 0.045f - 0.01765, 90.0f);
    TurnParameter turn_p;
    turn_p.set(3.5, 3.5, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 10.0, 10.0);
    makeFastestDiagonalPath(1000, turn_p, md.goal_x, md.goal_y, maze, path_vec);
    float necessary_time = HF_calcPlayPathTime(turn_p, path_vec);
    printf("3rd run estimation time:%f\n", necessary_time);    
    
    //printf("empty %d\n", trajCommander.empty());
    HF_playPath(turn_p, path_vec, trajCommander);

    tick_count = 0;
    real_time = 0.0;
    coor_x_pre = 0;
    coor_y_pre = 0;
    
    real_time = 0.0;
    
    sendReload();    
    Sleep(5000);
    sendMazeWall((uint32_t*)md.walls_vertical, (uint32_t*)md.walls_horizontal);

    while(trajCommander.empty() == false) {
        trajCommander.update();
        real_time += 0.0005;
        if(tick_count % 30 == 0) {
            sendRobotPos(trajCommander.x, trajCommander.y, trajCommander.ang, trajCommander.v);
            //sendTargetPos(trajCommander.x, trajCommander.y, trajCommander.ang);
            Sleep(5);
        }
        if(coor_x_pre != (int)(trajCommander.x/0.09) || coor_y_pre != (int)(trajCommander.y/0.09)) {
            sendNumberSqure(1, (int)(trajCommander.x/0.09), (int)(trajCommander.y/0.09));
        }
        coor_x_pre = (int)(trajCommander.x/0.09);
        coor_y_pre = (int)(trajCommander.y/0.09);
        tick_count++;

    }
    printf("3rd run real time:%f\n", real_time);


    std::cout << "=========== 4th run <- all wall known =====" << std::endl;
    Sleep(3000);
    for(int i=0; i<31; i++) {
        maze.walls_vertical[i] = md.walls_vertical[i];
        maze.walls_horizontal[i] = md.walls_horizontal[i];
    }
    for(int i=0; i<32; i++) maze.reached[i]=0xFFFFFFFF;

   //----------- パスを計算
    trajCommander.clear();
    trajCommander.reset(0.045f, 0.045f - 0.01765, 90.0f);
    //TurnParameter turn_p;
    //turn_p.set(3.5, 3.5, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 10.0, 10.0);
    makeFastestDiagonalPath(1000, turn_p, md.goal_x, md.goal_y, maze, path_vec);
    necessary_time = HF_calcPlayPathTime(turn_p, path_vec);
    printf("4th run estimation time:%f\n", necessary_time);    
    
    HF_playPath(turn_p, path_vec, trajCommander);

    tick_count = 0;
    real_time = 0.0;
    coor_x_pre = 0;
    coor_y_pre = 0;
    
    real_time = 0.0;
    
    sendNumberSqure(0, (int)(trajCommander.x/0.09), (int)(trajCommander.y/0.09));

    sendMazeWall((uint32_t*)md.walls_vertical, (uint32_t*)md.walls_horizontal);

    while(trajCommander.empty() == false) {
        std::chrono::system_clock::time_point  start, end; // 型は auto で可
        start = std::chrono::system_clock::now(); // 計測開始時間
        trajCommander.update();
        real_time += 0.0005;
        if(tick_count % 30 == 0) {
            sendRobotPos(trajCommander.x, trajCommander.y, trajCommander.ang, trajCommander.v);
            //sendTargetPos(trajCommander.x, trajCommander.y, trajCommander.ang);
            Sleep(5);
        }
        if(coor_x_pre != (int)(trajCommander.x/0.09) || coor_y_pre != (int)(trajCommander.y/0.09)) {
            sendNumberSqure(1, (int)(trajCommander.x/0.09), (int)(trajCommander.y/0.09));
        }
        coor_x_pre = (int)(trajCommander.x/0.09);
        coor_y_pre = (int)(trajCommander.y/0.09);
        tick_count++;

    }
    printf("4th run real time:%f\n", real_time);




    finalizeWebAppConnection();
    return 0;
}
