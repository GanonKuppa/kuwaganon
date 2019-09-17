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

using namespace umouse;

int main(int argc, const char *argv[])
{
    std::cout << "test" << std::endl;
    initWebAppConnection();

    Maze maze;
    TrajectoryCommander trajCommander;
    
    //----------- 迷路に壁をセット
    maze_archive::Hokushinetsu2019_HF md;
    
    for(int i=0;i<31;i++){
        maze.walls_vertical[i] = md.walls_vertical[i];        
        maze.walls_horizontal[i] = md.walls_horizontal[i];
    }

    for(int i=0;i<32;i++) maze.reached[i]=0xFFFFFFFF;

    sendMazeWall(maze.walls_vertical, maze.walls_horizontal);
    //----------- パスを計算
    std::vector<Path> path_vec;
    TurnParameter turn_p;
    //turn_p.set(3.0, 0.7, 15.0);
    turn_p.set(3.0, 1.5, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 10.0, 10.0);
    makeMinStepPath(md.goal_x, md.goal_y, maze, path_vec); //(3,2もおかしい)


    //translatePathSpin(path_vec);
    //translatePath90Deg(path_vec);
    //translatePathLong(path_vec);
    translatePathDiagonal(path_vec);
    printPath(path_vec);
    HF_playPath(turn_p, path_vec, trajCommander);
    //HF_playPathSpin(turn_p, path_vec, trajCommander);
    //HF_playPathSpinDiagonal(turn_p, path_vec, trajCommander);
    
    long tick_count = 0;
    double real_time = 0.0;
    while(trajCommander.empty() == false){
        std::chrono::system_clock::time_point  start, end; // 型は auto で可
        start = std::chrono::system_clock::now(); // 計測開始時間
        trajCommander.update();
        real_time += 0.0005;
        if(tick_count % 30 == 0){
            sendRobotPos(trajCommander.x, trajCommander.y, trajCommander.ang, trajCommander.v);
            //sendTargetPos(trajCommander.x, trajCommander.y, trajCommander.ang);                
        }
        

        tick_count++;
        
        while(1){
            end = std::chrono::system_clock::now();  // 計測終了時間
            double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count(); //処理に要した時間をミリ秒に変換
            if(elapsed > 1.0) break;
        }
        
    }
    printf("real time:%f\n", real_time);
    finalizeWebAppConnection();
    return 0;
}
