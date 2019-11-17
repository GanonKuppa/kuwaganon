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

int main(int argc, const char *argv[])
{
    srand((unsigned int)time(NULL));
    std::cout << "test" << std::endl;
    initWebAppConnection();        
    //sendReload();
    //Sleep(5000);
    
    Maze maze;
    TrajectoryCommander trajCommander;
    
    //----------- 迷路に壁をセット
    maze_archive::AllJapan2018Final_HF md;
    
    for(int i=0;i<31;i++){
        maze.walls_vertical[i] = md.walls_vertical[i];        
        maze.walls_horizontal[i] = md.walls_horizontal[i];
    }

    for(int i=0;i<32;i++) maze.reached[i]=0xFFFFFFFF;

    sendMazeWall(maze.walls_vertical, maze.walls_horizontal);
    //----------- パスを計算
    std::vector<Path> path_vec;
    
    TurnParameter turn_p;
    //turn_p.set(1.0, 1.0, 15.0);
    turn_p.set(5.0, 4.0, 1.36, 1.55, 1.3, 1.32, 1.3, 1.3, 17.0, 12.0);
    //makeMinStepPath(md.goal_x, md.goal_y, maze, path_vec);
    makeFastestDiagonalPath(1000, turn_p, md.goal_x, md.goal_y, maze, path_vec);
    //makeQuasiMinStepPath(2, 5, maze, path_vec);
 
    //translatePathSpin(path_vec);
    //translatePath90Deg(path_vec);
    //translatePathLong(path_vec);
    //translatePathDiagonal(path_vec);
    //printPath(path_vec);    
    float necessary_time = HF_calcPlayPathTime(turn_p, path_vec);
    printf("necessary_time:%f\n", necessary_time);
    HF_playPath(turn_p, path_vec, trajCommander);
    //HF_playPathSpin(turn_p, path_vec, trajCommander);
    //HF_playPathSpinDiagonal(turn_p, path_vec, trajCommander);
    
    long tick_count = 0;
    double real_time = 0.0;
    int coor_x_pre = 0;
    int coor_y_pre = 0;

    while(trajCommander.empty() == false){
        std::chrono::system_clock::time_point  start, end; // 型は auto で可
        start = std::chrono::system_clock::now(); // 計測開始時間
        trajCommander.update();
        real_time += 0.0005;
        if(tick_count % 30 == 0){
            sendRobotPos(trajCommander.x, trajCommander.y, trajCommander.ang, trajCommander.v);
            //sendTargetPos(trajCommander.x, trajCommander.y, trajCommander.ang);                
        }
        if(coor_x_pre != (int)(trajCommander.x/0.09) || coor_y_pre != (int)(trajCommander.y/0.09)){
            sendNumberSqure(1, (int)(trajCommander.x/0.09), (int)(trajCommander.y/0.09));
        }
        coor_x_pre = (int)(trajCommander.x/0.09);
        coor_y_pre = (int)(trajCommander.y/0.09);
        tick_count++;
        /*
        while(1){
            end = std::chrono::system_clock::now();  // 計測終了時間
            double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count(); //処理に要した時間をミリ秒に変換
            if(elapsed > 0.5) break;
        }
        */
        
    }
    printf("real time:%f\n", real_time);
    finalizeWebAppConnection();
    return 0;
}
