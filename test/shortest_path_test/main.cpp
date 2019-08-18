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

using namespace umouse;

int main(int argc, const char *argv[])
{
    std::cout << "test" << std::endl;
    initWebAppConnection();

    Maze maze;
    TrajectoryCommander trajCommander;
    
    //----------- 迷路に壁をセット
        constexpr uint32_t walls_vertical[31] = {0x87ffff9d,0x33ffffc0,0x79f94460,0x3efe12d6,0x1ff7e429,0x70fe6954,0x000396a8,0x3f07094c,0x01bbf6b0,0x00165128,0x181f2954,0x00021628,0x0084eb20,0x02005458,0x0036a82c,0x41e15754,0x3f4aad58,0xfc8592ac,0x7e8af750,0xf757c0a0,0xa8abb060,0xa84f7ff0,0x54b03f80,0x55481fc8,0xaa8724f4,0xadf8ff78,0x5a0076c6,0x5400eda2,0xa8007bf2,0x5000bc02,0x20017ffc};
    constexpr uint32_t walls_horizontal[31] = {0x87ffffcc,0x35ffffee,0x7af82a10,0x3d7d1560,0x4992e8de,0x284d1aa1,0x4002f750,0x263dfadc,0x41020534,0x30002ae4,0x003ed558,0x0a0aaba4,0x01115458,0x0086b034,0x41e15dc8,0x3f42ae58,0xfc857000,0x7e8a200c,0xf755bbc0,0xa8abf030,0xa8573c00,0x549e7fc0,0x5560cb80,0xaa907cc0,0xad0ebe68,0x5bf1dfc4,0x5400fea6,0xa801f382,0x5000fea2,0x20017f92,0x4002fffc};

    
    
    for(int i=0;i<31;i++){
        maze.walls_vertical[i] = walls_vertical[i];        
        maze.walls_horizontal[i] = walls_horizontal[i];
    }

    for(int i=0;i<32;i++) maze.reached[i]=0xFFFFFFFF;

    sendMazeWall(maze.walls_vertical, maze.walls_horizontal);
    //----------- パスを計算
    std::vector<Path> path_vec;
    TurnParameter turn_p;
    //turn_p.set(3.0, 0.5, 8.0);
    turn_p.set(3.0, 1.5, 0.7, 0.7, 0.8, 0.8, 0.8, 0.8, 15.0, 3.0);
    makeMinStepPath(2, 3, maze, path_vec); //(3,2もおかしい)


    //translatePathSpin(path_vec);
    //translatePath90Deg(path_vec);
    //translatePathLong(path_vec);
    translatePathDiagonal(path_vec);
    printPath(path_vec);
    HF_playPath(turn_p, path_vec, trajCommander);
    //HF_playPathSpin(turn_p, path_vec, trajCommander);
    
    long tick_count = 0;
    while(trajCommander.empty() == false){
        std::chrono::system_clock::time_point  start, end; // 型は auto で可
        start = std::chrono::system_clock::now(); // 計測開始時間
        trajCommander.update();

        if(tick_count % 10 == 0){
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
    finalizeWebAppConnection();
    return 0;
}
