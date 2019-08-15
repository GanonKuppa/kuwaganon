#include <iostream>
#include <memory>
#include <time.h>
#include "sendData2WebApp.h"
#include "maze.h"
#include "communication.h"
#include "pathCalculation.h"


using namespace umouse;

int main(int argc, const char *argv[])
{
    std::cout << "test" << std::endl;
    initWebAppConnection();

    sendRobotPos(1.0, 1.0, 0.0);

    Maze maze;
    
    //----------- 迷路に壁をセット
    uint32_t walls_vertical[31] = {0x000000ff,0x00000000,0xffefffff,0x00000000,0xbfffffff,0x00000000,0x01000000,0xf7ffffff,0x00000000,0x00000000,0xdbbffefb,0x00400000,0x00000000,0xbebfffef,0x00000000,0xbbbffeef,0x00000000,0x00048000,0xbbfffff7,0x00000220,0x00800000,0x00000000,0xdfeeffef,0x00040000,0x04000000,0xffeffff7,0x00000008,0xbdeedfff,0x00000000,0x00000000,0x00000000};
    uint32_t walls_horizontal[31] = {0x00000000,0x00000000,0x00000020,0x00800000,0x00080000,0x00400020,0x7eddfdfa,0x00080000,0x00000250,0x00000000,0xbefdfdfe,0x00028000,0x00020000,0x00020000,0x00000000,0xfeffedfe,0x00000000,0x01000000,0xfbddedbe,0x00000020,0x00000000,0x00000000,0x00200000,0xdffdffad,0x00000040,0x00000000,0x01000000,0xfb777f6d,0x00000000,0x00000000,0x00000000};

    for(int i=0;i<31;i++){
        maze.walls_vertical[i] = walls_vertical[i];        
        maze.walls_horizontal[i] = walls_horizontal[i];
    }

    for(int i=0;i<32;i++) maze.reached[i]=0xFFFFFFFF;

    sendMazeWall(maze.walls_vertical, maze.walls_horizontal);
    //----------- パスを計算
    std::vector<Path> path_vec;
    makeMinStepPath(31, 31, maze, path_vec);
    printfAsync("--- makeMinStepPath ----\n");
    printPath(path_vec);
    printfAsync("--- jointStraightPath ----\n");
    jointStraightPath(path_vec);

    //if(mode == 1 ) HF_playPathPivot(turn_p, path_vec);
    //HF_playPath(turn_p, path_vec);



    finalizeWebAppConnection();
    return 0;
}
