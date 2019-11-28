#pragma once

#include "stdint.h"
#include "maze.h"
#include "path.h"
#include "turnParameter.h"
#include "trajectoryCommander.h"
#include <vector>

namespace umouse {

    void setEntryArea(uint16_t start_x, uint16_t start_y, uint16_t goal_x, uint16_t goal_y, Maze& maze);
    void makeMinStepPath(uint16_t goal_x, uint16_t goal_y, Maze& maze, std::vector<Path>& path_vec);
    void makeQuasiMinStepPath(uint16_t goal_x, uint16_t goal_y, Maze& maze, std::vector<Path>& path_vec);
    void makeFastestDiagonalPath(uint32_t trial_times, TurnParameter turn_p, uint16_t goal_x, uint16_t goal_y, Maze& maze, std::vector<Path>& path_vec);


    void translatePathSpin(std::vector<Path>& path_vec);
    void translatePath90Deg(std::vector<Path>& path_vec);
    void translatePathLong(std::vector<Path>& path_vec);
    void translatePathDiagonal(std::vector<Path>& path_vec);
    void HF_playPath(TurnParameter turn_p, std::vector<Path>& path_vec, TrajectoryCommander& trajCommander);
    float HF_calcPlayPathTime(TurnParameter turn_p, std::vector<Path>& path_vec);

    void HF_playPathSpin(TurnParameter turn_p, std::vector<Path>& path_vec, TrajectoryCommander& trajCommander);
    void HF_playPathSpinDiagonal(TurnParameter turn_p, std::vector<Path>& path_vec, TrajectoryCommander& trajCommander);
    void printPath(std::vector<Path>& path_vec);

} // namespace umouse
