#pragma once

#include <myUtil.h>
#include "stdint.h"
#include "maze.h"
#include <map> // pair
#include <vector>
#include "communication.h"
#include "timer.h"
#include "sound.h"
#include "curveFactory.h"
#include "trajectory.h"
#include "parameterManager.h"
#include "path.h"
#include "turnParameter.h"

namespace umouse
{

inline void makeMinStepPath(uint16_t goal_x, uint16_t goal_y, Maze &maze, std::vector<Path> &path_vec)
{

    enum direction_e dir = N;
    uint16_t p_x = 0;
    uint16_t p_y = 0;

    maze.makeFastestMap(goal_x, goal_y);
    path_vec.push_back(Path(turn_type_e::STRAIGHT, 1, turn_dir_e::NO_TURN));
    p_y++;

    while ((p_x != goal_x) || (p_y != goal_y))
    {
        //waitmsec(3000);
        printfAsync("%d %d\n", p_x, p_y);
        direction_e min_dir = maze.getMinDirection(p_x, p_y, dir);
        if (min_dir == dir)
        {
            path_vec.push_back(Path(turn_type_e::STRAIGHT, 1, turn_dir_e::NO_TURN));
            path_vec.push_back(Path(turn_type_e::STRAIGHT, 1, turn_dir_e::NO_TURN));
        }
        else
        {
            int8_t dir_diff = (int8_t)min_dir - (int8_t)dir;
            if (dir_diff == 6)
                dir_diff = -2;
            if (dir_diff == -6)
                dir_diff = 2;
            turn_dir_e turn_dir = (turn_dir_e)(SIGN(dir_diff));
            path_vec.push_back(Path(turn_type_e::TURN_90, 1, turn_dir));
        }

        dir = min_dir;
        if (dir == direction_e::E)
            p_x++;
        else if (dir == direction_e::N)
            p_y++;
        else if (dir == direction_e::W)
            p_x--;
        else if (dir == direction_e::S)
            p_y--;
    }

    path_vec.push_back(Path(turn_type_e::STRAIGHT, 1, turn_dir_e::NO_TURN));
}

inline void jointStraightPath(std::vector<Path> &path_vec)
{
    for (uint16_t i = 0; i < (uint16_t)path_vec.size(); i++)
    {
        printfAsync("-------%d turntype %d \n", i, path_vec[i].turn_type);
        //waitmsec(3000);
        if (i == (uint16_t)path_vec.size())
        {
            break;
        }
        else if (path_vec[i].turn_type == turn_type_e::STRAIGHT)
        {
            if (i + 1 == (uint16_t)path_vec.size())
                break;
            while (path_vec[i + 1].turn_type == turn_type_e::STRAIGHT)
            {
                path_vec.erase(path_vec.begin() + i + 1);
                printfAsync("          -- i=%d -- size=%d --block_num=%d \n", i, path_vec.size(), path_vec[i].block_num);
                path_vec[i].block_num++;
                if (i + 1 == (uint16_t)path_vec.size())
                    break;
            }
        }
    }
}

inline void translatePath2NoDiagPath(std::vector<Path> &path_vec)
{
    // TURN_L_90
    for (int i=1;i< path_vec.size()-2; i++){
        if(path_vec[i].turn_type   == turn_type_e::STRAIGHT  &&
           path_vec[i+1].turn_type == turn_type_e::TURN_90   &&
           path_vec[i+2].turn_type == turn_type_e::STRAIGHT)
        {
               path_vec[i].turn_type = turn_type_e::TURN_L_90;
               path_vec[i].turn_dir = path_vec[i+1].turn_dir;
               path_vec[i+1].erase();
               path_vec[i+1].erase();
        }
    }

    // TURN_180
    for (int i=1;i< path_vec.size()-3; i++){
        if(path_vec[i].turn_type   == turn_type_e::STRAIGHT &&
           path_vec[i+1].turn_type == turn_type_e::TURN_90  &&
           path_vec[i+2].turn_type == turn_type_e::TURN_90  &&
           path_vec[i+3].turn_type == turn_type_e::STRAIGHT &&
           path_vec[i+1].turn_dir != path_vec[i+1].turn_dir)
        {
               path_vec[i].turn_type = turn_type_e::TURN_180;
               path_vec[i].turn_dir = path_vec[i+1].turn_dir;
               path_vec[i+1].erase();
               path_vec[i+1].erase();
               path_vec[i+1].erase();
        }
    }
 
}

inline void translatePath2DiagPath()
{
}

inline void HF_playPath2(TurnParameter turn_p, std::vector<Path> &path_vec)
{
}

inline void HF_playPath(TurnParameter turn_p, std::vector<Path> &path_vec)
{
    UMouse &m = UMouse::getInstance();
    float v_pre = 0.0;
    float v_fol = 0.0;
    bool is_start_section = true;

    float x, v, a, v_max;
    turn_dir_e dir;
    for (uint16_t i = 0; i < (uint16_t)path_vec.size(); i++)
    {
        //v_pre = v_fol;
        switch (path_vec[i].turn_type)
        {
        case turn_type_e::STRAIGHT:
            x = float(path_vec[i].block_num) * 0.045;
            if (is_start_section == true)
                x += m.WALL2MOUSE_CENTER_DIST;
            is_start_section = false;

            a = turn_p.a_straight;
            v_max = turn_p.getTurnV(turn_type_e::STRAIGHT);

            if (i == 0)
                v_pre = 0.0;
            else
                v_pre = turn_p.v_turn_90;

            if (i + 1 == (uint16_t)path_vec.size())
                v_fol = 0.1;
            else
                v_fol = turn_p.v_turn_90;

            {
                auto traj_straight0 = StraightTrajectory::createAsWallCenter(x - 0.045, v_pre, v_max, v_fol, a, a);
                auto traj_straight1 = StraightTrajectory::createAsWallCenter(0.045, v_fol, v_fol, v_fol, a, a);
                m.trajCommander.push(std::move(traj_straight0));
                m.trajCommander.push(std::move(traj_straight1));
            }
            break;
        case turn_type_e::TURN_90:
            v = turn_p.v_turn_90;
            dir = path_vec[i].turn_dir;
            {
                auto traj0 = StraightTrajectory::create(CurveFactory::getPreDist(turn_type_e::TURN_90), v, v, v, a, a);
                auto traj1 = CurveTrajectory::createAsNoStraght(v, turn_type_e::TURN_90, dir);
                auto traj2 = StraightTrajectory::create(CurveFactory::getFolDist(turn_type_e::TURN_90), v, v, v, a, a);

                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));
                m.trajCommander.push(std::move(traj2));
            }
            break;
        }
    }
    auto traj_stop = StopTrajectory::create(1.0);
    m.trajCommander.push(std::move(traj_stop));
}

inline void HF_playPathPivot(TurnParameter turn_p, std::vector<Path> &path_vec)
{
    UMouse &m = UMouse::getInstance();
    float v_pre = 0.0;
    float v_fol = 0.0;
    bool is_start_section = true;
    ParameterManager &pm = ParameterManager::getInstance();

    float x, v, a, v_max, dir;
    for (uint16_t i = 0; i < (uint16_t)path_vec.size(); i++)
    {
        v_pre = v_fol;
        switch (path_vec[i].turn_type)
        {
        case turn_type_e::STRAIGHT:
            x = float(path_vec[i].block_num) * 0.045;
            if (is_start_section == true)
                x += m.WALL2MOUSE_CENTER_DIST;

            is_start_section = false;
            a = turn_p.a_straight;
            v_max = turn_p.getTurnV(turn_type_e::STRAIGHT);

            if (i + 1 == (uint16_t)path_vec.size())
                v_fol = 0.1;
            else
                v_fol = turn_p.getTurnV((turn_type_e)path_vec[i + 1].turn_type);

            {
                auto traj_straight = StraightTrajectory::createAsWallCenter(x, v_pre, v_max, v_fol, a, a);
                m.trajCommander.push(std::move(traj_straight));
            }
            break;
        case turn_type_e::TURN_90:
            v = turn_p.v_turn_90;
            a = turn_p.a_straight;
            dir = SIGN((float)path_vec[i].turn_dir);
            {
                auto traj0 = StraightTrajectory::createAsWallCenter(0.03, v, v, 0.1, a, a);
                auto traj1 = StraightTrajectory::createAsWallCenter(0.015, 0.1, 0.1, 0.0, a, a);

                auto traj2 = StopTrajectory::create(0.05);
                auto traj3 = SpinTurnTrajectory::create(dir * 90.0f, pm.spin_ang_v, pm.spin_ang_a);
                auto traj4 = StopTrajectory::create(0.2);
                auto traj5 = StraightTrajectory::createAsWallCenter(0.09 / 2.0, 0.0, v, v, a, a);

                m.trajCommander.push(std::move(traj0));
                m.trajCommander.push(std::move(traj1));
                m.trajCommander.push(std::move(traj2));
                m.trajCommander.push(std::move(traj3));
                m.trajCommander.push(std::move(traj4));
                m.trajCommander.push(std::move(traj5));
            }
            break;
        }
    }
    auto traj_stop = StopTrajectory::create(3.0);
    m.trajCommander.push(std::move(traj_stop));
}

inline void printPath(std::vector<Path> &path_vec)
{
    printfAsync("==== path_vec ====\n");
    for (auto &path : path_vec)
    {
        path.print();
    }
    printfAsync("==================\n");
}

} // namespace umouse
