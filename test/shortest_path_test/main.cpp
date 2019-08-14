

#include <iostream>
#include <fstream>

#include <chrono>
#include "picojson.h"
#include <windows.h>
#include "udp.h"
#include "trajectory.h"
#include <memory>
#include <random>
#include "positionEstimator.h"
#include "controlMixer.h"
#include <time.h>
#include "trajectoryCommander.h"

using namespace umouse ;

void sendRobotPos(float x, float y, float ang){
    picojson::object obj;

    // データの追加
    obj.emplace(std::make_pair("type", "robot_pos"));
    obj.emplace(std::make_pair("x", x));
    obj.emplace(std::make_pair("y", y));
    obj.emplace(std::make_pair("ang", ang));
    // 文字列にするためにvalueを使用
    picojson::value val(obj);
    // return std::string
    val.serialize();    
    sendUdpString( val.serialize());
}

void sendTargetPos(float x, float y, float ang){
    picojson::object obj;

    // データの追加
    obj.emplace(std::make_pair("type", "target_pos"));
    obj.emplace(std::make_pair("x", x));
    obj.emplace(std::make_pair("y", y));
    obj.emplace(std::make_pair("ang", ang));
    // 文字列にするためにvalueを使用
    picojson::value val(obj);
    // return std::string
    val.serialize();    
    sendUdpString( val.serialize());
}


int main(int argc, const char *argv[])
{
   
    uint64_t round = 0;
    float min_error_ave = 10000.0f;
float min_P = 0.0;
float min_I = 0.0;
float min_D = 0.0;
float error_ave = 0.0f;
float P = 135.0;
float I = 0.0013;
float D = 10.0;


    initUdpClient("127.0.0.1", 2020);
while(1) {
//P+= 0.01;
//D+= 0.01;
//I+= 0.0001;

float error_ave = 0.0f;
for(int i=0;i<10;i++){    
    float x = 0.09/2;
    float y = 0.09/2;
    float ang = 90.0;
    
    TrajectoryCommander& trajCommander = TrajectoryCommander::getInstance();
    std::cout<< trajCommander.empty() << " ?emp" << std::endl;
    trajCommander.clear();
    float dummy = 0.0;
    auto traj0 = StraightTrajectory::create(0.09 * 6.5, x, y, ang, 0.0, 1.0, 0.3, 3.0, 4.0);
    auto traj1 = CurveTrajectory::create(dummy, dummy, dummy, 0.3, turn_type_e::TURN_90, turn_dir_e::CW);
    auto traj2 = StraightTrajectory::create(0.09 * 6.0, dummy, dummy, dummy, 0.3, 1.0, 0.3, 3.0, 4.0);    
    auto traj3 = CurveTrajectory::create(dummy, dummy, dummy, 0.3, turn_type_e::TURN_90, turn_dir_e::CW);
    auto traj4 = CurveTrajectory::create(dummy, dummy, dummy, 0.3, turn_type_e::TURN_90, turn_dir_e::CCW);
    auto traj5 = CurveTrajectory::create(dummy, dummy, dummy, 0.3, turn_type_e::TURN_90, turn_dir_e::CCW);
    auto traj6 = CurveTrajectory::create(dummy, dummy, dummy, 0.3, turn_type_e::TURN_90, turn_dir_e::CW);
    auto traj7 = CurveTrajectory::create(dummy, dummy, dummy, 0.3, turn_type_e::TURN_90, turn_dir_e::CW);
    auto traj8 = StraightTrajectory::create(0.09 * 6.5, dummy, dummy, dummy, 0.3, 1.0, 0.0, 5.0, 5.0);

    trajCommander.push(std::move(traj0));
    trajCommander.push(std::move(traj1));
    trajCommander.push(std::move(traj2));
    trajCommander.push(std::move(traj3));
    trajCommander.push(std::move(traj4));
    trajCommander.push(std::move(traj5));
    trajCommander.push(std::move(traj6));
    trajCommander.push(std::move(traj7));
    trajCommander.push(std::move(traj8));
    
//    auto traj = CompositeTrajectory::create(std::move(traj0), std::move(traj1), std::move(traj2));
    
    PositionEstimator esti = PositionEstimator(x, y ,ang);
    ControlMixer mixer = ControlMixer(P,I,D);
  

    long tick_count = 0;
    SYSTEMTIME st;
    GetLocalTime(&st);
    std::default_random_engine gen(st.wMilliseconds);
    std::normal_distribution<float> n_dist_v(0.0,0.5);
    std::normal_distribution<float> n_dist_ang_v(0.0,500);
    std::normal_distribution<float> n_dist_ang(0.0,1.5);
    //std::normal_distribution<float> n_dist_x(0.0,500);
    //std::normal_distribution<float> n_dist_y(0.0,500);
    float r_x_sum = 0.0f;
    float r_y_sum = 0.0f;
    float r_ang_sum = 0.0f;
  
    float robot_v;
    float robot_ang_v;

    float error = 0.0f;
    

    while(1){
        std::chrono::system_clock::time_point  start, end; // 型は auto で可
        start = std::chrono::system_clock::now(); // 計測開始時間
        
        trajCommander.update();
        
        if(trajCommander.empty() == true){
            std::cout <<"break" << std::endl;
            break;
        }
        
        auto traj = trajCommander.getTraj();


        //robot_v = traj->v + n_dist_v(gen);
        //if(tick_count%10 == 0)robot_ang_v = traj->ang_v + n_dist_ang_v(gen);
        //traj->update();        
        
        mixer.update(traj, esti);
        robot_ang_v = mixer.target_rot_v;
        robot_v = mixer.target_trans_v;
        esti.update(robot_ang_v, robot_v);
        
        if(tick_count %200 == 0 && traj.v > 0.2) esti.ang += n_dist_ang(gen);
        
        if(tick_count % 5 == 0){
            sendTargetPos(traj.x, traj.y, traj.ang);
            sendRobotPos(esti.x, esti.y, esti.ang);
        }
        
        error += sqrt((traj.x - esti.x) * (traj.x - esti.x) + (traj.y - esti.y) * (traj.y - esti.y)) ;
        tick_count++;
        while(1){
            end = std::chrono::system_clock::now();  // 計測終了時間
            double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count(); //処理に要した時間をミリ秒に変換
            if(elapsed > 500.0) break;
        }
        
    }
    error_ave += error;
    
}

if(min_error_ave > error_ave){
    min_error_ave = error_ave;
    min_P = P;
    min_D = D;
    min_I = I;
} 
std::cout <<"round:" << round << " error:" << error_ave << "MinPID" << min_P << " " << min_I << " " << min_D << " " << min_error_ave << std::endl;
round++;

}
    finalizeUdpClient();
    return 0;
}
