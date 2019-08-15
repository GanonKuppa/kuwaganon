
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <chrono>
#include "picojson.h"
#include <windows.h>
#include "udp.h"


void initWebAppConnection(){
    initUdpClient("127.0.0.1", 2020);
}

void finalizeWebAppConnection(){
    finalizeUdpClient();
}

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


void sendMazeWall(uint32_t* walls_vertical, uint32_t* walls_horizontal){
    picojson::object obj;
    obj.emplace(std::make_pair("type", "maze_wall"));
    
    std::stringstream vertical_ss;
    std::stringstream horizontal_ss;

    for(int i=0;i<31;i++){
        uint32_t v_val = ((walls_vertical[i] & 0xFF000000) >> (8*3)) +
                         ((walls_vertical[i] & 0x00FF0000) >> (8*1)) +
                         ((walls_vertical[i] & 0x0000FF00) << (8*1)) +
                         ((walls_vertical[i] & 0x000000FF) << (8*3));    

        vertical_ss  << std::setw(8) << std::setfill('0') << std::hex << v_val;
        
        uint32_t h_val = ((walls_horizontal[i] & 0xFF000000) >> (8*3)) +
                         ((walls_horizontal[i] & 0x00FF0000) >> (8*1)) +
                         ((walls_horizontal[i] & 0x0000FF00) << (8*1)) +
                         ((walls_horizontal[i] & 0x000000FF) << (8*3));    
        horizontal_ss << std::setw(8) << std::setfill('0') << std::hex << h_val;
    }
    obj.emplace(std::make_pair("walls_vertical", vertical_ss.str()));
    obj.emplace(std::make_pair("walls_horizontal", horizontal_ss.str()));
    // 文字列にするためにvalueを使用
    picojson::value val(obj);
    // return std::string
    val.serialize();    
    sendUdpString( val.serialize());
}
