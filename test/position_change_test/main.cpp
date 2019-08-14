

#include <iostream>
#include <fstream>

#include <chrono>
#include "picojson.h"
#include <windows.h>
#include "udp.h"
#include <memory>
#include <random>
#include <time.h>



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
    std::cout << "test" << std::endl;
    initUdpClient("127.0.0.1", 2020);
    sendRobotPos(1.0,1.0, 0.0);
    finalizeUdpClient();
    return 0;
}
