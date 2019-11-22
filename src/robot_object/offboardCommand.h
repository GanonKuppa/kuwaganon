#pragma once

namespace umouse {


    void id_100_exec(uint8_t* buf); // id:100 StopTrajectory挿入
    void id_101_exec(uint8_t* buf); // id:101 StraightTrajectory挿入
    void id_102_exec(uint8_t* buf); // id:102 SpinTurnTrajectory挿入
    void id_103_exec(uint8_t* buf); // id:103 CurveTrajectory挿入
    void id_104_exec(uint8_t* buf); // id:104 SteadySTtateCircularTrajectory挿入

    void id_251_exec(uint8_t* buf); //id:251 パラメータ書換
    void id_254_exec(uint8_t* buf); //id:254 ゲームパッド入力
    void id_255_exec(uint8_t* buf); //id:255 位置角度書換

}