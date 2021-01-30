#include "logger.h"
#include "timer.h"




#if FULL_PARAM
static float _log_data[1200][33];
#else
static float _log_data[1200][22];
#endif

namespace umouse{
void Logger::printHeadder(){
    printfSync(
        "timestamp,"
        "v_enc,"
        "v_ave,"
        "v_comp,"
        "v_acc,"
        "ang,"
        "ang_v,"
        "acc_cor_x,"
        "acc_cor_y,"
        "acc_x,"
        "acc_y,"
        "x,"
        "y,"
        "beta,"

        "v_setp,"
        "x_setp,"
        "y_setp,"
        "a_setp,"
        "ang_v_traj,"
        "ang_v_setp,"
        "ang_traj,"
        "ang_setp,"
#if FULL_PARAM
        "d_la,"
        "d_l,"
        "d_r,"
        "d_ra,"
        "ws_la,"
        "ws_l,"
        "ws_r,"
        "ws_ra,"
        "voltage,"
        "duty_l,"
        "duty_r"
#endif
            "\n"
    );
}

void Logger::print(){

    if(_logging) return;

    stopCMT0();
    stopCMT1();

    printHeadder();
    for(uint32_t i=0; i<_data_num; i++){
#if FULL_PARAM
        printfSync( "%f,"
                    "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,"
                    "%f, %f, %f, %f, %f, %f, %f, %f, %f,"
                    "%f, %f, %f, %f, %f, %f, %f,"
                    "%f, %f, %f,\n",
                    _log_data[i][0],
                    _log_data[i][1], _log_data[i][2],_log_data[i][3],_log_data[i][4],_log_data[i][5],
                    _log_data[i][6], _log_data[i][7],_log_data[i][8],_log_data[i][9],_log_data[i][10],
                    _log_data[i][11], _log_data[i][12],_log_data[i][13],_log_data[i][14],_log_data[i][15],
                    _log_data[i][16], _log_data[i][17],_log_data[i][18],_log_data[i][19],_log_data[i][20],
                    _log_data[i][21], _log_data[i][22],_log_data[i][23],_log_data[i][24],_log_data[i][25],
                    _log_data[i][26], _log_data[i][27],_log_data[i][28],_log_data[i][29],_log_data[i][30],
                    _log_data[i][31], _log_data[i][32]
        );
#else
        printfSync( "%f," 
                    "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,"
                    "%f, %f, %f, %f, %f, %f, %f, %f, %f,"
                    "\n",
                    _log_data[i][0],
                    _log_data[i][1], _log_data[i][2],_log_data[i][3],_log_data[i][4],_log_data[i][5],
                    _log_data[i][6], _log_data[i][7],_log_data[i][8],_log_data[i][9],_log_data[i][10],
                    _log_data[i][11], _log_data[i][12],_log_data[i][13],_log_data[i][14],_log_data[i][15],
                    _log_data[i][16], _log_data[i][17],_log_data[i][18],_log_data[i][19],_log_data[i][20],
                    _log_data[i][21], _log_data[i][22]
        );

#endif
    }
    startCMT0();
    startCMT1();
}

void Logger::start(){
    _data_num =0;
    _logging = true;
    _start_time_ms = getElapsedMsec();
}



void Logger::end(){
    _logging = false;
}

void Logger::update(){
    if(_data_num >= _max_data_num){
        _logging = false;
        _data_num = _max_data_num;
    }

    if(_logging){
        UMouse& m = UMouse::getInstance();
        BatVoltageMonitor& bvm = BatVoltageMonitor::getInstance();
        WallSensor& ws = WallSensor::getInstance();
        WheelOdometry& wo = WheelOdometry::getInstance();
        ICM20602& icm = ICM20602::getInstance();

        PowerTransmission& pt = PowerTransmission::getInstance();

        _log_data[_data_num][0] = getElapsedMsec() - _start_time_ms;
        _log_data[_data_num][1] = wo.getV();
        _log_data[_data_num][2] = wo.getAveV();
        _log_data[_data_num][3] = m.posEsti.getV();
        _log_data[_data_num][4] = m.posEsti.getVAcc();

        _log_data[_data_num][5] = m.posEsti.getAng();
        _log_data[_data_num][6] = m.posEsti.getAngV();
        _log_data[_data_num][7] = icm.acc_f_cor[0];
        _log_data[_data_num][8] = icm.acc_f_cor[1];
        _log_data[_data_num][9] = icm.acc_f[0];
        _log_data[_data_num][10] = icm.acc_f[1];
        _log_data[_data_num][11] = m.posEsti.getX();
        _log_data[_data_num][12] = m.posEsti.getY();
        _log_data[_data_num][13] = m.posEsti.getBeta();

        _log_data[_data_num][14] = m.trajCommander.v;
        _log_data[_data_num][15] = m.trajCommander.x;
        _log_data[_data_num][16] = m.trajCommander.y;
        _log_data[_data_num][17] = m.trajCommander.a;
        _log_data[_data_num][18] = m.trajCommander.ang_v;
        _log_data[_data_num][19] = m.ctrlMixer.target_rot_v;
        _log_data[_data_num][20] = m.trajCommander.ang;
        _log_data[_data_num][21] = m.ctrlMixer.target_rot_x;
#if FULL_PARAM
        _log_data[_data_num][22] = ws.ahead_dist_l();
        _log_data[_data_num][23] = ws.ahead_dist_l();
        _log_data[_data_num][24] = ws.ahead_dist_r();
        _log_data[_data_num][25] = ws.ahead_dist_r();
        _log_data[_data_num][26] = (float)ws.ahead_l();
        _log_data[_data_num][27] = (float)ws.left();
        _log_data[_data_num][28] = (float)ws.right();
        _log_data[_data_num][29] = (float)ws.ahead_r();
        _log_data[_data_num][30] = bvm.bat_vol;
        _log_data[_data_num][31] = pt.getDuty_L();
        _log_data[_data_num][32] = pt.getDuty_R();
#endif
        _data_num++;
    }

}


}

