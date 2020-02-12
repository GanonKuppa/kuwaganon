/***************************************************************/
/*                                                             */
/*      PROJECT NAME :  umouse forte                             */
/*      FILE         :  main.cpp                         */
/*      DESCRIPTION  :  Main Program                           */
/*      CPU SERIES   :  RX700                                  */
/*      CPU TYPE     :  RX71M                                  */
/*                                                             */
/*      This file is generated by e2 studio.                   */
/*                                                             */
/***************************************************************/

/************************************************************************/
/*    File Version: V1.00                                               */
/*    Date Generated: 08/07/2013                                        */
/************************************************************************/

#include "iodefine.h"

#ifdef CPPAPP
//Initialize global constructors
extern "C" void __main() {
    static int initialized;
    if (!initialized) {
        typedef void (*pfunc)();
        extern pfunc __ctors[];
        extern pfunc __ctors_end[];
        pfunc* p;

        initialized = 1;
        for (p = __ctors_end; p > __ctors;)
            (*--p)();
    }
}

#endif

//その他
#include <stdint.h>
#include <string>
#include <array>
#include <memory>
#include <myUtil.h>
#include <functional>

//peripheral_RX71M
#include "clock.h"
#include <uart.h>
#include "gpio.h"
#include <timeInterrupt.h>
#include "spi.h"
#include "timer.h"
#include "ad.h"
#include "pwm.h"
#include "phaseCounting.h"
#include "dataFlash.h"
#include "da.h"
#include "wdt.h"

//robot_object


#include "sound.h"

#include "fcled.h"
#include "communication.h"
#include "parameterManager.h"
#include "gamepad.h"
#include "wallsensor.h"

#include "batVoltageMonitor.h"
#include "wheelOdometry.h"
#include "ICM20602.h"
#include "adis16470.h"

#include "powerTransmission.h"
#include "pseudoDial.h"

//umouse_object
#include "maze.h"
#include <mouse.h>

//mode
#include "baseActivity.h"
#include "activityFactory.h"

//プロトタイプ宣言
void periperalInit();
void startUpInit();
void object_init();

//-------------タイマ割り込み関数---------------//
extern "C" void timeInterrupt(void);

//250usec毎に呼ばれる
void timeInterrupt(void) {
    //http://japan.renesasrulz.com/cafe_rene/f/69/t/1515.aspx 多重割り込み 資料
    __builtin_rx_setpsw('I');

    static uint64_t int_tick_count = 0;

    //--------------------------------------//
    umouse::ICM20602& icm = umouse::ICM20602::getInstance();
    umouse::adis16470& adis = umouse::adis16470::getInstance();
    umouse::FcLed& fcled = umouse::FcLed::getInstance();
    umouse::WallSensor& wallSen = umouse::WallSensor::getInstance();
    umouse::Gamepad& gamepad = umouse::Gamepad::getInstance();
    umouse::UMouse& mouse = umouse::UMouse::getInstance();
    umouse::BatVoltageMonitor& batVolMan = umouse::BatVoltageMonitor::getInstance();
    umouse::WheelOdometry& wheelOdometry = umouse::WheelOdometry::getInstance();
    umouse::PseudoDialL& dialL = umouse::PseudoDialL::getInstance();
    umouse::PseudoDialR& dialR = umouse::PseudoDialR::getInstance();

    //--------------------------------------//

    //UARTの送受信処理
    sendDataSCIFA9();
    recieveDataSCIFA9();
    umouse::fetchCommand();
    //3000msec毎の処理
    if (int_tick_count % 1200 == 0) {
        if (getElapsedMsec() > 6000) {
            batVolMan.lowVoltageCheck();
        }
    }

    //30msec毎の処理=120  20msec毎=80 10msec=40 7.5msec=30
    if (int_tick_count % 60 == 0) {
        umouse::sendPeriodicMsg();
    }
    //スロット0
    if (int_tick_count % 4 == 0) {
        if (getElapsedMsec() > 5000) {
            std::function< void(void) > w1 = [&wallSen]() {wallSen.update();};
            std::function< void(void) > w2 = [&icm]() {icm.update();};
            std::function< void(void) > w3 = [&mouse]() {mouse.update();};
            std::function< void(void) > w4 = []() {waitusec_sub(20);};
            adis.update(w1, w2, w3, w4);
            sendDataSCIFA9();
        } else {
            wallSen.update();
        }
    }
    //スロット1
    if (int_tick_count % 4 == 0) {
        wheelOdometry.update();
        gamepad.update();

    }
    //スロット2
    if (int_tick_count % 4 == 2) {
        if (getElapsedMsec() > 5000) {
            std::function< void(void) > w1 = [&wallSen]() {wallSen.update();};
            std::function< void(void) > w2 = [&icm]() {icm.update();};
            std::function< void(void) > w3 = [&mouse]() {mouse.update();};
            std::function< void(void) > w4 = []() {waitusec_sub(20);};
            adis.update(w1, w2, w3, w4);
            sendDataSCIFA9();
        } else {
            wallSen.update();
        }
    }
    //スロット3
    if (int_tick_count % 4 == 3) {
        wheelOdometry.update();
        dialL.update();
        dialR.update();
        fcled.update();
    }

    /////毎回行う処理/////

    //リセットコマンドの監視
    if (gamepad.BACK > 0 && gamepad.LB > 0) {
        SYSTEM.PRCR.WORD = 0xA502; //ソフトウェアリセット
        SYSTEM.SWRR = 0xA501;
    }

    batVolMan.update();
    soundUpdate();
    endTimeuCountIntCMT0();

    int_tick_count++;
    //resetWdt();
}

//-------------メイン関数---------------//
int main() {
    periperalInit();
    startUpInit();

    while (1) {
        auto activity = umouse::ActivityFactory::cteateModeSelect();
        activity->start();
    };

    return 0;
}

//各ペリフェラルの初期化
void periperalInit() {
    //クロック
    initClock();
    //IOピン
    initGPIO();
    //UART
    initSCI1();
    initSCIFA9();

    //割り込み関数
    initCMT0();
    initCMT1();

    //SPI
    initRSPI0();
    initRSPI1();

    //時間測定
    initTPU0();
    initCMTW0();
    initCMTW1();

    //AD
    initAD();

    //位相係数
    initMTU1();
    initMTU2();

    //PWM
    initMTU3();
    initMTU4();


    //DA
    initDA();
    umouse::printfAsync("-------各種ペリフェラル初期化完了-------\n");

    //データフラッシュ
    initDataFlash();
    umouse::ParameterManager& pm = umouse::ParameterManager::getInstance();
    pm.init();

    //ウォッチドックタイマー
    //initWdt();
    //startWdt();
}


//起動時の処理
void startUpInit() {
    setDutyMTU3(0.0);
    setDutyMTU4(0.0);

    setPriorityCMT0(12);
    setPriorityCMT1(15);
    startCMT0();
    umouse::printfAsync("-------CMT0割り込み開始-------\n");

    umouse::ICM20602& icm = umouse::ICM20602::getInstance();
    umouse::adis16470& adis = umouse::adis16470::getInstance();

    icm.init();
    adis.init();

    //sound
    startCMT1();
    umouse::printfAsync("-------CMT1割り込み開始-------\n");
    //
    object_init();
    /////////////コンパイル時固有文字列/////////
    umouse::printfAsync("Compile Date\n %s\n", __DATE__);
    uint16_t compile_hash = 0;
    for (int i = 0; i < sizeof(__TIME__); i++) {
        compile_hash += __TIME__[i];
    }

    umouse::printfAsync("Compile HASH: %d\n", compile_hash);
    umouse::printfAsync("Compile TIME: %s\n", __TIME__);
    umouse::printfAsync("Compile FILE: %s\n", __FILE__);
    umouse::printfAsync("---------------------------\n");
    GB();
    waitmsec(1000);
    randomNote(compile_hash);
    waitmsec(300);
    /////////////電池電圧警告////////////////
    umouse::BatVoltageMonitor::getInstance().voltageSoundCount();

    umouse::printfAsync("===finish init====\n");

}


void object_init() {
    //robot_object
    umouse::WallSensor::getInstance();
    umouse::ParameterManager::getInstance();
    umouse::Gamepad::getInstance();
    umouse::FcLed::getInstance();
    umouse::BatVoltageMonitor::getInstance();
    umouse::WheelOdometry::getInstance();
    umouse::ICM20602::getInstance();
    umouse::adis16470::getInstance();
    umouse::UMouse::getInstance();
    umouse::PowerTransmission::getInstance();
    umouse::PseudoDialL::getInstance();
    umouse::PseudoDialR::getInstance();

    //umouse_object

}

