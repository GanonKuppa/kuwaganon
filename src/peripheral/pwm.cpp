/*
 * pwm.cpp
 *
 *  Created on: 2016/11/06
 *      Author: ryota
 */

#include <myUtil.h>
#include "iodefine.h"

#include <stdint.h>

#include "pwm.h"
#include "clock.h"

static float dutyMTU3;
static float dutyMTU4;

/////////////////////////////////////////////////////////////
void initMTU3() {
    SYSTEM.PRCR.WORD = 0xA502;
    SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //MTUモジュールON
    SYSTEM.PRCR.WORD = 0xA500;

    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;
    MPC.P14PFS.BIT.PSEL = 1; //MTIOC3A
    MPC.PWPR.BYTE = 0x80;

    PORT1.PMR.BIT.B4 = 1;

    MTU3.TCR.BIT.TPSC = 0; //PCLKA/1
    MTU3.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
    MTU3.TIORH.BIT.IOA = 1; //初期出力0ンペアマッチ0出力
    MTU3.TIORH.BIT.IOB = 2; //初期出力0コンペアマッチ1出力
    MTU3.TGRA = 500; //500
    MTU3.TGRB = 1;
    MTU3.TGRC = 500;
    MTU3.TGRD = 1;
    MTU3.TMDR1.BIT.MD = 2; //PWM1
    //MTU3.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
    MTU3.TMDR1.BIT.BFB = 1;
}

/////////////////////////////////////////////////////////////
void initMTU4() {
    SYSTEM.PRCR.WORD = 0xA502;
    SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //MTUモジュールON
    SYSTEM.PRCR.WORD = 0xA500;

    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;
    MPC.P21PFS.BIT.PSEL = 0b001000; //MTIOC4A
    MPC.PWPR.BYTE = 0x80;

    PORT2.PMR.BIT.B1 = 1; //左PWM
    MTU.TSTRA.BIT.CST4 = 1;
    MTU.TOERA.BIT.OE4A = 1; //MTU出力端子を出力許可する

    MTU4.TCR.BIT.TPSC = 0; //PCLKA/1
    MTU4.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
    MTU4.TIORH.BIT.IOA = 1; //初期出力0ンペアマッチ0出力
    MTU4.TIORH.BIT.IOB = 2; //初期出力0コンペアマッチ1出力
    MTU4.TGRA = 500;
    MTU4.TGRB = 1;
    MTU4.TGRC = 500;
    MTU4.TGRD = 1;
    MTU4.TMDR1.BIT.MD = 2; //PWM1
    //MTU4.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
    MTU4.TMDR1.BIT.BFB = 1; //バッファーモードに設定
}

void setDutyMTU3(float duty) {
    duty = constrain(duty, 0.0, 1.0);
    dutyMTU3 = duty;
    if (duty == 0.0) {
        MTU3.TGRD = MTU3.TGRC;
    } else {
        MTU3.TGRD = (uint16_t) (MTU3.TGRC * (1.0 - duty));
    }
    MTU.TSTRA.BIT.CST3 = 1;

}

void setDutyMTU4(float duty) {
    duty = constrain(duty, 0.0, 1.0);
    dutyMTU4 = duty;
    if (duty == 0.0) {
        MTU4.TGRD = MTU4.TGRC-1;
    } else {
        MTU4.TGRD = (uint16_t) (MTU4.TGRC * (1.0 - duty));
    }
    MTU.TSTRA.BIT.CST4 = 1;

}

float getDutyMTU3() {
    return dutyMTU3;
}

float getDutyMTU4() {
    return dutyMTU4;
}

