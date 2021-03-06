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

static float dutyMTU0;
static float dutyMTU3;
static float dutyMTU4;
static float dutyTPU3;

static const uint16_t FREQ_COUNT = 400;
/*
MOTOR1_PHA 33 P13/MTIOC0B
MOTOR1_PWM 27 P21/MTIOC4A
MOTOR2_PHA 28 P20/TIOCB3
MOTOR2_PWM 32 P14/MTIOC3A

////左輪
27 MTIOC4A
33
右輪
28
32 MTIOC3A
*/

void initMTU0() {
    SYSTEM.PRCR.WORD = 0xA502;
    SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //MTUモジュールON
    SYSTEM.PRCR.WORD = 0xA500;

    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;
    MPC.P13PFS.BIT.PSEL = 1; //MTIOC0B
    MPC.PWPR.BYTE = 0x80;

    PORT1.PMR.BIT.B3 = 1;
    
    MTU.TSTRA.BIT.CST0 = 0;
    MTU0.TCR.BIT.TPSC = 0; //PCLKA/1
    MTU0.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア
    MTU0.TIORH.BIT.IOA = 6; //初期出力1 コンペアマッチ1出力
    MTU0.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力
    MTU0.TGRA = FREQ_COUNT; //500
    MTU0.TGRB = 1;
    MTU0.TGRC = FREQ_COUNT;
    MTU0.TGRD = 1;
    MTU0.TMDR1.BIT.MD = 3; //PWM2
    MTU0.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
    MTU0.TMDR1.BIT.BFB = 1;
}
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
    
    MTU.TSTRA.BIT.CST3 = 0;
    MTU3.TCR.BIT.TPSC = 0; //PCLKA/1
    MTU3.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア
    MTU3.TIORH.BIT.IOA = 6; //初期出力1 コンペアマッチ1出力
    MTU3.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力
    MTU3.TGRA = FREQ_COUNT; //500
    MTU3.TGRB = 1;
    MTU3.TGRC = FREQ_COUNT;
    MTU3.TGRD = 1;
    MTU3.TMDR1.BIT.MD = 2; //PWM1
    MTU3.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
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

    MTU.TSTRA.BIT.CST4 = 0;
    MTU.TOERA.BIT.OE4A = 1; //MTU出力端子を出力許可する

    MTU4.TCR.BIT.TPSC = 0; //PCLKA/1
    MTU4.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
    MTU4.TIORH.BIT.IOA = 6; //初期出力1 コンペアマッチ1出力
    MTU4.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力
    MTU4.TGRA = FREQ_COUNT;
    MTU4.TGRB = 1;
    MTU4.TGRC = FREQ_COUNT;
    MTU4.TGRD = 1;
    MTU4.TMDR1.BIT.MD = 2; //PWM1
    MTU4.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
    MTU4.TMDR1.BIT.BFB = 1; //バッファーモードに設定
}


void initTPU3(){
    SYSTEM.PRCR.WORD = 0xA502;
    SYSTEM.MSTPCRA.BIT.MSTPA13 = 0; //TPUモジュールON
    SYSTEM.PRCR.WORD = 0xA500;
    
    PORT2.PMR.BIT.B0 = 0;
    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;
    MPC.P20PFS.BIT.PSEL = 3; 
    MPC.PWPR.BYTE = 0x80;

    PORT2.PMR.BIT.B0 = 1;

    TPUA.TSTR.BIT.CST3 = 0;
        
    TPU3.TCR.BIT.TPSC = 0; //PCLKA/1
    TPU3.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
    TPU3.TIORH.BIT.IOA = 6; //初期出力1 コンペアマッチ1出力
    TPU3.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力

    TPU3.TGRA = FREQ_COUNT; //500
    TPU3.TGRB = 1;
    TPU3.TGRC = FREQ_COUNT;
    TPU3.TGRD = 1;
    TPU3.TMDR.BIT.MD = 3; //PWM2
    TPU3.TMDR.BIT.BFA = 1;
    TPU3.TMDR.BIT.BFB = 1;
}


void setDutyMTU0(float duty) {
    duty = constrain(duty, 0.0, 1.0);
    dutyMTU0 = duty;
    if (duty == 0.0) {
        PORT1.PMR.BIT.B3 = 0;
        PORT1.PODR.BIT.B3 = 0;
        //MTU0.TGRD = 1;
    }
    else if(duty == 1.0){
        PORT1.PMR.BIT.B3 = 0;
        PORT1.PODR.BIT.B3 = 1;
        //MTU0.TGRD = MTU0.TGRC-1;
    }
    else {
        PORT1.PMR.BIT.B3 = 1;
        MTU0.TGRD = (uint16_t) (MTU0.TGRC * duty);
    }
    MTU.TSTRA.BIT.CST0 = 1;

}


void setDutyMTU3(float duty) {
    duty = constrain(duty, 0.0, 1.0);
    dutyMTU3 = duty;
    if (duty == 0.0) {
        PORT1.PMR.BIT.B4 = 0;
        PORT1.PODR.BIT.B4 = 0;
        //MTU3.TGRD = 1;
    }
    else if(duty == 1.0){
        PORT1.PMR.BIT.B4 = 0;
        PORT1.PODR.BIT.B4 = 1;
        //MTU3.TGRD = MTU3.TGRC -1;
    }    
    else {
        PORT1.PMR.BIT.B4 = 1;
        MTU3.TGRD = (uint16_t) (MTU3.TGRC * duty);
    }
    MTU.TSTRA.BIT.CST3 = 1;

}

void setDutyMTU4(float duty) {
    duty = constrain(duty, 0.0, 1.0);
    dutyMTU4 = duty;
    if (duty == 0.0) {
        PORT2.PMR.BIT.B1 = 0; //左PWM
        PORT2.PODR.BIT.B1 = 0; //左PWM
        //MTU4.TGRD = 1;
    }
    else if(duty == 1.0){
        PORT2.PMR.BIT.B1 = 0; //左PWM
        PORT2.PODR.BIT.B1 = 1; //左PWM
        //MTU4.TGRD = MTU4.TGRC-1;
    }    
    else {
        PORT2.PMR.BIT.B1 = 1; //左PWM
        MTU4.TGRD = (uint16_t) (MTU4.TGRC * duty);
    }
    MTU.TSTRA.BIT.CST4 = 1;

}

void setDutyTPU3(float duty) {
    duty = constrain(duty, 0.0, 1.0);
    dutyMTU4 = duty;
    if (duty == 0.0) {
        PORT2.PMR.BIT.B0 = 0;
        PORT2.PODR.BIT.B0 = 0;

        //TPU3.TGRD = 1;
    }
    else if(duty == 1.0){
        PORT2.PMR.BIT.B0 = 0;
        PORT2.PODR.BIT.B0 = 1;
        //TPU3.TGRD = TPU3.TGRC -1;
    }    
    else {
        PORT2.PMR.BIT.B0 = 1;
        TPU3.TGRD = (uint16_t) (TPU3.TGRC * duty);
    }
    TPUA.TSTR.BIT.CST3 = 1;

}



float getDutyMTU0() {
    return dutyMTU0;
}

float getDutyMTU3() {
    return dutyMTU3;
}

float getDutyMTU4() {
    return dutyMTU4;
}

float getDutyTPU3() {
    return dutyTPU3;
}

