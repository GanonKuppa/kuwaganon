/*
 * timer.cpp
 *
 *  Created on: 2016/08/15
 *      Author: ryota
 */

#include <stdint.h>

#include <timer.h>
#include <uart.h>
#include "iodefine.h"
#include "clock.h"
//タイマを三つ使用
//CMTW0 elepsedTimeを刻む
//CMTW1 メインのタイマに使用
//TPU0  サブのタイマに使用

// PCLKBがクロックソース48MHz かつ タイマの分周比=8 --> 1μ秒　＝　6 カウント
static const unsigned int u_count = 6;
static const unsigned int m_count = 6000;

void initTPU0() {
    SYSTEM.PRCR.WORD = 0xA502;
    SYSTEM.MSTPCRA.BIT.MSTPA13 = 0; //TPU0-5 ON    //モジュールストップを解除
    SYSTEM.PRCR.WORD = 0xA500;

    TPUA.TSTR.BIT.CST0 = 0;
    TPU0.TCR.BIT.TPSC = 2; //1:PCLK/4 2:PCLK/16
    TPU0.TCNT = 0;

}

void initCMTW0() {
    SYSTEM.PRCR.WORD = 0xA502;
    MSTP(CMTW0) = 0; //モジュールストップを解除
    SYSTEM.PRCR.WORD = 0xA500;

    CMTW0.CMWSTR.BIT.STR = 0; //カウント停止
    CMTW0.CMWCR.BIT.CKS = 2; //0:PCLK/8  2:PCLK/128
    CMTW0.CMWCNT = 0;
    CMTW0.CMWCR.BIT.CCLR = 1;
    CMTW0.CMWCR.BIT.CMS = 0; // 32bitカウンタ

    CMTW0.CMWCR.BIT.CCLR = 1; //カウンタのクリア禁止

    CMTW0.CMWSTR.BIT.STR = 1; //カウント開始
}

void initCMTW1() {
    SYSTEM.PRCR.WORD = 0xA502;
    MSTP(CMTW1) = 0; //モジュールストップを解除
    SYSTEM.PRCR.WORD = 0xA500;

    CMTW1.CMWCR.BIT.CKS = 0; //0:PCLK/8  2:PCLK/128
    CMTW1.CMWCNT = 0;
    CMTW1.CMWCR.BIT.CCLR = 1;
    CMTW1.CMWCR.BIT.CMS = 0; // 32bitカウンタ

    CMTW1.CMWCR.BIT.CCLR = 1; //カウンタのクリア禁止
    CMTW1.CMWCNT = 0;
}

void waitClockCount(uint32_t cCount) {
    //PCLKBでカウント;
    //count_clock = PCLKB/8;
    CMTW1.CMWCNT = 0; //カウント値を0にする
    CMTW1.CMWSTR.BIT.STR = 1; //カウント開始

    while (1) {
        if ( CMTW1.CMWCNT >= cCount) break;
    }
    CMTW1.CMWSTR.BIT.STR = 0; //カウント停止

}

void waitusec(uint32_t usec) {
    //PCLKBでカウント;
    //count_clock = PCLKB/8;
    CMTW1.CMWCNT = 0; //カウント値を0にする
    CMTW1.CMWSTR.BIT.STR = 1; //カウント開始

    while (1) {
        if ( CMTW1.CMWCNT >= u_count * usec) break;
    }
    CMTW1.CMWSTR.BIT.STR = 0; //カウント停止
}

void waitmsec(uint32_t msec) {
    //PCLKBでカウント;
    //count_clock = PCLKB/8;
    CMTW1.CMWCNT = 0; // 32bitカウント値を0にする
    CMTW1.CMWSTR.BIT.STR = 1; //カウント開始

    while (1) {
        if ( CMTW1.CMWCNT >= m_count * msec) break;
    }
    CMTW1.CMWSTR.BIT.STR = 0; //カウント停止
}

void startTimeuCount() {
    CMTW1.CMWCNT = 0; //カウント値を0にする
    CMTW1.CMWSTR.BIT.STR = 1; //カウント開始
}


unsigned int getTimeuCount(void) {
    return CMTW1.CMWCNT / u_count;
}

unsigned int endTimeuCount() {
    CMTW1.CMWSTR.BIT.STR = 0; //カウント停止
    return CMTW1.CMWCNT / u_count;
}

void waitClockCount_sub(uint32_t cCount) {
    TPU0.TCNT = 0; //カウント値を0にする
    TPUA.TSTR.BIT.CST0 = 1; //カウント開始

    while (1) {
        if ( TPU0.TCNT >= cCount) break;
    }
    TPUA.TSTR.BIT.CST0 = 0; //カウント停止
}

void waitusec_sub(uint32_t usec) {
    //PCLKBでカウント;
    //count_clock = PCLKB/16;

    TPU0.TCNT = 0;
    TPUA.TSTR.BIT.CST0 = 1;

    while (1) {
        if ( TPU0.TCNT >= 3 * usec) break;
    }
    TPUA.TSTR.BIT.CST0 = 0; //カウント停止
}

void waitmsec_sub(uint32_t msec) {
    //PCLKBでカウント;
    //count_clock = PCLKB/8;
    TPU0.TCNT = 0;
    TPUA.TSTR.BIT.CST0 = 1;

    while (1) {
        if ( TPU0.TCNT >= 3000 * msec) break;
    }
    TPUA.TSTR.BIT.CST0 = 0; //カウント停止
}

void startTimeuCount_sub() {
    TPU0.TCNT = 0; //カウント値を0にする
    TPUA.TSTR.BIT.CST0 = 1; //カウント開始
}
;

unsigned int getTimeuCount_sub(void) {
    return TPU0.TCNT / 3;
}

unsigned int endTimeuCount_sub() {
    TPUA.TSTR.BIT.CST0; //カウント停止
    return TPUA.TSTR.BIT.CST0 / 3;
}


unsigned int getElapsedMsec() {
    //48MHz,128分周 -> 375カウントで1msec
    //190分くらいまでしか計れない
    return CMTW0.CMWCNT / 375;
}

unsigned int getElapsedUsec() {
    //48MHz,128分周 -> 375カウントで1msec
    //190分くらいまでしか計れない
    return CMTW0.CMWCNT / 375 * 1000;
}



unsigned int getElapsedSec() {
    return getElapsedMsec() / 1000;
};



