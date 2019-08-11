/*
 * uart.h
 *
 *  Created on: 2016/07/24
 *      Author: ryota
 */

#pragma once

#include <stdint.h>

#include <deque>
#include <queue>


void initSCI1();
void put1byteSCI1(char c);
void putnbyteSCI1(char *buf, int len);

void initSCIFA9();
void putnbyteSCIFA9(uint8_t *buf, uint16_t len);
void recieveDataSCIFA9();
void sendDataSCIFA9();

extern std::queue<uint8_t> transBuff; //送信用データバッファ
extern uint8_t recieveBuff[512];
extern uint16_t recieveBuffCount;

//下dequeを受信バッファにしてみたがうまく動かなかった。
//extern deque<uint8_t> recieveBuff;//受信用データバッファ

