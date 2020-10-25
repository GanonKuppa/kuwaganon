#pragma once

#include <stdint.h>

void initCMTW0();
void initCMTW1();
void initTPU0();

void waitClockCount(uint32_t cCount);
void waitusec(uint32_t usec);
void waitmsec(uint32_t msec);
void startTimeuCount(void);
unsigned int getTimeuCount(void);
unsigned int endTimeuCount(void);

void waitClockCount_sub(uint32_t cCount);
void waitusec_sub(uint32_t usec);
void waitmsec_sub(uint32_t msec);
void startTimeuCount_sub(void);
unsigned int getTimeuCount_sub(void);
unsigned int endTimeuCount_sub(void);

unsigned int getElapsedUsec(void);
unsigned int getElapsedMsec(void);
unsigned int getElapsedSec(void);

