#pragma once

#include <stdint.h>

void initCMT0();
void setPriorityCMT0(uint8_t priori);
void startCMT0();
void stopCMT0();

uint32_t endTimeuCountIntCMT0(void);
uint32_t getTimeuCountIntCMT0(void);

void initCMT1();
void setPriorityCMT1(uint8_t priori);
void startCMT1();
void stopCMT1();

uint32_t endTimeuCountIntCMT1(void);
uint32_t getTimeuCountIntCMT1(void);

