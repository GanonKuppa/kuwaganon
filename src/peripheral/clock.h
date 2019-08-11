/**
 * @file   clock.h
 * @brief  クロックの設定
 *
 * @date 2016/7/23
 * @author ryota
 */

#pragma once

#include <stdint.h>

const uint32_t ICLK = 96000000;
const uint32_t BCLK = 96000000;
const uint32_t PCLKA = 96000000;
const uint32_t PCLKB = 48000000;
const uint32_t PCLKC = 48000000;
const uint32_t PCLKD = 48000000;

void initClock(void);

