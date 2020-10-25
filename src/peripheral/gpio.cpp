/**
 * @file   clock.cpp
 * @brief  GPIOピンの設定
 *
 * @date 2016/7/23
 * @author ryota
 */

#include "iodefine.h"
#include "gpio.h"
#include <stdint.h>

void initGPIO() {
    //未使用ピンの処理
    PORT0.PDR.BYTE = (uint8_t) (PORT0.PDR.BYTE | 0x0F);
    PORT1.PDR.BYTE = (uint8_t) (PORT1.PDR.BYTE | 0x03);
    PORT5.PDR.BYTE = (uint8_t) (PORT5.PDR.BYTE | 0x40);
    PORT6.PDR.BYTE = (uint8_t) (PORT6.PDR.BYTE | 0xFF);
    PORT7.PDR.BYTE = (uint8_t) (PORT7.PDR.BYTE | 0xFF);
    PORT8.PDR.BYTE = (uint8_t) (PORT8.PDR.BYTE | 0xCF);
    PORT9.PDR.BYTE = (uint8_t) (PORT9.PDR.BYTE | 0xFF);
    PORTF.PDR.BYTE = (uint8_t) (PORTF.PDR.BYTE | 0x3F);
    PORTG.PDR.BYTE = (uint8_t) (PORTG.PDR.BYTE | 0xFF);
    PORTJ.PDR.BYTE = (uint8_t) (PORTJ.PDR.BYTE | 0x20);

    //FCLEDピン設定
    PORTD.PDR.BIT.B4 = 1; //B
    PORTD.PDR.BIT.B3 = 1; //G
    PORTD.PDR.BIT.B2 = 1; //R

    //MOTOR1ピン設定
    PORT1.PDR.BIT.B3 = 1; //MOTOR1_PHA
    PORT2.PDR.BIT.B1 = 1; //MOTOR1_PWM

    //MOTOR1ピン初期化
    PORT1.PODR.BIT.B3 = 0; //MOTOR1_PHA
    PORT2.PODR.BIT.B1 = 0; //MOTOR1_PWM

    //MOTOR2ピン設定
    PORT2.PDR.BIT.B0 = 1; //MOTOR2_PHA
    PORT1.PDR.BIT.B4 = 1; //MOTOR2_PWM

    //MOTOR2ピン初期化
    PORT2.PODR.BIT.B0 = 0; //MOTOR2_PHA
    PORT1.PODR.BIT.B4 = 0; //MOTOR2_PWM

    //センサLED
    PORTE.PDR.BIT.B0 = 1; //SLED_OUT1
    PORTD.PDR.BIT.B7 = 1; //SLED_OUT2
    PORTD.PDR.BIT.B0 = 1; //SLED_OUT3
    PORT0.PDR.BIT.B7 = 1; //SLED_OUT4

    //センサLED消灯
    PORTE.PODR.BIT.B0 = 0; //SLED_OUT1
    PORTD.PODR.BIT.B7 = 0; //SLED_OUT2
    PORTD.PODR.BIT.B0 = 0; //SLED_OUT3
    PORT0.PODR.BIT.B7 = 0; //SLED_OUT4

    // エンコーダZ相
    PORTB.PDR.BIT.B3 = 0; //57ピンENC_1_Z
    PORTA.PDR.BIT.B2 = 0; //68ピンENC_2_Z
}

