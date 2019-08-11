#include <led.h>


namespace umouse {

LED::LED(){};
LED::~LED(){};
void LED::setState(bool state) {};

/**
 * LEDの状態の更新
 * 1msec間隔でこの関数を呼び出すこと
 */
void LED::update() {
    if (flag_flash_LED == 1) {
        //LEDの点滅
        if (LED_count < on_count_LED) setState(1);
        else setState(0);
        if ((on_count_LED + off_count_LED) < LED_count) LED_count = 0;
        LED_count++;
    }
}

bool LED::getState(){return state;}

/**
 * LEDの点灯と消灯.  LEDが点滅状態で
 * 呼んだ場合は常時点灯(消灯)状態になる.
 *
 * @param  state 1:点灯 0:消灯
 */
void LED::turn(bool state) {
    flag_flash_LED = 0;
    setState(state);
}

/**
 * LEDを指定した時間間隔で点滅させる <br>
 * 例.LEDを100msecの間on, 50msecの間offで点滅させるには <br>
 *    on_count = 100, off_count = 50 <br>
 * LEDを常時点灯する場合は <br>
 *    turnLED(1) <br>
 * LEDを消灯する場合は <br>
 *    turnLED(0) <br>
 *
 * @param  on_count  [msec]
 * @param  off_count [msec]
 * @return void
 */
void LED::flash(uint16_t on_count, uint16_t off_count) {
    flag_flash_LED = 1;
    LED_count = 0;
    on_count_LED = on_count;
    off_count_LED = off_count;
}

}
