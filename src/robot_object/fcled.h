#pragma once

#include <led.h>
#include <stdint.h>

#define LED_R_PIN   PORTD.PODR.BIT.B2 //R
#define LED_G_PIN   PORTD.PODR.BIT.B3 //G
#define LED_B_PIN   PORTD.PODR.BIT.B4 //B

namespace umouse {

///////////////////////////////////////////////////////////////////
class LED_R : public LED {
public:
    void setState(bool state_){
        LED_R_PIN = 1-state_;
        state = state_;
    };
};
///////////////////////////////////////////////////////////////////
class LED_G : public LED {
public:
    void setState(bool state_){
        LED_G_PIN = 1-state_;
        state = state_;
    };
};
///////////////////////////////////////////////////////////////////
class LED_B : public LED {
public:
    void setState(bool state_){
        LED_B_PIN = 1-state_;
        state = state_;
    };
};
///////////////////////////////////////////////////////////////////
class FcLed {
public:
    static FcLed& getInstance() {
        static FcLed instance;
        return instance;
    }

    LED_R R;
    LED_G G;
    LED_B B;
    void update() {
        R.update();
        G.update();
        B.update();
    }
    void turn(bool r, bool g, bool b) {
        R.turn(r);
        G.turn(g);
        B.turn(b);
    }

    void flash(bool r, bool g, bool b, uint16_t on_count, uint16_t off_count) {
        if(r == 1) R.flash(on_count, off_count);
        else R.turn(0);
        
        if(g == 1) G.flash(on_count, off_count);
        else G.turn(0);
        
        if(b == 1) B.flash(on_count, off_count);
        else B.turn(0);
    }
private:
    FcLed() {turn(0,0,0);}
    ~FcLed() {turn(0,0,0);}
    FcLed(FcLed&) {turn(0,0,0);}
};

}
