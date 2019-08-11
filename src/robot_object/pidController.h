/*
 * pidController.h
 *
 *  Created on: 2018/11/06
 *      Author: qtfdl
 */

#pragma once

#include "myUtil.h"

namespace umouse {

class VelocityTypePidfController {
public:
    float Kp;
    float Ki;
    float Kd;
    float F;
    float e_k0;
    float e_k1;
    float ud_k0;
    float ud_k1;
    float u_k0;
    float u_k1;
    float satuation;
    bool enable;
    bool satuation_enable;


    VelocityTypePidfController() {
           Kp = 0.0f;
           Ki = 0.0f;
           Kd = 0.0f;
           F  = 0.0f;
           e_k0 = 0.0f;
           e_k1 = 0.0f;
           u_k0 = 0.0f;
           u_k1 = 0.0f;
           ud_k0 = 0.0f;
           ud_k1 = 0.0f;
           satuation = 0.0f;
           bool enable = true;
           bool satuation_enable = true;

    }

    virtual void update(float target_, float observed_val_) {
        e_k0 = target_ - observed_val_;

        ud_k0 = F * ud_k1 + (1.0f - F) * (e_k0 - e_k1);
        float delta_u_k = (Kp * (e_k0 - e_k1)) +  (Ki * e_k0) + (Kd * (ud_k0 - ud_k1));
        u_k0 = u_k1 + delta_u_k;
        if(satuation != 0.0f && satuation_enable == true)u_k0 = constrain(u_k0, -satuation, satuation);

        u_k1 = u_k0;
        e_k1 = e_k0;
        ud_k1 = ud_k0;

    };



    float getControlVal() {
        if(enable == true)return u_k0;
        else return 0.0f;
    }

    void set(float Kp_, float Ki_, float Kd_, float F_) {
            Kp = Kp_;
            Ki = Ki_;
            Kd = Kd_;
            F = F_;
    }

    void setSatuation(float satuation_){
        satuation = satuation_;
        if(satuation != 0.0f && satuation_enable == true)u_k0 = constrain(u_k0, -satuation, satuation);
    }

    void setEnable(bool enable_){
        enable = enable_;
    }

    void setSatuationEnable(float se){
        satuation_enable = se;
    }

    void reset(){
        e_k0 = 0.0;
        e_k1 = 0.0;
        u_k0 = 0.0;
        u_k1 = 0.0;
        ud_k0 = 0.0;
        ud_k1 = 0.0;
    }

};

class AngPidfController : public VelocityTypePidfController{
public:

    virtual void update(float target_, float observed_val_) {
        float ang_diff = target_ - observed_val_;
        while(ang_diff >  180.0f) ang_diff -= 360.0f;
        while(ang_diff < -180.0f) ang_diff += 360.0f;
        e_k0 = ang_diff;

        ud_k0 = F * ud_k1 + (1 - F) * (e_k0 - e_k1);
        float delta_u_k = (Kp * (e_k0 - e_k1)) +  (Ki * e_k0) + (Kd * (ud_k0 - ud_k1));
        u_k0 = u_k1 + delta_u_k;

        u_k1 = u_k0;
        e_k1 = e_k0;
        ud_k1 = ud_k0;

    };


};




class VelocityTypePidController {
public:
    float Kp;
    float Ki;
    float Kd;
    float e_k0; // 現在偏差
    float e_k1;// 前回偏差
    float e_k2;// 前々回偏差
    float u_k0;// 現在制御量
    float u_k1;// 前回制御量
    float T_s;// 制御周期

    VelocityTypePidController() {
        Kp = 0.0;
        Ki = 1.0;
        Kd = 0.0;
        e_k0 = 0.0;
        e_k1 = 0.0;
        e_k2 = 0.0;
        u_k0 = 0.0;
        u_k1 = 0.0;
        T_s = 0.001;
    }

    VelocityTypePidController(const VelocityTypePidController &pidc) {
        Kp = pidc.Kp;
        Ki = pidc.Ki;
        Kd = pidc.Kd;
        e_k0 = pidc.e_k0;
        e_k1 = pidc.e_k1;
        e_k2 = pidc.e_k2;
        u_k0 = pidc.u_k0;
        u_k1 = pidc.u_k1;
        T_s = pidc.T_s;

    }

    void delegateInternalState(const VelocityTypePidController &pidc) {
        e_k0 = pidc.e_k0;
        e_k1 = pidc.e_k1;
        e_k2 = pidc.e_k2;
        u_k0 = pidc.u_k0;
        u_k1 = pidc.u_k1;
        T_s = pidc.T_s;
    }

    VelocityTypePidController &operator=(const VelocityTypePidController &pidc) {
        return *this;
    }

    virtual void update(float target_, float observed_val_) {
        if(Kp == 0.0) return;
        e_k0 = target_ - observed_val_;
        float delta_u_k = Kp * (e_k0 - e_k1 + T_s / Ki * e_k0 + Kd / T_s * (e_k0 - 2 * e_k1 + e_k2) );
        u_k0 = u_k1 + delta_u_k;

        u_k1 = u_k0;
        e_k2 = e_k1;
        e_k1 = e_k0;

    };

    float getControlVal() {
        if(Kp == 0.0) return 0.0;
        else return u_k0;
    }

    void set(float Kp_, float Ki_, float Kd_) {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
    }

};

class PidController {
public:
    float Kp;
    float Ki;
    float Kd;
    float int_lim;
    float target;
    float error_p;
    float error_i;
    float error_d;
    float error_p_pre;

    void update(float target_, float observed_val) {
        target = target_;
        error_p = target - observed_val;
        error_i = constrain(error_p+error_i, -int_lim, int_lim);
        error_d = error_p - error_p_pre;
        error_p_pre = error_p;
    };

    float calc() {
        float control_val = Kp * error_p + Ki * error_i + Kd * error_d;
        return control_val;
    };

    void set(float Kp_, float Ki_, float Kd_) {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
    }

    void set(float Kp_, float Ki_, float Kd_, float int_lim_) {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
        int_lim = int_lim_;
    }

    PidController(float Kp_, float Ki_, float Kd_, float int_lim_) {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
        int_lim = int_lim_;
        target = 0.0;
        error_p = 0.0;
        error_i = 0.0;
        error_d = 0.0;
        error_p_pre = 0.0;
    }

    PidController() {
        Kp = 0.0;
        Ki = 0.0;
        Kd = 0.0;
        int_lim = 0.0;
        target = 0.0;
        error_p = 0.0;
        error_i = 0.0;
        error_d = 0.0;
        error_p_pre = 0.0;
    }

};

}
