#pragma once

#include "pwm.h"
#include "parameterManager.h"
#include "communication.h"
#include "timer.h"
#include "batVoltageMonitor.h"
#include <cfloat>
#include <math.h>
#include "myUtil.h"
#include "fcled.h"
#include "wheelOdometry.h"

#include <Eigen/Core>

namespace umouse {

    class PowerTransmission {
      public:
        static PowerTransmission& getInstance() {
            static PowerTransmission instance;
            return instance;
        }


        void setDuty(float duty_l, float duty_r) {
            setDuty_L(duty_l);
            setDuty_R(duty_r);
        }

        void setRawDuty(float duty_l, float duty_r) {
            setRawDuty_L(duty_l);
            setRawDuty_R(duty_r);
        }


        void setDuty_R(float duty) {
            if(duty_R == duty) return;
            ParameterManager& pm = ParameterManager::getInstance();
            float abs_duty = 0.0;
            abs_duty = constrain(ABS(duty), 0.0, pm.duty_limit);

            if(std::isnan(duty) == true) {
                printfAsync("duty NaN error R\n");
                duty_R = 0.0;
            } else duty_R = SIGN(duty) * abs_duty;

            if (duty_R == 0.0) {
                setDutyTPU3(1.0);
                setDutyMTU3(1.0);
            } else if (duty_R > 0) {
                float duty_real = (OFFSET_VOLTAGE_R_P + 4.2 * abs_duty * CORRECTION_FACTOR_R_P) / getVoltage();
                setDutyTPU3(1.0);
                setDutyMTU3(1.0 - duty_real);

            } else {
                float duty_real = ( - OFFSET_VOLTAGE_R_M + 4.2 * abs_duty * CORRECTION_FACTOR_R_M) / getVoltage();
                setDutyTPU3(1.0 - duty_real);
                setDutyMTU3(1.0);
            }
        }

        void setDuty_L(float duty) {
            if(duty_L == duty) return;
            ParameterManager& pm = ParameterManager::getInstance();
            float abs_duty = 0.0;
            abs_duty = constrain(ABS(duty), 0.0, pm.duty_limit);

            if(std::isnan(duty) == true) {
                printfAsync("duty NaN error L\n");
                duty_L = 0.0;
            } else duty_L = SIGN(duty) * abs_duty;

            // voltage_in = 4.2 * duty_in;
            // voltage_real * duty_real = offset_v + voltage_in 
            // voltage_real * duty_real = offset_v + 4.2 * duty_in

            if (duty_L == 0.0) {
                setDutyMTU0(1.0);
                setDutyMTU4(1.0);
            } else if (duty_L > 0) {
                float duty_real = (OFFSET_VOLTAGE_L_P + 4.2 * abs_duty * CORRECTION_FACTOR_L_P) / getVoltage();
                setDutyMTU0(1.0 - duty_real);
                setDutyMTU4(1.0);
            } else {
                float duty_real = (- OFFSET_VOLTAGE_L_M + 4.2 * abs_duty * CORRECTION_FACTOR_L_M) / getVoltage();
                setDutyMTU0(1.0);
                setDutyMTU4(1.0 - duty_real);
            }
        }


       void setRawDuty_R(float duty) {
            if(duty == duty_R) return;
            ParameterManager& pm = ParameterManager::getInstance();
            float abs_duty = 0.0;
            abs_duty = constrain((ABS(duty)), 0.0, pm.duty_limit);

            if(std::isnan(duty) == true) {
                printfAsync("duty NaN error R\n");
                duty_R = 0.0;
            } else duty_R = SIGN(duty) * abs_duty;

            if (duty_R == 0.0) {
                setDutyTPU3(1.0);
                setDutyMTU3(1.0);
            } else if (duty_R > 0) {
                setDutyTPU3(1.0);
                setDutyMTU3(1.0 - abs_duty);

            } else {
                setDutyTPU3(1.0 - abs_duty);
                setDutyMTU3(1.0);
            }
        }

        void setRawDuty_L(float duty) {
            if(duty == duty_L) return;
            ParameterManager& pm = ParameterManager::getInstance();
            float abs_duty = 0.0;
            abs_duty = constrain(ABS(duty), 0.0, pm.duty_limit);

            if(std::isnan(duty) == true) {
                printfAsync("duty NaN error L\n");
                duty_L = 0.0;
            } //else duty_L = SIGN(duty) * abs_duty;
            duty_L = duty;
            if (duty_L == 0.0) {
                setDutyMTU0(1.0);
                setDutyMTU4(1.0);
            } else if (duty_L > 0) {
                setDutyMTU0(1.0 - abs_duty);
                setDutyMTU4(1.0);
            } else {
                setDutyMTU0(1.0);
                setDutyMTU4(1.0 - abs_duty);
            }
        }

        float getDuty_R() {
            return duty_R;
        }

        float getDuty_L() {
            return duty_L;
        }


        float K_T2K_E(float K_T_) {
            return (2.0f * PI) / 60.0f * K_T_;
        }

        Eigen::Vector2f transAccDuty(float a) {
            Eigen::Vector2f duty;
            float Vcc = getVoltage();
            ParameterManager& pm = ParameterManager::getInstance();


            float F_trans = pm.mass * a / 2.0f;
            float r = 0.5f * pm.dia_tire;
            float torque_L = F_trans * r / GEAR_RATIO;
            float torque_R = F_trans * r / GEAR_RATIO;
            float K_T_L = pm.K_T_left;
            float K_T_R = pm.K_T_right;
            float R_L = pm.circuit_res_left;
            float R_R = pm.circuit_res_right;


            duty(0) = (R_L * torque_L / K_T_L ) / Vcc;
            duty(1) = (R_R * torque_R / K_T_R ) / Vcc;
            return duty;
        }



        Eigen::Vector2f transBackEmfDuty(float v) {
            Eigen::Vector2f duty;
            ParameterManager& pm = ParameterManager::getInstance();
            float Vcc = getVoltage();
            float rpm = GEAR_RATIO * v * 60.0f /(PI * pm.dia_tire);
            float K_E_L = K_T2K_E(pm.K_T_left);
            float K_E_R = K_T2K_E(pm.K_T_right);

            duty(0) = (K_E_L * rpm) / Vcc;
            duty(1) = (K_E_R * rpm) / Vcc;
            return duty;
        }

        Eigen::Vector2f rotAccDuty(float alpha) {
            Eigen::Vector2f duty;
            ParameterManager& pm = ParameterManager::getInstance();

            float Vcc = getVoltage();
            float F_rot = (pm.inertia * DEG2RAD(alpha))/ pm.tread;
            float R_L = pm.circuit_res_left;
            float R_R = pm.circuit_res_right;
            float r = 0.5f * pm.dia_tire;
            float K_T_L = pm.K_T_left;
            float K_T_R = pm.K_T_right;

            float torque_L = - F_rot * r / GEAR_RATIO;
            float torque_R =   F_rot * r / GEAR_RATIO;

            duty(0) = (R_L * torque_L / K_T_L ) / Vcc;
            duty(1) = (R_R * torque_R / K_T_R ) / Vcc;
            return duty;
        }

        Eigen::Vector2f rotBackEmfDuty(float omega) {
            Eigen::Vector2f duty;
            ParameterManager& pm = ParameterManager::getInstance();
            float Vcc = getVoltage();
            float v_L = - pm.tread * PI * omega / 360.0f;
            float v_R =   pm.tread * PI * omega / 360.0f;
            float rpm_L = GEAR_RATIO * v_L * 60.0f /(PI * pm.dia_tire);
            float rpm_R = GEAR_RATIO * v_R * 60.0f /(PI * pm.dia_tire);
            float K_E_L = K_T2K_E(pm.K_T_left);
            float K_E_R = K_T2K_E(pm.K_T_right);
            duty(0) = (K_E_L * rpm_L) / Vcc;
            duty(1) = (K_E_R * rpm_R) / Vcc;
            return duty;

        }

        Eigen::Vector2f rotFrictionCompensationDuty(float omega) {
            ParameterManager& pm = ParameterManager::getInstance();
            Eigen::Vector2f duty;
            float Vcc = getVoltage();
            duty(0) = - SIGN(omega) * (pm.rot_friction_coef * omega + pm.rot_friction_offset) * 4.2f / Vcc;
            duty(1) =   SIGN(omega) * (pm.rot_friction_coef * omega + pm.rot_friction_offset) * 4.2f / Vcc;
            return duty;
        }


        Eigen::Vector2f transFrictionCompensationDuty(float v) {
            ParameterManager& pm = ParameterManager::getInstance();
            Eigen::Vector2f duty;
            float Vcc = getVoltage();
            duty(0) = SIGN(v) * (pm.trans_friction_coef * v + pm.trans_friction_offset) * 4.2f / Vcc;
            duty(1) = SIGN(v) * (pm.trans_friction_coef * v + pm.trans_friction_offset) * 4.2f / Vcc;
            return duty;
        }


        Eigen::Vector2f transFFWithParamDuty(float v, float a) {
            if(- 0.01 < v && v < 0.01) {
                Eigen::Vector2f duty;
                duty(0) = 0.0;
                duty(1) = 0.0;
                return duty;
            }
            Eigen::Vector2f duty_v;
            Eigen::Vector2f duty_a;            
            ParameterManager& pm = ParameterManager::getInstance();
            duty_v(0) = pm.ff_v_coef * v + pm.ff_v_offset;
            duty_v(1) = pm.ff_v_coef * v + pm.ff_v_offset;
            duty_a(0) = pm.ff_a_coef * a + pm.ff_a_offset;
            duty_a(1) = pm.ff_a_coef * a + pm.ff_a_offset;
            return duty_v + duty_a;
        }


        Eigen::Vector2f rotFFWithParamDuty(float rot_v, float rot_a) {
            if(ABS(rot_v) < 45.0f) {
                Eigen::Vector2f duty;
                duty(0) = 0.0;
                duty(1) = 0.0;
                return duty;
            }

            Eigen::Vector2f duty_rot_v;
            Eigen::Vector2f duty_rot_a;
            float Vcc = getVoltage();
            ParameterManager& pm = ParameterManager::getInstance();
            duty_rot_v(0) = - SIGN(rot_v) * (pm.ff_rot_v_coef * ABS(rot_v) + pm.ff_rot_v_offset) / Vcc;
            duty_rot_v(1) =   SIGN(rot_v) * (pm.ff_rot_v_coef * ABS(rot_v) + pm.ff_rot_v_offset) / Vcc;
            duty_rot_a(0) = - SIGN(rot_a) * (pm.ff_rot_a_coef * ABS(rot_a) + pm.ff_rot_a_offset) / Vcc;
            duty_rot_a(1) =   SIGN(rot_a) * (pm.ff_rot_a_coef * ABS(rot_a) + pm.ff_rot_a_offset) / Vcc;
            return duty_rot_v + duty_rot_a;
        }

        void debug_duty_l() {
            WheelOdometry &wodo = WheelOdometry::getInstance();
            printfSync("-----duty l------\n");
            for(int i=0;i<100;i++){
                LED_R_PIN = 1;
                LED_G_PIN = 0;
                LED_B_PIN = 0;
                float duty = (float)i*0.01;
                setRawDuty_L(duty);
                
                waitmsec(2000);
                float vol = getVoltage() * getDuty_L();
                printfSync("%d,%f, %f, %f, %f, %f\n",i,duty,getDuty_L(),getVoltage() , vol, wodo.v_L);
                LED_R_PIN = 0;
                LED_G_PIN = 0;
                LED_B_PIN = 0;
                setDuty(0.0, 0.0);
                waitmsec(1000);
            }
        }

        void debug_duty_r() {
            printfSync("-----duty r------\n");
            WheelOdometry &wodo = WheelOdometry::getInstance();
            
            for(int i=0;i<100;i++){
                LED_R_PIN = 1;
                LED_G_PIN = 0;
                LED_B_PIN = 0;
                setRawDuty_R(i*0.01);
                waitmsec(2000);
                float vol = getVoltage() * getDuty_R();
                printfSync("%d, %f, %f\n",i , vol, wodo.v_R);
                LED_R_PIN = 0;
                LED_G_PIN = 0;
                LED_B_PIN = 0;
                setDuty(0.0, 0.0);
                waitmsec(1000);
            }
        }



        void debug() {

            for(int i=0;i<30;i++){
                LED_R_PIN = 1;
                LED_G_PIN = 0;
                LED_B_PIN = 0;
                setRawDuty(i*0.01, i*0.01);
                waitmsec(5000);
                LED_R_PIN = 0;
                LED_G_PIN = 0;
                LED_B_PIN = 0;
                setDuty(0.0, 0.0);
                waitmsec(2000);
            }

            for(int i=0;i<30;i++){
                LED_R_PIN = 0;
                LED_G_PIN = 1;
                LED_B_PIN = 0;
                setDuty(-i*0.01, -i*0.01);
                waitmsec(5000);
                LED_R_PIN = 0;
                LED_G_PIN = 0;
                LED_B_PIN = 0;
                setDuty(0.0, 0.0);
                waitmsec(2000);
            }


            setDuty_L(0.5);
            waitmsec(4000);
            setDuty_L(0.0);
            waitmsec(2000);
            setDuty_L(-0.5);
            waitmsec(4000);
            setDuty_L(0.0);
            waitmsec(2000);

            setDuty_R(0.5);
            waitmsec(4000);
            setDuty_R(0.0);
            waitmsec(2000);

            setDuty_R(-0.5);
            waitmsec(4000);
            setDuty_R(0.0);
            waitmsec(2000);

            setDuty(0.4, 0.4);
            waitmsec(500);
            setDuty(0.0, 0.0);
            waitmsec(4000);
            setDuty(0.5, 0.5);
            waitmsec(500);
            setDuty(0.0, 0.0);
            waitmsec(4000);
            setDuty(0.6, 0.6);
            waitmsec(500);
            setDuty(0.0, 0.0);
            waitmsec(4000);

            setDuty(-0.4, -0.4);
            waitmsec(500);
            setDuty(-0.0, -0.0);
            waitmsec(500);
            setDuty(-0.5, -0.5);
            waitmsec(500);
            setDuty(0.0, 0.0);
            waitmsec(4000);
            setDuty(-0.6, -0.6);
            waitmsec(500);
            setDuty(0.0, 0.0);
            waitmsec(500);

            setDuty(0.4, -0.4);
            waitmsec(1000);
            setDuty(0.0, -0.0);
            waitmsec(4000);
            setDuty(0.5, -0.5);
            waitmsec(1000);
            setDuty(0.0, 0.0);
            waitmsec(4000);
            setDuty(0.6, -0.6);
            waitmsec(1000);
            setDuty(0.0, 0.0);
            waitmsec(4000);

            setDuty(-0.4, 0.4);
            waitmsec(1000);
            setDuty(0.0, 0.0);
            waitmsec(4000);
            setDuty(-0.5, 0.5);
            waitmsec(1000);
            setDuty(0.0, 0.0);
            waitmsec(4000);
            setDuty(-0.6, 0.6);
            waitmsec(1000);
            setDuty(0.0, 0.0);
            waitmsec(4000);

        }

      private:
        float duty_R;
        float duty_L;

        const float PI = 3.1415926535;
        const float TIRE_GEAR_NUM = 41.0;
        const float PINION_GEAR_NUM = 9.0;
        const float GEAR_RATIO = TIRE_GEAR_NUM / PINION_GEAR_NUM;

        const float OFFSET_VOLTAGE_L_M = -0.08036225;
        const float OFFSET_VOLTAGE_L_P = 0.088720018;
        const float OFFSET_VOLTAGE_R_M = -0.07486831;
        const float OFFSET_VOLTAGE_R_P = 0.093748036;

        const float CORRECTION_FACTOR_L_M = 1.0;
        const float CORRECTION_FACTOR_L_P = 0.984144854;
        const float CORRECTION_FACTOR_R_M = 0.980893308;
        const float CORRECTION_FACTOR_R_P = 0.988689915;


        float getVoltage() {
            return BatVoltageMonitor::getInstance().bat_vol;
        }

    };

}
