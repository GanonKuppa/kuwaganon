#pragma once

#include "pwm.h"
#include "parameterManager.h"
#include "communication.h"
#include "timer.h"
#include "batVoltageMonitor.h"
#include <cfloat>
#include <math.h>
#include "myUtil.h"

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

        void setDuty_R(float duty) {
            ParameterManager& pm = ParameterManager::getInstance();
            float abs_duty = constrain(ABS(duty), 0.0, pm.duty_limit);
            if(std::isnan(duty) == true) {
                printfAsync("duty NaN error R\n");
                duty_R = 0.0;
            } else duty_R = SIGN(duty) * abs_duty;

            if (duty == 0.0) {

            } else if (duty > 0) {
                PORT2.PODR.BIT.B0 = 0;
            } else {
                PORT2.PODR.BIT.B0 = 1;
            }
            setDutyMTU3(ABS(duty));
        }

        void setDuty_L(float duty) {
            ParameterManager& pm = ParameterManager::getInstance();
            float abs_duty = constrain(ABS(duty), 0.0, pm.duty_limit);
            if(std::isnan(duty) == true) {
                printfAsync("duty NaN error L\n");
                duty_L = 0.0;
            } else duty_L = SIGN(duty) * abs_duty;

            if (duty == 0.0) {

            } else if (duty > 0) {
                PORT1.PODR.BIT.B3 = 1;
            } else {
                PORT1.PODR.BIT.B3 = 0;
            }
            setDutyMTU4(ABS(duty));
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
            float Vcc = getVoltage();
            ParameterManager& pm = ParameterManager::getInstance();
            duty_v(0) = SIGN(v) * (pm.ff_v_coef * ABS(v) + pm.ff_v_offset) / Vcc;
            duty_v(1) = SIGN(v) * (pm.ff_v_coef * ABS(v) + pm.ff_v_offset) / Vcc;
            duty_a(0) =           (pm.ff_a_coef * a + pm.ff_a_offset) / Vcc;
            duty_a(1) =           (pm.ff_a_coef * a + pm.ff_a_offset) / Vcc;
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


        void debug() {

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

        float getVoltage() {
            return BatVoltageMonitor::getInstance().bat_vol;
        }

    };

}
