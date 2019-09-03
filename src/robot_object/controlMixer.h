#pragma once

#include <math.h>
#include "trajectory.h"
#include "pidController.h"
#include "wallController.h"
#include "positionEstimator.h"
#include "powerTransmission.h"
#include "parameterManager.h"
#include "wheelOdometry.h"
#include "ICM20602.h"
#include <Eigen/Core>
#include "batVoltageMonitor.h"
#include "fcLed.h"

namespace umouse {

    class ControlMixer {
    public:
        VelocityTypePidfController v_pidf;
        VelocityTypePidfController ang_v_pidf;
        VelocityTypePidfController local_x_pidf;
        VelocityTypePidfController local_y_pidf;
        VelocityTypePidfController r_pidf;
        AngPidfController ang_pidf;

        WallPidfController wall_pidf;
        const float DELTA_T = 0.0005;

        float target_x_dd;
        float target_y_dd;

        float target_trans_v;
        float target_trans_a;

        float target_rot_v_pre;
        float target_rot_v;
        float target_rot_a;

        Eigen::Vector2f duty;

        ControlMixer() {
            target_x_dd = 0.0f;
            target_y_dd = 0.0f;

            target_trans_v = 0.0f;
            target_trans_a = 0.0f;

            target_rot_v_pre = 0.0f;
            target_rot_v = 0.0f;
            target_rot_a = 0.0f;

            local_x_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
            local_y_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
            v_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
            ang_v_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
            ang_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);

            error_sec = 0.0f;
            error_limit_sec = 0.2f; // 秒
            ang_v_error_th = 270.0f;
            v_error_th = 0.5;

            duty = Eigen::Vector2f::Zero();
        }

        void reset() {
            target_x_dd = 0.0f;
            target_y_dd = 0.0f;

            target_trans_v = 0.0f;
            target_trans_a = 0.0f;

            target_rot_v_pre = 0.0f;
            target_rot_v = 0.0f;
            target_rot_a = 0.0f;
            local_x_pidf.reset();
            local_y_pidf.reset();
            v_pidf.reset();
            ang_v_pidf.reset();
            ang_pidf.reset();
            wall_pidf.reset();

            error_sec = 0.0f;

            duty(0) = 0.0f;
            duty(1) = 0.0f;
        }

        Eigen::Vector2f getDuty() {
            return duty;
        }

        bool isOutOfControl() {
            WheelOdometry &wodo = WheelOdometry::getInstance();
            ICM20602 &icm = ICM20602::getInstance();
            if( ABS(v_pidf.e_k0) > v_error_th ||
                    ABS(ang_v_pidf.e_k0) > ang_v_error_th ||
                    ABS(icm.omega_f[2] - wodo. getAng_v()) > ang_v_error_th ) error_sec += DELTA_T;
            else error_sec = 0.0f;

            if(error_sec > error_limit_sec) return true;
            else return false;

        }

        void update( const BaseTrajectory& traj, const PositionEstimator& esti, bool isRWall, bool isLWall) {
            ParameterManager &pm = ParameterManager::getInstance();


            setPIDF(traj);
            local_x_pidf.update(traj.x, esti.x);
            local_y_pidf.update(traj.y, esti.y);

            if(traj.motion_type == EMotionType::STOP) FcLed::getInstance().turn(1,0,0);
            else FcLed::getInstance().turn(0,0,0);

            if(ABS(traj.v) < 0.11 ||
                    traj.motion_type == EMotionType::STOP ||
                    traj.motion_type == EMotionType::SPINTURN ||
                    true) {
                target_trans_v = traj.v;
                target_trans_a = traj.a;
                target_rot_v = traj.ang_v;
                target_rot_a = traj.ang_a;
                local_x_pidf.reset();
                local_y_pidf.reset();
            }
            else {
                target_x_dd = traj.x_dd + local_x_pidf.getControlVal();
                target_y_dd = traj.y_dd + local_y_pidf.getControlVal();

                float traj_ang_rad = DEG2RAD(traj.ang);
                target_trans_a = target_x_dd * cosf(traj_ang_rad) + target_y_dd * sinf(traj_ang_rad);
                // constrainLのつけ方がおかしい。
                target_trans_v += target_trans_a * DELTA_T;

                target_rot_v = RAD2DEG((target_y_dd * cosf(traj_ang_rad) - target_x_dd * sinf(traj_ang_rad))/ target_trans_v );
                target_rot_a = (target_rot_v - target_rot_v_pre) / DELTA_T;
                target_rot_v_pre = target_rot_v;
            }

            if(traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER && traj.v > 0.1 //&&
//                UMouse::getInstance().isWallControllable();
               ) {
                wall_pidf.update(WallSensor::getInstance(), isRWall, isLWall);
                target_rot_v += wall_pidf.getControlVal();
            }
            else {
                wall_pidf.reset();
            }
/*
            if(traj.motion_type == EMotionType::DIAGONAL_CENTER){
                float diag_error = WallSensor::getInstance().ahead_l()
                if( > )target_rot_v += 
            }
*/

            if(traj.motion_type == EMotionType::STOP ||
               traj.motion_type == EMotionType::SPINTURN ||
               traj.motion_type == EMotionType::CURVE ||
               traj.motion_type == EMotionType::DIAGONAL ||
               traj.motion_type == EMotionType::DIAGONAL_CENTER ||
               traj.motion_type == EMotionType::STRAIGHT ||
               (traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER
                && 
                (WallSensor::getInstance().isRight_for_ctrl() == false &&
                WallSensor::getInstance().isLeft_for_ctrl() == false)
               )
               //traj.v < 0.11 ||
               //1
            ) {                
                if(traj.motion_type == EMotionType::DIAGONAL_CENTER){
                    float target_ang = traj.ang;
                    if (WallSensor::getInstance().ahead_l() > pm.wall_diagonal_ahead_l_threshold) target_ang -= pm.wall_diagonal_avoid_add_ang;
                    else if (WallSensor::getInstance().ahead_r() > pm.wall_diagonal_ahead_r_threshold) target_ang += pm.wall_diagonal_avoid_add_ang;
                    ang_pidf.update(target_ang, esti.ang);
                }

                ang_pidf.update(traj.ang, esti.ang);
                target_rot_v += ang_pidf.getControlVal();
            }
            else {
                ang_pidf.reset();
            }

            if( traj.motion_type != motion_type_pre){
                ang_v_pidf.reset();
                ang_pidf.reset();
                printfAsync("reset!!\n");
            }
            



            v_pidf.update(target_trans_v, esti.v);
            ang_v_pidf.update(target_rot_v, esti.ang_v);

            PowerTransmission &pt = PowerTransmission::getInstance();
            duty(0) = 0.0f;
            duty(1) = 0.0f;


            Eigen::Vector2f duty_v_FF(0.0f, 0.0f);
            duty_v_FF += pt.transAccDuty(target_trans_a);
            duty_v_FF += pt.transBackEmfDuty(target_trans_v);
            duty_v_FF += pt.transFrictionCompensationDuty(target_trans_v);
            if(pm.trans_v_FF_enable == true) duty += duty_v_FF;

            Eigen::Vector2f duty_ang_v_FF(0.0f, 0.0f);
            duty_ang_v_FF += pt.rotAccDuty(target_rot_a);
            duty_ang_v_FF += pt.rotBackEmfDuty(target_rot_v);
            duty_ang_v_FF += pt.rotFrictionCompensationDuty(target_rot_v);
            if(pm.rot_v_FF_enable == true) duty+= duty_ang_v_FF;

            float voltage = BatVoltageMonitor::getInstance().bat_vol;
            float duty_v_satuation = pm.trans_v_satuation_FF_multiplier * ABS(duty_v_FF(0)) + pm.trans_v_satuation_offset_duty * 4.2f / voltage;
            float duty_ang_v_satuation = pm.rot_v_satuation_FF_multiplier * ABS(duty_ang_v_FF(0)) + pm.rot_v_satuation_offset_duty * 4.2f / voltage;

            if(pm.trans_v_PIDF_satuation_enable == true) v_pidf.setSatuation(duty_v_satuation);
            if(pm.rot_v_PIDF_satuation_enable == true ) ang_v_pidf.setSatuation(duty_ang_v_satuation);

            duty(0) += v_pidf.getControlVal() - ang_v_pidf.getControlVal();
            duty(1) += v_pidf.getControlVal() + ang_v_pidf.getControlVal();
            motion_type_pre = traj.motion_type;
        }

    private:
        float error_sec;
        float error_limit_sec;
        float ang_v_error_th;
        float v_error_th;
        EMotionType motion_type_pre;

        void setPIDF(const BaseTrajectory& traj) {
            ParameterManager &pm = ParameterManager::getInstance();

            // 速度PIDFゲイン設定
            if(traj.motion_type == EMotionType::STOP || traj.motion_type == EMotionType::SPINTURN) {
                v_pidf.set(pm.trans_v_spin_P, pm.trans_v_spin_I, pm.trans_v_spin_D, pm.trans_v_spin_F);
            }
            else if(traj.motion_type == EMotionType::STRAIGHT || traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                    traj.motion_type == EMotionType::DIAGONAL || traj.motion_type == EMotionType::DIAGONAL_CENTER
            ){
                v_pidf.set(pm.trans_v_P, pm.trans_v_I, pm.trans_v_D, pm.trans_v_F);
            }
            else if(traj.motion_type == EMotionType::CURVE ){
                v_pidf.set(pm.trans_v_slalom_P, pm.trans_v_slalom_I, pm.trans_v_slalom_D, pm.trans_v_slalom_F);
            }

            // 角速度PIDFゲイン設定
            if(traj.motion_type == EMotionType::STOP || traj.motion_type == EMotionType::SPINTURN) {
                ang_v_pidf.set(pm.rot_v_spin_P, pm.rot_v_spin_I, pm.rot_v_spin_D, pm.rot_v_spin_F);
            }
            else if(traj.motion_type == EMotionType::STRAIGHT || traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                    traj.motion_type == EMotionType::DIAGONAL || traj.motion_type == EMotionType::DIAGONAL_CENTER
            ){
                ang_v_pidf.set(pm.rot_v_P, pm.rot_v_I, pm.rot_v_D, pm.rot_v_F);
            }
            else if(traj.motion_type == EMotionType::CURVE ){
                ang_v_pidf.set(pm.rot_v_slalom_P, pm.rot_v_slalom_I, pm.rot_v_slalom_D, pm.rot_v_slalom_F);
            }

            // 角度PIDFゲイン設定
            if(traj.motion_type == EMotionType::STOP || traj.motion_type == EMotionType::SPINTURN) {
                ang_pidf.set(pm.rot_x_spin_P, pm.rot_x_spin_I, pm.rot_x_spin_D, pm.rot_x_spin_F);
            }
            else{ //if(traj.motion_type == EMotionType::CURVE){
                ang_pidf.set(pm.rot_x_slalom_P, pm.rot_x_slalom_I, pm.rot_x_slalom_D, pm.rot_x_slalom_F);
            }

            // 速度PIDFサチュレーション設定
            if(traj.motion_type == EMotionType::STOP || traj.motion_type == EMotionType::SPINTURN) {
                v_pidf.setSatuationEnable(pm.trans_v_PIDF_satuation_enable);
            }
            else {
                v_pidf.setSatuationEnable(false);
            }

            // 速度PIDFイネーブル設定
            v_pidf.setEnable(pm.trans_v_PIDF_enable);


            //角速度サチュレーション設定
            if(traj.motion_type == EMotionType::STOP || traj.motion_type == EMotionType::SPINTURN) {
                ang_v_pidf.setSatuationEnable(false);
            }
            else {
                ang_v_pidf.setSatuationEnable(pm.rot_v_PIDF_satuation_enable);
            }


            ang_v_pidf.setEnable(pm.rot_v_PIDF_enable);

            local_x_pidf.setEnable(pm.pos_PIDF_enable);
            local_x_pidf.setSatuationEnable(pm.pos_PIDF_satuation_enable);
            local_x_pidf.setSatuation(pm.pos_satuation_xy_dd);

            local_y_pidf.setEnable(pm.pos_PIDF_enable);
            local_y_pidf.setSatuationEnable(pm.pos_PIDF_satuation_enable);
            local_y_pidf.setSatuation(pm.pos_satuation_xy_dd);

            if(traj.motion_type == EMotionType::CURVE){
                local_x_pidf.set(pm.pos_slalom_P, pm.pos_slalom_I, pm.pos_slalom_D, pm.pos_slalom_F);
                local_y_pidf.set(pm.pos_slalom_P, pm.pos_slalom_I, pm.pos_slalom_D, pm.pos_slalom_F);
            }else {
                local_x_pidf.set(pm.pos_P, pm.pos_I, pm.pos_D, pm.pos_F);
                local_y_pidf.set(pm.pos_P, pm.pos_I, pm.pos_D, pm.pos_F);
            }

            //ang_pidf.set(pm.rot_x_spin_P, pm.rot_x_spin_I, pm.rot_x_spin_D, pm.rot_x_spin_F);
            ang_pidf.setEnable(true);
            ang_pidf.setSatuationEnable(false);

            wall_pidf.set(pm.wall_P, pm.wall_I, pm. wall_D, pm.wall_F);
            wall_pidf.setEnable(pm.wall_PIDF_enable);

        }

    };

}
