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
        VelocityTypePidfController pos_pidf;
        AngPidfController ang_pidf;
        WallPidfController wall_pidf;

        const float DELTA_T = 0.0005;

        float target_trans_v;
        float target_trans_a;

        float target_rot_v_pre;
        float target_rot_x;
        float target_rot_v;
        float target_rot_a;

        float v_back_emf_FF;
        float a_acc_FF;
        float v_fric_FF;
        float ang_v_back_emf_FF;
        float ang_a_acc_FF;
        float ang_v_fric_FF;

        Eigen::Vector2f duty;

        ControlMixer() {

            target_trans_v = 0.0f;
            target_trans_a = 0.0f;

            target_rot_v_pre = 0.0f;
            target_rot_x = 0.0f;
            target_rot_v = 0.0f;
            target_rot_a = 0.0f;

            v_back_emf_FF = 0.0f;
            a_acc_FF = 0.0f;
            v_fric_FF = 0.0f;
            ang_v_back_emf_FF = 0.0f;
            ang_a_acc_FF = 0.0f;
            ang_v_fric_FF = 0.0f;

            pos_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
            v_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
            ang_v_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
            ang_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);

            error_sec = 0.0f;
            error_limit_sec = 0.2f; // 秒
            ang_v_error_th = 180.0f;
            v_error_th = 0.3;

            duty = Eigen::Vector2f::Zero();
        }

        void reset() {
            target_trans_v = 0.0f;
            target_trans_a = 0.0f;

            target_rot_v_pre = 0.0f;
            target_rot_x = 0.0f;
            target_rot_v = 0.0f;
            target_rot_a = 0.0f;

            v_back_emf_FF = 0.0f;
            a_acc_FF = 0.0f;
            v_fric_FF = 0.0f;
            ang_v_back_emf_FF = 0.0f;
            ang_a_acc_FF = 0.0f;
            ang_v_fric_FF = 0.0f;

            pos_pidf.reset();
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
            WheelOdometry& wodo = WheelOdometry::getInstance();
            ICM20602& icm = ICM20602::getInstance();
            if( ABS(v_pidf.e_k0) > v_error_th ||
                    ABS(ang_v_pidf.e_k0) > ang_v_error_th ||
                    ABS(icm.omega_f[2] - wodo. getAng_v()) > ang_v_error_th ||
                    ABS(ang_pidf.e_k0) > 5.0f ||
                    ABS(v_pidf.e_k0) > ABS(v_error_th)
                    ) error_sec += DELTA_T;
            else error_sec = 0.0f;
            error_sec = 0.0f;
            if(error_sec > error_limit_sec) return true;
            else return false;

        }

        void update(BaseTrajectory& traj, PositionEstimator& esti, bool isRWall, bool isLWall) {
            ParameterManager& pm = ParameterManager::getInstance();

            setPIDF(traj);
            target_trans_v = traj.v;
            target_trans_a = traj.a;

            target_rot_x = traj.ang;
            target_rot_v = traj.ang_v;
            target_rot_a = traj.ang_a;


            if(traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER) {
                wall_pidf.update(WallSensor::getInstance(), isRWall, isLWall);
            } else {
                wall_pidf.reset();
            }

            if( (isRWall || isLWall) &&
                    (WallSensor::getInstance().isRight_for_ctrl() ||
                     WallSensor::getInstance().isLeft_for_ctrl()) &&
                    traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER &&
                    pm.wall_PIDF_enable == true
                    //fabs(esti.calcWallCenterOffset()) > 0.001
              ) {

                wall_pidf.update(WallSensor::getInstance(), isRWall, isLWall);
                target_rot_x += wall_pidf.getControlVal();
                pos_pidf.reset();
            } else if( (traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                        traj.motion_type == EMotionType::STRAIGHT &&
                        pm.pos_PIDF_enable == true) &&
                       fabs(esti.calcWallCenterOffset()) > pm.rot_x_wall_abs_center_offset
                     ) {
                pos_pidf.update(0.0, - esti.calcWallCenterOffset());
                target_rot_x += pos_pidf.getControlVal();
                wall_pidf.reset();
            } else if(traj.motion_type == EMotionType::DIAGONAL_CENTER &&
                      pm.pos_PIDF_enable == true
                     ) {
                pos_pidf.update(0.0, - esti.calcDiagWallCenterOffset());
                target_rot_x += pos_pidf.getControlVal();
            }

            // 直進時の衝突回避
            if(traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER || 
               traj.motion_type == EMotionType::STRAIGHT) {
                if (WallSensor::getInstance().ahead_l() > pm.wall_diagonal_ahead_l_threshold &&
                    WallSensor::getInstance().ahead_r() < pm.wall_diagonal_ahead_r_threshold) {
                    target_rot_x -= pm.wall_diagonal_avoid_add_ang;
                } else if (WallSensor::getInstance().ahead_r() > pm.wall_diagonal_ahead_r_threshold &&
                           WallSensor::getInstance().ahead_r() < pm.wall_diagonal_ahead_l_threshold) {
                    target_rot_x += pm.wall_diagonal_avoid_add_ang;
                }
            }
            // 斜め直進時の衝突回避
            if(traj.motion_type == EMotionType::DIAGONAL_CENTER) {
                if (WallSensor::getInstance().ahead_l() > pm.wall_diagonal_ahead_l_threshold) {
                    target_rot_x -= pm.wall_diagonal_avoid_add_ang;
                } else if (WallSensor::getInstance().ahead_r() > pm.wall_diagonal_ahead_r_threshold) {
                    target_rot_x += pm.wall_diagonal_avoid_add_ang;
                }
            }
            // 斜めターン時の衝突回避
            if(traj.motion_type == EMotionType::CURVE &&
                    (( esti.getAng() > 30.0f  && esti.getAng() < 60.0f  ) ||
                     ( esti.getAng() > 120.0f && esti.getAng() < 150.0f ) ||
                     ( esti.getAng() > 210.0f && esti.getAng() < 240.0f ) ||
                     ( esti.getAng() > 300.0f && esti.getAng() < 330.0f )) &&

                    (traj.turn_type == turn_type_e::TURN_D_90 ||
                     traj.turn_type == turn_type_e::TURN_D2S_135 ||
                     traj.turn_type == turn_type_e::TURN_S2D_135 ||
                     traj.turn_type == turn_type_e::TURN_D2S_45 ||
                     traj.turn_type == turn_type_e::TURN_S2D_45)


              ) {
                if (WallSensor::getInstance().ahead_l() > 150
                        && WallSensor::getInstance().ahead_l() < pm.wall_diagonal_ahead_l_threshold) {
                    target_rot_x -= pm.wall_diagonal_avoid_add_ang;
                } else if (WallSensor::getInstance().ahead_r() > 150 &&
                           WallSensor::getInstance().ahead_r() < pm.wall_diagonal_ahead_l_threshold
                          ) {
                    target_rot_x += pm.wall_diagonal_avoid_add_ang;
                }
            }


            ang_pidf.update(target_rot_x, esti.getAng());
            target_rot_v += ang_pidf.getControlVal();
            //target_rot_v += (target_rot_x - traj.ang)/0.05; // 1度を50msecで回る角速度を足す


            if( (traj.motion_type != motion_type_pre &&
                    motion_type_pre == EMotionType::STOP)
                    || traj.motion_type == EMotionType::DIRECT_DUTY_SET) {
                //ang_v_pidf.reset();
                //ang_pidf.reset();
                //v_pidf.reset();
                wall_pidf.reset();
                pos_pidf.reset();
            }

            WheelOdometry& wodo = WheelOdometry::getInstance();
            v_pidf.update(target_trans_v, esti.getV());
            ang_v_pidf.update(target_rot_v, esti.getAngV());

            PowerTransmission& pt = PowerTransmission::getInstance();
            duty(0) = 0.0f;
            duty(1) = 0.0f;


            Eigen::Vector2f duty_v_FF(0.0f, 0.0f);
            v_back_emf_FF = pt.transBackEmfDuty(target_trans_v)(0);
            a_acc_FF = pt.transAccDuty(target_trans_a)(0);
            v_fric_FF = pt.transFrictionCompensationDuty(target_trans_v)(0);
            /*
                        duty_v_FF += pt.transAccDuty(target_trans_a);
                        duty_v_FF += pt.transBackEmfDuty(target_trans_v);
                        duty_v_FF += pt.transFrictionCompensationDuty(target_trans_v);
            */
            duty_v_FF += pt.transFFWithParamDuty(target_trans_v, target_trans_a);
            if(pm.trans_v_FF_enable == true) duty += duty_v_FF;

            Eigen::Vector2f duty_ang_v_FF(0.0f, 0.0f);
            ang_v_back_emf_FF = pt.rotBackEmfDuty(target_rot_v)(0);
            ang_a_acc_FF = pt.rotAccDuty(target_rot_a)(0);
            ang_v_fric_FF = pt.rotFrictionCompensationDuty(target_rot_v)(0);
            /*
                        duty_ang_v_FF += pt.rotAccDuty(target_rot_a);
                        duty_ang_v_FF += pt.rotBackEmfDuty(target_rot_v);
                        duty_ang_v_FF += pt.rotFrictionCompensationDuty(target_rot_v);
            */
            duty_ang_v_FF += pt.rotFFWithParamDuty(target_rot_v, target_rot_a);
            if(pm.rot_v_FF_enable == true) duty += duty_ang_v_FF;

            float voltage = BatVoltageMonitor::getInstance().bat_vol;
            float duty_v_saturation = (pm.trans_v_saturation_FF_multiplier * ABS(duty_v_FF(0)) + pm.trans_v_saturation_offset_duty);
            float duty_ang_v_saturation = (pm.rot_v_saturation_FF_multiplier * ABS(duty_ang_v_FF(0)) + pm.rot_v_saturation_offset_duty);

            if(pm.trans_v_PIDF_saturation_enable == true) v_pidf.setSaturation(duty_v_saturation);
            if(pm.rot_v_PIDF_saturation_enable == true ) ang_v_pidf.setSaturation(duty_ang_v_saturation);

            duty(0) += (v_pidf.getControlVal() - ang_v_pidf.getControlVal());
            duty(1) += (v_pidf.getControlVal() + ang_v_pidf.getControlVal());
            // duty飽和時には回転系制御を優先
            if(duty(0) > 1.0 || duty(1) > 1.0) {
                float duty_overflow = 0.0f;
                if(duty(0) > duty(1) ) {
                    duty_overflow = duty(0) - 1.0f;
                } else {
                    duty_overflow = duty(1) - 1.0f;
                }
                duty(0) -= duty_overflow;
                duty(1) -= duty_overflow;

            }

            motion_type_pre = traj.motion_type;
        }

      private:
        float error_sec;
        float error_limit_sec;
        float ang_v_error_th;
        float v_error_th;
        EMotionType motion_type_pre;

        void setPIDF(const BaseTrajectory& traj) {
            ParameterManager& pm = ParameterManager::getInstance();

            // 速度PIDFゲイン設定
            if(traj.motion_type == EMotionType::STOP || traj.motion_type == EMotionType::SPINTURN) {
                v_pidf.set(pm.trans_v_spin_P, pm.trans_v_spin_I, pm.trans_v_spin_D, pm.trans_v_spin_F);
            } else if(traj.motion_type == EMotionType::STRAIGHT || traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                      traj.motion_type == EMotionType::DIAGONAL || traj.motion_type == EMotionType::DIAGONAL_CENTER
                     ) {
                v_pidf.set(pm.trans_v_P, pm.trans_v_I, pm.trans_v_D, pm.trans_v_F);
            } else if(traj.motion_type == EMotionType::CURVE ) {
                v_pidf.set(pm.trans_v_slalom_P, pm.trans_v_slalom_I, pm.trans_v_slalom_D, pm.trans_v_slalom_F);
            }

            // 角速度PIDFゲイン設定
            if(traj.motion_type == EMotionType::STOP || traj.motion_type == EMotionType::SPINTURN) {
                ang_v_pidf.set(pm.rot_v_spin_P, pm.rot_v_spin_I, pm.rot_v_spin_D, pm.rot_v_spin_F);
            } else if(traj.motion_type == EMotionType::STRAIGHT || traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                      traj.motion_type == EMotionType::DIAGONAL || traj.motion_type == EMotionType::DIAGONAL_CENTER
                     ) {
                ang_v_pidf.set(pm.rot_v_P, pm.rot_v_I, pm.rot_v_D, pm.rot_v_F);
            } else if(traj.motion_type == EMotionType::CURVE ) {
                ang_v_pidf.set(pm.rot_v_slalom_P, pm.rot_v_slalom_I, pm.rot_v_slalom_D, pm.rot_v_slalom_F);
            }

            // 角度PIDFゲイン設定
            if(traj.motion_type == EMotionType::STOP || traj.motion_type == EMotionType::SPINTURN) {
                ang_pidf.set(pm.rot_x_spin_P, pm.rot_x_spin_I, pm.rot_x_spin_D, pm.rot_x_spin_F);
            } else if(traj.motion_type == EMotionType::STRAIGHT || traj.motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                      traj.motion_type == EMotionType::DIAGONAL || traj.motion_type == EMotionType::DIAGONAL_CENTER) {
                ang_pidf.set(pm.rot_x_P, pm.rot_x_I, pm.rot_x_D, pm.rot_x_F);
            } else if(traj.motion_type == EMotionType::CURVE) {
                ang_pidf.set(pm.rot_x_slalom_P, pm.rot_x_slalom_I, pm.rot_x_slalom_D, pm.rot_x_slalom_F);
            }

            // 速度PIDFサチュレーション設定
            if(traj.motion_type == EMotionType::STOP || traj.motion_type == EMotionType::SPINTURN) {
                //v_pidf.setSaturationEnable(false);
                v_pidf.setSaturationEnable(pm.trans_v_PIDF_saturation_enable);
            } else {
                v_pidf.setSaturationEnable(pm.trans_v_PIDF_saturation_enable);
            }

            // 速度PIDFイネーブル設定
            v_pidf.setEnable(pm.trans_v_PIDF_enable);


            // 角速度サチュレーション設定
            if(traj.motion_type == EMotionType::STOP || traj.motion_type == EMotionType::SPINTURN) {
                //ang_v_pidf.setSaturationEnable(false);
                ang_v_pidf.setSaturationEnable(pm.rot_v_PIDF_saturation_enable);
            } else {
                ang_v_pidf.setSaturationEnable(pm.rot_v_PIDF_saturation_enable);
            }

            // 角速度PIDFイネーブル設定
            ang_v_pidf.setEnable(pm.rot_v_PIDF_enable);

            // 位置PIDF設定
            pos_pidf.setEnable(pm.pos_PIDF_enable);
            pos_pidf.setSaturationEnable(true);
            pos_pidf.setSaturation(pm.target_rot_x_saturation);
            pos_pidf.set(pm.pos_P, pm.pos_I, pm.pos_D, pm.pos_F);

            // 角度PIDF設定
            ang_pidf.setEnable(true);
            ang_pidf.setSaturationEnable(true);

            // 壁PIDF設定
            wall_pidf.set(pm.wall_P, pm.wall_I, pm. wall_D, pm.wall_F);
            wall_pidf.setEnable(pm.wall_PIDF_enable);
            wall_pidf.setSaturation(pm.target_rot_x_saturation);

            // 積分サチュレーション設定
            v_pidf.setIntegralSaturation(pm.trans_v_PIDF_integral_saturation);
            ang_v_pidf.setIntegralSaturation(pm.rot_v_PIDF_integral_saturation);
            ang_pidf.setIntegralSaturation(pm.rot_x_PIDF_integral_saturation);
            pos_pidf.setIntegralSaturation(pm.pos_PIDF_integral_saturation);
            wall_pidf.setIntegralSaturation(pm.wall_PIDF_integral_saturation);
        }

    };

}
