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
        PidfController v_pidf;
        PidfController ang_v_pidf;        
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
            
            v_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
            ang_v_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
            ang_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);

            error_sec = 0.0f;
            error_limit_sec = 0.2f; // 秒
            ang_v_error_th = 180.0f;
            v_error_th = 1.0;

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
            if(error_sec > error_limit_sec) return true;
            else return false;

        }

        void update(float v, float ang_v, PositionEstimator& esti, bool wallCenter){
            ParameterManager& pm = ParameterManager::getInstance();
            ang_v_pidf.set(pm.rot_v_P, pm.rot_v_I, pm.rot_v_D, pm.rot_v_F);
            v_pidf.set(pm.trans_v_P, pm.trans_v_I, pm.trans_v_D, pm.trans_v_F);

            v_pidf.update(target_trans_v, esti.getV());
            ang_v_pidf.update(target_rot_v, esti.getAngV());


            duty(0) = (v_pidf.getControlVal() - ang_v_pidf.getControlVal());
            duty(1) = (v_pidf.getControlVal() + ang_v_pidf.getControlVal());
            return;
        }



        void update(BaseTrajectory& traj, PositionEstimator& esti, bool isRWall, bool isLWall, bool isPillar) {
            ParameterManager& pm = ParameterManager::getInstance();
            EMotionType motion_type = traj.motion_type;
            turn_type_e turn_type = traj.turn_type;
            setPIDF(motion_type);
            target_trans_v = traj.v;
            target_trans_a = traj.a;

            target_rot_x = traj.ang;
            target_rot_v = traj.ang_v;
            target_rot_a = traj.ang_a;


            if(motion_type == EMotionType::STRAIGHT_WALL_CENTER) {
                wall_pidf.update(WallSensor::getInstance(), isRWall, isLWall, isPillar);
            } else {
                wall_pidf.reset();
            }

            if( (isRWall || isLWall) &&
                 motion_type == EMotionType::STRAIGHT_WALL_CENTER &&
                 pm.wall_PIDF_enable == true
              ) {
                wall_pidf.update(WallSensor::getInstance(), isRWall, isLWall, isPillar);
                // 壁制御量は曲率とみなし, 速度をかけることで角速度に変換
                float v_now = constrainL(esti.getV(), 0.1f);
                target_rot_v += v_now *  wall_pidf.getControlVal();
            }
/*
            // 直進時の衝突回避
            if(motion_type == EMotionType::STRAIGHT_WALL_CENTER || 
               motion_type == EMotionType::STRAIGHT) {
                if (WallSensor::getInstance().ahead_l() > pm.wall_diagonal_ahead_l_threshold &&
                    WallSensor::getInstance().ahead_r() < pm.wall_diagonal_ahead_r_threshold) {
                    target_rot_x -= pm.wall_diagonal_avoid_add_ang;
                } else if (WallSensor::getInstance().ahead_r() > pm.wall_diagonal_ahead_r_threshold &&
                           WallSensor::getInstance().ahead_l() < pm.wall_diagonal_ahead_l_threshold) {
                    target_rot_x += pm.wall_diagonal_avoid_add_ang;
                }
            }
*/
            // 斜め直進時の衝突回避
            if(motion_type == EMotionType::DIAGONAL_CENTER) {
                float v_now = constrainL(esti.getV(), 0.1f);
                if (WallSensor::getInstance().ahead_l() < pm.wall_diagonal_ahead_l_threshold) {
                    target_rot_v -= v_now * pm.wall_diagonal_avoid_add_ang;
                } else if (WallSensor::getInstance().ahead_r() < pm.wall_diagonal_ahead_r_threshold) {
                    target_rot_v += v_now * pm.wall_diagonal_avoid_add_ang;
                }
            }
            // 斜めターン時の衝突回避
/*            
            if(motion_type == EMotionType::CURVE &&
                    (( esti.getAng() > 30.0f  && esti.getAng() < 60.0f  ) ||
                     ( esti.getAng() > 120.0f && esti.getAng() < 150.0f ) ||
                     ( esti.getAng() > 210.0f && esti.getAng() < 240.0f ) ||
                     ( esti.getAng() > 300.0f && esti.getAng() < 330.0f )) &&

                    (turn_type == turn_type_e::TURN_D_90 ||
                     turn_type == turn_type_e::TURN_D2S_135 ||
                     turn_type == turn_type_e::TURN_S2D_135 ||
                     turn_type == turn_type_e::TURN_D2S_45 ||
                     turn_type == turn_type_e::TURN_S2D_45)


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
*/

            ang_pidf.update(target_rot_x, esti.getAng());
            float ang_pidf_controlval = ang_pidf.getControlVal();
            if(wall_pidf.engaged()){
                ang_pidf.reset();
            } 

            target_rot_v += ang_pidf_controlval;


            if( (motion_type != motion_type_pre &&
                 (motion_type_pre == EMotionType::STOP || motion_type_pre == EMotionType::SPINTURN)
                 )
                || motion_type == EMotionType::DIRECT_DUTY_SET) {
                ang_v_pidf.reset();
                ang_pidf.reset();
                v_pidf.reset();
                wall_pidf.reset();                
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
            if(pm.trans_v_FF_enable) duty += duty_v_FF;

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
            if(pm.rot_v_FF_enable) duty += duty_ang_v_FF;

            float voltage = BatVoltageMonitor::getInstance().bat_vol;
            float duty_v_saturation = (pm.trans_v_saturation_FF_multiplier * ABS(duty_v_FF(0)) + pm.trans_v_saturation_offset_duty);
            float duty_ang_v_saturation = (pm.rot_v_saturation_FF_multiplier * ABS(duty_ang_v_FF(0)) + pm.rot_v_saturation_offset_duty);


            if(motion_type == EMotionType::STOP || motion_type == EMotionType::SPINTURN) {
                // do nothing
            } else {
                v_pidf.setSaturation(duty_v_saturation);
                ang_v_pidf.setSaturation(duty_ang_v_saturation);
            }

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

            motion_type_pre = motion_type;
        
            {
                if( ABS(ang_v_pidf.getError()) > ang_v_error_th ||
                    ABS(ang_pidf.getError()) > 5.0f ||
                    ABS(v_pidf.getError()) > ABS(v_error_th)
                        ) error_sec += DELTA_T;
                else error_sec = 0.0f;
            }
        
        }

      private:
        float error_sec;
        float error_limit_sec;
        float ang_v_error_th;
        float v_error_th;
        EMotionType motion_type_pre;

        void setPIDF(EMotionType motion_type) {
            ParameterManager& pm = ParameterManager::getInstance();

            // 速度PIDFゲイン設定
            if(motion_type == EMotionType::STOP || motion_type == EMotionType::SPINTURN) {
                v_pidf.set(pm.trans_v_spin_P, pm.trans_v_spin_I, pm.trans_v_spin_D, pm.trans_v_spin_F);
            } else if(motion_type == EMotionType::STRAIGHT || motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                      motion_type == EMotionType::DIAGONAL || motion_type == EMotionType::DIAGONAL_CENTER
                     ) {
                v_pidf.set(pm.trans_v_P, pm.trans_v_I, pm.trans_v_D, pm.trans_v_F);
            } else if(motion_type == EMotionType::CURVE ) {
                v_pidf.set(pm.trans_v_slalom_P, pm.trans_v_slalom_I, pm.trans_v_slalom_D, pm.trans_v_slalom_F);
            }

            // 角速度PIDFゲイン設定
            if(motion_type == EMotionType::STOP || motion_type == EMotionType::SPINTURN) {
                ang_v_pidf.set(pm.rot_v_spin_P, pm.rot_v_spin_I, pm.rot_v_spin_D, pm.rot_v_spin_F);
            } else if(motion_type == EMotionType::STRAIGHT || motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                      motion_type == EMotionType::DIAGONAL || motion_type == EMotionType::DIAGONAL_CENTER
                     ) {
                ang_v_pidf.set(pm.rot_v_P, pm.rot_v_I, pm.rot_v_D, pm.rot_v_F);
            } else if(motion_type == EMotionType::CURVE ) {
                ang_v_pidf.set(pm.rot_v_slalom_P, pm.rot_v_slalom_I, pm.rot_v_slalom_D, pm.rot_v_slalom_F);
            }

            // 角度PIDFゲイン設定
            if(motion_type == EMotionType::STOP || motion_type == EMotionType::SPINTURN) {
                ang_pidf.set(pm.rot_x_spin_P, pm.rot_x_spin_I, pm.rot_x_spin_D, pm.rot_x_spin_F);
            } else if(motion_type == EMotionType::STRAIGHT || motion_type == EMotionType::STRAIGHT_WALL_CENTER ||
                      motion_type == EMotionType::DIAGONAL || motion_type == EMotionType::DIAGONAL_CENTER) {
                ang_pidf.set(pm.rot_x_P, pm.rot_x_I, pm.rot_x_D, pm.rot_x_F);
            } else if(motion_type == EMotionType::CURVE) {
                ang_pidf.set(pm.rot_x_slalom_P, pm.rot_x_slalom_I, pm.rot_x_slalom_D, pm.rot_x_slalom_F);
            }
            
            // 壁PIDFゲイン設定
            wall_pidf.set(pm.wall_P, pm.wall_I, pm.wall_D, pm.wall_F);


            // ------------------------------------------------------------------------- //
            // PIDFイネーブル設定
            v_pidf.setEnable(pm.trans_v_PIDF_enable);
            ang_v_pidf.setEnable(pm.rot_v_PIDF_enable);            
            wall_pidf.setEnable(pm.wall_PIDF_enable);
            ang_pidf.setEnable(pm.rot_v_PIDF_enable); // 角度制御と角速度制御のイネーブルは兼用

            // サチュレーションenable設定
            if(motion_type == EMotionType::STOP || motion_type == EMotionType::SPINTURN) {
                v_pidf.setSaturationEnable(false);
            } else {
                v_pidf.setSaturationEnable(pm.trans_v_PIDF_saturation_enable);
            }

            if(motion_type == EMotionType::STOP || motion_type == EMotionType::SPINTURN) {
                ang_v_pidf.setSaturationEnable(false);
            } else {
                ang_v_pidf.setSaturationEnable(pm.rot_v_PIDF_saturation_enable);
            }
            
            ang_pidf.setSaturationEnable(true);
            wall_pidf.setSaturationEnable(true);

            // サチュレーション値設定

            
            ang_pidf.setSaturation(360.0f);
            wall_pidf.setSaturation(360.0f);
            
            // 積分サチュレーションイネーブル設定
            v_pidf.setIntegralSaturationEnable(true);
            ang_v_pidf.setIntegralSaturationEnable(true);            
            wall_pidf.setIntegralSaturationEnable(true);
            ang_pidf.setIntegralSaturationEnable(true); // 角度制御と角速度制御のイネーブルは兼用


            // 積分サチュレーション値設定
            if(motion_type == EMotionType::STOP || motion_type == EMotionType::SPINTURN) {
                v_pidf.setIntegralSaturation(3.0);
                ang_v_pidf.setIntegralSaturation(3600.0);
                ang_pidf.setIntegralSaturation(360.0);
            } else {
                v_pidf.setIntegralSaturation(pm.trans_v_PIDF_integral_saturation);
                ang_v_pidf.setIntegralSaturation(pm.rot_v_PIDF_integral_saturation);
                ang_pidf.setIntegralSaturation(pm.rot_x_PIDF_integral_saturation);
            }
            
            wall_pidf.setIntegralSaturation(pm.wall_PIDF_integral_saturation);
        }

    };

}
