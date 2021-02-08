#pragma once



#include "pidController.h"
#include "wallsensor.h"



namespace umouse {


    class WallPidfController : public PidfController {
      public:

        virtual void update(WallSensor& ws, bool isRWall, bool isLWall, bool isPillar) {
            if( ws.calcAheadWallDist() < 0.08 ||
                (isRWall == false &&
                 isLWall == false ) ) {
                reset();
                return;
            }

            float error = 0.0f;
            float target_line = 0.045f;
            e_r0 =   10000 * ws.center_dist_r(target_line);            
            e_l0 = - 10000 * ws.center_dist_l(0.09f - target_line);


            bool is_right = ws.isRight_for_ctrl() && isRWall;
            bool is_left =  ws.isLeft_for_ctrl() && isLWall;

            // 柱を見ている際は柱に近づき過ぎている場合のみ壁制御をかける
            if(isPillar){
                if( ws.dist_r() > 0.045f) is_right = false;
                if( ws.dist_l() > 0.045f) is_left = false;
            }


            if(is_right && is_left) error = (float)(e_r0 + e_l0) / 2.0f;
            else if(is_right) error = (float)(e_r0);
            else if(is_left) error = (float)(e_l0);
            else error = 0.0f;

            e_p0 = error;

            
            if(integral_saturation_enable){
                e_i0 = constrain(e_p0 + e_i1, -ABS(integral_saturation), ABS(integral_saturation));
            }
            else {
                e_i0 = e_p0 + e_i1;
            }

            e_d0 = F * e_d1 + (1.0f - F) * (e_p0 - e_p1);
        
            u_k0 = constrain(Kp * e_p0 + Ki * e_i0 + Kd * e_d0, -ABS(saturation), ABS(saturation));

            e_p1 = e_p0;
            e_i1 = e_i0;
            e_d1 = e_d0;
            u_k1 = u_k0;

        };
      
      WallPidfController(){
        e_r0 = 0;
        e_l0 = 0;
        
        e_r1 = 0;
        e_l1 = 0;

      }
      
      private:
        int16_t e_r0;
        int16_t e_l0;

        int16_t e_r1;
        int16_t e_l1;
        
    };


}




