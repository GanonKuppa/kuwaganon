#pragma once



#include "pidController.h"
#include "wallsensor.h"



namespace umouse {


    class WallPidfController : public PidfController {
      public:

        virtual void update(WallSensor& ws, bool isRWall, bool isLWall, bool wallCenter) {
            if( ws.calcAheadWallDist() < 0.08 ||
                (isRWall == false &&
                 isLWall == false ) ) {
                reset();
                return;
            }

            float error = 0.0f;
            float dist = 0.045f;
            if( (isLWall && ws.isLeft()) && (isRWall && ws.isRight()) ) dist = (0.09f - ws.dist_l() + ws.dist_r()) * 0.5;
            else if((isLWall && ws.isLeft())) dist = 0.09f - ws.dist_l();
            else if(isRWall && ws.isRight()) dist = ws.dist_r();

            float target_line = 0.045f;
            wallCenter = true;
            if     (wallCenter                   ) target_line = 0.045f;
            else if(    dist < 0.039f            ) target_line = 0.039f;
            else if(ABS(dist - 0.041f) <= 0.002f ) target_line = 0.041f;
            else if(ABS(dist - 0.045f) <= 0.002f ) target_line = 0.045f;
            else if(ABS(dist - 0.049f) <= 0.002f ) target_line = 0.049f;
            else if(    dist > 0.051f            ) target_line = 0.051f;
            else                                   target_line = 0.045f;
            

            e_r0 =   10000 * ws.center_dist_r(target_line);
            if(e_r0 > -20 && e_r0 < 20) e_r0 = 0;
            e_l0 = - 10000 * ws.center_dist_l(0.09f - target_line);
            if(e_l0 > -20 && e_l0 < 20) e_l0 = 0;

            bool is_r_high_volatility = (ABS(e_r0 - e_r1) > 150);
            bool is_l_high_volatility = (ABS(e_l0 - e_l1) > 150);            

            bool is_right = ws.isRight() && isRWall && !is_r_high_volatility;
            bool is_left =  ws.isLeft() && isLWall && !is_l_high_volatility;

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




