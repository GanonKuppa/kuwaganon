#pragma once



#include "pidController.h"
#include "wallsensor.h"



namespace umouse {

    class WallController : public VelocityTypePidController {

      public:
        using VelocityTypePidController::VelocityTypePidController;

        void update(WallSensor& ws) {

            float error;

            int16_t e_fr = 0;
            int16_t e_fl = 0;
            int16_t e_br = 0;
            int16_t e_bl = 0;
            uint8_t valid_error_num = 0;

//        if(ws.isRight_for_ctrl() == 1) e_fr = -(ws.right() - ws.center_r() );
//        if(ws.isLeft_for_ctrl() == 1) e_fl = +(ws.left() - ws.center_l() );
            //if(ws.isRight_for_ctrl_b() == 1) e_br = +(ws.right_b() - ws.center_r_b());
            //if(ws.isLeft_for_ctrl_b()  == 1) e_bl = -(ws.left_b()  - ws.center_l_b());

            //if(ws.ahead() > 900){ e_fr = 0; e_fl=0;}
            //if(ws.ahead_b() > 900){ e_br = 0; e_bl=0;}

            if(e_fr != 0) valid_error_num ++;
            if(e_fl != 0) valid_error_num ++;
            //if(e_br != 0) valid_error_num ++;
            //if(e_bl != 0) valid_error_num ++;

            if(valid_error_num != 0) error = (float)(e_fr + e_fl + e_br + e_bl) / (float)(valid_error_num);
            else error = 0.0f;

            e_k0 = -error;

            float delta_u_k = Kp * (e_k0 - e_k1 + T_s / Ki * e_k0 + Kd / T_s * (e_k0 - 2 * e_k1 + e_k2) );
            u_k0 = u_k1 + delta_u_k;

            u_k1 = u_k0;
            e_k2 = e_k1;
            e_k1 = e_k0;

            if(valid_error_num == 0) u_k0 = 0.0;

        };

    };

    class WallPidfController : public VelocityTypePidfController {
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

            if     (wallCenter                   ) target_line = 0.045f;
            else if(    dist < 0.039f            ) target_line = 0.039f;
            else if(ABS(dist - 0.041f) <= 0.002f ) target_line = 0.041f;
            else if(ABS(dist - 0.045f) <= 0.002f ) target_line = 0.045f;
            else if(ABS(dist - 0.049f) <= 0.002f ) target_line = 0.049f;
            else if(    dist > 0.051f            ) target_line = 0.051f;
            else                                   target_line = 0.045f;
            

            e_r0 =   10000 * ws.center_dist_r(target_line);
            e_l0 = - 10000 * ws.center_dist_l(0.09f - target_line);
            

            bool is_r_high_volatility = (ABS(e_r0 - e_r1) > 15);
            bool is_l_high_volatility = (ABS(e_l0 - e_l1) > 15);            

            bool is_right = ws.isRight() && isRWall && !is_r_high_volatility;
            bool is_left =  ws.isLeft() && isLWall && !is_l_high_volatility;

            if(is_right && is_left) error = (float)(e_r0 + e_l0) / 2.0f;
            else if(is_right) error = (float)(e_r0);
            else if(is_left) error = (float)(e_l0);
            else error = 0.0f;

            e_k0 = error;

            ud_k0 = F * ud_k1 + (1 - F) * (e_k0 - e_k1);
            float delta_u_k = calc_delta_u_k();
            u_k0 = u_k1 + delta_u_k;

            u_k1 = u_k0;
            e_k1 = e_k0;
            ud_k1 = ud_k0;
            
            e_r1 = e_r0;
            e_l1 = e_l0;
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




