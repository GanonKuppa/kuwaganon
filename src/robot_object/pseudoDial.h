#pragma once

#include "pidController.h"
#include "wheelOdometry.h"
#include "powerTransmission.h"
#include "communication.h"
#include "myUtil.h"


namespace umouse {


    class PseudoDial {
      protected:
        bool enable;
        VelocityTypePidController ang_ctrl;

        float target_ang;
        float duty;
        uint8_t dial_position;
        uint8_t division_num;

        PseudoDial() {
            enable = false;
            duty = 0.0f;
            ang_ctrl.set(0.003, 1000000.0, 0.0);
            dial_position = 0;
            division_num = 8;
            target_ang = 0.0f;

        }

        virtual ~PseudoDial() {}

        virtual float getAngle() = 0;
        virtual float getVelocity() = 0;
      public:
        void setEnable(bool enable_) {
            enable = enable_;
            if(enable == false) duty = 0.0f;
        }

        bool getEnable() {
            return enable;
        }

        float getDuty(){return duty;}

        void update() {
            if(enable == true) {
                WheelOdometry& wodo = WheelOdometry::getInstance();
                dial_position = (int)((getAngle() / 360.0) * (float)division_num);
                target_ang = fmod(360.0 * (float)dial_position / (float)division_num, 360.0);
                ang_ctrl.update(target_ang, getAngle());

                duty = constrain(ang_ctrl.getControlVal(), -0.65, 0.65) ;
                if(ABS(getVelocity()) >0.05) duty = 0.0;                
            }
        }

        void incrementPosition() {
            dial_position = (dial_position + (division_num + 1)) % division_num;
        }

        void decrementPosition() {
            dial_position = (dial_position + (division_num - 1)) % division_num;
        }


        void setDivisionNum(uint8_t num) {
            division_num = num;
        }

        uint8_t getDialPosition() {
            return dial_position;
        }

        void reset() {
            WheelOdometry& wodo = WheelOdometry::getInstance();
            wodo.resetTireAng();
            dial_position = 0;
        }

        void debug() {
            printfAsync("------------------- \n");
            printfAsync("dial pos:%d target_ang:%f\n",dial_position, target_ang);
            printfAsync("angle:%f duty:%f \n",getAngle(), ang_ctrl.getControlVal());

        }
    };


    class PseudoDialL : public PseudoDial {
      public:
        static PseudoDialL& getInstance() {
            static PseudoDialL instance;
            return instance;
        }

      private:
        PseudoDialL() {};
        ~PseudoDialL() {};

        float getAngle() {
            WheelOdometry& wodo = WheelOdometry::getInstance();
            return wodo.getTireAng_L();
        }


        float getVelocity() {
            WheelOdometry& wodo = WheelOdometry::getInstance();
            return wodo.getV_L();
        }


    };

    class PseudoDialR : public PseudoDial {
      public:
        static PseudoDialR& getInstance() {
            static PseudoDialR instance;
            return instance;
        }
      private:
        PseudoDialR() {};
        ~PseudoDialR() {};

        float getAngle() {
            WheelOdometry& wodo = WheelOdometry::getInstance();
            return wodo.getTireAng_R();
        }


        float getVelocity() {
            WheelOdometry& wodo = WheelOdometry::getInstance();
            return wodo.getV_R();
        }

    };



}

