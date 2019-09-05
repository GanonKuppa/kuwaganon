#pragma once

#include <math.h>
#include <memory>
#include <string>
#include "curveFactory.h"
#include "arcLengthParameterizedCurve.h"
#include "kappa.h"
#include "myUtil.h"

namespace umouse
{


enum class EMotionType{
    DIAGONAL = 0,
    DIAGONAL_CENTER,
    STRAIGHT,
    STRAIGHT_WALL_CENTER,
    SPINTURN,
    CURVE,
    STOP,
    STOP_DIRECT_DUTY_SET
};


class BaseTrajectory
{
public:
    
    float x_0;
    float y_0;
    float ang_0;
    float x;
    float x_d;
    float x_dd;
    float y;
    float y_d;
    float y_dd;
    float ang;
    float ang_v;
    float ang_a;
    float v;
    float a;
    float target_dist;
    double cumulative_dist;
    double cumulative_ang;
    float cumulative_t;
    EMotionType motion_type;
    uint16_t hash;
    const double DELTA_T = 0.0005;

    void setInitPos(float x_, float y_, float ang_){
        x = x_;
        y = y_;
        ang = ang_;
        x_0 = x_;
        y_0 = y_;
        ang_0 = ang_;
    }

    virtual float getEndX(){
        return 0.0f;
    }

    virtual float getEndY(){
        return 0.0f;
    }

    virtual float getEndAng(){
        return 0.0f;
    }

    virtual void forceEnd(){
        x = getEndX();
        y = getEndY();
        ang = getEndAng();
    }


    void init(float x_, float y_, float v_, float a_, float ang_, float ang_v_, float ang_a_)
    {
        x_0 = 0.0f;
        y_0 = 0.0f;
        ang_0 = 0.0f;
        x = x_;
        y = y_;
        v = v_;
        a = a_;
        ang = ang_;
        ang_v = ang_v_;
        ang_a = ang_a_;
        cumulative_dist = 0.0f;
        cumulative_ang = 0.0f;
        cumulative_t = 0.0f;
        x_d = 0.0f;
        x_dd = 0.0f;
        y_d = 0.0f;
        y_dd = 0.0f;
        motion_type =EMotionType::STOP;
        hash = (uint16_t)xor32();
    }

    std::string getMotionTypeString(){
        std::string str;
        if(motion_type == EMotionType::DIAGONAL) str = "DIAGONAL";
        else if(motion_type == EMotionType::DIAGONAL_CENTER) str = "DIAGONAL_CENTER";
        else if(motion_type == EMotionType::STRAIGHT) str = "STRAIGHT";
        else if(motion_type == EMotionType::STRAIGHT_WALL_CENTER) str = "STRAIGHT_WALL_CENTER";
        else if(motion_type == EMotionType::SPINTURN) str = "SPINTURN";
        else if(motion_type == EMotionType::CURVE) str = "CURVE";
        else if(motion_type == EMotionType::STOP) str = "STOP";
        return str;
    }

    virtual void update()
    {
        v += DELTA_T * a;
        cumulative_dist += DELTA_T * v;
        cumulative_ang += DELTA_T * ang_v;
        cumulative_t += DELTA_T;

        ang_v += DELTA_T * ang_a;
        ang += DELTA_T * ang_v;
        ang = fmod(ang + 360.0, 360.0);

        float ang_rad = DEG2RAD(ang);
        float ang_v_rad = DEG2RAD(ang_v);

        x_d = v * cosf(ang_rad);
        y_d = v * sinf(ang_rad);

        x_dd = a * cosf(ang_rad) - v * ang_v_rad * sinf(ang_rad);
        y_dd = a * sinf(ang_rad) + v * ang_v_rad * cosf(ang_rad);

        x += DELTA_T * x_d;
        y += DELTA_T * y_d;
    }

    virtual bool isEnd()
    {
        return false;
    }

    virtual ~BaseTrajectory(){};
};

class StraightTrajectory : public BaseTrajectory
{
public:
    StraightTrajectory(float target_dist_, float v_0_)
    {
        float x_ = 0.0;
        float y_ = 0.0;
        float ang_ = 0.0;
        a_acc = 0.0;
        a_dec = 0.0;
        target_dist = target_dist_;
        v_end = v_0_;
        v_max = v_0_;
        v_0 = v_0_;
        wall_contact = false;
        init(x_, y_, v_0_, a_acc, ang_, 0.0f, 0.0f);
        motion_type = EMotionType::STRAIGHT;

    }

    StraightTrajectory(float target_dist_, float v_0_, float v_max_, float v_end_, float a_acc_, float a_dec_)
    {
        float x_ = 0.0;
        float y_ = 0.0;
        float ang_ = 0.0;
        a_acc = a_acc_;
        a_dec = a_dec_;
        target_dist = target_dist_;
        v_end = v_end_;
        v_max = v_max_;
        v_0 = v_0_;
        wall_contact = false;
        init(x_, y_, v_0_, a_acc, ang_, 0.0f, 0.0f);
        motion_type = EMotionType::STRAIGHT;
    }

    static std::unique_ptr<BaseTrajectory> create(float target_dist_, float v_0_){
        return std::unique_ptr<BaseTrajectory>(new StraightTrajectory(target_dist_, v_0_) );
    }

    static std::unique_ptr<BaseTrajectory> create(float target_dist_, float v_0_, float v_max_, float v_end_, float a_acc_, float a_dec_){
        return std::unique_ptr<BaseTrajectory>(new StraightTrajectory(target_dist_, v_0_, v_max_, v_end_, a_acc_, a_dec_  )  );
    }

    static std::unique_ptr<BaseTrajectory> createAsContactWall(float target_dist_, float v_0_){
        auto traj = new StraightTrajectory(target_dist_, v_0_);
        traj->setWallContact(true);
        return std::unique_ptr<BaseTrajectory>(traj);
    }

    static std::unique_ptr<BaseTrajectory> createAsContactWall(float target_dist_, float v_0_, float v_max_, float v_end_, float a_acc_, float a_dec_){
        auto traj = new StraightTrajectory(target_dist_, v_0_, v_max_, v_end_, a_acc_, a_dec_  );
        traj->setWallContact(true);
        return std::unique_ptr<BaseTrajectory>(traj);
    }


    static std::unique_ptr<BaseTrajectory> createAsWallCenter(float target_dist_, float v_0_){
        auto traj = new StraightTrajectory(target_dist_, v_0_);
        traj->motion_type=EMotionType::STRAIGHT_WALL_CENTER;
        return std::unique_ptr<BaseTrajectory>(traj);
    }

    static std::unique_ptr<BaseTrajectory> createAsWallCenter(float target_dist_, float v_0_, float v_max_, float v_end_, float a_acc_, float a_dec_){
        auto traj = new StraightTrajectory(target_dist_, v_0_, v_max_, v_end_, a_acc_, a_dec_  );
        traj->motion_type=EMotionType::STRAIGHT_WALL_CENTER;
        return std::unique_ptr<BaseTrajectory>(traj);
    }

    static std::unique_ptr<BaseTrajectory> createAsDiagonal(float target_dist_, float v_0_){
        auto traj = new StraightTrajectory(target_dist_, v_0_);
        traj->motion_type=EMotionType::DIAGONAL;
        return std::unique_ptr<BaseTrajectory>(traj);
    }

    static std::unique_ptr<BaseTrajectory> createAsDiagonal(float target_dist_, float v_0_, float v_max_, float v_end_, float a_acc_, float a_dec_){
        auto traj = new StraightTrajectory(target_dist_, v_0_, v_max_, v_end_, a_acc_, a_dec_  );
        traj->motion_type=EMotionType::DIAGONAL;
        return std::unique_ptr<BaseTrajectory>(traj);
    }

    static std::unique_ptr<BaseTrajectory> createAsDiagonalCenter(float target_dist_, float v_0_){
        auto traj = new StraightTrajectory(target_dist_, v_0_);
        traj->motion_type=EMotionType::DIAGONAL_CENTER;
        return std::unique_ptr<BaseTrajectory>(traj);
    }

    static std::unique_ptr<BaseTrajectory> createAsDiagonalCenter(float target_dist_, float v_0_, float v_max_, float v_end_, float a_acc_, float a_dec_){
        auto traj = new StraightTrajectory(target_dist_, v_0_, v_max_, v_end_, a_acc_, a_dec_  );
        traj->motion_type=EMotionType::DIAGONAL_CENTER;
        return std::unique_ptr<BaseTrajectory>(traj);
    }


    virtual float getEndX(){
        float ang_rad = DEG2RAD(ang_0);
        return x_0 + target_dist * cosf(ang_rad);
    }

    virtual float getEndY(){
        float ang_rad = DEG2RAD(ang_0);
        return y_0 + target_dist * sinf(ang_rad);
    }

    virtual float getEndAng(){
        return ang_0;
    }

    virtual void forceEnd(){
        x = getEndX();
        y = getEndY();
        ang = getEndAng();
        float epsilon = 1.0;
        cumulative_dist = target_dist + epsilon;
    }



    virtual void update()
    {
        BaseTrajectory::update();
        if( (a_acc == 0.0f && a_dec == 0.0f)||
            (v_max == v_0 && v_max == v_end)
        ){
            a_acc = 0.0f;
            a_dec = 0.0f;
            a = 0.0f;
            return;
        } 
        
        float x_bre = 0.0;
        if (a_dec != 0.0)
            x_bre = (v * v - v_end * v_end) / (2.0f * a_dec);

        if (x_bre > (target_dist - cumulative_dist))
            a = -a_dec;

        if (v > v_max)
        {
            v = v_max;
            a = 0.0;
        }
        if (v < v_end && a < 0.0f)
        {
            v = v_end;
            if(wall_contact == true && v_end == 0.0f) v = 0.1f;
            cumulative_dist = target_dist;
            //printfAsync("EEEEEEE\n");
        }
    }

    void setWallContact(bool enable){
        wall_contact = enable;
    }

    virtual bool isEnd()
    {
        if(wall_contact == false){
            if (cumulative_dist >= target_dist){
                x = getEndX();
                y = getEndY();
                ang = getEndAng();
                return true;
            }
            else
                return false;
        }
        else{
            WallSensor &ws = WallSensor::getInstance();

            if (cumulative_dist >= target_dist){
                x = getEndX();
                y = getEndY();
                ang = getEndAng();
            }

            if (ws.getContactWallTime()>0.3 && cumulative_dist >= target_dist)
                return true;
            else
                return false;

        }
    }

    virtual ~StraightTrajectory(){};

private:    
    float v_max;
    float v_end;
    float a_acc;
    float a_dec;
    float v_0;
    bool wall_contact;
};



class SpinTurnTrajectory : public BaseTrajectory
{
public:
    SpinTurnTrajectory(float target_cumulative_ang_, float abs_ang_v_max_, float abs_ang_a_)
    {
        float x_ = 0.0;
        float y_ = 0.0;
        float ang_ = 0.0;
        init(x_, y_, 0.0f, 0.0f, ang_, 0.0f, SIGN(target_cumulative_ang_) * abs_ang_a_);
        motion_type = EMotionType::SPINTURN;
        target_cumulative_ang = target_cumulative_ang_;
        abs_ang_v_max = abs_ang_v_max_;
        abs_ang_a = abs_ang_a_;
    }

    static std::unique_ptr<BaseTrajectory> create(float target_cumulative_ang_, float abs_ang_v_max_, float abs_ang_a_ ){
        return std::unique_ptr<BaseTrajectory>(new SpinTurnTrajectory(target_cumulative_ang_, abs_ang_v_max_, abs_ang_a_  )  );
    }

    virtual float getEndX(){
        return x_0;
    }

    virtual float getEndY(){
        return y_0;
    }

    virtual float getEndAng(){
        return fmod(ang_0 + target_cumulative_ang + 360.0f, 360.0f);        
    }

    virtual void update()
    {
        ang_v_end = 0.0;
        BaseTrajectory::update();
        float ang_bre = 0.0;
        //std::cout << "ang_a:" << ang_a << " ang_v:" << ang_v << " ang:" << ang << std::endl;

        if (abs_ang_a != 0.0)
            ang_bre = (ang_v * ang_v - 0.0f * 0.0f) / (2.0f * abs_ang_a);

        if (ang_bre > (ABS(target_cumulative_ang) - ABS(cumulative_ang)))
            ang_a = -SIGN(target_cumulative_ang) * abs_ang_a;

        if (ABS(ang_v) > abs_ang_v_max)
        {
            ang_v = SIGN(target_cumulative_ang) * abs_ang_v_max;
            ang_a = 0.0;
        }

        if (ABS(ang_v) < ABS(ang_v_end))
            ang_v = ang_v_end;
    }

    virtual bool isEnd()
    {
        if(target_cumulative_ang == 0.0f) return true;

        if (ABS(cumulative_ang) >= ABS(target_cumulative_ang)){
            x = getEndX();
            y = getEndY();
            ang = getEndAng();
            return true;
        }            
        else
            return false;
    }

private:
    float target_cumulative_ang;
    float abs_ang_a;
    float abs_ang_v_max;
    float ang_v_end;
};


class CurveTrajectory : public BaseTrajectory{
public:
    CurveTrajectory(float v_, turn_type_e turn_type_, turn_dir_e turn_dir_){
        float x_ = 0.0;
        float y_ = 0.0;
        float ang_ = 0.0;

        init(x_, y_, v_, 0.0f, ang_, 0.0f, 0.0f);

        alp_curve = CurveFactory::create(turn_type_);
        turn_dir = turn_dir_;
        motion_type = EMotionType::CURVE;
    }




    static std::unique_ptr<BaseTrajectory> create(float v_, turn_type_e turn_type_, turn_dir_e turn_dir_){
        return std::unique_ptr<BaseTrajectory>(new CurveTrajectory(v_, turn_type_, turn_dir_));
    }

    static std::unique_ptr<BaseTrajectory> createAsNoStraght(float v_, turn_type_e turn_type_, turn_dir_e turn_dir_){
        auto traj = new CurveTrajectory(v_, turn_type_, turn_dir_);
        traj->alp_curve->pre_dist = 0.0f;
        traj->alp_curve->fol_dist = 0.0f;
        return std::unique_ptr<BaseTrajectory>(traj);
    }

    virtual ~CurveTrajectory(){
        delete alp_curve;
    };
    
    virtual float getEndX(){
        float x;
        float y;
        float ang_rad = DEG2RAD(ang_0);
        if(alp_curve->pre_dist == 0.0f && alp_curve->fol_dist == 0.0f){
            x = alp_curve->x;
            y = alp_curve->y * (float)(turn_dir);
        } 
        else{
            x = alp_curve->x_with_pf;
            y = alp_curve->y_with_pf * (float)(turn_dir);
        } 
        return x_0 + x * cosf(ang_rad) - y * sinf(ang_rad);
    }

    virtual float getEndY(){
        float x;
        float y;
        float ang_rad = DEG2RAD(ang_0);
        if(alp_curve->pre_dist == 0.0f && alp_curve->fol_dist == 0.0f){
            x = alp_curve->x;
            y = alp_curve->y * (float)(turn_dir);
        } 
        else{
            x = alp_curve->x_with_pf;
            y = alp_curve->y_with_pf * (float)(turn_dir);
        } 
        return y_0 + x * sinf(ang_rad) + y * cosf(ang_rad);
    }

    virtual float getEndAng(){
        return fmod(ang_0 + alp_curve->ang * (float)(turn_dir) + 360.0f, 360.0f);        
    }


    virtual void update(){
        //std::cout << ang <<" " << ang_v << std::endl;

        v += DELTA_T * a;
        cumulative_dist += DELTA_T * v;
        cumulative_ang += DELTA_T * ang_v;
        cumulative_t += DELTA_T;
        if(cumulative_dist < (alp_curve->pre_dist) ){
            ang_a = 0.0f;
            ang_v = 0.0f;
        }
        else if (cumulative_dist < (alp_curve->pre_dist + alp_curve->arc_len)){
            ang_v = float(turn_dir) * alp_curve->getOmega(cumulative_dist, v);
        }
        else{
            ang_a = 0.0f;
            ang_v = 0.0f;
        }

        //ang_v += DELTA_T * ang_a;
        ang += DELTA_T * ang_v;
        ang = fmod(ang + 360.0, 360.0);

        float ang_rad = DEG2RAD(ang);
        float ang_v_rad = DEG2RAD(ang_v);

        x_d = v * cosf(ang_rad);
        y_d = v * sinf(ang_rad);

        x_dd = a * cosf(ang_rad) - v * ang_v_rad * sinf(ang_rad);
        y_dd = a * sinf(ang_rad) + v * ang_v_rad * cosf(ang_rad);

        x += DELTA_T * x_d;
        y += DELTA_T * y_d;
    }

    virtual bool isEnd(){
        float end_cumulative_dist = alp_curve->pre_dist + alp_curve->arc_len + alp_curve->fol_dist;
        if(end_cumulative_dist < cumulative_dist){
             x = getEndX();
             y = getEndY();
             ang = getEndAng();
             return true;
        }
        else return false;
    }
    ArcLengthParameterizedCurve *alp_curve;
private:

    turn_dir_e turn_dir;
};


class StopTrajectory : public BaseTrajectory{
public:
    StopTrajectory(float stop_time_){
        float x_ = 0.0;
        float y_ = 0.0;
        float ang_ = 0.0;

        init(x_, y_, 0.0f, 0.0f, ang_, 0.0f, 0.0f);
        motion_type = EMotionType::STOP;
        stop_time = stop_time_;
    }

    static std::unique_ptr<BaseTrajectory> create(float stop_time_){
        return std::unique_ptr<BaseTrajectory>(new StopTrajectory(stop_time_));
    }



    static std::unique_ptr<BaseTrajectory> createAsDirectDutySet(float stop_time_){


        auto traj = new StopTrajectory(stop_time_);
        traj->motion_type=EMotionType::STOP_DIRECT_DUTY_SET;
        return std::unique_ptr<BaseTrajectory>(traj);

    }


    virtual float getEndX(){
        return x_0;
    }

    virtual float getEndY(){
        return y_0;
    }

    virtual float getEndAng(){
        return ang_0;
    }


    virtual void update(){
        cumulative_t += DELTA_T;
    }

    virtual bool isEnd(){
        if(stop_time < cumulative_t) return true;
        else return false;
    }

private:
    float stop_time;
};



} // namespace umouse
