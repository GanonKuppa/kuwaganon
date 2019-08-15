#pragma once
#include <stdint.h>

namespace umouse{

class ArcLengthParameterizedCurve{
public:
    uint32_t length;

    ArcLengthParameterizedCurve(const float *kappa_array_, uint16_t array_size_, float arc_len_, float delta_s_, float pre_dist_, float fol_dist_){
        kappa_array = kappa_array_;
        array_size = array_size_;
        arc_len = arc_len_;
        delta_s = delta_s_;
        pre_dist = pre_dist_;
        fol_dist = fol_dist_;
    }

    float getOmega(float s, float v){
        uint16_t index = uint16_t( (s - pre_dist) / delta_s);
        if (index >= array_size -1) return 0.0;
        else return rad2deg(kappa_array[index] * v);
    }

    float arc_len;
    float delta_s;
    uint16_t array_size;
    const float *kappa_array;
    float pre_dist;
    float fol_dist;

private:
    float rad2deg(float rad){
        return ((rad)*180.0) / 3.1415926535;
    }


};

}

