#include "curveFactory.h"
#include "arcLengthParameterizedCurve.h"
#include "kappa.h"

namespace umouse{




ArcLengthParameterizedCurve* CurveFactory::create(turn_type_e turn_type_){
    switch(turn_type_){
        case turn_type_e::TURN_90 :
            return new ArcLengthParameterizedCurve(KAPPA_90DEG_HALF, KAPPA_SIZE_90DEG_HALF,
                            ARC_LEN_90DEG_HALF, DELTA_S_90DEG_HALF, PRE_90DEG_HALF, FOL_90DEG_HALF,
                            X_90DEG_HALF, Y_90DEG_HALF, X_WITH_PF_90DEG_HALF, Y_WITH_PF_90DEG_HALF, ANG_90DEG_HALF);
            break;

       case turn_type_e::TURN_L_90 :
            return new ArcLengthParameterizedCurve(KAPPA_L90DEG_HALF, KAPPA_SIZE_L90DEG_HALF,
                            ARC_LEN_L90DEG_HALF, DELTA_S_L90DEG_HALF, PRE_L90DEG_HALF, FOL_L90DEG_HALF,
                            X_L90DEG_HALF, Y_L90DEG_HALF, X_WITH_PF_L90DEG_HALF, Y_WITH_PF_L90DEG_HALF, ANG_L90DEG_HALF);
            break;

        case turn_type_e::TURN_180 :
            return new ArcLengthParameterizedCurve(KAPPA_180DEG_HALF, KAPPA_SIZE_180DEG_HALF,
                            ARC_LEN_180DEG_HALF, DELTA_S_180DEG_HALF, PRE_180DEG_HALF, FOL_180DEG_HALF,
                            X_180DEG_HALF, Y_180DEG_HALF, X_WITH_PF_180DEG_HALF, Y_WITH_PF_180DEG_HALF, ANG_180DEG_HALF);
            break;
        case turn_type_e::TURN_S2D_45 :
            return new ArcLengthParameterizedCurve(KAPPA_S2D45DEG_HALF, KAPPA_SIZE_S2D45DEG_HALF,
                            ARC_LEN_S2D45DEG_HALF, DELTA_S_S2D45DEG_HALF,PRE_S2D45DEG_HALF, FOL_S2D45DEG_HALF,
                            X_S2D45DEG_HALF, Y_S2D45DEG_HALF, X_WITH_PF_S2D45DEG_HALF, Y_WITH_PF_S2D45DEG_HALF, ANG_S2D45DEG_HALF);

            break;
        case turn_type_e::TURN_S2D_135 :
            return new ArcLengthParameterizedCurve(KAPPA_S2D135DEG_HALF, KAPPA_SIZE_S2D135DEG_HALF,
                            ARC_LEN_S2D135DEG_HALF, DELTA_S_S2D135DEG_HALF, PRE_S2D135DEG_HALF, FOL_S2D135DEG_HALF,
                            X_S2D135DEG_HALF, Y_S2D135DEG_HALF, X_WITH_PF_S2D135DEG_HALF, Y_WITH_PF_S2D135DEG_HALF, ANG_S2D135DEG_HALF);
            break;
        case turn_type_e::TURN_D_90 :
            return new ArcLengthParameterizedCurve(KAPPA_D90DEG_HALF, KAPPA_SIZE_D90DEG_HALF,
                            ARC_LEN_D90DEG_HALF, DELTA_S_D90DEG_HALF, PRE_D90DEG_HALF, FOL_D90DEG_HALF,
                            X_D90DEG_HALF, Y_D90DEG_HALF, X_WITH_PF_D90DEG_HALF, Y_WITH_PF_D90DEG_HALF, ANG_D90DEG_HALF);
            break;
        case turn_type_e::TURN_D2S_45 :
            return new ArcLengthParameterizedCurve(KAPPA_D2S45DEG_HALF, KAPPA_SIZE_D2S45DEG_HALF,
                            ARC_LEN_D2S45DEG_HALF, DELTA_S_D2S45DEG_HALF, PRE_D2S45DEG_HALF, FOL_D2S45DEG_HALF,
                            X_D2S45DEG_HALF, Y_D2S45DEG_HALF, X_WITH_PF_D2S45DEG_HALF, Y_WITH_PF_D2S45DEG_HALF, ANG_D2S45DEG_HALF);

            break;
        case turn_type_e::TURN_D2S_135 :
            return new ArcLengthParameterizedCurve(KAPPA_D2S135DEG_HALF, KAPPA_SIZE_D2S135DEG_HALF,
                            ARC_LEN_D2S135DEG_HALF, DELTA_S_D2S135DEG_HALF, PRE_D2S135DEG_HALF, FOL_D2S135DEG_HALF,
                            X_D2S135DEG_HALF, Y_D2S135DEG_HALF, X_WITH_PF_D2S135DEG_HALF, Y_WITH_PF_D2S135DEG_HALF, ANG_D2S135DEG_HALF);
            break;

        default:
            return nullptr;
    }
};


float CurveFactory::getPreDist(turn_type_e turn_type_){
    switch(turn_type_){
        case turn_type_e::TURN_90 :
            return PRE_90DEG_HALF;
        case turn_type_e::TURN_L_90 :
            return PRE_L90DEG_HALF;
        case turn_type_e::TURN_180 :
            return PRE_180DEG_HALF;
        case turn_type_e::TURN_S2D_45 :
            return PRE_S2D45DEG_HALF;
        case turn_type_e::TURN_S2D_135 :
            return PRE_S2D135DEG_HALF;
        case turn_type_e::TURN_D_90 :
            return PRE_D90DEG_HALF;
        case turn_type_e::TURN_D2S_45 :
            return PRE_D2S45DEG_HALF;
        case turn_type_e::TURN_D2S_135 :
            return PRE_D2S135DEG_HALF;
        default:
            return 0.0;
    }
};

float CurveFactory::getFolDist(turn_type_e turn_type_){
    switch(turn_type_){
        case turn_type_e::TURN_90 :
            return FOL_90DEG_HALF;
        case turn_type_e::TURN_L_90 :
            return FOL_L90DEG_HALF;
        case turn_type_e::TURN_180 :
            return FOL_180DEG_HALF;
        case turn_type_e::TURN_S2D_45 :
            return FOL_S2D45DEG_HALF;
        case turn_type_e::TURN_S2D_135 :
            return FOL_S2D135DEG_HALF;
        case turn_type_e::TURN_D_90 :
            return FOL_D90DEG_HALF;
        case turn_type_e::TURN_D2S_45 :
            return FOL_D2S45DEG_HALF;
        case turn_type_e::TURN_D2S_135 :
            return FOL_D2S135DEG_HALF;
        default:
            return 0.0;
    }
};



}


