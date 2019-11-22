#include "activityFactory.h"

#include "modeSelectActivity.h"
#include "shortestRunActivity.h"
#include "debugActivity.h"
#include "deleteMazeActivity.h"
#include "searchRunActivity.h"
#include "calibrateWallCenterActivity.h"
#include "fullAutonomousRun.h"
#include "radioControlActivity.h"
#include "subModeSelectActivity.h"

namespace umouse {

    std::unique_ptr<BaseActivity> ActivityFactory::create(EActivityColor name) {
        switch(name) {
            case EActivityColor::BLACK:
                return std::unique_ptr<BaseActivity>(new ModeSelectActivity());

            case EActivityColor::RED:
                return std::unique_ptr<BaseActivity>(new ShortestRunActivity());

            case EActivityColor::GREEN:
                return std::unique_ptr<BaseActivity>(new DebugActivity());

            case EActivityColor::YELLOW:
                return std::unique_ptr<BaseActivity>(new DeleteMazeActivity());

            case EActivityColor::BLUE:
                return std::unique_ptr<BaseActivity>(new SearchRunActivity());

            case EActivityColor::MAGENTA:
                return std::unique_ptr<BaseActivity>(new CalibrateWallCenterActivity());

            case EActivityColor::CYAN:
                return std::unique_ptr<BaseActivity>(new FullAutonomousRunActivity());

            case EActivityColor::WHITE:
                return std::unique_ptr<BaseActivity>(new RadioControlActivity());

            default:
                return std::unique_ptr<BaseActivity>(new ModeSelectActivity());
        }
    }

    std::unique_ptr<BaseActivity> ActivityFactory::cteateSubModeSelect() {
        return unique_ptr < BaseActivity > (new SubModeSelectActivity());
    }


    std::unique_ptr<BaseActivity> ActivityFactory::cteateModeSelect() {
        return unique_ptr < BaseActivity > (new ModeSelectActivity());
    }

}
