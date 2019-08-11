/*
 * sound.cpp
 *
 *  Created on: 2016/12/03
 *      Author: ryota
 */

#include <myUtil.h>
#include <myUtil.h>
#include <timeInterrupt.h>
#include <uart.h>
#include <wallsensor.h>
#include "sound.h"
#include "da.h"
#include "timer.h"
#include <queue>
#include <array>

#include "parameterManager.h"




struct Note;
class SoundManager;

struct Note {
    uint8_t pitch;
    uint16_t len;
};

class SoundManager {
public:
    uint16_t max_volume;
    uint16_t note_off_count;
    uint16_t bpm;
    uint16_t time_resolution;
    bool enable;

    static const uint8_t track_num = 8;
    uint8_t wave_form_data[track_num];      //各トラックの波形
    uint8_t wave_volume_data[track_num];    //各トラックのボリューム  volume_resolution段階

    static const uint16_t wave_size = 2000;
    static const uint16_t volume_resolution = 16;

    std::array<std::queue<Note>, track_num> noteBuff;
    int64_t note_len[track_num];

    uint16_t wave[track_num][wave_size];     //各トラックの音の一周期分の波形
    uint16_t wave_period[track_num];         //各トラックの音の一周期分の波形カウント数

    uint16_t wave_master_volume[track_num];

    SoundManager(void) {
        init();
    }
    ;
    void init();
    void setWave(uint8_t track_num_, uint8_t pitch, uint8_t waveform, uint8_t wave_volume);
    void update();
    void clearBuff();
    uint16_t calcWaveVal(uint32_t num_count);
};

void SoundManager::init() {
    for (int j = 0; j < track_num; j++) {
        for (int i = 0; i < wave_size; i++) {
            wave[j][i] = 0;
        }
        wave_period[j] = 0;
        wave_form_data[j] = 0;
        wave_volume_data[j] = 16;
        wave_master_volume[j] = 1000;         //500;
    }
    note_off_count = 50;
    enable = false;
    bpm = 80;
    time_resolution = 480;

}

void SoundManager::setWave(uint8_t track, uint8_t pitch, uint8_t waveform, uint8_t wave_volume) {
    uint16_t wave_count = 0;
    uint16_t wave_count_max = note10uCnt[pitch];
    uint16_t wave_amplitude_max = 0;
    wave_period[track] = wave_count_max;

    if (wave_volume != 0) wave_amplitude_max = wave_master_volume[track] * wave_volume / 16;
    else wave_amplitude_max = 0;

    if (pitch == nR || wave_amplitude_max == 0 || waveform == REST) {
        wave_period[track] = 1;
        wave[track][0] = 0;
        return;
    }

    switch (waveform) {
        case SANKAKU:
            for (int i = 0; i < wave_count_max; i++) {
                if (i < wave_count_max / 2) wave[track][i] = i * (2 * wave_amplitude_max) / wave_count_max;
                else {
                    int16_t val = ((wave_count / 2) / (0 - wave_amplitude_max)) * (i - wave_count_max);
                    if (val > 0) wave[track][i] = val;
                    else wave[track][i] = 0;
                }
            }
            break;
        case NOKOGIRI:
            for (int i = 0; i < wave_count_max; i++) {
                wave[track][i] = i * wave_amplitude_max / wave_count_max;
            }

            break;
        case KUKEI12_5:
            for (int i = 0; i < wave_count_max; i++) {
                if (i < wave_count_max / 8) wave[track][i] = wave_amplitude_max;
                else wave[track][i] = 0;
            }
            break;
        case KUKEI25:
            for (int i = 0; i < wave_count_max; i++) {
                if (i < wave_count_max / 4) wave[track][i] = wave_amplitude_max;
                else wave[track][i] = 0;
            }
            break;
        case KUKEI50:
            for (int i = 0; i < wave_count_max; i++) {
                if (i < wave_count_max / 2) wave[track][i] = wave_amplitude_max;
                else wave[track][i] = 0;
            }
            break;
        case KUKEI75:
            for (int i = 0; i < wave_count_max; i++) {
                if (i < wave_count_max * 3 / 4) wave[track][i] = wave_amplitude_max;
                else wave[track][i] = 0;
            }
            break;
        case NOISE3:
            for (int i = 0; i < wave_count_max; i++) {
                wave[track][i] = xor32() % wave_amplitude_max;
            }
            break;
    }
}

void SoundManager::update() {
    umouse::ParameterManager &pm = umouse::ParameterManager::getInstance();
    if (enable == false || pm.silent_flag == 1){
        for(int i=0; i<track_num; i++) while(!noteBuff[i].empty()) noteBuff[i].pop() ;
        return;
    }
    for (int i = 0; i < track_num; i++) {
        if (note_len[i] != 0) {
            note_len[i]--;
            if (note_len[i] < note_off_count) setWave(i, nR, REST, 0);
        } else {
            if (noteBuff[i].size() != 0) {

                Note note = noteBuff[i].front();

                uint16_t pitch = note.pitch + 1;
                uint16_t len = note.len;
                note.len = 0;
                //myprintf3("i:%d p:%d l:%d \n",i,note.pitch,note.len);
//myprintf3("w:%d \n",wave_form_data[i]);
                noteBuff[i].pop();
                note_len[i] = (len * 1000 * 4 * 60) / (time_resolution * bpm);
                setWave(i, pitch, wave_form_data[i], wave_volume_data[i]);
//if (noteBuff[i].empty() ==true  )myprintf3("empty%d\n",i);

            } else setWave(i, nR, REST, 0);
        }
    }

}

void SoundManager::clearBuff() {
    for (int i = 0; i < track_num; i++) {
        while (!noteBuff[i].empty())
            noteBuff[i].pop();
        wave_period[i] = 1;
        wave[i][0] = 0;
        wave_volume_data[i] = 0;
    }
}

uint16_t SoundManager::calcWaveVal(uint32_t num_count) {
    uint16_t wave_val = 0;
    for (int i = 0; i < track_num; i++) {
        if (wave_period[i] != 0) {
            static uint16_t wave_val_ch = 0;
            if (wave_form_data[i] == NOISE1 && wave_period[i] != 1 && wave[i][0] != 0) wave_val_ch = (2 * xor32())
                    % (wave_master_volume[i] * wave_volume_data[i] / volume_resolution);
            else if (wave_form_data[i] == NOISE2 && wave_period[i] != 1 && wave[i][0] != 0) {
                if (xor32() % 5 == 0) wave_val_ch = xor32()
                        % (wave_master_volume[i] * wave_volume_data[i] / volume_resolution);
            } else wave_val_ch = wave[i][num_count % wave_period[i]];
            wave_val += wave_val_ch;
        }
    }

    return wave_val;
}

static SoundManager SEM;
static SoundManager BGMM;

typedef void (*FUNC)();
static std::queue<FUNC> bgmList;

extern "C" void timeInterrupt_sub(void);

void timeInterrupt_sub() {
    static uint64_t num_count = 0;

    uint16_t SEM_wave_val = SEM.calcWaveVal(num_count);
    uint16_t BGMM_wave_val = BGMM.calcWaveVal(num_count);
    uint16_t da_val = SEM_wave_val + BGMM_wave_val;

    setDA((da_val < 4000) ? da_val : 4000);

    num_count++;
    endTimeuCountIntCMT1();

}

//外部のタイマ割り込み内で呼ぶこと
void soundUpdate() {
    SEM.update();
    BGMM.update();

    bool isEmp = true;
    bool isAllNoteOff = true;
    for (int i = 0; i < BGMM.track_num; i++) {
        if (BGMM.noteBuff[i].empty() == false) isEmp = false;
        if (BGMM.note_len[i] != 0) isEmp = false;

    }
    if (isEmp == true && bgmList.empty() != true && isAllNoteOff) {
        bgmList.front()();
//myprintf3("in\n");
        bgmList.pop();
    }
}

void bgmClear() {
    while (!bgmList.empty())
        bgmList.pop();
    BGMM.clearBuff();
}

void addBgmList(FUNC bgm) {
    bgmList.push(bgm);
    waitmsec(10);
}

bool isEmptyBgmList() {
    if (bgmList.empty() == 0) return false;
    else return true;
}

void bpmTest(float bairitu) {
    for (int i = 0; i < BGMM.track_num; i++) {
        BGMM.note_len[i] *= bairitu;
    }
    BGMM.bpm = BGMM.bpm * bairitu;

}

//se
#include "soundEffect.h"
#include "famima.h"

//bgm
/*
#include "puyo1.h"
#include "nekuro.h"

#include "wily.h"
#include "robotol.h"
#include "gamecube.h"
#include "togetoge.h"

#include "road27.h"
#include "sky_high.h"

#include "meiji.h"
#include "otenba.h"
#include "aaa.h"
#include "owen.h"
//#include "nekuro.h"
#include "saigetu.h"
*/

