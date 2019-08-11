/*
 * sound.cpp
 *
 *  Created on: 2016/12/03
 *      Author: ryota
 */

#ifdef REGA

#include "sound.h"
#include "myUtil.h"
#include "timeInterrupt.hpp"
#include "da.h"
#include "myUtil.h"
#include "timer.h"
#include "uart.hpp"
#include <queue>


using std::queue;

struct Note;
class SoundEffectManager;


uint16_t wave[8][1000];          //各トラックの音の一周期分の波形
uint16_t wave_period[8];         //各トラックの音の一周期分のカウント数
uint8_t  wave_form_data[8];      //各トラックの波形
uint8_t  wave_volume_data[8];    //各トラックのボリューム 16段階




const uint16_t wave_master_volume[8]={
	1000,1000,1000,500,
	500,500,500,500
};


void waveInit(){
	for(int j=0;j<8;j++){
		for(int i=0;i<1000;i++){
			wave[j][i] = 0;
		}
	}

	for(int k=0;k<8;k++) wave_period[k]=0;
	for(int k=0;k<8;k++) wave_form_data[k]=0;

}


void setWave(uint8_t track_num, uint8_t pitch, uint8_t waveform, uint8_t wave_volume){
	uint16_t wave_count = 0;
	uint16_t wave_count_max = note10uCnt[pitch];
	uint16_t wave_amplitude_max = 0;

	wave_period[track_num]      = wave_count_max;
	wave_form_data[track_num]   = waveform;
	wave_volume_data[track_num] = wave_volume;

	if(wave_volume != 0) wave_amplitude_max = wave_master_volume[track_num] *wave_volume/16;
	else wave_amplitude_max = 0;

	if(pitch==nR || wave_amplitude_max == 0 || waveform == REST){
		wave_period[track_num]      = 1;
		wave[track_num][0] = 0;
		return;
	}

	switch(waveform){
		case SANKAKU:
		for(int i=0;i< wave_count_max;i++){
			if(i < wave_count_max/2) wave[track_num][i]= i*(2*wave_amplitude_max)/wave_count_max;
			else{
				int16_t val = ((wave_count/2)/(0-wave_amplitude_max))*(i-wave_count_max);
				if(val > 0) wave[track_num][i] = val;
				else wave[track_num][i] = 0;
			}
		}
			break;
	    case NOKOGIRI:
	    	for(int i=0;i< wave_count_max;i++){
	    		wave[track_num][i] = i*wave_amplitude_max/wave_count_max;
	    	}

			break;
	    case KUKEI12_5:
	    	for(int i=0;i< wave_count_max;i++){
				if(i < wave_count_max/8) wave[track_num][i]= wave_amplitude_max;
				else wave[track_num][i] = 0;
			}
			break;
	    case KUKEI25:
	    	for(int i=0;i< wave_count_max;i++){
				if(i < wave_count_max/4) wave[track_num][i]= wave_amplitude_max;
				else wave[track_num][i] = 0;
	    	}
	    	break;
	    case KUKEI50:
	    	for(int i=0;i< wave_count_max;i++){
				if(i< wave_count_max/2) wave[track_num][i]= wave_amplitude_max;
				else wave[track_num][i] = 0;
	    	}
	    	break;
	    case KUKEI75:
	    	for(int i=0;i< wave_count_max;i++){
				if(i< wave_count_max*3/4) wave[track_num][i]= wave_amplitude_max;
				else wave[track_num][i] = 0;
	    	}
			break;
	}

};

struct Note{
	uint8_t pitch;
	uint16_t len;
};

class SoundEffectManager{
public:
	const uint16_t timeResolution = 480;
	uint16_t bpm = 80;

	queue<Note> noteBuff6;
	queue<Note> noteBuff7;
	uint32_t note_len6;
	uint32_t note_len7;
	uint8_t  note_form6;
	uint8_t  note_form7;
	uint8_t  note_volume6;
	uint8_t  note_volume7;
	//60秒に480*100 -> 1秒に800 1カウントは1/800秒
	//(1/(timeResolution*bpm/60))*1000*1000/10= time*100000*60/(timeResolutio*bpm)
	SoundEffectManager(void){
		note_len6=0;
		note_len7=0;
		note_form6 = KUKEI75;
		note_form7 = KUKEI25;
		note_volume6 =16;
		note_volume7 =10;
	}


} SEM ;


extern "C" void timeInterrupt_sub(void);

void timeInterrupt_sub(){
	static uint16_t num_count = 0;


	uint16_t wave_val = 0;
	for(int i=0;i<8;i++){
		if(wave_period[i]!=0){
			static uint16_t wave_val_ch = 0;
			if(wave_form_data[i] == NOISE1) wave_val_ch = (2*xor32()) % (wave_master_volume[i] * wave_volume_data[i]/16);
			else if(wave_form_data[i] == NOISE2){
				if(xor32() %5 == 0) wave_val_ch = xor32() % (wave_master_volume[i] * wave_volume_data[i]/16);
			}
			//else if(wave_form_data[i] == REST) wave_val_ch =0;
			else wave_val_ch = wave[i][num_count % wave_period[i]];
			wave_val += wave_val_ch;
		}
	}
	//if(getElapsedMsec() %500 <240)  wave_val += xor128() % 600;
	setDA(wave_val);



	//if(SEM.note_len8 !=0)SEM.note_len8--;
	//else

	num_count ++;
	endTimeuCountIntCMT1();

}


class  BgmManager{

};








const uint16_t famima_u[14][2] ={
	{nR,l______l},
	{nFS6,lE},
	{nD6,lE},
	{nA5,lE},
	{nD6,lE},
	{nE6,lE},
	{nA6,lQd},
	{nR,l______l},
	{nE6,lE},
	{nFS6,lE},
	{nE6,lE},
	{nA5,lE},
	{nD6,lH},
	{nR,l______l}
};

const uint16_t famima_l[11][2] ={
	{nR,l______l},
	{nR,lQ},
	{nFS5,lQ},
	{nCS6,lQd},
	{nA5,lE},
	{nR,l______l},
	{nA5,lQ},
	{nA5,lE},
	{nA5,lE},
	{nFS5,lH},
	{nR,l______l}
};


const uint16_t famimaMinor_u[14][2] ={
	{nR,l______l},
	{nF6,lE},
	{nD6,lE},
	{nA5,lE},
	{nD6,lE},
	{nE6,lE},
	{nA6,lQd},
	{nR,l______l},
	{nE6,lE},
	{nF6,lE},
	{nE6,lE},
	{nA5,lE},
	{nD6,lH},
	{nR,l______l}
};

const uint16_t famimaMinor_l[11][2] ={
	{nR,l______l},
	{nR,lQ},
	{nF5,lQ},
	{nCS6,lQd},
	{nA5,lE},
	{nR,l______l},
	{nA5,lQ},
	{nA5,lE},
	{nA5,lE},
	{nFS5,lH},
	{nR,l______l}
};






void famima(void){
	Note note;
	for(int i=0;i<14;i++){
		note.pitch = famima_u[i][0];
		note.len = famima_u[i][1];
		SEM.noteBuff6.push(note);
	}

	for(int i=0;i<12;i++){
		note.pitch = famima_l[i][0];
		note.len = famima_l[i][1];
		SEM.noteBuff7.push(note);
	}

	/*
	for(int i=0;i<14;i++){
			note.pitch = famima_m[i][0];
			note.len = famima_m[i][1];
			myprintf3("i:%d p:%d \n",i,note.pitch);
			SEM.noteBuff7.push(note);
		}
		*/
}


void famimaMinor(void){
	Note note;
	for(int i=0;i<14;i++){
		note.pitch = famimaMinor_u[i][0];
		note.len = famimaMinor_u[i][1];
		myprintf3("i:%d p:%d \n",i,note.pitch);
		SEM.noteBuff6.push(note);
	}

	for(int i=0;i<12;i++){
		note.pitch = famimaMinor_l[i][0];
		note.len = famimaMinor_l[i][1];
		myprintf3("i:%d p:%d \n",i,note.pitch);
		SEM.noteBuff7.push(note);
	}

	/*
	for(int i=0;i<14;i++){
			note.pitch = famima_m[i][0];
			note.len = famima_m[i][1];
			myprintf3("i:%d p:%d \n",i,note.pitch);
			SEM.noteBuff7.push(note);
		}
		*/
}



void updateSEM(){
	uint16_t note_off_count = 0; //発音された音の最後を少し削る

	if(SEM.note_len6 !=0){
		SEM.note_len6--;
		if(SEM.note_len6 < 150) setWave(6,nR, REST, 0);
	}

	else{
		if(SEM.noteBuff6.size() != 0){
			myprintf3("note_len6: %d\n",SEM.note_len6);

			myprintf3("num: %d\n",SEM.noteBuff6.size());
			Note note = SEM.noteBuff6.front();
			uint16_t pitch = note.pitch;
			uint16_t len   = note.len;
			myprintf3("P:%d,L:%d\n",pitch,len);
			SEM.noteBuff6.pop();
			SEM.note_len6 = (len*1000 *4 *60)/(SEM.timeResolution*SEM.bpm);
			setWave(6, pitch, SEM.note_form6,  SEM.note_volume6);
		}
		else  setWave(6, nR, REST,  0);
	}


	if(SEM.note_len7 > 0){
		SEM.note_len7--;
		if(SEM.note_len7 < 150) setWave(7,nR, REST, 0);
	}
	else{
		if(SEM.noteBuff7.size() > 0){
			myprintf3("note_len7: %d\n",SEM.note_len7);

			myprintf3("num: %d\n",SEM.noteBuff7.size());
			Note note = SEM.noteBuff7.front();
			uint16_t pitch = note.pitch;
			uint16_t len   = note.len;
			myprintf3("P:%d,L:%d\n",pitch,len);
			SEM.noteBuff7.pop();
			SEM.note_len7 = (len*1000 *4 *60)/(SEM.timeResolution*SEM.bpm);
			setWave(7, pitch, SEM.note_form7,  SEM.note_volume7);
		}
		else  setWave(7, nR, REST,  0);
	}


}



#endif
