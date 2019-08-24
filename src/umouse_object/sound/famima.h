
const uint16_t famima_u[][2] ={
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

const uint16_t famima_l[][2] ={
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

const uint16_t famimaMinor_u[][2] ={
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

const uint16_t famimaMinor_l[][2] ={
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

const uint16_t GB1[][2] ={
	{nR,l______l},
	{nC6,lS},
	{nC7,lH},
	{nR,l______l}
};

const uint16_t SEA1[][2] ={
	{nR,l______l},
	{nB7,lS},
	{nR,lS},
	{nR,l______l}
};

const uint16_t SEB1[][2] ={
	{nR,l______l},
	{nBF7,lS},
	{nR,lS},
	{nR,l______l}
};

const uint16_t SEC1[][2] ={
	{nR,l______l},
	{nB5,lS},
	{nR,lS},
	{nR,l______l}
};

const uint16_t SED1[][2] ={
    {nR,l______l},
    {nE8,lS},
    {nC8,lE},
    {nR,lS},
    {nR,l______l}
};

const uint16_t SEE1[][2] ={
    {nR,l______l},
    {nD8,lTS},
    {nE8,lTS},
    {nFS8,lE},
    {nR,lS},
    {nR,l______l}
};

const uint16_t SEF1[][2] ={
    {nR,l______l},
    {nA7,lE},
    {nA7,lE},
    {nF7,lE},
    {nF7,lE},
    {nG7,lQ},
    {nD7,lE},
    {nD7,lE},
    {nR,l______l},
    {nC7,lE},
    {nC7,lE},
    {nD7,lQ},
    {nG7,lQ},
    {nF7,lQ},
    {nR,l______l}
};

const uint16_t SEG1[][2] ={
    {nR,l______l},
    {nG5,lEd},
    {nC6,lS},
    {nR,lS},
    {nA5,lS},
    {nD6,lE+lQ+lQ},
    {nR,l______l},
};

const uint16_t SEG2[][2] ={
    {nR,l______l},
    {nF5,lEd},
    {nF5,lS},
    {nR,lS},
    {nF5,lS},
    {nF5,lE+lQ+lQ},
    {nR,l______l},
};

const uint16_t SEG3[][2] ={
    {nR,l______l},
    {nR,lEd},
    {nR,lS},
    {nR,lS},
    {nA5,lS},
    {nR,lE+lQ+lQ},
    {nR,l______l},
};



const uint16_t RAMEN1[][2]{
    {nR,l______l},
    {nCS7,lQT},
    {nD7,lQT},
    {nE7,lQT},
    {nR,l______l},
    {nB7,lQT*2},
    {nG7,lQT},
    {nR, lQT*2},
    {nG7,lQT},
    {nD7,lQT},
    {nFS7,lQT},
    {nG7,lQT},
    {nR, lQT*2},
    {nG7,lQT},
    {nR,l______l},
    {nA7,lQT*2},
    {nG7,lQT},
    {nFS7,lQT*2},
    {nG7,lQT+lQ},
    {nR,lQ}
};

const uint16_t RAMEN2[][2]{
    {nR,l______l},
    {nR,lQT},
    {nR,lQT},
    {nR,lQT},
    {nR,l______l},
    {nC7,lQT*2},
    {nR,lQ+lQT},
    {nB6,lQT*2-lQT},
    {nR,lQ+lQT},
    {nR,l______l},
    {nA6,lQT*2},
    {nR,lQT},
    {nR,lQT*2},
    {nG6,lQT+lQ},
    {nR,lQ}
};

const uint16_t SEH1[][2] ={
    {nR,l______l},
    {nD8,lTS},
    {nDS8,lTS},
    {nR,lS},
    {nR,l______l}
};

const uint16_t SEI1[][2] ={
    {nR,l______l},
    {nD8,lTS},
    {nCS8,lTS},
    {nR,lS},
    {nR,l______l}
};


const uint16_t SEJ1[][2] ={
    {nR,l______l},
    {nE8,lTS},
    {nFS8,lTS},
    {nE8,lTS},
    {nFS8,lTS},
    {nGS8,lTS},
    {nR,lS},
    {nR,l______l}
};




void GB(void){
	Note note;
	SEM.enable=false;
	SEM.clearBuff();
	SEM.wave_form_data[6] = SANKAKU;
	SEM.wave_form_data[7] = SANKAKU;
	SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
	SEM.wave_volume_data[7] = 16;
	SEM.bpm = 120;
	uint16_t len1 = sizeof(GB1)/sizeof(GB1[0]);
	for(int i=0;i<len1;i++){
		note.pitch = GB1[i][0];
		note.len = GB1[i][1];
		SEM.noteBuff[6].push(note);
	}
	SEM.enable=true;
}

void SEA(void){
	Note note;
	SEM.enable=false;
	SEM.clearBuff();
	SEM.wave_form_data[6] = SANKAKU;
	SEM.wave_form_data[7] = SANKAKU;
	SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
	SEM.wave_volume_data[7] = 16;
	SEM.bpm = 120;
	uint16_t len1 = sizeof(SEA1)/sizeof(SEA1[0]);
	for(int i=0;i<len1;i++){
		note.pitch = SEA1[i][0];
		note.len = SEA1[i][1];
		SEM.noteBuff[6].push(note);
	}
	SEM.enable=true;
}


void SEB(void){
	Note note;
	SEM.enable=false;
	SEM.clearBuff();
	SEM.wave_form_data[6] = SANKAKU;
	SEM.wave_form_data[7] = SANKAKU;
	SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
	SEM.wave_volume_data[7] = 16;
	SEM.bpm = 120;
	uint16_t len1 = sizeof(SEB1)/sizeof(SEB1[0]);
	for(int i=0;i<len1;i++){
		note.pitch = SEB1[i][0];
		note.len = SEB1[i][1];
		SEM.noteBuff[6].push(note);
	}
	SEM.enable=true;
}

void SEC(void){
	Note note;
	SEM.enable=false;
	SEM.clearBuff();
	SEM.wave_form_data[6] = SANKAKU;
	SEM.wave_form_data[7] = SANKAKU;
	SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
	SEM.wave_volume_data[7] = 16;
	SEM.bpm = 120;
	uint16_t len1 = sizeof(SEC1)/sizeof(SEC1[0]);
	for(int i=0;i<len1;i++){
		note.pitch = SEC1[i][0];
		note.len = SEC1[i][1];
		SEM.noteBuff[6].push(note);
	}
	SEM.enable=true;
}

void SED(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = SANKAKU;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    uint16_t len1 = sizeof(SED1)/sizeof(SED1[0]);
    for(int i=0;i<len1;i++){
        note.pitch = SED1[i][0];
        note.len = SED1[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}

void SEE(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = SANKAKU;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    uint16_t len1 = sizeof(SEE1)/sizeof(SEE1[0]);
    for(int i=0;i<len1;i++){
        note.pitch = SEE1[i][0];
        note.len = SEE1[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}

void SEF(int8_t offset){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = SANKAKU;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 240;
    uint16_t len1 = sizeof(SEF1)/sizeof(SEF1[0]);
    for(int i=0;i<len1;i++){
        note.pitch = SEF1[i][0] + offset;
        note.len = SEF1[i][1];
        SEM.noteBuff[6].push(note);
        SEM.noteBuff[7].push(note);
    }
    SEM.enable=true;
}

void SEG(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[5] = SANKAKU;
    SEM.wave_form_data[6] = SANKAKU;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[5] = 12;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[6] = 6;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 6;
    SEM.bpm = 170;
    uint16_t len1 = sizeof(SEG1)/sizeof(SEG1[0]);
    for(int i=0;i<len1;i++){
        note.pitch = SEG1[i][0];
        note.len = SEG1[i][1];
        SEM.noteBuff[5].push(note);
    }

    uint16_t len2 = sizeof(SEG2)/sizeof(SEG2[0]);
    for(int i=0;i<len1;i++){
        note.pitch = SEG2[i][0];
        note.len = SEG2[i][1];
        SEM.noteBuff[6].push(note);
    }


    uint16_t len3 = sizeof(SEG3)/sizeof(SEG3[0]);
    for(int i=0;i<len1;i++){
        note.pitch = SEG3[i][0];
        note.len = SEG3[i][1];
        SEM.noteBuff[7].push(note);
    }

    SEM.enable=true;
}


void RAMEN(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] =  KUKEI75;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[6] = 32;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 0;
    SEM.bpm = 250;
    uint16_t len1 = sizeof(RAMEN1)/sizeof(RAMEN1[0]);
    for(int i=0;i<len1;i++){
        note.pitch = RAMEN1[i][0];
        note.len = RAMEN1[i][1];
        SEM.noteBuff[6].push(note);
    }

    SEM.enable=true;
}

void SEH(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = SANKAKU;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    uint16_t len1 = sizeof(SEH1)/sizeof(SEH1[0]);
    for(int i=0;i<len1;i++){
        note.pitch = SEH1[i][0];
        note.len = SEH1[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}


void SEI(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = SANKAKU;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    uint16_t len1 = sizeof(SEI1)/sizeof(SEI1[0]);
    for(int i=0;i<len1;i++){
        note.pitch = SEI1[i][0];
        note.len = SEI1[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}



void SEJ(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = SANKAKU;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    uint16_t len1 = sizeof(SEJ1)/sizeof(SEJ1[0]);
    for(int i=0;i<len1;i++){
        note.pitch = SEJ1[i][0];
        note.len = SEJ1[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}

const uint16_t SE_I7_1[][2] ={
    {nR,l______l},
    {nC5,lEd},
    {nR,l______l},
};

const uint16_t SE_I7_2[][2] ={
    {nR,l______l},
    {nE5,lEd},
    {nR,l______l},
};

const uint16_t SE_I7_3[][2] ={
    {nR,l______l},
    {nG5,lEd},
    {nR,l______l},
};

const uint16_t SE_I7_4[][2] ={
    {nR,l______l},
    {nBF5,lEd},
    {nR,l______l},
};

void SE_I7(void){
    Note note;
      SEM.enable=false;
      SEM.clearBuff();
      SEM.wave_form_data[4] = SANKAKU;
      SEM.wave_form_data[5] = SANKAKU;
      SEM.wave_form_data[6] = SANKAKU;
      SEM.wave_form_data[7] = SANKAKU;
      SEM.wave_volume_data[4] = 10;    //各トラックのボリューム  volume_resolution段階
      SEM.wave_volume_data[5] = 10;    //各トラックのボリューム  volume_resolution段階
      SEM.wave_volume_data[6] = 10;    //各トラックのボリューム  volume_resolution段階
      SEM.wave_volume_data[7] = 10;
      SEM.bpm = 170;
      uint16_t len1 = sizeof(SE_I7_1)/sizeof(SE_I7_1[0]);
      for(int i=0;i<len1;i++){
          note.pitch = SE_I7_1[i][0];
          note.len = SE_I7_1[i][1];
          SEM.noteBuff[4].push(note);
      }

      uint16_t len2 = sizeof(SE_I7_2)/sizeof(SE_I7_2[0]);
      for(int i=0;i<len2;i++){
          note.pitch = SE_I7_2[i][0];
          note.len = SE_I7_2[i][1];
          SEM.noteBuff[5].push(note);
      }


      uint16_t len3 = sizeof(SE_I7_3)/sizeof(SE_I7_3[0]);
      for(int i=0;i<len3;i++){
          note.pitch = SE_I7_3[i][0];
          note.len = SE_I7_3[i][1];
          SEM.noteBuff[6].push(note);
      }

      uint16_t len4 = sizeof(SE_I7_4)/sizeof(SE_I7_4[0]);
      for(int i=0;i<len4;i++){
          note.pitch = SE_I7_4[i][0];
          note.len = SE_I7_4[i][1];
          SEM.noteBuff[7].push(note);
      }


      SEM.enable=true;
}


const uint16_t SE_Im7_1[][2] ={
    {nR,l______l},
    {nC5,lEd},
    {nR,l______l},
};

const uint16_t SE_Im7_2[][2] ={
    {nR,l______l},
    {nEF5,lEd},
    {nR,l______l},
};

const uint16_t SE_Im7_3[][2] ={
    {nR,l______l},
    {nG5,lEd},
    {nR,l______l},
};

const uint16_t SE_Im7_4[][2] ={
    {nR,l______l},
    {nBF5,lEd},
    {nR,l______l},
};

void SE_Im7(void){
    Note note;
      SEM.enable=false;
      SEM.clearBuff();
      SEM.wave_form_data[4] = SANKAKU;
      SEM.wave_form_data[5] = SANKAKU;
      SEM.wave_form_data[6] = SANKAKU;
      SEM.wave_form_data[7] = SANKAKU;
      SEM.wave_volume_data[4] = 10;    //各トラックのボリューム  volume_resolution段階
      SEM.wave_volume_data[5] = 10;    //各トラックのボリューム  volume_resolution段階
      SEM.wave_volume_data[6] = 10;    //各トラックのボリューム  volume_resolution段階
      SEM.wave_volume_data[7] = 10;
      SEM.bpm = 170;
      uint16_t len1 = sizeof(SE_Im7_1)/sizeof(SE_Im7_1[0]);
      for(int i=0;i<len1;i++){
          note.pitch = SE_Im7_1[i][0];
          note.len = SE_Im7_1[i][1];
          SEM.noteBuff[4].push(note);
      }

      uint16_t len2 = sizeof(SE_Im7_2)/sizeof(SE_Im7_2[0]);
      for(int i=0;i<len2;i++){
          note.pitch = SE_Im7_2[i][0];
          note.len = SE_Im7_2[i][1];
          SEM.noteBuff[5].push(note);
      }


      uint16_t len3 = sizeof(SE_Im7_3)/sizeof(SE_Im7_3[0]);
      for(int i=0;i<len3;i++){
          note.pitch = SE_Im7_3[i][0];
          note.len = SE_Im7_3[i][1];
          SEM.noteBuff[6].push(note);
      }

      uint16_t len4 = sizeof(SE_Im7_4)/sizeof(SE_Im7_4[0]);
      for(int i=0;i<len4;i++){
          note.pitch = SE_Im7_4[i][0];
          note.len = SE_Im7_4[i][1];
          SEM.noteBuff[7].push(note);
      }


      SEM.enable=true;
}


void famima(void){
	Note note;
	SEM.enable=false;
	SEM.clearBuff();
	SEM.wave_form_data[6] = SANKAKU;
	SEM.wave_form_data[7] = SANKAKU;
	SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
	SEM.wave_volume_data[7] = 16;
	SEM.bpm = 80;
	uint16_t len1 = sizeof(famima_u)/sizeof(famima_u[0]);
	for(int i=0;i<len1;i++){
		note.pitch = famima_u[i][0];
		note.len = famima_u[i][1];
		SEM.noteBuff[6].push(note);
	}
	uint16_t len2 = sizeof(famima_l)/sizeof(famima_l[0]);
	for(int i=0;i<len2;i++){
		note.pitch = famima_l[i][0];
		note.len = famima_l[i][1];
		SEM.noteBuff[7].push(note);
	}
	SEM.enable=true;

}

void famimaMinor(void){
	Note note;

	SEM.enable=false;
	SEM.clearBuff();
	SEM.wave_form_data[6] = SANKAKU;
	SEM.wave_form_data[7] = SANKAKU;
	SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
	SEM.wave_volume_data[7] = 16;
	SEM.bpm = 80;
	uint16_t len1 = sizeof(famimaMinor_u)/sizeof(famimaMinor_u[0]);
	for(int i=0;i<len1;i++){
		note.pitch = famimaMinor_u[i][0];
		note.len = famimaMinor_u[i][1];
		SEM.noteBuff[6].push(note);
	}
	uint16_t len2 = sizeof(famimaMinor_l)/sizeof(famimaMinor_l[0]);

	for(int i=0;i<len2;i++){
		note.pitch = famimaMinor_l[i][0];
		note.len = famimaMinor_l[i][1];
		SEM.noteBuff[7].push(note);
	}
	SEM.enable=false;

}


#include <myUtil.h>
void randomNote(uint16_t seed){
	Note note;

	SEM.enable=false;
	SEM.clearBuff();
	SEM.wave_form_data[6] = SANKAKU;
	SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
	SEM.bpm = 80;

	note.pitch = 72 + ((seed * xor32() ) %24);
	note.len = lS;
	SEM.noteBuff[6].push(note);

	note.pitch = 72 + ((seed * xor32() ) %24);
	note.len = lS;
	SEM.noteBuff[6].push(note);
	SEM.enable=true;

}
