


static const uint16_t CURSOR_MOVE[][2] ={
    {nR,l______l},
    {nB7,lS},
    {nR,lS},
    {nR,l______l}
};

void SE_CURSOR_MOVE(){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = SANKAKU;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    
    uint16_t len1 = sizeof(CURSOR_MOVE)/sizeof(CURSOR_MOVE[0]);
    for(int i=0;i<len1;i++){
        note.pitch = CURSOR_MOVE[i][0];
        note.len = CURSOR_MOVE[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}


void SE_CURSOR_MOVE(int8_t offset){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = SANKAKU;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;

    uint16_t len1 = sizeof(CURSOR_MOVE)/sizeof(CURSOR_MOVE[0]);
    for(int i=0;i<len1;i++){
        note.pitch = CURSOR_MOVE[i][0] + offset;
        note.len = CURSOR_MOVE[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}

static const uint16_t ON_ACTIVITY_1[][2] ={
    {nR,l______l},
    {nDS6,lS},
    {nF6,lS},
    {nR,lS},
    {nFS6,lS},
    {nR,l______l}
};


static const uint16_t ON_ACTIVITY_2[][2] ={
    {nR,l______l},
    {nDS6-7,lS},
    {nF6-7,lS},
    {nR,lS},
    {nFS6-7,lS},
    {nR,l______l}
};


void SE_ON_ACTIVITY(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = NOISE1;
    SEM.wave_form_data[7] = NOISE1;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 360;
    uint16_t len1 = sizeof(ON_ACTIVITY_1)/sizeof(ON_ACTIVITY_1[0]);
    for(int i=0;i<len1;i++){
        note.pitch = ON_ACTIVITY_1[i][0];
        note.len = ON_ACTIVITY_1[i][1];
        SEM.noteBuff[6].push(note);
    }

    uint16_t len2 = sizeof(ON_ACTIVITY_2)/sizeof(ON_ACTIVITY_2[0]);
    for(int i=0;i<len2;i++){
        note.pitch = ON_ACTIVITY_2[i][0];
        note.len = ON_ACTIVITY_2[i][1];
        SEM.noteBuff[7].push(note);
    }

    SEM.enable=true;
}

static const uint16_t END_ACTIVITY[][2] ={
    {nR,l______l},
    {nB5,lS},
    {nA5,lS},
    {nG5,lS},
    {nFS5,lS},
    {nF5,lS},
    {nE5,lS},
    {nDS5,lS},
    {nR,l______l}
};

void SE_END_ACTIVITY(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = KUKEI12_5;
    SEM.wave_form_data[7] = KUKEI12_5;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 160;
    uint16_t len1 = sizeof(END_ACTIVITY)/sizeof(END_ACTIVITY[0]);
    for(int i=0;i<len1;i++){
        note.pitch = END_ACTIVITY[i][0];
        note.len = END_ACTIVITY[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}

static const uint16_t CONFIRM[][2] ={
    {nR,l______l},
    {nA7,lS},
    {nA7,lS},
    {nR,l______l}
};

void SE_CONFIRM(int8_t offset){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = SANKAKU;
    SEM.wave_form_data[7] = SANKAKU;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    uint16_t len1 = sizeof(CONFIRM)/sizeof(CONFIRM[0]);
    for(int i=0;i<len1;i++){
        note.pitch = CONFIRM[i][0] + offset;
        note.len = CONFIRM[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}


static const uint16_t POSITION_CHANGE[][2] ={
    {nR,l______l},
    {nB6,lS},
    {nB6,lS},
    {nR,lS},
    {nB6,lS},
    {nR,l______l}
};

void SE_POSITION_CHANGE(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = KUKEI75;
    SEM.wave_form_data[7] = KUKEI75;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    uint16_t len1 = sizeof(POSITION_CHANGE)/sizeof(POSITION_CHANGE[0]);
    for(int i=0;i<len1;i++){
        note.pitch = POSITION_CHANGE[i][0];
        note.len = POSITION_CHANGE[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}

static const uint16_t CONTACT_WALL[][2] ={
    {nR,l______l},
    {nE6,lS},
    {nDS6,lS},
    {nR,l______l}
};

void SE_CONTACT_WALL(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = KUKEI75;
    SEM.wave_form_data[7] = KUKEI75;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    uint16_t len1 = sizeof(CONTACT_WALL)/sizeof(CONTACT_WALL);
    for(int i=0;i<len1;i++){
        note.pitch = CONTACT_WALL[i][0];
        note.len = CONTACT_WALL[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}



static const uint16_t CORNER_R[][2] ={
    {nR,l______l},
    {nDS7,lS},
    {nCS7,lS},
    {nC7,lS},
    {nR,l______l}
};

void SE_CORNER_R(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = KUKEI75;
    SEM.wave_form_data[7] = KUKEI75;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    uint16_t len1 = sizeof(CORNER_R)/sizeof(CORNER_R[0]);
    for(int i=0;i<len1;i++){
        note.pitch = CORNER_R[i][0];
        note.len = CORNER_R[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}

static const uint16_t CORNER_L[][2] ={
    {nR,l______l},
    {nCS7,lS},
    {nDS7,lS},
    {nE7,lS},
    {nR,l______l}
};

void SE_CORNER_L(void){
    Note note;
    SEM.enable=false;
    SEM.clearBuff();
    SEM.wave_form_data[6] = KUKEI75;
    SEM.wave_form_data[7] = KUKEI75;
    SEM.wave_volume_data[6] = 16;    //各トラックのボリューム  volume_resolution段階
    SEM.wave_volume_data[7] = 16;
    SEM.bpm = 120;
    uint16_t len1 = sizeof(CORNER_L)/sizeof(CORNER_L[0]);
    for(int i=0;i<len1;i++){
        note.pitch = CORNER_L[i][0];
        note.len = CORNER_L[i][1];
        SEM.noteBuff[6].push(note);
    }
    SEM.enable=true;
}


