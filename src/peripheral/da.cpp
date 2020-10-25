#include <stdint.h>
#include "iodefine.h"
#include "da.h"

#define SOUND_ENABLE 1
/**************DA変換の初期設定*****************/
void initDA(void) {
#if SOUND_ENABLE
    SYSTEM.PRCR.WORD = 0xA502;
    MSTP(DA) = 0;
    SYSTEM.PRCR.WORD = 0xA500;

    PORT0.PDR.BIT.B5 = 0;
    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;
    MPC.P05PFS.BYTE = 0x80;
    MPC.PWPR.BYTE = 0x80;
    PORT0.PMR.BIT.B5 = 0;

    DA.DACR.BYTE = 0x3F;

    DA.DADR1 = 0;
    DA.DACR.BYTE = 0xFF;
#endif
}

//RX71MのDACは12bit分解能なので0から4095までの値を設定可能
void setDA(uint16_t da) {
#if SOUND_ENABLE
    if(DA.DADR1 != da){
        DA.DADR1 = da;
        DA.DACR.BIT.DAOE1 = 1;
    } 
#endif
}

uint16_t getDA() {
    return DA.DADR1;
}


