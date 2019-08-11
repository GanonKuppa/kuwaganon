#ifndef SOUND_H
#define SOUND_H

#include <stdint.h>


//
void soundUpdate();
typedef void (*FUNC)();
void addBgmList(FUNC);
void bgmClear();
void bpmTest(float bairitu);
bool isEmptyBgmList();

//se
void famima(void);
void famimaMinor(void);
void GB(void);
void SEA(void);
void SEB(void);
void SEC(void);
void SED(void);
void SEE(void);
void SEF(void);
void SEG(void);
void SEH(void);
void SEI(void);
void SEJ(void);
void RAMEN(void);
void randomNote(uint16_t seed);
void SE_I7(void);
void SE_Im7(void);
void SE_CursorMove(void);

void SE_CURSOR_MOVE();
void SE_CURSOR_MOVE(int8_t note_offset);
void SE_ON_ACTIVITY();
void SE_END_ACTIVITY();
void SE_CONFIRM();
void SE_POSITION_CHANGE();
void SE_CONTACT_WALL();
void SE_CORNER_R();
void SE_CORNER_L();

//bgm
/*
void oten(void);
void wily(void);
void robotol(void);
void gamecube(void);
void togetoge(void);
void road27(void);
void sky_high(void);

void meiji(void);
void aaa(void);
void otenba(void);
void owen(void);
void nekuro(void);
void saigetu(void);


void puyo1(void);
*/
//defineによる定数定義
#define nCm1   0
#define nCSm1  1
#define nDm1   2
#define nDSm1  3
#define nEm1   4
#define nFm1   5
#define nFSm1  6
#define nGm1   7
#define nGSm1  8
#define nAm1   9
#define nASm1  10
#define nBm1   11

#define nC0   12
#define nCS0  13
#define nD0   14
#define nDS0  15
#define nE0   16
#define nF0   17
#define nFS0  18
#define nG0   19
#define nGS0  20
#define nA0   21
#define nAS0  22
#define nB0   23

#define nC1   24
#define nCS1  25
#define nD1   26
#define nDS1  27
#define nE1   28
#define nF1   29
#define nFS1  30
#define nG1   31
#define nGS1  32
#define nA1   33
#define nAS1  34
#define nB1   35

#define nC2   36
#define nCS2  37
#define nD2   38
#define nDS2  39
#define nE2   40
#define nF2   41
#define nFS2  42
#define nG2   43
#define nGS2  44
#define nA2   45
#define nAS2  46
#define nB2   47

#define nC3   48
#define nCS3  49
#define nD3   50
#define nDS3  51
#define nE3   52
#define nF3   53
#define nFS3  54
#define nG3   55
#define nGS3  56
#define nA3   57
#define nAS3  58
#define nB3   59

#define nC4   60
#define nCS4  61
#define nD4   62
#define nDS4  63
#define nE4   64
#define nF4   65
#define nFS4  66
#define nG4   67
#define nGS4  68
#define nA4   69
#define nAS4  70
#define nB4   71

#define nC5   72
#define nCS5  73
#define nD5   74
#define nDS5  75
#define nE5   76
#define nF5   77
#define nFS5  78
#define nG5   79
#define nGS5  80
#define nA5   81
#define nAS5  82
#define nB5   83

#define nC6   84
#define nCS6  85
#define nD6   86
#define nDS6  87
#define nE6   88
#define nF6   89
#define nFS6  90
#define nG6   91
#define nGS6  92
#define nA6   93
#define nAS6  94
#define nB6   95

#define nC7   96
#define nCS7  97
#define nD7   98
#define nDS7  99
#define nE7   100
#define nF7   101
#define nFS7  102
#define nG7   103
#define nGS7  104
#define nA7   105
#define nAS7  106
#define nB7   107

#define nC8   108
#define nCS8  109
#define nD8   110
#define nDS8  111
#define nE8   112
#define nF8   113
#define nFS8  114
#define nG8   115
#define nGS8  116
#define nA8   117
#define nAS8  118
#define nB8   119

#define nDFm1 nCSm1
#define nEFm1 nDSm1
#define nGFm1 nFSm1
#define nAFm1 nGSm1
#define nBFm1 nASm1

#define nDF0  nCS0
#define nEF0  nDS0
#define nGF0  nFS0
#define nAF0  nGS0
#define nBF0  nAS0

#define nDF1  nCS1
#define nEF1  nDS1
#define nGF1  nFS1
#define nAF1  nGS1
#define nBF1  nAS1

#define nDF2  nCS2
#define nEF2  nDS2
#define nGF2  nFS2
#define nAF2  nGS2
#define nBF2  nAS2

#define nDF3  nCS3
#define nEF3  nDS3
#define nGF3  nFS3
#define nAF3  nGS3
#define nBF3  nAS3

#define nDF4  nCS4
#define nEF4  nDS4
#define nGF4  nFS4
#define nAF4  nGS4
#define nBF4  nAS4

#define nDF5  nCS5
#define nEF5  nDS5
#define nGF5  nFS5
#define nAF5  nGS5
#define nBF5  nAS5

#define nDF6  nCS6
#define nEF6  nDS6
#define nGF6  nFS6
#define nAF6  nGS6
#define nBF6  nAS6

#define nDF7  nCS7
#define nEF7  nDS7
#define nGF7  nFS7
#define nAF7  nGS7
#define nBF7  nAS7

#define nDF8  nCS8
#define nEF8  nDS8
#define nGF8  nFS8
#define nAF8  nGS8
#define nBF8  nAS8


#define nR    120 //休符


#define SANKAKU     0
#define NOKOGIRI    1
#define KUKEI12_5   2
#define KUKEI25     3
#define KUKEI50     4
#define KUKEI75     5
#define NOISE1      6
#define NOISE2      7
#define NOISE3      8
#define REST        9

//4分音符480tickとした際の各音符の長さ
#define lW 1920
#define lH  960
#define lQ  480
#define lE  240
#define lS  120
#define lTS  60

#define lWd  2880
#define lHd  1440
#define lQd   720
#define lEd   360
#define lSd   180


#define lHT 320
#define lQT 160
#define lE_H 1200

#define l______l 0


const uint16_t note10uCnt[] ={
	12231,11545,10897,10285,9708,9163,8649,8163,7705,7273,6865,6479, //Cm1 0
	6116,5772,5448,5143,4854,4582,4324,4082,3853,3636,3432,3240,     //C0  12
	3058,2886,2724,2571,2427,2291,2162,2041,1926,1818,1716,1620,     //C1  24
	1529,1443,1362,1286,1213,1145,1081,1020,963,909,858,810,         //C2  36
	764,722,681,643,607,573,541,510,482,455,429,405,                 //C3  48
	382,361,341,321,303,286,270,255,241,227,215,202,                 //C4  60
	191,180,170,161,152,143,135,128,120,114,107,101,                 //C5  72
 	 96, 90, 85, 80, 76, 72, 68, 64, 60, 57, 54, 51,                 //C6  84
	 48, 45, 43, 40, 38, 36, 34, 32, 30, 28, 27, 25,                 //C7  96
	 24, 23, 21, 20, 19, 18, 17, 16, 15, 14,  0,  0,                 //C8  108
	 0 ,  0,  0,  0,  0,   0,  0,  0,  0, 0,   0,  0

};







#endif
