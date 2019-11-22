#ifndef MYUTIL_H
#define MYUTIL_H

#include <stdint.h>

int16_t concatenate2Byte_int(uint8_t H_byte, uint8_t L_byte);
uint16_t concatenate2Byte_uint(uint8_t H_byte, uint8_t L_byte);
uint32_t concatenate4Byte_uint(uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0);

void quickSort_int16(int16_t numbers[], uint32_t left, uint32_t right);

float constrain(float amt, float low, float high);
float constrainH(float amt, float high);
float constrainL(float amt, float low);

uint8_t byteH(uint16_t val);
uint8_t byteL(uint16_t val);


uint32_t xor32(void);
uint32_t xor64(void);
uint32_t xor96(void);
uint32_t xor128(void);
uint32_t shortFreqNoise(void);
uint32_t longFreqNoise(void);

/***********便利なマクロ関数*********************/
#define ABS(IN) ((IN) < 0 ? - (IN) : (IN))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define SWAP(a,b) ((a != b) && (a += b,b = a - b,a -= b))
#define MAX3(a, b, c) ((a) > (MAX(b, c)) ? (a) : (MAX(b, c)))
#define MAX4(a, b, c, d) ((a) > (MAX3(b, c, d)) ? (a) : (MAX3(b, c, d)))
#define MIN3(a, b, c) ((a) < (MIN(b, c)) ? (a) : (MIN(b, c)))
#define MIN4(a, b, c, d) ((a) < (MIN3(b, c, d)) ? (a) : (MIN3(b, c, d)))
#define SIGN(x) ((x<0)?(-1):((x>0)?1:0))
#define DEG2RAD(deg) (((deg) * 3.1415926535897932) / 180.0)
#define RAD2DEG(rad) (((rad) * 180.0) / 3.1415926535)




#endif
