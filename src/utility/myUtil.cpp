/*
 * myUtil.cpp
 *
 *  Created on: 2016/10/02
 *      Author: ryota
 */

#include <myUtil.h>
#include <stdio.h>
#include <stdint.h>



int16_t concatenate2Byte_int(uint8_t H_byte, uint8_t L_byte) {
  int32_t con;
  con = L_byte + (H_byte<<8);
  if (con > 32767) {
    con -=  65536;
  }
  return (int16_t)con;
}

uint16_t concatenate2Byte_uint(uint8_t H_byte, uint8_t L_byte) {
  int32_t con;
  con = L_byte + (H_byte<<8);
  return (uint16_t)con;
}

uint32_t concatenate4Byte_uint(uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0) {
	uint32_t con;
	con = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0;
	return con;
}


void quickSort_int16(int16_t numbers[], uint32_t left, uint32_t right)
{
    uint32_t pivot, l_hold, r_hold;

    l_hold = left;
    r_hold = right;
    pivot = numbers[left];
    while (left < right)
    {
        while ((numbers[right] >= pivot) && (left < right))
            right--;
        if (left != right)
        {
            numbers[left] = numbers[right];
            left++;
        }
        while ((numbers[left] <= pivot) && (left < right))
            left++;
        if (left != right)
        {
            numbers[right] = numbers[left];
            right--;
        }
    }
    numbers[left] = pivot;
    pivot = left;
    left = l_hold;
    right = r_hold;
    if (left < pivot)
        quickSort_int16(numbers, left, pivot-1);
    if (right > pivot)
        quickSort_int16(numbers, pivot+1, right);
}

float constrain(float amt, float low, float high){
	if(amt > high) return high;
	else if(amt < low) return low;
	else return amt;
}

float constrainH(float amt, float high){
	if(amt > high) return high;
	else return amt;
}

float constrainL(float amt, float low){
	if(amt < low) return low;
	else return amt;
}

uint8_t byteH(uint16_t val){
    return (0x0000ff00 & val) >> 8;
}

uint8_t byteL(uint16_t val){
    return (0x000000ff & val);
}


uint32_t xor32(void) {
  static uint32_t y = 2463534242;
  y = y ^ (y << 13); y = y ^ (y >> 17);
  return y = y ^ (y << 5);
}

uint32_t xor64(void) {
  static uint64_t x = 88172645463325252ULL;
  x = x ^ (x << 13); x = x ^ (x >> 7);
  return x = x ^ (x << 17);
}

uint32_t xor96(void) {
  static uint32_t x = 123456789;
  static uint32_t y = 362436069;
  static uint32_t z = 521288629;
  uint32_t t;

  t = (x ^ (x << 3)) ^ (y ^ (y >> 19)) ^ (z ^ (z << 6));
  x = y; y = z;
  return z = t;
}

uint32_t xor128(void) {
  static uint32_t x = 123456789;
  static uint32_t y = 362436069;
  static uint32_t z = 521288629;
  static uint32_t w = 88675123;
  uint32_t t;

  t = x ^ (x << 11);
  x = y; y = z; z = w;
  return w = (w ^ (w >> 19)) ^ (t ^ (t >> 8));
}



uint32_t shortFreqNoise(){
static uint32_t reg = 0x8000;

	//以下の3行を回す。
	//　shortFreqは、短周期フラグ。1にすると有効になる。
	//　これで得られるoutputの値が、ノイズチャンネルの波形。
	uint32_t shortFreq = 1;
	reg >>= 1;
	reg |= ((reg ^ (reg >> (shortFreq ? 6 : 1))) & 1) << 15;
	return reg & 1;
}

uint32_t longFreqNoise(){
static uint32_t reg = 0x8000;

	//以下の3行を回す。
	//　shortFreqは、短周期フラグ。1にすると有効になる。
	//　これで得られるoutputの値が、ノイズチャンネルの波形。
	uint32_t shortFreq = 0;
	reg >>= 1;
	reg |= ((reg ^ (reg >> (shortFreq ? 6 : 1))) & 1) << 15;
	return reg & 1;
}


