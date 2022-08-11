#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define setBit(arr, k) (arr[k / 8] |= (1 << (7 - k % 8)))
#define clearBit(arr, k) (arr[k / 8] &= ~(1 << (7 - k % 8)))
#define testBit(arr, k) ((arr[k / 8] & (1 << (7 - k % 8))) != 0)

uint8_t *compress_data(uint8_t *data, uint8_t *len);
uint8_t set_N_bits_C(int8_t value, uint8_t num_bits);
float change_interval_C(float value, float in_min, float in_max, float out_min, float out_max);