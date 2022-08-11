#include "compress.h"

uint8_t *compress_data(uint8_t *data, uint8_t *len)
{
    static uint8_t bytes[100];
    memset(bytes, 0, 100);
    int16_t data_difference[*len];
    memset(data_difference, 0, *len);

    data_difference[0] = data[0];
    for (uint8_t i = 1; i < *len; i++)
        data_difference[i] = data[i] - data[i - 1];

    int16_t cur_diff;
    uint8_t repeated_value_counter = 0;
    uint16_t cur_bit = 0;
    uint8_t ranges[] = {2, 6, 14, 30};
    // Start of the algorithm
    // Compress rules:
    // 00 - Differences
    //     00 + 2 bits [-2, -1] [1, 2] - 00 (-2) -> 11 (2)
    //     01 + 3 bits [-6, -3] [3, 6] - 000 (-6) -> 111 (6)
    //     10 + 4 bits [-14, -7] [7, 14] - 0000 (-14) -> 1111 (14)
    //     11 + 5 bits [-30, -15] [15, 30] - 0000 (-30) -> 1111 (30)
    // 01 - Repetitions
    //     3 bits counter of repetitions
    //     Max rep. 8, if > repeat rule
    //     000 (1) -> 111 (8)
    // 10 - Absolute coding
    //     Use if diff > 30
    //     9 bits - 0/1 sign + 8 bits for diff
    // 11 - End
    //     Use when reached the end of difference array
    //     01 - 3 bit [-6, -3] [3, 6]
    //     10 - 4 bit [-14, -7] [7, 14]
    //     11 - 5 bit [-30, -15] [15, 30]

    // First value as original value from array as 8-bit
    bytes[0] = data_difference[0];
    cur_bit += 8;
    // Continue through whole difference array and use compress rules
    for (uint8_t idx = 1; idx < *len; idx++)
    {
        cur_diff = data_difference[idx];
        if (cur_diff == 0)
        {
            repeated_value_counter++;
            if (idx + 1 < *len)
                continue;
        }
        // Check if counter is > 0 -> compress with rule 01
        if (repeated_value_counter != 0)
        {
            for (int8_t counter = repeated_value_counter; counter > 0; counter -= 8)
            {
                // 01
                clearBit(bytes, cur_bit);
                cur_bit++;
                setBit(bytes, cur_bit);
                cur_bit++;
                // 3 bits for (counter - 1)
                uint8_t temp[] = {counter > 8 ? 8 - 1 : counter - 1};
                for (uint8_t k = 5; k < 8; k++, cur_bit++)
                	testBit(temp, k) ? setBit(bytes, cur_bit) : clearBit(bytes, cur_bit);
            }
            repeated_value_counter = 0;
            if (cur_diff == 0)
                continue;
        }
        // Check size of diff -> if > 30 compress with rule 10
        if (cur_diff < -ranges[3] || cur_diff > ranges[3])
        {
            // 10
            setBit(bytes, cur_bit);
            cur_bit++;
            clearBit(bytes, cur_bit);
            cur_bit++;
            // check sign 1/0
            cur_diff < 0 ? setBit(bytes, cur_bit) : clearBit(bytes, cur_bit);
            cur_bit++;
            // 8 bits for value (positive)
            uint8_t temp[] = {abs(cur_diff)};
            for (uint8_t k = 0; k < 8; k++, cur_bit++)
            	testBit(temp, k) ? setBit(bytes, cur_bit) : clearBit(bytes, cur_bit);
            continue;
        }
        // Compress with rule 00
        // 00
        clearBit(bytes, cur_bit);
        cur_bit++;
        clearBit(bytes, cur_bit);
        cur_bit++;
        for (uint8_t i = 0; i < 4; i++)
        {
            if (abs(cur_diff) <= ranges[i])
            {
                // 00 or 01 or 10 or 11
                uint8_t temp[] = {i};
                for (uint8_t k = 6; k < 8; k++, cur_bit++)
                	testBit(temp, k) ? setBit(bytes, cur_bit) : clearBit(bytes, cur_bit);
                // 2 - 5 bits for cur_value
                uint8_t needed_bits = i + 2;
                // Set N bits for cur_value in i-th range
                uint8_t temp2[] = {set_N_bits_C(cur_diff, needed_bits)};
                for (uint8_t k = 8 - needed_bits; k < 8; k++, cur_bit++)
                	testBit(temp2, k) ? setBit(bytes, cur_bit) : clearBit(bytes, cur_bit);
                break;
            }
        }
    }
    // Reached the end of difference array -> compress with rule 11
    // 11
    setBit(bytes, cur_bit);
    cur_bit++;
    setBit(bytes, cur_bit);
    cur_bit++;
    *len = ceil((float)cur_bit / 8);
    return bytes;
}

uint8_t set_N_bits_C(int8_t value, uint8_t num_bits)
{
    uint8_t max_diff = pow(2, num_bits) - 2;
    int8_t start = -max_diff;
    int8_t end = max_diff;
    uint8_t half = pow(2, num_bits) / 2;
    int8_t first_half_end = start + half - 1;
    int8_t second_half_start = end - half + 1;
    return value < 0 ? (uint8_t)change_interval_C(value, start, first_half_end, 0, half - 1) : (uint8_t)change_interval_C(value, second_half_start, end, half, end + 1);
}

float change_interval_C(float value, float in_min, float in_max, float out_min, float out_max)
{
    return (out_max - out_min) * ((value - in_min) / (in_max - in_min)) + out_min;
}
