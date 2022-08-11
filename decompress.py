import numpy as np
import sys

dtype = np.dtype('B')


def setBit(arr, k):
    arr[int(k / 8)] |= (1 << (7 - k % 8))


def clearBit(arr, k):
    arr[int(k / 8)] &= ~(1 << (7 - k % 8))


def testBit(arr, k):
    return (arr[int(k / 8)] & (1 << (7 - k % 8))) != 0


def decompress(data, length):
    # print("Compressed data:")
    # for i in range(length * 8):
    #     if i > 0 and i % 8 == 0:
    #         print(end=" ")
    #     print("%d" % testBit(data, i),  end="")
    # print()
    ranges = np.array([2, 6, 14, 30])
    # Start of the algorithm
    # Decompress rules:
    # 00 - Differences
    #     00 + 2 bits [-2, -1] [1, 2] - 00 (-2) -> 11 (2)
    #     01 + 3 bits [-6, -3] [3, 6] - 000 (-6) -> 111 (6)
    #     10 + 4 bits [-14, -7] [7, 14] - 0000 (-14) -> 1111 (14)
    #     11 + 5 bits [-30, -15] [15, 30] - 0000 (-30) -> 1111 (30)
    # 01 - Repetitions
    #     3 bits counter of repetitions
    #     Max rep. 8, if > repeat rule
    #     000 (1) -> 111 (8)
    # 10 - Absolute coding
    #     Use if diff > 30
    #     9 bits - 0/1 sign + 8 bits for diff
    # 11 - End
    #     Use when reached the end of difference array
    #     01 - 3 bit [-6, -3] [3, 6]
    #     10 - 4 bit [-14, -7] [7, 14]
    #     11 - 5 bit [-30, -15] [15, 30]

    # for num_bits in range(2, 6):
    #     start = -ranges[num_bits - 2]
    #     end = ranges[num_bits - 2]
    #     half = np.power(2, num_bits) / 2
    #     first_half_end = start + half - 1
    #     second_half_start = end - half + 1
    #     print("Range for max diff: ", ranges[num_bits - 2])
    #     for i in range(start, end + 1):
    #         if i <= first_half_end or i >= second_half_start:
    #             print("%3d" % set_N_bits_D(i, num_bits),  end=" ")
    #     print()
    #     for i in range(start, end + 1):
    #         if i <= first_half_end or i >= second_half_start:
    #             print("%3d" % i,  end=" ")
    #     print()

    data_difference = []
    end = False
    cur_bit = 0

    # print("START")
    # First value as original value from array as 8-bit
    (diff_value, cur_bit) = read_N_bits(data, cur_bit, 8)
    data_difference.append(diff_value)
    # arr = np.array([data_difference[0]])
    # for i in range(8):
    #     print("%d" % testBit(arr, i), end="")
    # print(" | ", end="")
    # Go through whole data reading bits and use decompress rules
    while not end:
        (rule, cur_bit) = read_N_bits(data, cur_bit, 2)
        # arr = np.array([rule])
        # for i in range(6, 8):
        #     print("%d" % testBit(arr, i), end="")
        # print(end=" ")
        if rule == 0:   # 00 - Differences
            (rule, cur_bit) = read_N_bits(data, cur_bit, 2)
            num_bits = rule + 2  # 0 -> 2 bits, 1 -> 3 bits, 2 -> 4 bits, 3 -> 5 bits
            # arr = np.array([rule])
            # for i in range(6, 8):
            #     print("%d" % testBit(arr, i), end="")
            # print(end=" ")
            (rule, cur_bit) = read_N_bits(data, cur_bit, num_bits)
            diff_value = get_N_bits_value_D(rule, num_bits)
            # arr = np.array([rule])
            # for i in range(8 - num_bits, 8):
            #     print("%d" % testBit(arr, i), end="")
            data_difference.append(diff_value)
        elif rule == 1:  # 01 - Repetitions
            (rule, cur_bit) = read_N_bits(data, cur_bit, 3)
            # arr = np.array([rule])
            # for i in range(5, 8):
            #     print("%d" % testBit(arr, i), end="")
            for i in range(rule + 1):
                data_difference.append(0)
        elif rule == 2:  # 10 - Absolute coding
            (sign, cur_bit) = read_N_bits(data, cur_bit, 1)
            # print("%d" % sign, end=" ")
            (rule, cur_bit) = read_N_bits(data, cur_bit, 8)
            # arr = np.array([rule])
            # for i in range(8):
            #     print("%d" % testBit(arr, i), end="")
            data_difference.append(-rule) if sign else data_difference.append(rule)
        elif rule == 3:  # 11 - End
            end = True
        # if not end:
        #     print(" | ", end="")
    # print()
    # print("END")
    # print("Difference array:")
    length = len(data_difference)
    bytes = np.empty(length)
    bytes[0] = data_difference[0]
    # print("%3d" % data_difference[0], end=" ")
    for i in range(1, length):
        # print("%3d" % data_difference[i], end=" ")
        bytes[i] = bytes[i - 1] + data_difference[i]
    # print()
    # print("Original array:")
    # for i in range(length):
    #     print("%3d" % bytes[i], end=" ")
    # print()
    return (bytes, length)


def read_N_bits(data, cur_bit, num_bits):
    temp = np.array([0])
    for k in range(8 - num_bits, 8):
        setBit(temp, k) if testBit(data, cur_bit) else clearBit(temp, k)
        cur_bit += 1
    return (temp[0], cur_bit)


def get_N_bits_value_D(value, num_bits):
    max_diff = np.power(2, num_bits) - 2
    start = -max_diff
    end = max_diff
    half = np.power(2, num_bits) / 2
    first_half_end = start + half - 1
    second_half_start = end - half + 1
    return change_interval_D(value, 0, half - 1, start, first_half_end) if value < half else change_interval_D(value, half, end + 1, second_half_start, end)


def set_N_bits_D(value, num_bits):
    max_diff = np.power(2, num_bits) - 2
    start = -max_diff
    end = max_diff
    half = np.power(2, num_bits) / 2
    first_half_end = start + half - 1
    second_half_start = end - half + 1
    return change_interval_D(value, start, first_half_end, 0, half - 1) if value < 0 else change_interval_D(value, second_half_start, end, half, end + 1)


def change_interval_D(value, in_min, in_max, out_min, out_max):
    return (out_max - out_min) * ((value - in_min) / (in_max - in_min)) + out_min


# if __name__ == "__main__":
#     try:
#         with open(sys.argv[1], "rb") as f:
#             numpy_data = np.fromfile(f, dtype)
#         # print(numpy_data, numpy_data.shape)
#         (data, length) = decompress(numpy_data, len(numpy_data))
#         data = data.astype(int)
#         f = open("decompressed.txt", "w")
#         for i in range(length):
#             f.write(chr(data[i]))
#         f.close()
#     except IOError:
#         print('Error While Opening the file!')
