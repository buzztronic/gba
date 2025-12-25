#pragma once

#include <stdint.h>

typedef unsigned int uint;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

#define len(arr) (sizeof(arr) / sizeof((arr)[0]))
#define is_clear(data, bit) (!((data) & (1 << (bit))))
#define is_set(data, bit) ((data) & (1 << (bit)))

#define set_bit(dst, bit) (dst |= (1 << (bit)))
#define clear_bit(dst, bit) (dst &= ~(1 << (bit)))

#define in_range(data, start, end) ((data) >= (start) && (data) <= (end))

u32 ror32(u32 data, u32 n);
u32 asr32(u32 data, u32 n);

inline int bit(u32 val, u32 idx)
{
    return (val >> idx) & 1;
}

inline u32 bits(u32 val, u32 p, u32 n)
{
    return (val >> p) & ((u32)~0 >> (32-n));
}

#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)
#define BIT_3 (1 << 3)
#define BIT_4 (1 << 4)
#define BIT_5 (1 << 5)
#define BIT_6 (1 << 6)
#define BIT_7 (1 << 7)
#define BIT_8 (1 << 8)
#define BIT_9 (1 << 9)
#define BIT_10 (1 << 10)
#define BIT_11 (1 << 11)
#define BIT_12 (1 << 12)
#define BIT_13 (1 << 13)
#define BIT_14 (1 << 14)
#define BIT_15 (1 << 15)
#define BIT_16 (1 << 16)
#define BIT_17 (1 << 17)
#define BIT_18 (1 << 18)
#define BIT_19 (1 << 19)
#define BIT_20 (1 << 20)
#define BIT_21 (1 << 21)
#define BIT_22 (1 << 22)
#define BIT_23 (1 << 23)
#define BIT_24 (1 << 24)
#define BIT_25 (1 << 25)
#define BIT_26 (1 << 26)
#define BIT_27 (1 << 27)
#define BIT_28 (1 << 28)
#define BIT_29 (1 << 29)
#define BIT_30 (1 << 30)
#define BIT_31 (1 << 31)
#define BIT_32 (1 << 32)
