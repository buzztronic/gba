#include "common.h"

int bit(u32 val, u32 idx);
u32 bits(u32 val, u32 p, u32 n);

u32 ror32(u32 data, u32 n)
{
    u32 saved_bits;

    n %= 32;

    // save the n bits
    saved_bits = data & (~0 >> (32-n));
    saved_bits <<= 32-n;

    // shift data
    data >>= n;

    // restore the saved bits
    data |= saved_bits;

    return data;
}

u32 asr32(u32 data, u32 n)
{
    u32 bit_31;

    bit_31 = data & BIT_31;

    if (n >= 32) {
        if (bit_31) {
            return ~0;
        } else {
            return 0;
        }
    }

    data >>= n;

    if (!bit_31)
        return data;

    data |= (~(u32)0 << (32 - n));

    return data;
}
