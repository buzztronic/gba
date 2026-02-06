#pragma once

#include "cpu.h"

void arm_build_decode_table(void);
uint arm_step(Cpu *this);
