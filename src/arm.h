#pragma once

#include "cpu.h"

void arm_build_decode_table(void);
uint arm_step(Cpu *this);
void arm_flush_pipeline(Cpu *this);
