#pragma once

#include "cpu.h"

void arm_build_decode_table(Cpu *this);
void arm_build_condition_table(Cpu *this);
uint arm_step(Cpu *this);
