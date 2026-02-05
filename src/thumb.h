#pragma once

#include "cpu.h"

uint thumb_step(Cpu *this);
void thumb_build_decode_table(Cpu *this);
