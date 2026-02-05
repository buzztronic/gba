#include <stdlib.h>

#include "cpu.h"
#include "arm.h"
#include "thumb.h"

Cpu *cpu_init(Bus *bus)
{
    Cpu *cpu = malloc(sizeof(Cpu));

    for (int i = 0; i < 16; ++i) {
		cpu->reg[i] = &(cpu->reg_usr[i]);
	}

    // initial state
    cpu->cpsr = 0x5F;
    *cpu->reg[15] = 0x8000000;
    *cpu->reg[13] = 0x3007F00;

    cpu->spsr[CPU_MODE_SVC] = 0;
    cpu->spsr[CPU_MODE_IRQ] = 0;
    cpu->reg_svc[0] = 0x03007FE0;
    cpu->reg_irq[0] = 0x03007FA0;
    cpu->reg_svc[1] = 0;
    cpu->reg_irq[1] = 0;

    for (u32 i = 0x3007E00; i < 0x3008000; i++) {
        bus_write(bus, i, 0);
    }

    bus_write16(bus, 0x4000208, 0);
    bus_write16(bus, 0x4000200, 0);
    bus_write16(bus, 0x4000202, 0xFFFF);

    cpu->bus = bus;
    cpu->pc_changed = 1;

    arm_build_condition_table(cpu);
    arm_build_decode_table(cpu);
    thumb_build_decode_table(cpu);

    return cpu;
}

uint cpu_step(Cpu *this)
{
    cpu_handle_interrupts(this);

    if (is_clear(this->cpsr, PSR_BIT_T)) {
        return arm_step(this);
    } else {
        return thumb_step(this);
    }
    return 0;
}

void cpu_handle_interrupts(Cpu *this)
{
    u8 irq_master = bus_read(this->bus, 0x4000208);

    if (is_set(this->cpsr, PSR_BIT_I) || is_clear(irq_master, 0))
        return;

    u16 irq_flag = bus_read16(this->bus, 0x4000202);
    u16 irq_enable = bus_read16(this->bus, 0x4000200);

    if (irq_flag & irq_enable) {
        puts("IRQ");
        printf("irq_flag:    %04X\n", irq_flag);
        printf("irq_enable:  %04X\n", irq_enable);

        this->reg_irq[1] = reg(15) + 4;
        if (!this->pc_changed) {
            if (is_clear(this->cpsr, PSR_BIT_T)) {
                this->reg_irq[1] -= 4;
                this->reg_irq[1] &= ~3;
            } else {
                this->reg_irq[1] -= 2;
                this->reg_irq[1] &= ~1;
            }
        }

        this->spsr[CPU_MODE_IRQ] = this->cpsr;

        this->cpsr &= ~0xF;
        this->cpsr |= CPU_MODE_IRQ;
        cpu_bank_registers(this);

        // switch to ARM
        clear_bit(this->cpsr, PSR_BIT_T);

        // disable IRQ
        set_bit(this->cpsr, PSR_BIT_I);

        reg(15) = 0x00000018;
        this->pc_changed = 1;
    }
}

uint cpu_software_interrupt(Cpu *this, u32 opcode)
{
    if (is_clear(this->cpsr, PSR_BIT_T)) {
        this->reg_svc[1] = (reg(15) - 4) & ~3;
    } else {
        this->reg_svc[1] = (reg(15) - 2) & ~1;
    }

    this->spsr[CPU_MODE_SVC] = this->cpsr;

    this->cpsr &= 0xFFFFFFF0;
    this->cpsr |= CPU_MODE_SVC;
    clear_bit(this->cpsr, PSR_BIT_T);
    set_bit(this->cpsr, PSR_BIT_I);

    cpu_bank_registers(this);

    reg(15) = 0x00000008;
    this->pc_changed = 1;

    return 1;
}

void cpu_bank_registers(Cpu *this)
{
    switch (bits(this->cpsr, 0, 4)) {
        case CPU_MODE_SYS:
        case CPU_MODE_USR:
            for (uint i = 8; i < 15; ++i) {
                this->reg[i] = &this->reg_usr[i];
            }
        break;
        case CPU_MODE_FIQ:
            for (uint i = 8; i < 15; ++i) {
                this->reg[i] = &this->reg_fiq[i-8];
            }
        break;
        case CPU_MODE_IRQ:
            this->reg[13] = &this->reg_irq[0];
            this->reg[14] = &this->reg_irq[1];
        break;
        case CPU_MODE_SVC:
            this->reg[13] = &this->reg_svc[0];
            this->reg[14] = &this->reg_svc[1];
        break;
        case CPU_MODE_ABT:
            this->reg[13] = &this->reg_abt[0];
            this->reg[14] = &this->reg_abt[1];
        break;
        case CPU_MODE_UND:
            this->reg[13] = &this->reg_und[0];
            this->reg[14] = &this->reg_und[1];
        break;
    }
}

u32 alu_and(u32 op1, u32 op2, u32 *cpsr)
{
    return op1 & op2;
}

u32 alu_eor(u32 op1, u32 op2, u32 *cpsr)
{
    return op1 ^ op2;
}

u32 alu_sub(u32 op1, u32 op2, u32 *cpsr)
{
    set_bit(*cpsr, PSR_BIT_C);
    return alu_adc(op1, ~op2, cpsr);
}

u32 alu_rsb(u32 op1, u32 op2, u32 *cpsr)
{
    set_bit(*cpsr, PSR_BIT_C);
    return alu_adc(op2, ~op1, cpsr);
}

u32 alu_add(u32 op1, u32 op2, u32 *cpsr)
{
    const u32 result = op1 + op2;
    const u8 result_s = bit(result, 31);
    const u8 op1_s = bit(op1, 31);
    const u8 op2_s = bit(op2, 31);

    if (op1_s == op2_s && result_s != op1_s)
        set_bit(*cpsr, PSR_BIT_V);
    else
        clear_bit(*cpsr, PSR_BIT_V);

    if (result < op1 || result < op2)
        set_bit(*cpsr, PSR_BIT_C);
    else
        clear_bit(*cpsr, PSR_BIT_C);

    return result;
}

u32 alu_adc(u32 op1, u32 op2, u32 *cpsr)
{
    const u32 carry = bit(*cpsr, PSR_BIT_C);

    const u32 sum = alu_add(op1, op2, cpsr);

    if (carry == 0)
        return sum;

    const u8 op1_s = bit(op1, 31);
    const u8 op2_s = bit(op2, 31);
    const u8 result_s = bit(op1+op2+1, 31);

    if (op1_s == op2_s && result_s != op1_s)
        set_bit(*cpsr, PSR_BIT_V);
    else
        clear_bit(*cpsr, PSR_BIT_V);

    if (sum == ~(u32)0) {
        set_bit(*cpsr, PSR_BIT_C);
    }

    return sum + 1;
}

u32 alu_sbc(u32 op1, u32 op2, u32 *cpsr)
{
    return alu_adc(op1, ~op2, cpsr);
    u32 not_carry = !bit(*cpsr, PSR_BIT_C);

    u32 diff = alu_sub(op1, op2, cpsr);

    if (not_carry == 0)
        return diff;

    if (bit(diff, 31) && !bit(diff-1, 31)) {
        set_bit(*cpsr, PSR_BIT_V);
    }
    if (diff != 0) {
        set_bit(*cpsr, PSR_BIT_C);
    }

    return diff - 1;
}

u32 alu_rsc(u32 op1, u32 op2, u32 *cpsr)
{
    return alu_sbc(op2, op1, cpsr);
}

u32 alu_orr(u32 op1, u32 op2, u32 *cpsr)
{
    return op1 | op2;
}

u32 alu_mov(u32 op1, u32 op2, u32 *cpsr)
{
    return op2;
}

u32 alu_bic(u32 op1, u32 op2, u32 *cpsr)
{
    return op1 & ~op2;
}

u32 alu_mvn(u32 op1, u32 op2, u32 *cpsr)
{
    return ~op2;
}

void cpu_update_zn(u32 result, u32 *cpsr)
{
    // Zero
    if (result == 0) {
        set_bit(*cpsr, PSR_BIT_Z);
    } else {
        clear_bit(*cpsr, PSR_BIT_Z);
    }

    // Negative
    if (bit(result, 31)) {
        set_bit(*cpsr, PSR_BIT_N);
    } else {
        clear_bit(*cpsr, PSR_BIT_N);
    }
}
