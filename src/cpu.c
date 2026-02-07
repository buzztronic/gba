#include <stdlib.h>
#include <string.h>

#include "cpu.h"
#include "arm.h"
#include "thumb.h"

// global
u8 cpu_cond_lut[1 << 8];

// declarations
static void cpu_build_condition_table(void);

// functions
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

// just so the compiler doesn't complain
#ifndef RUN_CPU_TESTS
    for (u32 i = 0x3007E00; i < 0x3008000; i++) {
        bus_write(bus, i, 0);
    }

    bus_write16(bus, 0x4000208, 0);
    bus_write16(bus, 0x4000200, 0);
    bus_write16(bus, 0x4000202, 0xFFFF);
#endif

    cpu->bus = bus;
    cpu->pc_changed = 0;

    if (is_clear(cpu->cpsr, PSR_BIT_T)) {
        arm_flush_pipeline(cpu);
    } else {
        thumb_flush_pipeline(cpu);
    }

    return cpu;
}

void cpu_init_global(void)
{
    cpu_build_condition_table();
    arm_build_decode_table();
    thumb_build_decode_table();
}

void cpu_build_condition_table(void)
{
    for (uint idx = 0; idx < (1 << 8); ++idx) {
        u32 cpsr = bits(idx, 4, 4) << 28;
        switch (bits(idx, 0, 4)) {
            case 0x0:
                // EQ
                cpu_cond_lut[idx] = !!is_set(cpsr, PSR_BIT_Z);
            break;
            case 0x1:
                // NE
                cpu_cond_lut[idx] = !!is_clear(cpsr, PSR_BIT_Z);
            break;
            case 0x2:
                // CS
                cpu_cond_lut[idx] = !!is_set(cpsr, PSR_BIT_C);
            break;
            case 0x3:
                // CC
                cpu_cond_lut[idx] = !!is_clear(cpsr, PSR_BIT_C);
            break;
            case 0x4:
                // MI
                cpu_cond_lut[idx] = !!is_set(cpsr, PSR_BIT_N);
            break;
            case 0x5:
                // PL
                cpu_cond_lut[idx] = !!is_clear(cpsr, PSR_BIT_N);
            break;
            case 0x6:
                // VS
                cpu_cond_lut[idx] = !!is_set(cpsr, PSR_BIT_V);
            break;
            case 0x7:
                // VC
                cpu_cond_lut[idx] = !!is_clear(cpsr, PSR_BIT_V);
            break;
            case 0x8:
                // HI
                cpu_cond_lut[idx] = is_set(cpsr, PSR_BIT_C) && is_clear(cpsr, PSR_BIT_Z);
            break;
            case 0x9:
                // LS
                cpu_cond_lut[idx] = is_clear(cpsr, PSR_BIT_C) || is_set(cpsr, PSR_BIT_Z);
            break;
            case 0xA:
                // GE
                cpu_cond_lut[idx] = bit(cpsr, PSR_BIT_N) == bit(cpsr, PSR_BIT_V);
            break;
            case 0xB:
                // LT
                cpu_cond_lut[idx] = bit(cpsr, PSR_BIT_N) != bit(cpsr, PSR_BIT_V);
            break;
            case 0xC:
                // GT
                cpu_cond_lut[idx] = is_clear(cpsr, PSR_BIT_Z) && bit(cpsr, PSR_BIT_N) == bit(cpsr, PSR_BIT_V);
            break;
            case 0xD:
                // LE
                cpu_cond_lut[idx] = !!is_set(cpsr, PSR_BIT_Z) || (bit(cpsr, PSR_BIT_N) != bit(cpsr, PSR_BIT_V));
            break;
            case 0xE:
                // AL
                cpu_cond_lut[idx] = 1;
            break;
        }
    }
}

uint cpu_step(Cpu *this)
{
#ifndef RUN_CPU_TESTS
    cpu_handle_interrupts(this);
#endif

    u32 cycles = 0;
    if (is_clear(this->cpsr, PSR_BIT_T)) {
        cycles = arm_step(this);
    } else {
        cycles = thumb_step(this);
    }

    if (is_clear(this->cpsr, PSR_BIT_T)) {
        reg(15) &= ~3;
        if (this->pc_changed) {
            arm_flush_pipeline(this);
        } else {
            reg(15) += 4;
        }
    } else {
        reg(15) &= ~1;
        if (this->pc_changed) {
            thumb_flush_pipeline(this);
        } else {
            reg(15) += 2;
        }
    }

    this->pc_changed = 0;

    return cycles;
}

void cpu_handle_interrupts(Cpu *this)
{
    u8 irq_master = bus_read(this->bus, 0x4000208);

    if (is_set(this->cpsr, PSR_BIT_I) || is_clear(irq_master, 0))
        return;

    u16 irq_flag = bus_read16(this->bus, 0x4000202);
    u16 irq_enable = bus_read16(this->bus, 0x4000200);

    if (irq_flag & irq_enable) {
        this->reg_irq[1] = reg(15) + 4;
        if (is_clear(this->cpsr, PSR_BIT_T)) {
            this->reg_irq[1] -= 8;
        } else {
            this->reg_irq[1] -= 4;
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

        arm_flush_pipeline(this);
    }
}

uint cpu_software_interrupt(Cpu *this, u32 opcode)
{
    if (is_clear(this->cpsr, PSR_BIT_T)) {
        this->reg_svc[1] = reg(15) - 4;
    } else {
        this->reg_svc[1] = reg(15) - 2;
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

#ifdef RUN_CPU_TESTS
u8 *read_file(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        perror(path);
        exit(1);
    }

    long bsize = fsize(f);
    u8 *buff = malloc(bsize);

    fread(buff, 1, bsize, f);

    fclose(f);

    return buff;
}

u32 load_cpu_state(Cpu *this, const u8 *ptr)
{
    u32 *list[] = {
        this->reg_usr+0, this->reg_usr+1, this->reg_usr+2, this->reg_usr+3,
        this->reg_usr+4, this->reg_usr+5, this->reg_usr+6, this->reg_usr+7,
        this->reg_usr+8, this->reg_usr+9, this->reg_usr+10, this->reg_usr+11,
        this->reg_usr+12, this->reg_usr+13, this->reg_usr+14, this->reg_usr+15,

        this->reg_fiq+0, this->reg_fiq+1, this->reg_fiq+2, this->reg_fiq+3,
        this->reg_fiq+4, this->reg_fiq+5, this->reg_fiq+6,

        this->reg_svc+0, this->reg_svc+1,
        this->reg_abt+0, this->reg_abt+1,
        this->reg_irq+0, this->reg_irq+1,
        this->reg_und+0, this->reg_und+1,

        &this->cpsr,

        this->spsr + CPU_MODE_FIQ, this->spsr + CPU_MODE_SVC,
        this->spsr + CPU_MODE_ABT, this->spsr + CPU_MODE_IRQ,
        this->spsr + CPU_MODE_UND,

        &this->execute_opcode, &this->decode_opcode,
    };

    const u32 size = *(u32 *)ptr;

    ptr += 8;
    for (u32 i = 0; i < len(list); i++) {
        *list[i] = *(u32 *)ptr;
        ptr += 4;
    }

    return size;
}

u32 load_transactions(Transaction **tlist, u32 *len, const u8 *ptr)
{
    const u32 size = *(u32 *)ptr;
    const u32 magic = *(u32 *)(ptr+4);

    assert(magic == 3);

    *len = *(u32 *)(ptr+8);

    assert(*len > 0);
    *tlist = malloc(*len * sizeof(**tlist));
    assert(*tlist != NULL);

    Transaction *list = *tlist;
    ptr += 12;
    for (u32 i = 0; i < *len; i++) {
        list[i].kind = *(u32 *)(ptr+0);
        list[i].size = *(u32 *)(ptr+4);
        list[i].addr = *(u32 *)(ptr+8);
        list[i].data = *(u32 *)(ptr+12);

        assert(list[i].kind < 3);

        ptr += 4 * 6;
    }

    return size;
}

u32 load_opcode(u32 *opcode, u32 *base, const u8 *ptr)
{
    *opcode = *(u32 *)(ptr+8);
    *base = *(u32 *)(ptr+12);

    return *(u32 *)ptr;
}

const char *flags_to_str(u32 cpsr)
{
    static char str[] = "----";
    strcpy(str, "----");

    if (bit(cpsr, PSR_BIT_N))
        str[0] = 'N';
    if (bit(cpsr, PSR_BIT_Z))
        str[1] = 'Z';
    if (bit(cpsr, PSR_BIT_C))
        str[2] = 'C';
    if (bit(cpsr, PSR_BIT_V))
        str[3] = 'V';

    return str;
}

void test_compare_cpu(const Cpu *real, const Cpu *expected)
{
    // sorry couldn't think of a good name
    typedef struct State {
        char *name;
        u32 real;
        u32 expected;
    } State;

    State list[] = {
        { "usr_r0", real->reg_usr[0], expected->reg_usr[0] },
        { "usr_r1", real->reg_usr[1], expected->reg_usr[1] },
        { "usr_r2", real->reg_usr[2], expected->reg_usr[2] },
        { "usr_r3", real->reg_usr[3], expected->reg_usr[3] },
        { "usr_r4", real->reg_usr[4], expected->reg_usr[4] },
        { "usr_r5", real->reg_usr[5], expected->reg_usr[5] },
        { "usr_r6", real->reg_usr[6], expected->reg_usr[6] },
        { "usr_r7", real->reg_usr[7], expected->reg_usr[7] },
        { "usr_r8", real->reg_usr[8], expected->reg_usr[8] },
        { "usr_r9", real->reg_usr[9], expected->reg_usr[9] },
        { "usr_r10", real->reg_usr[10], expected->reg_usr[10] },
        { "usr_r11", real->reg_usr[11], expected->reg_usr[11] },
        { "usr_r12", real->reg_usr[12], expected->reg_usr[12] },
        { "usr_r13", real->reg_usr[13], expected->reg_usr[13] },
        { "usr_r14", real->reg_usr[14], expected->reg_usr[14] },
        { "usr_r15", real->reg_usr[15], expected->reg_usr[15] },

        { "fiq_r0", real->reg_fiq[0], expected->reg_fiq[0] },
        { "fiq_r1", real->reg_fiq[1], expected->reg_fiq[1] },
        { "fiq_r2", real->reg_fiq[2], expected->reg_fiq[2] },
        { "fiq_r3", real->reg_fiq[3], expected->reg_fiq[3] },
        { "fiq_r4", real->reg_fiq[4], expected->reg_fiq[4] },
        { "fiq_r5", real->reg_fiq[5], expected->reg_fiq[5] },
        { "fiq_r6", real->reg_fiq[6], expected->reg_fiq[6] },
        { "fiq_r7", real->reg_fiq[7], expected->reg_fiq[7] },

        { "svc_r0", real->reg_svc[0], expected->reg_svc[0] },
        { "svc_r1", real->reg_svc[1], expected->reg_svc[1] },

        { "abt_r0", real->reg_abt[0], expected->reg_abt[0] },
        { "abt_r1", real->reg_abt[1], expected->reg_abt[1] },

        { "irq_r0", real->reg_irq[0], expected->reg_irq[0] },
        { "irq_r1", real->reg_irq[1], expected->reg_irq[1] },

        { "und_r0", real->reg_und[0], expected->reg_und[0] },
        { "und_r1", real->reg_und[1], expected->reg_und[1] },

        { "cpsr", real->cpsr, expected->cpsr },

        { "spsr_fiq", real->spsr[CPU_MODE_FIQ], expected->spsr[CPU_MODE_FIQ] },
        { "spsr_svc", real->spsr[CPU_MODE_SVC], expected->spsr[CPU_MODE_SVC] },
        { "spsr_abt", real->spsr[CPU_MODE_ABT], expected->spsr[CPU_MODE_ABT] },
        { "spsr_irq", real->spsr[CPU_MODE_IRQ], expected->spsr[CPU_MODE_IRQ] },
        { "spsr_und", real->spsr[CPU_MODE_UND], expected->spsr[CPU_MODE_UND] },

        { "pipeline[0]", real->execute_opcode, expected->execute_opcode },
        { "pipeline[1]", real->decode_opcode, expected->decode_opcode },
    };

    if (is_clear(expected->cpsr, PSR_BIT_T))
        list[15].expected &= ~3;
    else
        list[15].expected &= ~1;

    for (u32 i = 0; i < len(list); i++) {
        if (list[i].real != list[i].expected) {
            eprintf("test  : %u\n", real->test_no);
            eprintf("opcode: %08X | %08X\n", real->topcode, real->topcode_reverse);
            eprintf("flags : %s\n\n", flags_to_str(real->initial->cpsr));

            eprintf("%s\n", list[i].name);
            if (!strcmp(list[i].name, "cpsr")) {
                eprintf("expected:    %s | %08X | %u\n", flags_to_str(list[i].expected), list[i].expected, list[i].expected);
                eprintf("sad reality: %s | %08X | %u\n", flags_to_str(list[i].real), list[i].real, list[i].real);
            } else {
                eprintf("expected:    %08X | %u\n", list[i].expected, list[i].expected);
                eprintf("sad reality: %08X | %u\n", list[i].real, list[i].real);
            }
            exit(1);
        }
    }
}

u32 cpu_test(const u8 *ptr, u32 test_no)
{
    const u32 size = *(u32 *)ptr;
    ptr += 4;

    Cpu cpu = {0};
    Cpu icpu = {0};
    Cpu fcpu = {0};

    ptr += load_cpu_state(&cpu, ptr);
    ptr += load_cpu_state(&fcpu, ptr);
    ptr += load_transactions(&cpu.tlist, &cpu.tlist_len, ptr);
    ptr += load_opcode(&cpu.topcode, &cpu.tbase, ptr);

    cpu.topcode_reverse = \
        bits(cpu.topcode, 0, 8) << 24 |
        bits(cpu.topcode, 8, 8) << 16 |
        bits(cpu.topcode, 16, 8) << 8 |
        bits(cpu.topcode, 24, 8);

    // TODO: I'm storing icpu so I can dump initial state but I'm not doing
    // that yet.
    icpu = cpu;
    cpu.initial = &icpu;
    cpu.test_no = test_no;
    
    for (u32 i = 0; i < 16; i++) {
        cpu.reg[i] = cpu.reg_usr + i;
    }
    cpu_bank_registers(&cpu);

    cpu_step(&cpu);

    test_compare_cpu(&cpu, &fcpu);

    free(cpu.tlist);

    return size;
}

void cpu_test_all(const char *path)
{
    u8 *buff = read_file(path);
    assert(buff != NULL);

    // NOTE: this code assumes the machine is little-endian

    const u32 magic = *(u32 *)buff;
    const u32 no_test = *(u32 *)(buff+4);

    assert(magic == 0xD33DBAE0);

    eprintf("\n%s\n", path);

    u8 *ptr = buff+8;
    for (u32 i = 0; i < no_test; i++) {
        ptr += cpu_test(ptr, i);
    }

    free(buff);
}

u32 test_read(Cpu *this, u32 kind, u32 size, u32 addr)
{
    const u32 i = this->tidx;
    const Transaction tran = this->tlist[i];
    this->tidx++;

    if (tran.kind != kind || tran.size != size || tran.addr != addr) {
        eprintf("test: %u\n", this->test_no);
        eprintf("opcode: %08X | %08X\n", this->topcode, this->topcode_reverse);
        eprintf("flags:  %s\n\n", flags_to_str(this->initial->cpsr));

        eprintf("expected | real\n");
        eprintf("kind: %u | %u \n", tran.kind, kind);
        eprintf("size: %u | %u \n", tran.size, size);
        eprintf("addr: %u | %u \n", tran.addr, addr);

        exit(1);
    }

    if (kind != 0)
        return tran.data;

    if (addr == this->tbase)
        return this->topcode;

    return addr & (u32 []){ 0, 0xFF, 0xFFFF, 0, 0xFFFFFFFF }[size];
}

void test_write(Cpu *this, u32 kind, u32 size, u32 addr, u32 data)
{
    const u32 i = this->tidx;
    const Transaction tran = this->tlist[i];
    this->tidx++;

    if (tran.kind != kind || tran.size != size ||
        tran.addr != addr || tran.data != data)
    {
        eprintf("test: %u\n", this->test_no);
        eprintf("opcode: %08X | %08X\n", this->topcode, this->topcode_reverse);
        eprintf("flags:  %s\n\n", flags_to_str(this->initial->cpsr));

        eprintf("expected | real\n");
        eprintf("kind: %u | %u \n", tran.kind, kind);
        eprintf("size: %u | %u \n", tran.size, size);
        eprintf("addr: %u | %u \n", tran.addr, addr);
        eprintf("data: %u | %u \n", tran.data, data);

        exit(1);
    }
}

#endif
