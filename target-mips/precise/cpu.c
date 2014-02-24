/*
 * Limited MIPS R3000 CPU emulation
 *
 * Copyright (C) 2014 Antony Pavlov <antonynpavlov@gmail.com>
 *
 * This CPU model is based on MIPS R3000 CPU emulation (VMIPS)
 * Copyright 2001, 2002, 2003, 2004 Brian R. Gaeke.
 *
 * See original vmips-1.4.1 sources here:
 *   http://www.dgate.org/vmips/releases/vmips-1.4.1/vmips-1.4.1.tar.gz
 *
 * VMIPS is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <stdlib.h>
#include <malloc.h>

#include "cpu.h"
#include "qemu/bswap.h"
#include "excnames.h"
#include "cpzeroreg.h"

int precise_cpu_mips_exec(CPUArchState *env);
void precise_mips_cpu_init(struct CPUMIPSState *env);

static void exception(VmipsCpu *cpu,
    uint16_t excCode, int mode /* = ANY */, int coprocno /* = -1 */);

/*
 * vmips-1.4.1/accesstypes.h
 */

/* Three kinds of memory accesses are possible.
 * There are two kinds of load and one kind of store:
 * INSTFETCH is a memory access due to an instruction fetch.
 * DATALOAD is a memory access due to a load instruction,
 * e.g., lw, lh, lb.
 * DATASTORE is a memory access due to a store instruction,
 * e.g., sw, sh, sb.
 *
 * ANY is a catch-all used in exception prioritizing which
 * implies that none of the kinds of memory accesses applies,
 * or that the type of memory access otherwise doesn't matter.
 */
#define INSTFETCH 0
#define DATALOAD 1
#define DATASTORE 2
#define ANY 3

/*
 * vmips-1.4.1/mapper.cc
 */

/* If the host processor is byte-swapped with respect to the target
 * we are emulating, we will need to swap data bytes around when we
 * do loads and stores. These functions implement the swapping.
 *
 * The mips_to_host_word(), etc. functions are aliases for
 * the swap_word() primitives with more semantically-relevant
 * names.
 */

/* Convert word W from big-endian to little-endian, or vice-versa,
 * and return the result of the conversion.
 */
static uint32_t swap_word(uint32_t w)
{
    return ((w & 0x0ff) << 24) | (((w >> 8) & 0x0ff) << 16) |
        (((w >> 16) & 0x0ff) << 8) | ((w >> 24) & 0x0ff);
}

/* Convert halfword H from big-endian to little-endian, or vice-versa,
 * and return the result of the conversion.
 */
static uint16_t swap_halfword(uint16_t h)
{
    return ((h & 0x0ff) << 8) | ((h >> 8) & 0x0ff);
}

static int byteswapped = 1;

/* Convert word W from target processor byte-order to host processor
 * byte-order and return the result of the conversion.
 */
static uint32_t mips_to_host_word(uint32_t w)
{
    if (byteswapped)
        w = swap_word(w);

    return w;
}

/* Convert word W from host processor byte-order to target processor
 * byte-order and return the result of the conversion.
 */
static uint32_t host_to_mips_word(uint32_t w)
{
    if (byteswapped)
        w = swap_word(w);

    return w;
}

/* Convert halfword H from target processor byte-order to host processor
 * byte-order and return the result of the conversion.
 */
static uint16_t mips_to_host_halfword(uint16_t h)
{
    if (byteswapped)
        h = swap_halfword(h);

    return h;
}

/* Convert halfword H from host processor byte-order to target processor
 * byte-order and return the result of the conversion.
 */
static uint16_t host_to_mips_halfword(uint16_t h)
{
    if (byteswapped)
        h = swap_halfword(h);

    return h;
}

/* Fetch a word from the physical memory from physical address
 * ADDR. MODE is INSTFETCH if this is an instruction fetch; DATALOAD
 * otherwise. CACHEABLE is true if this access should be routed through
 * the cache, false otherwise.  This routine is shared between instruction
 * fetches and word-wide data fetches.
 *
 * The routine returns either the specified word, if it is mapped and
 * the address is correctly aligned, or else a word consisting of all
 * ones is returned.
 *
 * Words are returned in the endianness of the target processor; since devices
 * are implemented as Ranges, devices should return words in the host
 * endianness.
 *
 * This routine may trigger exceptions IBE and/or DBE in the client
 * processor, if the address is unmapped.
 * This routine may trigger exception AdEL in the client
 * processor, if the address is unaligned.
 */
static uint32_t fetch_word(VmipsCpu *cpu, uint32_t addr, int32_t mode, bool cacheable)
{
    uint32_t word;

    if (addr % 4 != 0) {
        exception(cpu, AdEL, mode, -1);

        return 0xffffffff;
    }

    /* FIXME: use address_space_read() here */
    cpu_physical_memory_read(addr, &word, 4);

    return host_to_mips_word(word);
}

/* Fetch a halfword from the physical memory from physical address ADDR.
 * CACHEABLE is true if this access should be routed through the cache,
 * false otherwise.
 *
 * The routine returns either the specified halfword, if it is mapped
 * and the address is correctly aligned, or else a halfword consisting
 * of all ones is returned.
 *
 * Halfwords are returned in the endianness of the target processor;
 * since devices are implemented as Ranges, devices should return halfwords
 * in the host endianness.
 *
 * This routine may trigger exception DBE in the client processor,
 * if the address is unmapped.
 * This routine may trigger exception AdEL in the client
 * processor, if the address is unaligned.
 */
static uint16_t fetch_halfword(VmipsCpu *cpu, uint32_t addr, bool cacheable)
{
    uint16_t hword;

    if (addr % 2 != 0) {
        exception(cpu, AdEL, DATALOAD, -1);

        return 0xffff;
    }

    /* FIXME: use address_space_read() here */
    cpu_physical_memory_read(addr, &hword, 2);

    return host_to_mips_halfword(hword);
}

/* Fetch a byte from the physical memory from physical address ADDR.
 * CACHEABLE is true if this access should be routed through the cache,
 * false otherwise.
 *
 * The routine returns either the specified byte, if it is mapped,
 * or else a byte consisting of all ones is returned.
 *
 * This routine may trigger exception DBE in the client processor,
 * if the address is unmapped.
 */
static uint8_t fetch_byte(VmipsCpu *cpu, uint32_t addr, bool cacheable)
{
    uint8_t byte;

    /* FIXME: use address_space_read() here */
    cpu_physical_memory_read(addr, &byte, 1);

    return byte;
}

/* Store a word's-worth of DATA to physical address ADDR.
 * CACHEABLE is true if this access should be routed through the cache,
 * false otherwise.
 *
 * This routine may trigger exception AdES in the client processor,
 * if the address is unaligned.
 * This routine may trigger exception DBE in the client processor,
 * if the address is unmapped.
 */
static void store_word(VmipsCpu *cpu, uint32_t addr, uint32_t data, bool cacheable)
{
    if (addr % 4 != 0) {
        exception(cpu, AdES, DATASTORE, -1);

        return;
    }

    data = mips_to_host_word(data);

    /* FIXME: use address_space_write() here */
    cpu_physical_memory_write(addr, &data, 4);
}

/* Store half a word's-worth of DATA to physical address ADDR.
 * CACHEABLE is true if this access should be routed through the cache,
 * false otherwise.
 *
 * This routine may trigger exception AdES in the client processor,
 * if the address is unaligned.
 * This routine may trigger exception DBE in the client processor,
 * if the address is unmapped.
 */
static void store_halfword(VmipsCpu *cpu, uint32_t addr, uint16_t data, bool cacheable)
{
    if (addr % 2 != 0) {
        exception(cpu, AdES, DATASTORE, -1);

        return;
    }

    data = mips_to_host_halfword(data);

    /* FIXME: use address_space_write() here */
    cpu_physical_memory_write(addr, &data, 2);
}

/* Store a byte of DATA to physical address ADDR.
 * CACHEABLE is true if this access should be routed through the cache,
 * false otherwise.
 *
 * This routine may trigger exception DBE in the client processor,
 * if the address is unmapped.
 */
static void store_byte(VmipsCpu *cpu, uint32_t addr, uint8_t data, bool cacheable)
{
    /* FIXME: use address_space_write() here */
    cpu_physical_memory_write(addr, &data, 1);
}

/*
 * vmips-1.4.1/cpu.h
 */

static uint16_t opcode(const uint32_t i)
{
    return (i >> 26) & 0x03f;
}

static uint16_t rs(const uint32_t i)
{
    return (i >> 21) & 0x01f;
}

static uint16_t rt(const uint32_t i)
{
    return (i >> 16) & 0x01f;
}

static uint16_t rd(const uint32_t i)
{
    return (i >> 11) & 0x01f;
}

static uint16_t immed(const uint32_t i)
{
    return i & 0x0ffff;
}

static short s_immed(const uint32_t i)
{
    return i & 0x0ffff;
}

static uint16_t shamt(const uint32_t i)
{
    return (i >> 6) & 0x01f;
}

static uint16_t funct(const uint32_t i)
{
    return i & 0x03f;
}

static uint32_t jumptarg(const uint32_t i)
{
    return i & 0x03ffffff;
}

/*
 * vmips-1.4.1/cpu.cc
 */

static void control_transfer(VmipsCpu *cpu, uint32_t new_pc);

/* states of the delay-slot state machine -- see CPU::step() */
static const int NORMAL = 0, DELAYING = 1, DELAYSLOT = 2;

/* certain fixed register numbers which are handy to know */
static const int reg_zero = 0;  /* always zero */
static const int reg_sp = 29;   /* stack pointer */
static const int reg_ra = 31;   /* return address */

/* pointer to CPU method returning void and taking two uint32_t's */
typedef void (*emulate_funptr)(VmipsCpu *, uint32_t, uint32_t);

static void branch(VmipsCpu *cpu, uint32_t instr, uint32_t pc);

/* Called when the program wants to use coprocessor COPROCNO, and there
 * isn't any implementation for that coprocessor.
 * Results in a Coprocessor Unusable exception, along with an error
 * message being printed if the coprocessor is marked usable in the
 * CP0 Status register.
 */
static inline void cop_unimpl(VmipsCpu *cpu, int coprocno, uint32_t instr, uint32_t pc)
{
    /* FIXME: cheking for a cpzero->cop_usable(coprocno) is skipped */
    exception(cpu, CpU, ANY, coprocno);
}

static void cpone_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cop_unimpl(cpu, 1, instr, pc);
}

static void cptwo_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cop_unimpl(cpu, 2, instr, pc);
}

static void cpthree_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cop_unimpl(cpu, 3, instr, pc);
}

static void lwc1_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cop_unimpl(cpu, 1, instr, pc);
}

static void lwc2_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cop_unimpl(cpu, 2, instr, pc);
}

static void lwc3_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cop_unimpl(cpu, 3, instr, pc);
}

static void swc1_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cop_unimpl(cpu, 1, instr, pc);
}

static void swc2_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cop_unimpl(cpu, 2, instr, pc);
}

static void swc3_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cop_unimpl(cpu, 3, instr, pc);
}

static void exception(VmipsCpu *cpu,
    uint16_t excCode, int mode /* = ANY */, int coprocno /* = -1 */)
{
    fprintf(stderr, "exception excCode=%d\n", excCode);

    exit(1);
}

/* reserved instruction */
static void RI_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    exception(cpu, RI, ANY, -1);
}

static void sll_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = cpu->reg[rt(instr)] << shamt(instr);
}

static int32_t srl(int32_t a, int32_t b)
{
    if (b == 0) {
        return a;
    } else if (b == 32) {
        return 0;
    } else {
        return (a >> b) & ((1 << (32 - b)) - 1);
    }
}

static int32_t sra(int32_t a, int32_t b)
{
    if (b == 0)
        return a;

    return (a >> b) | (((a >> 31) & 0x01) * (((1 << b) - 1) << (32 - b)));
}

static void srl_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = srl(cpu->reg[rt(instr)], shamt(instr));
}

static void sra_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = sra(cpu->reg[rt(instr)], shamt(instr));
}

static void sllv_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = cpu->reg[rt(instr)] << (cpu->reg[rs(instr)] & 0x01f);
}

static void srlv_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = srl(cpu->reg[rt(instr)], cpu->reg[rs(instr)] & 0x01f);
}

static void srav_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = sra(cpu->reg[rt(instr)], cpu->reg[rs(instr)] & 0x01f);
}

static void jr_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (cpu->reg[rd(instr)] != 0) {
        exception(cpu, RI, ANY, -1);
        return;
    }

    control_transfer(cpu, cpu->reg[rs(instr)]);
}

static void jalr_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    control_transfer(cpu, cpu->reg[rs(instr)]);

    /* RA gets addr of instr after delay slot (2 words after this one). */
    cpu->reg[rd(instr)] = pc + 8;
}

static void syscall_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    exception(cpu, Sys, ANY, -1);
}

static void break_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    exception(cpu, Bp, ANY, -1);
}

static void mfhi_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = cpu->hi;
}

static void mthi_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (rd(instr) != 0) {
        exception(cpu, RI, ANY, -1);
        return;
    }

    cpu->hi = cpu->reg[rs(instr)];
}

static void mflo_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = cpu->lo;
}

static void mtlo_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (rd(instr) != 0) {
        exception(cpu, RI, ANY, -1);
        return;
    }

    cpu->lo = cpu->reg[rs(instr)];
}

static void mult64s(uint32_t *hi, uint32_t *lo, int32_t n, int32_t m);

static void mult_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (rd(instr) != 0) {
        exception(cpu, RI, ANY, -1);
        return;
    }

    mult64s(&cpu->hi, &cpu->lo, cpu->reg[rs(instr)], cpu->reg[rt(instr)]);
}

static void mult64(uint32_t *hi, uint32_t *lo, uint32_t n, uint32_t m)
{
#ifdef HAVE_LONG_LONG
    uint64_t result;
    result = ((uint64_t)n) * ((uint64_t)m);
    *hi = (uint32_t) (result >> 32);
    *lo = (uint32_t) result;
#else /* HAVE_LONG_LONG */
    /*           n = (w << 16) | x ; m = (y << 16) | z
     *     w x   g = a + e ; h = b + f ; p = 65535
     *   X y z   c = (z * x) mod p
     *   -----   b = (z * w + ((z * x) div p)) mod p
     *   a b c   a = (z * w + ((z * x) div p)) div p
     * d e f     f = (y * x) mod p
     * -------   e = (y * w + ((y * x) div p)) mod p
     * i g h c   d = (y * w + ((y * x) div p)) div p
     */
    uint16_t w, x, y, z, a, b, c, d, e, f, g, h, i;
    uint32_t p;

    p = 65536;
    w = (n >> 16) & 0x0ffff;
    x = n & 0x0ffff;
    y = (m >> 16) & 0x0ffff;
    z = m & 0x0ffff;
    c = (z * x) % p;
    b = (z * w + ((z * x) / p)) % p;
    a = (z * w + ((z * x) / p)) / p;
    f = (y * x) % p;
    e = (y * w + ((y * x) / p)) % p;
    d = (y * w + ((y * x) / p)) / p;
    h = (b + f) % p;
    g = ((a + e) + ((b + f) / p)) % p;
    i = d + (((a + e) + ((b + f) / p)) / p);
    *hi = (i << 16) | g;
    *lo = (h << 16) | c;
#endif /* HAVE_LONG_LONG */
}

static void mult64s(uint32_t *hi, uint32_t *lo, int32_t n, int32_t m)
{
#ifdef HAVE_LONG_LONG
    int64_t result;
    result = ((int64_t)n) * ((int64_t)m);
    *hi = (uint32_t) (result >> 32);
    *lo = (uint32_t) result;
#else /* HAVE_LONG_LONG */
    int32_t result_sign = (n < 0) ^ (m < 0);
    int32_t n_abs = n;
    int32_t m_abs = m;

    if (n_abs < 0)
        n_abs = -n_abs;
    if (m_abs < 0)
        m_abs = -m_abs;

    mult64(hi, lo, n_abs, m_abs);

    if (result_sign) {
        *hi = ~*hi;
        *lo = ~*lo;
        if (*lo & 0x80000000) {
            *lo += 1;
            if (!(*lo & 0x80000000)) {
                *hi += 1;
            }
        } else {
            *lo += 1;
        }
    }
#endif /* HAVE_LONG_LONG */
}

static void multu_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (rd(instr) != 0) {
        exception(cpu, RI, ANY, -1);
        return;
    }

    mult64(&cpu->hi, &cpu->lo, cpu->reg[rs(instr)], cpu->reg[rt(instr)]);
}

static void div_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    int32_t signed_rs = (int32_t)cpu->reg[rs(instr)];
    int32_t signed_rt = (int32_t)cpu->reg[rt(instr)];

    cpu->lo = signed_rs / signed_rt;
    cpu->hi = signed_rs % signed_rt;
}

static void divu_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->lo = cpu->reg[rs(instr)] / cpu->reg[rt(instr)];
    cpu->hi = cpu->reg[rs(instr)] % cpu->reg[rt(instr)];
}

static void add_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    int32_t a, b, sum;

    a = (int32_t)cpu->reg[rs(instr)];
    b = (int32_t)cpu->reg[rt(instr)];
    sum = a + b;

    if ((a < 0 && b < 0 && !(sum < 0)) || (a >= 0 && b >= 0 && !(sum >= 0))) {
        exception(cpu, Ov, ANY, -1);
        return;
    }

    cpu->reg[rd(instr)] = (uint32_t)sum;
}

static void addu_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    int32_t a, b, sum;

    a = (int32_t)cpu->reg[rs(instr)];
    b = (int32_t)cpu->reg[rt(instr)];
    sum = a + b;
    cpu->reg[rd(instr)] = (uint32_t)sum;
}

static void sub_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    int32_t a, b, diff;

    a = (int32_t)cpu->reg[rs(instr)];
    b = (int32_t)cpu->reg[rt(instr)];
    diff = a - b;
    if ((a < 0 && !(b < 0) && !(diff < 0)) || (!(a < 0) && b < 0 && diff < 0)) {
        exception(cpu, Ov, ANY, -1);
        return;
    }

    cpu->reg[rd(instr)] = (uint32_t)diff;
}

static void subu_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    int32_t a, b, diff;

    a = (int32_t)cpu->reg[rs(instr)];
    b = (int32_t)cpu->reg[rt(instr)];
    diff = a - b;
    cpu->reg[rd(instr)] = (uint32_t)diff;
}

static void and_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = cpu->reg[rs(instr)] & cpu->reg[rt(instr)];
}

static void or_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = cpu->reg[rs(instr)] | cpu->reg[rt(instr)];
}

static void xor_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = cpu->reg[rs(instr)] ^ cpu->reg[rt(instr)];
}

static void nor_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rd(instr)] = ~(cpu->reg[rs(instr)] | cpu->reg[rt(instr)]);
}

static void slt_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    int32_t s_rs = (int32_t)cpu->reg[rs(instr)];
    int32_t s_rt = (int32_t)cpu->reg[rt(instr)];

    if (s_rs < s_rt) {
        cpu->reg[rd(instr)] = 1;
    } else {
        cpu->reg[rd(instr)] = 0;
    }
}

static void sltu_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (cpu->reg[rs(instr)] < cpu->reg[rt(instr)]) {
        cpu->reg[rd(instr)] = 1;
    } else {
        cpu->reg[rd(instr)] = 0;
    }
}

static void bltz_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if ((int32_t)cpu->reg[rs(instr)] < 0)
        branch(cpu, instr, pc);
}

static void bgez_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if ((int32_t)cpu->reg[rs(instr)] >= 0)
        branch(cpu, instr, pc);
}

/* As with JAL, BLTZAL and BGEZAL cause RA to get the address of the
 * instruction two words after the current one (pc + 8).
 */
static void bltzal_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[reg_ra] = pc + 8;
    if ((int32_t)cpu->reg[rs(instr)] < 0)
        branch(cpu, instr, pc);
}

static void bgezal_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[reg_ra] = pc + 8;
    if ((int32_t)cpu->reg[rs(instr)] >= 0)
        branch(cpu, instr, pc);
}

static void funct_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    static const emulate_funptr functJumpTable[] = {
        &sll_emulate,     &RI_emulate,
        &srl_emulate,     &sra_emulate,
        &sllv_emulate,    &RI_emulate,
        &srlv_emulate,    &srav_emulate,
        &jr_emulate,      &jalr_emulate,
        &RI_emulate,      &RI_emulate,
        &syscall_emulate, &break_emulate,
        &RI_emulate,      &RI_emulate,
        &mfhi_emulate,    &mthi_emulate,
        &mflo_emulate,    &mtlo_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate,
        &mult_emulate,    &multu_emulate,
        &div_emulate,     &divu_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate,
        &add_emulate,     &addu_emulate,
        &sub_emulate,     &subu_emulate,
        &and_emulate,     &or_emulate,
        &xor_emulate,     &nor_emulate,
        &RI_emulate,      &RI_emulate,
        &slt_emulate,     &sltu_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate,
        &RI_emulate,      &RI_emulate
    };

    (*functJumpTable[funct(instr)])(cpu, instr, pc);
}

static void regimm_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    switch (rt(instr)) {
    case 0:
        bltz_emulate(cpu, instr, pc);
    break;
    case 1:
        bgez_emulate(cpu, instr, pc);
    break;
    case 16:
        bltzal_emulate(cpu, instr, pc);
    break;
    case 17:
        bgezal_emulate(cpu, instr, pc);
    break;
    default:
        exception(cpu, RI, ANY, -1);
    break; /* reserved instruction */
    }
}

/*
 * calc_branch_target - Calculate the address to jump to for the
 * PC-relative branch for which the offset is specified by the immediate field
 * of the branch instruction word INSTR, with the program counter equal to PC.
 */
static uint32_t calc_branch_target(uint32_t instr, uint32_t pc)
{
    return (pc + 4) + (s_immed(instr) << 2);
}

static void control_transfer(VmipsCpu *cpu, uint32_t new_pc)
{
    if (!new_pc)
        printf("Jumping to zero (PC = 0x%x)\n", cpu->pc);

    cpu->delay_state = DELAYING;
    cpu->delay_pc = new_pc;
}

/*
 * calc_jump_target - Calculate the address to jump to as a result of
 * the J-format (jump) instruction INSTR at address PC.  (PC is the address
 * of the jump instruction, and INSTR is the jump instruction word.)
 */
static uint32_t calc_jump_target(uint32_t instr, uint32_t pc)
{
    /* Must use address of delay slot (pc + 4) to calculate. */
    return ((pc + 4) & 0xf0000000) | (jumptarg(instr) << 2);
}

static void jump(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    control_transfer(cpu, calc_jump_target(instr, pc));
}

static void j_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    jump(cpu, instr, pc);
}

static void jal_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    jump(cpu, instr, pc);
    /* RA gets addr of instr after delay slot (2 words after this one). */
    cpu->reg[reg_ra] = pc + 8;
}

static void branch(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    control_transfer(cpu, calc_branch_target(instr, pc));
}

static void beq_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (cpu->reg[rs(instr)] == cpu->reg[rt(instr)])
        branch(cpu, instr, pc);
}

static void bne_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (cpu->reg[rs(instr)] != cpu->reg[rt(instr)])
        branch(cpu, instr, pc);
}

static void blez_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (rt(instr) != 0) {
        exception(cpu, RI, ANY, -1);
        return;
    }

    if (cpu->reg[rs(instr)] == 0 || (cpu->reg[rs(instr)] & 0x80000000))
        branch(cpu, instr, pc);
}

static void bgtz_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (rt(instr) != 0) {
        exception(cpu, RI, ANY, -1);
        return;
    }

    if (cpu->reg[rs(instr)] != 0 && (cpu->reg[rs(instr)] & 0x80000000) == 0)
        branch(cpu, instr, pc);
}

static void addi_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    int32_t a, b, sum;

    a = (int32_t)cpu->reg[rs(instr)];
    b = s_immed(instr);
    sum = a + b;

    if ((a < 0 && b < 0 && !(sum < 0)) || (a >= 0 && b >= 0 && !(sum >= 0))) {
        exception(cpu, Ov, ANY, -1);
        return;
    } else {
        cpu->reg[rt(instr)] = (uint32_t)sum;
    }
}

static void addiu_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    int32_t a, b, sum;

    a = (int32_t)cpu->reg[rs(instr)];
    b = s_immed(instr);
    sum = a + b;
    cpu->reg[rt(instr)] = (uint32_t)sum;
}

static void slti_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    int32_t s_rs = cpu->reg[rs(instr)];

    if (s_rs < s_immed(instr)) {
        cpu->reg[rt(instr)] = 1;
    } else {
        cpu->reg[rt(instr)] = 0;
    }
}

static void sltiu_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    if (cpu->reg[rs(instr)] < (uint32_t)(int32_t)s_immed(instr)) {
        cpu->reg[rt(instr)] = 1;
    } else {
        cpu->reg[rt(instr)] = 0;
    }
}

static void andi_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rt(instr)] = (cpu->reg[rs(instr)] & 0x0ffff) & immed(instr);
}

static void ori_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rt(instr)] = cpu->reg[rs(instr)] | immed(instr);
}

static void xori_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rt(instr)] = cpu->reg[rs(instr)] ^ immed(instr);
}

static void lui_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->reg[rt(instr)] = immed(instr) << 16;
}

/* Request for address translation (possibly using the TLB). */
/* FIXME: no TLB just now */
static uint32_t address_trans(uint32_t vaddr, int mode, bool *cacheable)
{
    switch (vaddr & KSEG_SELECT_MASK) {
    case KSEG0:
        *cacheable = true;
        return vaddr - KSEG0_CONST_TRANSLATION;
    case KSEG1:
        *cacheable = false;
        return vaddr - KSEG1_CONST_TRANSLATION;
    }

    return 0xffffffff;
}

static void lb_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, base;
    int8_t byte;
    int32_t offset;
    bool cacheable;

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;

    /* Translate virtual address to physical address. */
    phys = address_trans(virt, DATALOAD, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Fetch byte.
     * Because it is assigned to a signed variable (int32_t byte)
     * it will be sign-extended.
     */
    byte = fetch_byte(cpu, phys, cacheable);

    if (cpu->exception_pending)
        return;

    /* Load target register with data. */
    cpu->reg[rt(instr)] = byte;
}

static void lh_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, base;
    int16_t halfword;
    int32_t offset;
    bool cacheable;

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;

    /* This virtual address must be halfword-aligned. */
    if (virt % 2 != 0) {
        exception(cpu, AdEL, DATALOAD, -1);
        return;
    }

    /* Translate virtual address to physical address. */
    phys = address_trans(virt, DATALOAD, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Fetch halfword.
     * Because it is assigned to a signed variable (int32_t halfword)
     * it will be sign-extended.
     */
    halfword = fetch_halfword(cpu, phys, cacheable);

    if (cpu->exception_pending)
        return;

    /* Load target register with data. */
    cpu->reg[rt(instr)] = halfword;
}

/* The lwr and lwl algorithms here are taken from SPIM 6.0,
 * since I didn't manage to come up with a better way to write them.
 * Improvements are welcome.
 */
static uint32_t lwr(VmipsCpu *cpu, uint32_t regval, uint32_t memval, uint8_t offset)
{
    if (cpu->opt_bigendian) {
        switch (offset) {
        case 0:
            return (regval & 0xffffff00) |
                    ((unsigned)(memval & 0xff000000) >> 24);
        case 1:
            return (regval & 0xffff0000) |
                    ((unsigned)(memval & 0xffff0000) >> 16);
        case 2:
            return (regval & 0xff000000) |
                    ((unsigned)(memval & 0xffffff00) >> 8);
        case 3:
            return memval;
        }
    } else /* if MIPS target is little endian */ {
        switch (offset) {
        /* The SPIM source claims that "The description of the
         * little-endian case in Kane is totally wrong." The fact
         * that I ripped off the LWR algorithm from them could be
         * viewed as a sort of passive assumption that their claim
         * is correct.
         */
        case 0: /* 3 in book */
            return memval;
        case 1: /* 0 in book */
            return (regval & 0xff000000) | ((memval & 0xffffff00) >> 8);
        case 2: /* 1 in book */
            return (regval & 0xffff0000) | ((memval & 0xffff0000) >> 16);
        case 3: /* 2 in book */
            return (regval & 0xffffff00) | ((memval & 0xff000000) >> 24);
        }
    }

    fprintf(stderr, "Invalid offset %x passed to lwr\n", offset);

    return 0xffffffff;
}

static uint32_t lwl(VmipsCpu *cpu, uint32_t regval, uint32_t memval,
    uint8_t offset)
{
    if (cpu->opt_bigendian) {
        switch (offset) {
        case 0:
            return memval;
        case 1:
            return (memval & 0xffffff) << 8 | (regval & 0xff);
        case 2:
            return (memval & 0xffff) << 16 | (regval & 0xffff);
        case 3:
            return (memval & 0xff) << 24 | (regval & 0xffffff);
        }
    } else /* if MIPS target is little endian */ {
        switch (offset) {
        case 0:
            return (memval & 0xff) << 24 | (regval & 0xffffff);
        case 1:
            return (memval & 0xffff) << 16 | (regval & 0xffff);
        case 2:
            return (memval & 0xffffff) << 8 | (regval & 0xff);
        case 3:
            return memval;
        }
    }

    fprintf(stderr, "Invalid offset %x passed to lwl\n", offset);

    return 0xffffffff;
}

static void lwl_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, wordvirt, base, memword;
    uint8_t which_byte;
    int32_t offset;
    bool cacheable;

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;
    /* We request the word containing the byte-address requested. */
    wordvirt = virt & ~0x03UL;

    /* Translate virtual address to physical address. */
    phys = address_trans(wordvirt, DATALOAD, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Fetch word. */
    memword = fetch_word(cpu, phys, DATALOAD, cacheable);

    if (cpu->exception_pending)
        return;

    /* Insert bytes into the left side of the register. */
    which_byte = virt & 0x03;
    cpu->reg[rt(instr)] = lwl(cpu, cpu->reg[rt(instr)],
        memword, which_byte);
}

static void lw_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, base, word;
    int32_t offset;
    bool cacheable;

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;

    /* This virtual address must be word-aligned. */
    if (virt % 4 != 0) {
        exception(cpu, AdEL, DATALOAD, -1);
        return;
    }

    /* Translate virtual address to physical address. */
    phys = address_trans(virt, DATALOAD, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Fetch word. */
    word = fetch_word(cpu, phys, DATALOAD, cacheable);

    if (cpu->exception_pending)
        return;

    /* Load target register with data. */
    cpu->reg[rt(instr)] = word;
}

static void lbu_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, base, byte;
    int32_t offset;
    bool cacheable;

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;

    /* Translate virtual address to physical address. */
    phys = address_trans(virt, DATALOAD, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Fetch byte.  */
    byte = fetch_byte(cpu, phys, cacheable);
    byte &= 0x000000ff;

    if (cpu->exception_pending)
        return;

    /* Load target register with data. */
    cpu->reg[rt(instr)] = byte;
}

static void lhu_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, base, halfword;
    int32_t offset;
    bool cacheable;

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;

    /* This virtual address must be halfword-aligned. */
    if (virt % 2 != 0) {
        exception(cpu, AdEL, DATALOAD, -1);
        return;
    }

    /* Translate virtual address to physical address. */
    phys = address_trans(virt, DATALOAD, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Fetch halfword.  */
    halfword = fetch_halfword(cpu, phys, cacheable);
    halfword &= 0x0000ffff;

    if (cpu->exception_pending)
        return;

    /* Load target register with data. */
    cpu->reg[rt(instr)] = halfword;
}

static void lwr_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, wordvirt, base, memword;
    uint8_t which_byte;
    int32_t offset;
    bool cacheable;

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;
    /* We request the word containing the byte-address requested. */
    wordvirt = virt & ~0x03UL;

    /* Translate virtual address to physical address. */
    phys = address_trans(wordvirt, DATALOAD, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Fetch word. */
    memword = fetch_word(cpu, phys, DATALOAD, cacheable);

    if (cpu->exception_pending)
        return;

    /* Insert bytes into the left side of the register. */
    which_byte = virt & 0x03;
    cpu->reg[rt(instr)] = lwr(cpu, cpu->reg[rt(instr)], memword, which_byte);
}

static void sb_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, base;
    uint8_t data;
    int32_t offset;
    bool cacheable;

    /* Load data from register. */
    data = cpu->reg[rt(instr)] & 0x0ff;

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;

    /* Translate virtual address to physical address. */
    phys = address_trans(virt, DATASTORE, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Store byte. */
    store_byte(cpu, phys, data, cacheable);
}

static void sh_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, base;
    uint16_t data;
    int32_t offset;
    bool cacheable;

    /* Load data from register. */
    data = cpu->reg[rt(instr)] & 0x0ffff;

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;

    /* This virtual address must be halfword-aligned. */
    if (virt % 2 != 0) {
        exception(cpu, AdES, DATASTORE, -1);
        return;
    }

    /* Translate virtual address to physical address. */
    phys = address_trans(virt, DATASTORE, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Store halfword. */
    store_halfword(cpu, phys, data, cacheable);
}

static uint32_t swl(VmipsCpu *cpu, uint32_t regval, uint32_t memval,
    uint8_t offset)
{
    if (cpu->opt_bigendian) {
        switch (offset) {
        case 0:
            return regval;
        case 1:
            return (memval & 0xff000000) | (regval >> 8 & 0xffffff);
        case 2:
            return (memval & 0xffff0000) | (regval >> 16 & 0xffff);
        case 3:
            return (memval & 0xffffff00) | (regval >> 24 & 0xff);
        }
    } else /* if MIPS target is little endian */ {
        switch (offset) {
        case 0:
            return (memval & 0xffffff00) | (regval >> 24 & 0xff);
        case 1:
            return (memval & 0xffff0000) | (regval >> 16 & 0xffff);
        case 2:
            return (memval & 0xff000000) | (regval >> 8 & 0xffffff);
        case 3:
            return regval;
        }
    }

    fprintf(stderr, "Invalid offset %x passed to swl\n", offset);

    return 0xffffffff;
}

static uint32_t swr(VmipsCpu *cpu, uint32_t regval, uint32_t memval,
    uint8_t offset)
{
    if (cpu->opt_bigendian) {
        switch (offset) {
        case 0:
            return ((regval << 24) & 0xff000000) | (memval & 0xffffff);
        case 1:
            return ((regval << 16) & 0xffff0000) | (memval & 0xffff);
        case 2:
            return ((regval << 8) & 0xffffff00) | (memval & 0xff);
        case 3:
            return regval;
        }
    } else /* if MIPS target is little endian */ {
        switch (offset) {
        case 0:
            return regval;
        case 1:
            return ((regval << 8) & 0xffffff00) | (memval & 0xff);
        case 2:
            return ((regval << 16) & 0xffff0000) | (memval & 0xffff);
        case 3:
            return ((regval << 24) & 0xff000000) | (memval & 0xffffff);
        }
    }

    fprintf(stderr, "Invalid offset %x passed to swr\n", offset);

    return 0xffffffff;
}

static void swl_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, wordvirt, base, regdata, memdata;
    int32_t offset;
    uint8_t which_byte;
    bool cacheable;

    /* Load data from register. */
    regdata = cpu->reg[rt(instr)];

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;

    /* We request the word containing the byte-address requested. */
    wordvirt = virt & ~0x03UL;

    /* Translate virtual address to physical address. */
    phys = address_trans(wordvirt, DATASTORE, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Read data from memory. */
    memdata = fetch_word(cpu, phys, DATASTORE, cacheable);

    if (cpu->exception_pending)
        return;

    /* Write back the left side of the register. */
    which_byte = virt & 0x03UL;
    store_word(cpu, phys, swl(cpu, regdata, memdata, which_byte),
        cacheable);
}

static void sw_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, base, data;
    int32_t offset;
    bool cacheable;

    /* Load data from register. */
    data = cpu->reg[rt(instr)];

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;

    /* This virtual address must be word-aligned. */
    if (virt % 4 != 0) {
        exception(cpu, AdES, DATASTORE, -1);
        return;
    }

    /* Translate virtual address to physical address. */
    phys = address_trans(virt, DATASTORE, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Store word. */
    store_word(cpu, phys, data, cacheable);
}

static void swr_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t phys, virt, wordvirt, base, regdata, memdata;
    int32_t offset;
    uint8_t which_byte;
    bool cacheable;

    /* Load data from register. */
    regdata = cpu->reg[rt(instr)];

    /* Calculate virtual address. */
    base = cpu->reg[rs(instr)];
    offset = s_immed(instr);
    virt = base + offset;
    /* We request the word containing the byte-address requested. */
    wordvirt = virt & ~0x03UL;

    /* Translate virtual address to physical address. */
    phys = address_trans(wordvirt, DATASTORE, &cacheable);

    if (cpu->exception_pending)
        return;

    /* Read data from memory. */
    memdata = fetch_word(cpu, phys, DATASTORE, cacheable);

    if (cpu->exception_pending)
        return;

    /* Write back the right side of the register. */
    which_byte = virt & 0x03UL;
    store_word(cpu, phys, swr(cpu, regdata, memdata, which_byte),
        cacheable);
}

/* from cpzero.cc */
static void rfe_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    cpu->cp0_reg[Status] = (cpu->cp0_reg[Status] & 0xfffffff0)
            | ((cpu->cp0_reg[Status] >> 2) & 0x0f);
}

static uint32_t read_masks[] = {
    Index_MASK, Random_MASK, EntryLo_MASK, 0, Context_MASK,
    PageMask_MASK, Wired_MASK, Error_MASK, BadVAddr_MASK, Count_MASK,
    EntryHi_MASK, Compare_MASK, Status_MASK, Cause_MASK, EPC_MASK,
    PRId_MASK, Config_MASK, LLAddr_MASK, WatchLo_MASK, WatchHi_MASK,
    0, 0, 0, 0, 0, 0, ECC_MASK, CacheErr_MASK, TagLo_MASK, TagHi_MASK,
    ErrorEPC_MASK, 0
};

static uint32_t write_masks[] = {
    Index_MASK, 0, EntryLo_MASK, 0, Context_MASK & ~Context_BadVPN_MASK,
    PageMask_MASK, Wired_MASK, Error_MASK, 0, Count_MASK,
    EntryHi_MASK, Compare_MASK, Status_MASK,
    Cause_MASK & ~Cause_IP_Ext_MASK, 0, 0, Config_MASK, LLAddr_MASK,
    WatchLo_MASK, WatchHi_MASK, 0, 0, 0, 0, 0, 0, ECC_MASK,
    CacheErr_MASK, TagLo_MASK, TagHi_MASK, ErrorEPC_MASK, 0
};

static void cp0_reset(VmipsCpu *cpu)
{
    /* Turn off any randomly-set pending-interrupt bits, as these
     * can impact correctness. */
    cpu->cp0_reg[Cause] &= ~Cause_IP_MASK;

    /* Reset Random register to upper bound (8<=Random<=63) */
    cpu->cp0_reg[Random] = Random_UPPER_BOUND << 8;

    /* Reset Status register: clear KUc, IEc, SwC (i.e., caches are not
     * switched), TS (TLB shutdown has not occurred), and set
     * BEV (Bootstrap exception vectors ARE in effect).
     */
    cpu->cp0_reg[Status] = (cpu->cp0_reg[Status] | Status_DS_BEV_MASK) &
        ~(Status_KUc_MASK | Status_IEc_MASK | Status_DS_SwC_MASK |
          Status_DS_TS_MASK);
    cpu->cp0_reg[PRId] = 0x00000230; /* MIPS R3000A */
}

static void mfc0_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    int r;

    r = rd(instr);

    /* This ensures that non-existent CP0 registers read as zero. */
    cpu->reg[rt(instr)] = cpu->cp0_reg[r] & read_masks[r];
}

static void mtc0_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint32_t data;
    int r;

    r = rd(instr);

    /* This preserves the bits which are readable but not writable, and writes
     * the bits which are writable with new data, thus making it suitable
     * for mtc0-type operations.  If you want to write all the bits which
     * are _connected_, use: reg[r] = new_data & write_masks[r]; .
     */
    data = cpu->reg[rt(instr)];

    cpu->cp0_reg[r] = (cpu->cp0_reg[r] & (read_masks[r] & ~write_masks[r]))
             | (data & write_masks[r]);
}

/* Convention says that CP0's condition is TRUE if the memory
   write-back buffer is empty. Because memory writes are fast as far
   as the emulation is concerned, the write buffer is always empty
   for CP0. */
static inline bool cpCond(void)
{
    return true;
}

static void bc0x_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint16_t condition = rt(instr);

    switch (condition) {
    case 0: /* bc0f */
        if (!cpCond()) {
            branch(cpu, instr, pc);
        }
        break;

    case 1: /* bc0t */
        if (cpCond()) {
            branch(cpu, instr, pc);
        }
        break;

    case 2: /* bc0fl - not valid, but not reserved(A-17, H&K) - no-op. */
    case 3: /* bc0tl - not valid, but not reserved(A-21, H&K) - no-op. */
        break;
    default:
        exception(cpu, RI, ANY, 0);
        break; /* reserved */
    }
}

static void cpzero_emulate(VmipsCpu *cpu, uint32_t instr, uint32_t pc)
{
    uint16_t instr_rs = rs(instr);

    if (instr_rs > 15) {
        switch (funct(instr)) {
        /* FIXME: tlb*_emulate()s are skipped */

        case 16:
            rfe_emulate(cpu, instr, pc);
            break;

        default:
            exception(cpu, RI, ANY, 0);
            break;
        }
    } else {
        switch (instr_rs) {
        case 0:
            mfc0_emulate(cpu, instr, pc);
            break;
        case 2: /* cfc0 - reserved */
            exception(cpu, RI, ANY, 0);
            break;
        case 4:
            mtc0_emulate(cpu, instr, pc);
            break;
        case 6: /* ctc0 - reserved */
            exception(cpu, RI, ANY, 0);
            break;
        case 8:
            bc0x_emulate(cpu, instr, pc);
            break;
        default:
            exception(cpu, RI, ANY, 0);
            break;
        }
    }
}

#define PRECISE_DEBUG
#undef PRECISE_DEBUG
int precise_cpu_mips_exec(CPUArchState *env)
{
    VmipsCpu *cpu = &env->vmips;

    static const emulate_funptr opcodeJumpTable[] = {
    &funct_emulate, &regimm_emulate,  &j_emulate,     /*  1 */
    &jal_emulate,   &beq_emulate,     &bne_emulate,   /*  2 */
    &blez_emulate,  &bgtz_emulate,    &addi_emulate,  /*  3 */
    &addiu_emulate, &slti_emulate,    &sltiu_emulate, /*  4 */
    &andi_emulate,  &ori_emulate,     &xori_emulate,  /*  5 */
    &lui_emulate,   &cpzero_emulate,  &cpone_emulate, /*  6 */
    &cptwo_emulate, &cpthree_emulate, &RI_emulate,    /*  7 */
    &RI_emulate,    &RI_emulate,      &RI_emulate,    /*  8 */
    &RI_emulate,    &RI_emulate,      &RI_emulate,    /*  9 */
    &RI_emulate,    &RI_emulate,      &RI_emulate,    /* 10 */
    &RI_emulate,    &RI_emulate,      &lb_emulate,    /* 11 */
    &lh_emulate,    &lwl_emulate,     &lw_emulate,    /* 12 */
    &lbu_emulate,   &lhu_emulate,     &lwr_emulate,   /* 13 */
    &RI_emulate,    &sb_emulate,      &sh_emulate,    /* 14 */
    &swl_emulate,   &sw_emulate,      &RI_emulate,    /* 15 */
    &RI_emulate,    &swr_emulate,     &RI_emulate,    /* 16 */
    &RI_emulate,    &lwc1_emulate,    &lwc2_emulate,  /* 17 */
    &lwc3_emulate,  &RI_emulate,      &RI_emulate,    /* 18 */
    &RI_emulate,    &RI_emulate,      &RI_emulate,    /* 19 */
    &swc1_emulate,  &swc2_emulate,    &swc3_emulate,  /* 20 */
    &RI_emulate,    &RI_emulate,      &RI_emulate,    /* 21 */
    &RI_emulate
    };

    /* Clear exception_pending flag if it was set by a prior instruction. */
    cpu->exception_pending = false;

    /* FIXME */
#if 0
    /* Decrement Random register every clock cycle. */
    cpzero->adjust_random();
#endif

    /* Save address of instruction responsible for exceptions
       which may occur. */
    if (cpu->delay_state != DELAYSLOT)
        cpu->next_epc = cpu->pc;

    /* Get physical address of next instruction. */
    bool cacheable;
    uint32_t real_pc = address_trans(cpu->pc, INSTFETCH, &cacheable);

    if (cpu->exception_pending) {
        if (cpu->opt_excmsg)
            fprintf(stderr,
                "** PC address translation caused the exception! **\n");
        goto out;
    }

    cpu->instr = fetch_word(cpu, real_pc, INSTFETCH, cacheable);

    if (cpu->exception_pending) {
        if (cpu->opt_excmsg)
            fprintf(stderr, "** Instruction fetch caused the exception! **\n");
        goto out;
    }

    /* Disassemble the instruction, if the user requested it. */
    if (cpu->opt_instdump) {
        fprintf(stderr, "PC=0x%08x [%08x]\t%08x \n",
            cpu->pc, real_pc, cpu->instr);
        /* FIXME: use print_insn_big_mips */
        /* machine->disasm->disassemble(pc, instr); */
    }

    /* FIXME: checking for an interrupt is skipped */

    /* Emulate the instruction by jumping
     * to the appropriate emulation method. */
    (*opcodeJumpTable[opcode(cpu->instr)])(cpu, cpu->instr, cpu->pc);

out:
    /* Force register zero to contain zero. */
    cpu->reg[reg_zero] = 0;

    /*
     * If an exception is pending, then the PC has already been changed to
     * contain the exception vector.  Return now, so that we don't clobber it.
     */
    if (cpu->exception_pending) {
        /* Instruction at beginning of exception handler is NOT in delay slot,
           no matter what the last instruction was. */
        cpu->delay_state = NORMAL;
        #ifdef PRECISE_DEBUG
        return EXCP_DEBUG;
        #else
        return 0;
        #endif
    }

    /*
     * Recall the delay_state values: 0=NORMAL, 1=DELAYING, 2=DELAYSLOT.
     * This is what the delay_state values mean (at this point in the code):
     * DELAYING: The last instruction caused a branch to be taken.
     *  The next instruction is in the delay slot.
     *  The next instruction EPC will be PC - 4.
     * DELAYSLOT: The last instruction was executed in a delay slot.
     *  The next instruction is on the other end of the branch.
     *  The next instruction EPC will be PC.
     * NORMAL: No branch was executed; next instruction is at PC + 4.
     *  Next instruction EPC is PC.
     */

    /* Update the pc and delay_state values. */
    cpu->pc += 4;

    if (cpu->delay_state == DELAYSLOT)
        cpu->pc = cpu->delay_pc;

    /* 0->0, 1->2, 2->0 */
    cpu->delay_state = (cpu->delay_state << 1) & 0x03;

    #ifdef PRECISE_DEBUG
    {
        /* FIXME: drop this code */

        int i;

        env->active_tc.PC = cpu->pc;

        for (i = 0; i < 32; i++) {
            env->active_tc.gpr[i] = cpu->reg[i];
        }
    }

    return EXCP_DEBUG;
    #else
    return 0;
    #endif
}

void precise_mips_cpu_init(struct CPUMIPSState *env)
{
    VmipsCpu *cpu = &env->vmips;

    cpu->reg[reg_zero] = 0;
    cpu->pc = 0xbfc00000;
    cpu->exception_pending = false;

    cpu->last_epc = 0;
    cpu->last_prio = 0;
    cpu->delay_state = NORMAL;
    /* FIXME: drop it */
    cpu->delay_pc = 0;

    cpu->opt_excmsg = false;
    cpu->opt_reportirq = false;
    cpu->opt_excpriomsg = false;
    cpu->opt_haltbreak = false;
    cpu->opt_haltibe = false;
    cpu->opt_haltjrra = false;
    cpu->opt_instdump = false;

    cpu->opt_bigendian = true;

    cp0_reset(cpu);
}
