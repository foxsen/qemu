/*
 * Tiny Code Generator for QEMU
 * Copyright (c) 2020 Loongson Technology Corporation Limited
 * Copyright (c) 2008-2009 Arnaud Patard <arnaud.patard@rtp-net.org>
 * Copyright (c) 2009 Aurelien Jarno <aurelien@aurel32.net>
 * Copyright (c) 2008 Fabrice Bellard
 *
 * Based on i386/tcg-target.c and mips/tcg-target.c
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

 /*
  * Todo list:
  * 1. Softmmu support
  * 2. Some ops(bswap/orc/andc/div/mul/rem) need to be verified
  * 3. Host vec support
  * 4. Remove useless debug messages
  * 5. Movi API need to be rework, tcg pool will be removed.
  * 6. Performance improvements
  */

#include "../tcg-pool.inc.c"

//#define LOONGARCH_DEBUG

#ifdef CONFIG_DEBUG_TCG
static const char * const tcg_target_reg_names[TCG_TARGET_NB_REGS] = {
    "zero",
    "ra",
    "tp",
    "sp",
    "a0",
    "a1",
    "a2",
    "a3",
    "a4",
    "a5",
    "a6",
    "a7",
    "t0",
    "t1",
    "t2",
    "t3",
    "t4",
    "t5",
    "t6",
    "t7",
    "t8",
    "x",
    "fp",
    "s0",
    "s1",
    "s2",
    "s3",
    "s4",
    "s5",
    "s6",
    "s7",
    "s8",
};
#endif

static const int tcg_target_reg_alloc_order[] = {
    /* Call saved registers */
    /* TCG_REG_S0 reservered for TCG_AREG0 */
    TCG_REG_S1,
    TCG_REG_S2,
    TCG_REG_S3,
    TCG_REG_S4,
    TCG_REG_S5,
    TCG_REG_S6,
    TCG_REG_S7,
    TCG_REG_S8,

    /* Call clobbered registers */
    TCG_REG_T0,
    TCG_REG_T1,
    TCG_REG_T2,
    TCG_REG_T3,
    TCG_REG_T4,
    TCG_REG_T5,
    TCG_REG_T6,

    /* Argument registers */
    TCG_REG_A0,
    TCG_REG_A1,
    TCG_REG_A2,
    TCG_REG_A3,
    TCG_REG_A4,
    TCG_REG_A5,
    TCG_REG_A6,
    TCG_REG_A7,
};

static const int tcg_target_call_iarg_regs[] = {
    TCG_REG_A0,
    TCG_REG_A1,
    TCG_REG_A2,
    TCG_REG_A3,
    TCG_REG_A4,
    TCG_REG_A5,
    TCG_REG_A6,
    TCG_REG_A7,
};

static const int tcg_target_call_oarg_regs[] = {
    TCG_REG_A0,
    TCG_REG_A1,
};

#define TCG_CT_CONST_ZERO  0x100
#define TCG_CT_CONST_S12   0x200
#define TCG_CT_CONST_N12   0x400
#define TCG_CT_CONST_M12   0x800
#define TCG_CT_CONST_P2M1  0x1000    /* Power of 2 minus 1.  */
#define TCG_CT_CONST_U12   0x2000

static inline bool is_p2m1(tcg_target_long val)
{
    return val && ((val + 1) & val) == 0;
}

static inline tcg_target_long sextreg(tcg_target_long val, int pos, int len)
{
    if (TCG_TARGET_REG_BITS == 32) {
        return sextract32(val, pos, len);
    } else {
        return sextract64(val, pos, len);
    }
}

/* parse target specific constraints */
static const char *target_parse_constraint(TCGArgConstraint *ct,
                                           const char *ct_str, TCGType type)
{
    switch (*ct_str++) {
    case 'r':
        ct->ct |= TCG_CT_REG;
        ct->u.regs = 0xffffffff;
        break;
    case 'L':
        /* qemu_ld/qemu_st constraint */
        ct->ct |= TCG_CT_REG;
        ct->u.regs = 0xffffffff;
        tcg_regset_reset_reg(ct->u.regs, TCG_REG_A0);
        /* qemu_ld/qemu_st uses TCG_REG_TMP0 */
#if defined(CONFIG_SOFTMMU)
        tcg_regset_reset_reg(ct->u.regs, tcg_target_call_iarg_regs[0]);
        tcg_regset_reset_reg(ct->u.regs, tcg_target_call_iarg_regs[1]);
        tcg_regset_reset_reg(ct->u.regs, tcg_target_call_iarg_regs[2]);
        tcg_regset_reset_reg(ct->u.regs, tcg_target_call_iarg_regs[3]);
        tcg_regset_reset_reg(ct->u.regs, tcg_target_call_iarg_regs[4]);
#endif
        break;
    case 'I':
        ct->ct |= TCG_CT_CONST_S12;
        break;
    case 'N':
        ct->ct |= TCG_CT_CONST_N12;
        break;
    case 'M':
        ct->ct |= TCG_CT_CONST_M12;
        break;
    case 'Z':
        /* we can use a zero immediate as a zero register argument. */
        ct->ct |= TCG_CT_CONST_ZERO;
        break;
    case 'K':
        ct->ct |= TCG_CT_CONST_P2M1;
        break;
    case 'U':
        ct->ct |= TCG_CT_CONST_U12;
        break;
    default:
        return NULL;
    }
    return ct_str;
}

/* test if a constant matches the constraint */
static int tcg_target_const_match(tcg_target_long val, TCGType type,
                                  const TCGArgConstraint *arg_ct)
{
    int ct = arg_ct->ct;
    if (ct & TCG_CT_CONST) {
        return 1;
    }
    if ((ct & TCG_CT_CONST_ZERO) && val == 0) {
        return 1;
    }
    if ((ct & TCG_CT_CONST_S12) && val == sextreg(val, 0, 12)) {
        return 1;
    }
    if ((ct & TCG_CT_CONST_N12) && -val == sextreg(-val, 0, 12)) {
        return 1;
    }
    if ((ct & TCG_CT_CONST_M12) && val >= -0xfff && val <= 0xfff) {
        return 1;
    }
    if ((ct & TCG_CT_CONST_U12) && val == ((uint16_t)val & 0xFFF)) {
        return 1;
    }
    if ((ct & TCG_CT_CONST_P2M1) && is_p2m1(val)) {
        return 1;
    }
    return 0;
}

/*
 * RISC-V Base ISA opcodes (IM)
 */

typedef enum {
    OPC_CLZD = 0x09, /*LoongArch 0000000000000000000101 0x09 CLZ.D*/
    OPC_CTZD = 0x0B, /*LoongArch 0000000000000000001011 0x0B CTZ.D*/
    OPC_SEH = 0x16,/* LoongARCH: EXT.W.H 0000000000000000010110 */
    OPC_SEB = 0x17,/* LoongARCH: EXT.W.B 0000000000000000010111 */
    OPC_ADD =  0x21, /*LoongARCH: 00000000000100001 => 0x21 ADD.D*/
    OPC_SUB = 0x23, /*LoongArch 00000000000100011 0x23 SUB.D*/
    OPC_SLT = 0x24, /*LoongARCH: 00000000000100100 0x24 */
    OPC_SLTU = 0x25, /*LoongARCH: 00000000000100101 => 0x25 SLTU*/

    OPC_MASKEQZ = 0x26, /*LoongArch: 00000000000100110 0x26 SELNEZ*/
    OPC_MASKNEZ = 0x27, /*LoongArch: 00000000000100111 0x27 SELEQZ*/
    
    OPC_NOR = 0x28, /*LoongArch: 00000000000101000 0x28 NOR*/
    OPC_AND = 0x29, /*LoongArch: 00000000000101001 0x29*/
    OPC_OR = 0x2A, /*LoongARCH: OR: 00000000000101010 2A*/
    OPC_XOR = 0x2B, /*LoongARCH: XOR: 00000000000101011 2B*/
    OPC_ORN = 0x2C, /*LoongARCH: ORN: 00000000000101100 2C*/
    OPC_ANDN = 0x2D, /*LoongARCH: ANDN: 00000000000101101 2D*/

    OPC_SLL = 0x31, /*Loonarch: 00000000000110001 0x31 SLL.D*/
    OPC_SRL = 0x32, /*Loonarch: 00000000000110010 0x32 SRL.D*/
    OPC_SRA = 0x33, /*Loonarch: 00000000000110011 0x33 SRA.D*/
    OPC_ROTRD = 0x37, /*00000000000110111 ROTR.D*/


    OPC_MULHWU = 0x3A, /*00000000000111010 MULHWU*/
    OPC_MUL = 0x3B, /*LoongArch: 00000000000111011 MUL.D*/
    OPC_DMULH = 0x3C, /*00000000000111100 MULH.D*/
    OPC_MULHDU = 0x3D, /*00000000000111101 MULHDU*/
    OPC_DIVWU = 0x42, /*00000000001000010 DIVU@Mispr6*/
    OPC_DIVD= 0x44, /*00000000001000100 DIVD@Mispr6*/
    OPC_MODD= 0x45, /*00000000001000101 DMOD@Mispr6*/
    OPC_DIVDU = 0x46, /*00000000001000110 DDIVU@MipsR6*/
    OPC_MODDU = 0x47, /*00000000001000111 DMODU@MipsR6*/

    OPC_SLTI = 0x08,  /*LoongARCH: 0000001008 => 0x08*/
    OPC_SLTIU = 0x09, /*LoongARCH: 0000001001 => 0x09*/
    OPC_ADDI = 0x0B, /*LoongARCH: 0000001011 => 0xB ADDI.D*/
    OPC_ANDI = 0x0D, /*LoongARCH: 0000001101 => 0xD*/
    OPC_ORI = 0x0E, /*LoongARCH: 0000001110 => 0xE*/
    OPC_XORI = 0x0F, /*LoongARCH: 0000001111 => 0xF XORI*/

    OPC_AUIPC = 0x0E,/*LoongARCH: PCADDU12I 0001110 */
    OPC_BEQ = 0x16,  /*LoongArch: 010110 => 0x16*/
    OPC_BNE = 0x17,  /*LoongArch: 010111 => 0x17*/
    OPC_BLT = 0x18,  /*LoongArch: 011000 => 0x18*/
    OPC_BGE = 0x19,  /*LoongArch: 011001 => 0x19*/
    OPC_BLTU = 0x1A, /*LoongArch: 011010 => 0x1A*/
    OPC_BGEU = 0x1B, /*LoongArch: 011011 => 0x1B*/

    OPC_JIRL = 0x13, /* LoongArch 010011 => 0x13*/
    OPC_B = 0x14, /* LoongArch: 010100 => 0x14 instead of OPC_J*/
    OPC_BL = 0x15, /* LoongArch: 010101 => 0x15 instead of OPC_JAL*/

    OPC_LB = 0xA0, /*LoongArch: 0010100000 0xA0*/
    OPC_LH = 0xA1, /*0010100001 LD.H*/
    OPC_LW = 0xA2, /*LoongARCH: 0010100010 => 0xA2*/
    OPC_LD = 0xA3, /*LoongARCH: 0010100011 => 0xA3*/
    OPC_SB = 0xA4, /* LoongArch: 0010100100 ST.B*/
    OPC_SH = 0xA5, /*LoongArch: 0010100101 0xA5 ST.H*/
    OPC_SW = 0xA6, /*LoongARCH: 0010100110 => 0xA6*/
    OPC_SD = 0xA7, /*LoongARCH: 0010100111 => 0xA7 ST.D*/
    OPC_LBU = 0xA8, /*LoongARCH: 0010101000 => 0xA8*/
    OPC_LHU = 0xA9, /*LoongArch 0010101001 0xA9 LD.HU*/
    OPC_LWU = 0xAA, /*LoongArch: 0010101010 0xAA*/

    /*
     * Need to remove dup defination.
     */
    OPC_SLLI = 0x41, /*LoongARCH: 0000000001000001 => 0x41 SLLI.D*/
    OPC_DSLL = 0x41,/* LoongARCH: SLLI.D 0000000001000001 */
    OPC_SRLI = 0x45,/*LoongARCH: 0000000001000101 => 0x45 SRLI.D*/
    OPC_SRLID = 0x45,/* LoongARCH: SRLI.D 0000000001000101 */
    OPC_SRAI = 0x49, /*LoongARCH: 0000000001001001 => 0x49 SRAI.D*/
    OPC_ROTRID=0x4D, /*LoongArch: 0000000001001101 0x4D ROTR.D*/

    OPC_INS = (0x03 << 21) | (0 << 15),/* LoongARCH: BSTRINS.W BS00000000011 0x03 Bit15=0*/
    OPC_EXT = (0x03 << 21) | (1 << 15),/* LoongARCH: BSTRPICK.W BS00000000011 0x03 Bit15=1*/
    OPC_DINS = 0x02,/* LoongARCH: BSTRINS.D 000000010 0x02 */
    OPC_DEXT = 0x03,/* LoongARCH: BSTRPICK.D 000000011 0x03 */

    OPC_DBAR = 0x70E4, /* LoongARCH: DBAR 00111000011100100 */

    OPC_LUI = 0x0A, /*LoongARCH: LU12I 0001010 => 0xA, opc is 26..31*/
    OPC_LU32I =0x0B, /*LoongARCH: LU32I 0001011 => 0xB, opc is 26..31*/


#if TCG_TARGET_REG_BITS == 64
    /*
     * This OPC only affect 32 bits !!!
     */
    OPC_CLZW = 0x05, /*LoongArch:0000000000000000000101 0x05 CLZ.W*/
    OPC_CTZW = 0x07, /*LoongArch:0000000000000000000111 0x07 CTZ.W*/
    OPC_ADDIW = 0xA,/*LoongARCH: 0000001010 => 0xA ADDI.W*/
    OPC_ADDW = 0x20, /*LoongARCH: 00000000000100000 => 0x20 ADD.W*/
    OPC_ROTRW = 0x36, /*00000000000110110 ROTR.W*/
    OPC_MULW = 0x38, /*Loongarch: 00000000000111000 MUL.W*/
    OPC_MULHW = 0x39, /*Loongarch: 00000000000111001 MULH.W*/
    OPC_SLLIW = 0x81, /*LoongARCH: 00000000010000001 => 0x81 SLLI.W*/
    OPC_SLLW = 0x2E, /*LoongARch: 00000000000101110 0x2E SLL.W*/
    OPC_SRAIW = 0x91, /*00000000010010001 SRAI.W*/
    OPC_SRLIW = 0x89, /*LoongArch 00000000010001001 0x89 SRLI.W*/
    OPC_SRLW = 0x2F, /*LoongARch: 00000000000101111 0x2F SRL.W*/
    OPC_SRAW = 0x30, /*LoongARch: 00000000000110000 0x30 SRA.W*/
    OPC_SUBW = 0x22, /*LoongArch 00000000000100010 0x22 SUB.W*/

    OPC_ROTRIW=0x99, /*LoongArch: 00000000010011001 0x99 ROTR.W*/
#else
    /* Simplify code throughout by defining aliases for RV32.  */
    OPC_ADDIW = OPC_ADDI,
    OPC_ADDW = OPC_ADD,
    OPC_DIVUW = OPC_DIVU,
    OPC_DIVW = OPC_DIV,
    OPC_MULW = OPC_MUL,
    OPC_REMUW = OPC_REMU,
    OPC_REMW = OPC_REM,
    OPC_SLLIW = OPC_SLLI,
    OPC_SLLW = OPC_SLL,
    OPC_SRAIW = OPC_SRAI,
    OPC_SRAW = OPC_SRA,
    OPC_SRLIW = OPC_SRLI,
    OPC_SRLW = OPC_SRL,
    OPC_SUBW = OPC_SUB,
#endif

    /* To avoid SP calculate error*/
    /* Different with Riscv, we need to use different Ops for data load/store*/
    ALIAS_PADD     = sizeof(void *) == 4 ? OPC_ADDW : OPC_ADD,
    ALIAS_PADDI    = sizeof(void *) == 4 ? OPC_ADDIW : OPC_ADDI,
} LoongarchInsn;

static int32_t encode_r(LoongarchInsn opc, TCGReg rd, TCGReg rs1, TCGReg rs2)
{
    #ifdef LOONGARCH_DEBUG
    printf ("Create insn 0x%x\n", (opc << 15) | (rd & 0x1f) | (rs1 & 0x1f) << 5 | (rs2 & 0x1f) << 10);
    #endif
    return (opc << 15) | (rd & 0x1f) | (rs1 & 0x1f) << 5 | (rs2 & 0x1f) << 10;
}

static int32_t encode_imm12(uint32_t imm)
{
    return (imm & 0xfff) << 10;
}

static int32_t encode_i(LoongarchInsn opc, TCGReg rd, TCGReg rs1, uint32_t imm)
{
    return (opc << 22) | (rd & 0x1f) | (rs1 & 0x1f) << 5 | encode_imm12(imm);
}

static int32_t encode_simm12(uint32_t imm)
{
    return (imm & 0xFFF) << 10;
}

static int32_t encode_s(LoongarchInsn opc, TCGReg rs1, TCGReg rs2, uint32_t imm)
{
    return (opc << 22) | (rs1 & 0x1f) << 5 | (rs2 & 0x1f) | encode_simm12(imm);
}

static int32_t encode_uimm20(uint32_t imm)
{
    return ((imm & 0xfffff000) >> 12) << 5;
}

static int32_t encode_u(LoongarchInsn opc, TCGReg rd, uint32_t imm)
{
    return (opc << 25) | (rd & 0x1f) | encode_uimm20(imm);
}

/*LoongArch for B and BL insn*/
static int32_t encode_uj(LoongarchInsn opc, uint32_t imm)
{
    int32_t insn = 0;
    int32_t offset = 0;

    tcg_debug_assert((imm & 3) == 0);

    insn |= (opc << 26);
    offset |= ((imm >> 2) & 0xFFFF) << 10;
    offset |= ((imm >> 2)& 0x3FF0000) >> 16;
    insn |= offset;

    return insn;
}

/*
 * JIRL: nextPC=GR[rj]+sext(offs<<2), GR[rd]=PC+4
 * If you don't want to update PC value, set rd is REG_ZER0
 */
static void tcg_out_opc_jirl(TCGContext *s, TCGReg rd, TCGReg rs)
{
    int32_t insn = 0;
    insn = (OPC_JIRL << 26);
    insn |= (0 << 10); /*offset field is 0*/
    insn |= (rs & 0x1F) << 5;
    insn |= (rd & 0x1F);
    #ifdef LOONGARCH_DEBUG
    printf("[%s] insn=0x%x rs=0x%x,rd=0x%x\n",__func__,insn,rs,rd);
    #endif
    tcg_out32(s, insn);
}

/*
 * 3 Regs operation. rd,rj,rk
 */
static void tcg_out_opc_reg(TCGContext *s, LoongarchInsn opc,
                            TCGReg rd, TCGReg rs1, TCGReg rs2)
{
    tcg_out32(s, encode_r(opc, rd, rs1, rs2));
}


static int32_t encode_2r(LoongarchInsn opc, TCGReg rd, TCGReg rj)
{
    return (opc << 10) | (rd & 0x1f) | (rj & 0x1f) << 5;
}
/*
 * 2 Regs operation. rd,rj
 */
static void tcg_out_opc_2reg(TCGContext *s, LoongarchInsn opc,
                            TCGReg rd, TCGReg rj)
{
    tcg_out32(s, encode_2r(opc, rd, rj));
}

static void tcg_out_opc_imm(TCGContext *s, LoongarchInsn opc,
                            TCGReg rd, TCGReg rs1, TCGArg imm)
{
    #ifdef LOONGARCH_DEBUG
    printf("[%s] insn=0x%x,opc=0x%x ,rd=0x%x, rs1=0x%x, imm=%ld\n",__func__,encode_i(opc, rd, rs1, imm),opc,rd,rs1,imm);
    #endif
    tcg_out32(s, encode_i(opc, rd, rs1, imm));
}

static void tcg_out_opc_store(TCGContext *s, LoongarchInsn opc,
                              TCGReg rs1, TCGReg rs2, uint32_t imm)
{
    #ifdef LOONGARCH_DEBUG
    printf("[%s] insn=0x%x, opc=0x%x,addr=0x%x, data=0x%x, imm=0x%x\n",__func__,encode_s(opc, rs1, rs2, imm),opc,rs1,rs2,imm);
    #endif
    tcg_out32(s, encode_s(opc, rs1, rs2, imm));
}

/*
 * LoongArch encode BEQ BNE BLT BGE BLTU BGEU
 * TCG input format: rs1 (condition) rs2
 */
static int32_t encode_b(LoongarchInsn opc, TCGReg rs1, TCGReg rs2, uint32_t imm)
{
    int32_t insn = 0;
    insn |= (opc << 26);
    insn |= (rs2 & 0x1f);
    insn |= (rs1 & 0x1f) << 5;
    /*
     * Save 18bits coz insn run as below
     * nextPC=PC+sext(offs<<2)
     */
    insn |= ((imm & 0x3FFFF) >> 2) << 10;
    return insn;
}

static void tcg_out_opc_branch(TCGContext *s, LoongarchInsn opc,
                               TCGReg rs1, TCGReg rs2, uint32_t imm)
{
    #ifdef LOONGARCH_DEBUG
    printf("[%s] insn=0x%x, opc=0x%x,rs1=0x%x, rs2=0x%x\n",__func__,encode_b(opc, rs1, rs2, imm),opc,rs1,rs2);
    #endif
    tcg_out32(s, encode_b(opc, rs1, rs2, imm));
}

static void tcg_out_opc_upper(TCGContext *s, LoongarchInsn opc,
                              TCGReg rd, uint32_t imm)
{
    #ifdef LOONGARCH_DEBUG
    printf("[%s] insn=0x%x,opc=0x%x,rd=0x%x, imm=0x%x\n",__func__,encode_u(opc, rd, imm),opc,rd,imm);
    #endif
    tcg_out32(s, encode_u(opc, rd, imm));
}

/*
 * For INS/EXT.
 */
static void tcg_out_opc_bf(TCGContext *s, LoongarchInsn opc, TCGReg rd,
                                  TCGReg rs, int msb, int lsb)
{
    int32_t insn = 0;
    tcg_debug_assert((opc == OPC_INS) ||  (opc == OPC_EXT));

    insn = opc;
    insn |= (rs & 0x1F) << 5;
    insn |= (rd & 0x1F);
    insn |= (msb & 0x1F) << 16;
    insn |= (lsb & 0x1F) << 10;

    #ifdef LOONGARCH_DEBUG
    printf("[%s] insn=0x%x\n",__func__,insn);
    #endif
    tcg_out32(s, insn);
}

/*
 * For DINS/DEXT.
 */
static void tcg_out_opc_bf64(TCGContext *s, LoongarchInsn opc, TCGReg rd,
                                  TCGReg rs, int msb, int lsb)
{
    int32_t insn = 0;

    insn = (opc << 22);
    insn |= (rs & 0x1F) << 5;
    insn |= (rd & 0x1F);
    insn |= (msb & 0x3F) << 16;
    insn |= (lsb & 0x3F) << 10;
    #ifdef LOONGARCH_DEBUG
    printf("[%s] insn=0x%x, msb=%d, lsb=%d\n",__func__,insn,msb,lsb);
    #endif
    tcg_out32(s, insn);
}

static void tcg_out_nop_fill(tcg_insn_unit *p, int count)
{
    int i;
    for (i = 0; i < count; ++i) {
        p[i] = encode_i(OPC_ANDI, TCG_REG_ZERO, TCG_REG_ZERO, 0);
    }
}

/*
 * Relocations
 */
static void reloc_pc_10_16_s2(tcg_insn_unit *pc, tcg_insn_unit *target)
{
    intptr_t offs = (intptr_t)target - (intptr_t)pc;
    #ifdef LOONGARCH_DEBUG
    printf("[reloc debug]: target = %p, pc = %p, off = 0x%lx, *pc = 0x%x\n", target, pc, offs, *pc);
    #endif
    /* check 4-aligned */
    tcg_debug_assert(offs % 4 == 0);
    /* check 18-bit signed */
    offs >>= 2;
    tcg_debug_assert(((offs & ~0x7fffU) == 0) || ((offs & ~0x7fffU) == ~0x7fffU));
    /* (*PC) [25 ... 10] = offs [17 ... 2] */
    *pc = deposit32(*pc, 10, 16, offs & 0xffff);
    #ifdef LOONGARCH_DEBUG
    printf("[reloc debug]: *pc = 0x%x\n", *pc);
    #endif
}

static void reloc_pc_0_10_10_16_s2(tcg_insn_unit *pc, tcg_insn_unit *target)
{
    intptr_t offs = (intptr_t)target - (intptr_t)pc;
    /* check 4-aligned */
    tcg_debug_assert(offs % 4 == 0);
    /* check 28-bit signed */
    offs >>= 2;
    tcg_debug_assert(((offs & ~0x1ffffffU) == 0) || ((offs & ~0x1ffffffU) == ~0x1ffffffU));
    /*
     * (*PC) [9 ... 0] = offs [27 ... 18]
     * (*PC) [25 ... 10] = offs [17 ... 2]
     */
    *pc = deposit32(*pc, 10, 16, offs & 0xffff);
    *pc = deposit32(*pc, 0, 10, (offs & 0x3ff0000) >> 16);
}

static bool reloc_call(tcg_insn_unit *code_ptr, tcg_insn_unit *target)
{
    intptr_t offset = (intptr_t)target - (intptr_t)code_ptr;
    int32_t lo = sextreg(offset, 0, 12);
    int32_t hi = offset - lo;

    if (offset == hi + lo) {
        #ifdef LOONGARCH_DEBUG
        printf("Offset is %ld\n", offset);
        printf("code_ptr[0] is 0x%x\n", code_ptr[0]);
        printf("code_ptr[1] is 0x%x\n", code_ptr[1]);
        #endif
        code_ptr[0] |= encode_uimm20(hi);
        code_ptr[1] |= encode_imm12(lo);
        #ifdef LOONGARCH_DEBUG
        printf("Patched code_ptr[0] is 0x%x\n", code_ptr[0]);
        printf("Patched code_ptr[1] is 0x%x\n", code_ptr[1]);
        #endif
        return true;
    }

    return false;
}

static bool patch_reloc(tcg_insn_unit *code_ptr, int type,
                        intptr_t value, intptr_t addend)
{

    tcg_debug_assert(addend == 0);
    switch (type) {
    case R_LARCH_SOP_POP_32_S_10_16_S2:
        reloc_pc_10_16_s2(code_ptr, (tcg_insn_unit *)value);
        break;
    case R_LARCH_SOP_POP_32_S_0_10_10_16_S2:
        reloc_pc_0_10_10_16_s2(code_ptr, (tcg_insn_unit *)value);
        break;
    case R_RISCV_CALL:
        return reloc_call(code_ptr, (tcg_insn_unit *)value);
    default:
        #ifdef LOONGARCH_DEBUG
        printf("[Fix me!!!]: This is called by tcg.c, type is %d\n", type);
        #endif
        tcg_abort();
    }
    return true;
}

/*
 * TCG intrinsics
 */
static bool tcg_out_mov(TCGContext *s, TCGType type, TCGReg ret, TCGReg arg)
{
    /* Simple reg-reg move, optimising out the 'do nothing' case */
    if (ret != arg) {
        tcg_out_opc_reg(s, OPC_OR, ret, arg, TCG_REG_ZERO);
    }
    return true;
}

static int32_t encode_uimm5(uint32_t imm)
{
    return (imm & 0x1f) << 10;
}

static int32_t encode_shift_5bit(LoongarchInsn opc, TCGReg rd, TCGReg rs1,uint32_t imm)
{
    return (opc << 15) | (rs1 & 0x1f) << 5 | (rd & 0x1f) | encode_uimm5(imm);
}

static int32_t encode_uimm6(uint32_t imm)
{
    return (imm & 0x3f) << 10;
}

static int32_t encode_shift_6bit(LoongarchInsn opc, TCGReg rd, TCGReg rs1,uint32_t imm)
{
    return (opc << 16) | (rs1 & 0x1f) << 5 | (rd & 0x1f) | encode_uimm6(imm);
}

/*
 * 32bit insn, might be used later
 */
static void tcg_out_opc_sxli(TCGContext *s, LoongarchInsn opc,
                            TCGReg rd, TCGReg rs1, TCGArg imm)
{
    #ifdef LOONGARCH_DEBUG
    printf ("[%s] insn: 0x%x,opc is 0x%x, rd=0x%x, rs1=0x%x, imm =0x%lx\n",__func__,encode_shift_5bit(opc, rd, rs1, imm),opc,rd,rs1,imm);
    #endif
    tcg_out32(s, encode_shift_5bit(opc, rd, rs1, imm));
}

static void tcg_out_opc_dxli(TCGContext *s, LoongarchInsn opc,
                            TCGReg rd, TCGReg rs1, TCGArg imm)
{
    #ifdef LOONGARCH_DEBUG
    printf ("[%s] insn: 0x%x,opc is 0x%x, rd=0x%x, rs1=0x%x, imm =0x%lx\n",__func__,encode_shift_6bit(opc, rd, rs1, imm),opc,rd,rs1,imm);
    #endif
    tcg_out32(s, encode_shift_6bit(opc, rd, rs1, imm));
}

/*
 * Shift Insns.
 */
static void tcg_out_opc_sa(TCGContext *s, LoongarchInsn opc,
                                  TCGReg rd, TCGReg rt, TCGArg sa)
{
    int32_t insn;

    insn = (opc << 15);
    insn |= (rt & 0x1F) << 5;
    insn |= (rd & 0x1F);
    insn |= (sa & 0x1F) << 10;
    tcg_out32(s, insn);
}

static void tcg_out_opc_sa64(TCGContext *s, LoongarchInsn opc,
                             TCGReg rd, TCGReg rt, TCGArg sa)
{
    int32_t insn;

    insn = (opc << 16);
    insn |= (rt & 0x1F) << 5;
    insn |= (rd & 0x1F);
    insn |= (sa & 0x3F) << 10;
    tcg_out32(s, insn);
}

static void tcg_out_dsll(TCGContext *s, TCGReg rd, TCGReg rt, TCGArg sa)
{
    tcg_out_opc_sa64(s, OPC_DSLL, rd, rt, sa);
}

/* Riscv is differnet with LoongArch */
/* Need to refer to mips implement */
static void tcg_out_movi(TCGContext *s, TCGType type, TCGReg rd,
                         tcg_target_long val)
{
    tcg_target_long lo, tmp;
    int shift, ret;

    #ifdef LOONGARCH_DEBUG
    printf("%s at Line %d. val is 0x%lx\n",__func__,__LINE__,val);
    #endif

    if (TCG_TARGET_REG_BITS == 64 && type == TCG_TYPE_I32) {
        val = (int32_t)val;
    }
    lo = sextreg(val, 0, 12);
    if (val == lo) {
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d.\n",__func__,__LINE__);
        #endif
        tcg_out_opc_imm(s, ALIAS_PADDI, rd, TCG_REG_ZERO, lo);
        return;
    }

    if (TCG_TARGET_REG_BITS == 32 || val == (int32_t)val) {
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d.\n",__func__,__LINE__);
        #endif
        /*LUI is sext that is lead to [32:63] is 0xFF*/
        tcg_out_opc_upper(s, OPC_LUI, rd, val);
        if (lo != 0) {
            #ifdef LOONGARCH_DEBUG
            printf("[%s] at Line %d.\n",__func__,__LINE__);
            #endif
            /*
             * 32bit opc is enough,coz val/reg are all 32 bits
             * Fixme:Sometimes movi_i64 invoke in here.
             */
            //tcg_out_opc_imm(s, ALIAS_PADDI, rd, rd, lo);
            tcg_out_opc_imm(s, OPC_ORI, rd, rd, val);
        }
        return;
    }

    /* We can only be here if TCG_TARGET_REG_BITS != 32 */
    tmp = tcg_pcrel_diff(s, (void *)val);
    if (tmp == (int32_t)tmp) {
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d. tmp=%ld\n",__func__,__LINE__,tmp);
        #endif
        tcg_debug_assert(type == TCG_TYPE_PTR);
        tcg_out_opc_upper(s, OPC_AUIPC, rd, 0);
        tcg_out_opc_imm(s, ALIAS_PADDI, rd, rd, 0);
        ret = reloc_call(s->code_ptr - 2, (tcg_insn_unit *)val);
        tcg_debug_assert(ret == true);
        return;
    }

    /* Look for a single 20-bit section.  */
    shift = ctz64(val);
    tmp = val >> shift;
    if (tmp == sextreg(tmp, 0, 20)) {
        #ifdef LOONGARCH_DEBUG
        printf("%s at Line %d. shift is %d, tmp is 0x%lx, val is 0x%lx\n",__func__,__LINE__,shift, tmp, val);
        #endif
        /*Low 12bit is zero*/
        tcg_out_opc_upper(s, OPC_LUI, rd, tmp << 12);
        if (shift > 12) {
            /*
             * LoongArch: create a new API to implement SLLI etc...
             * Fixme: How to select the REG length here?
             */
            if ((TCG_TARGET_REG_BITS == 64)
                &&(type == TCG_TYPE_PTR)) {
                tcg_out_opc_dxli(s, OPC_DSLL, rd, rd, shift - 12);
            } else {
                tcg_debug_assert(0);
            }
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("TODO ==> [%s] at Line %d.\n",__func__,__LINE__);
            #endif
            tcg_debug_assert(0);
            tcg_out_opc_imm(s, OPC_SRAI, rd, rd, 12 - shift);
        }
        return;
    }

    /* Look for a few high zero bits, with lots of bits set in the middle.  */
    shift = clz64(val);
    tmp = val << shift;
    if (tmp == sextreg(tmp, 12, 20) << 12) {
        #ifdef LOONGARCH_DEBUG
        printf("[Fixme]: %s at Line %d. shift is %d, tmp is 0x%lx, val is 0x%lx\n",__func__,__LINE__,shift, tmp, val);
        #endif
        tcg_out_opc_upper(s, OPC_LUI, rd, tmp);
        tcg_out_opc_dxli(s, OPC_SRLI, rd, rd, shift);
        return;
    } else if (tmp == sextreg(tmp, 0, 12)) {
        #ifdef LOONGARCH_DEBUG
        printf("[Fixme]: %s at Line %d. shift is %d, tmp is 0x%lx, val is 0x%lx\n",__func__,__LINE__,shift, tmp, val);
        #endif
        tcg_out_opc_imm(s, OPC_ADDI, rd, TCG_REG_ZERO, tmp);
        tcg_out_opc_dxli(s, OPC_SRLI, rd, rd, shift);
        return;
    }
    /* Drop into the constant pool.  */
    /* Why put a jump label here?*/
    new_pool_label(s, val, R_RISCV_CALL, s->code_ptr, 0);
    tcg_out_opc_upper(s, OPC_AUIPC, rd, 0);
    tcg_out_opc_imm(s, OPC_LD, rd, rd, 0);
}

static void tcg_out_ext8u(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_opc_imm(s, OPC_ANDI, ret, arg, 0xff);
}

static void tcg_out_ext16u(TCGContext *s, TCGReg ret, TCGReg arg)
{
    /*
     * LA ANDI only has 12bit, has to implement as Risc-V did
     */
    tcg_out_opc_sxli(s, OPC_SLLIW, ret, arg, 16);
    tcg_out_opc_sxli(s, OPC_SRLIW, ret, ret, 16);
}

static void tcg_out_ext32u(TCGContext *s, TCGReg ret, TCGReg arg)
{
    int32_t insn = 0;

    insn = (OPC_DEXT << 22);
    insn |= (arg & 0x1F) << 5;
    insn |= (ret & 0x1F);
    insn |= (31 & 0x3F) << 16;
    insn |= (0 & 0x3F) << 10;
    tcg_out32(s, insn);
}
/*
 * LoongArch has SEB insn.
 */
static void tcg_out_ext8s(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_opc_2reg(s, OPC_SEB, ret, arg);
}
/*
 * LoongArch has SEH insn.
 */
static void tcg_out_ext16s(TCGContext *s, TCGReg ret, TCGReg arg)
{
    tcg_out_opc_2reg(s, OPC_SEH, ret, arg);
}

static void tcg_out_ext32s(TCGContext *s, TCGReg ret, TCGReg arg)
{
    /*
     * Todo: adapt to LA
     * Logic left shift 0
     */
    tcg_out_opc_sa(s, OPC_SLLIW, ret, arg, 0);
}

static void tcg_out_ldst(TCGContext *s, LoongarchInsn opc, TCGReg data,
                         TCGReg addr, intptr_t offset)
{
    intptr_t imm12 = sextreg(offset, 0, 12);

    /*
     * Different with Risc-V, need to identify ptr length
     * Select different OPC between 32bit and 64 bit.
     */
    if (offset != imm12) {
        intptr_t diff = offset - (uintptr_t)s->code_ptr;

        if (addr == TCG_REG_ZERO && diff == (int32_t)diff) {
            imm12 = sextreg(diff, 0, 12);
            #ifdef LOONGARCH_DEBUG
            printf("tcg_out_ldst at Line=%d,imm12=%ld, offset=%ld, diff - imm12=%ld\n",__LINE__,imm12,offset,diff - imm12);
            #endif
            tcg_out_opc_upper(s, OPC_AUIPC, TCG_REG_TMP2, diff - imm12);
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("tcg_out_ldst at Line %d\n",__LINE__);
            #endif
            tcg_out_movi(s, TCG_TYPE_PTR, TCG_REG_TMP2, offset - imm12);
            if (addr != TCG_REG_ZERO) {
            #ifdef LOONGARCH_DEBUG
            printf("tcg_out_ldst at Line %d\n",__LINE__);
            #endif
                tcg_out_opc_reg(s, OPC_ADD, TCG_REG_TMP2, TCG_REG_TMP2, addr);
            }
        }
        addr = TCG_REG_TMP2;
    }
    switch (opc) {
    case OPC_SB:
    case OPC_SH:
    case OPC_SW:
    case OPC_SD:
        tcg_out_opc_store(s, opc, addr, data, imm12);
        break;
    case OPC_LB:
    case OPC_LBU:
    case OPC_LH:
    case OPC_LHU:
    case OPC_LW:
    case OPC_LWU:
    case OPC_LD:
        #ifdef LOONGARCH_DEBUG
        printf("[tcg_out_ldst-LOAD] opc=0x%x, data=0x%x, addr=0x%x, imm12=0x%lx\n", opc,data,addr,imm12);
        #endif
        tcg_out_opc_imm(s, opc, data, addr, imm12);
        break;
    default:
        g_assert_not_reached();
    }
}

static void tcg_out_ld(TCGContext *s, TCGType type, TCGReg arg,
                       TCGReg arg1, intptr_t arg2)
{
    bool is32bit = (TCG_TARGET_REG_BITS == 32 || type == TCG_TYPE_I32);
    tcg_out_ldst(s, is32bit ? OPC_LW : OPC_LD, arg, arg1, arg2);
}

static void tcg_out_st(TCGContext *s, TCGType type, TCGReg arg,
                       TCGReg arg1, intptr_t arg2)
{
    bool is32bit = (TCG_TARGET_REG_BITS == 32 || type == TCG_TYPE_I32);
    tcg_out_ldst(s, is32bit ? OPC_SW : OPC_SD, arg, arg1, arg2);
}

static bool tcg_out_sti(TCGContext *s, TCGType type, TCGArg val,
                        TCGReg base, intptr_t ofs)
{
    if (val == 0) {
        tcg_out_st(s, type, TCG_REG_ZERO, base, ofs);
        return true;
    }
    return false;
}

static void tcg_out_addsub2(TCGContext *s,
                            TCGReg rl, TCGReg rh,
                            TCGReg al, TCGReg ah,
                            TCGArg bl, TCGArg bh,
                            bool cbl, bool cbh, bool is_sub, bool is32bit)
{
    const LoongarchInsn opc_add = is32bit ? OPC_ADDW : OPC_ADD;
    const LoongarchInsn opc_addi = is32bit ? OPC_ADDIW : OPC_ADDI;
    const LoongarchInsn opc_sub = is32bit ? OPC_SUBW : OPC_SUB;
    TCGReg th = TCG_REG_TMP1;

    /* If we have a negative constant such that negating it would
       make the high part zero, we can (usually) eliminate one insn.  */
    if (cbl && cbh && bh == -1 && bl != 0) {
        bl = -bl;
        bh = 0;
        is_sub = !is_sub;
    }

    /* By operating on the high part first, we get to use the final
       carry operation to move back from the temporary.  */
    if (!cbh) {
        tcg_out_opc_reg(s, (is_sub ? opc_sub : opc_add), th, ah, bh);
    } else if (bh != 0 || ah == rl) {
        tcg_out_opc_imm(s, opc_addi, th, ah, (is_sub ? -bh : bh));
    } else {
        th = ah;
    }

    /* Note that tcg optimization should eliminate the bl == 0 case.  */
    if (is_sub) {
        if (cbl) {
            tcg_out_opc_imm(s, OPC_SLTIU, TCG_REG_TMP0, al, bl);
            tcg_out_opc_imm(s, opc_addi, rl, al, -bl);
        } else {
            tcg_out_opc_reg(s, OPC_SLTU, TCG_REG_TMP0, al, bl);
            tcg_out_opc_reg(s, opc_sub, rl, al, bl);
        }
        tcg_out_opc_reg(s, opc_sub, rh, th, TCG_REG_TMP0);
    } else {
        if (cbl) {
            tcg_out_opc_imm(s, opc_addi, rl, al, bl);
            tcg_out_opc_imm(s, OPC_SLTIU, TCG_REG_TMP0, rl, bl);
        } else if (rl == al && rl == bl) {
            tcg_out_opc_imm(s, OPC_SLTI, TCG_REG_TMP0, al, 0);
            tcg_out_opc_reg(s, opc_addi, rl, al, bl);
        } else {
            tcg_out_opc_reg(s, opc_add, rl, al, bl);
            tcg_out_opc_reg(s, OPC_SLTU, TCG_REG_TMP0,
                            rl, (rl == bl ? al : bl));
        }
        tcg_out_opc_reg(s, opc_add, rh, th, TCG_REG_TMP0);
    }
}

static const struct {
    LoongarchInsn op;
    bool swap;
} tcg_brcond_to_riscv[] = {
    [TCG_COND_EQ] =  { OPC_BEQ,  false },
    [TCG_COND_NE] =  { OPC_BNE,  false },
    [TCG_COND_LT] =  { OPC_BLT,  false },
    [TCG_COND_GE] =  { OPC_BGE,  false },
    [TCG_COND_LE] =  { OPC_BGE,  true  },
    [TCG_COND_GT] =  { OPC_BLT,  true  },
    [TCG_COND_LTU] = { OPC_BLTU, false },
    [TCG_COND_GEU] = { OPC_BGEU, false },
    [TCG_COND_LEU] = { OPC_BGEU, true  },
    [TCG_COND_GTU] = { OPC_BLTU, true  }
};

/*
 * Cause LoongArch has 16 bits offsets for B insns.
 * So refer to mips implementatitons.
 */
static void tcg_out_brcond(TCGContext *s, TCGCond cond, TCGReg arg1,
                           TCGReg arg2, TCGLabel *l)
{
    LoongarchInsn op = tcg_brcond_to_riscv[cond].op;

    tcg_debug_assert(op != 0);

    if (tcg_brcond_to_riscv[cond].swap) {
        TCGReg t = arg1;
        arg1 = arg2;
        arg2 = t;
    }

    if (l->has_value) {
        intptr_t diff = tcg_pcrel_diff(s, l->u.value_ptr);

        if (diff == sextreg(diff, 0, 16)) {
            #ifdef LOONGARCH_DEBUG
            printf("tcg_out_brcond at Line %d, \t opc is %d\n",__LINE__,op);
            #endif
            tcg_out_opc_branch(s, op, arg1, arg2, diff);
        } else {
            /* Invert the conditional branch.  */
            tcg_debug_assert(0);
            //tcg_out_opc_branch(s, op ^ (1 << 12), arg1, arg2, 8);
            //tcg_out_opc_jump(s, OPC_JAL, TCG_REG_ZERO, diff - 4);
            #ifdef LOONGARCH_DEBUG
            printf("[Fix me!!!!] tcg_out_brcond at Line %d, \t opc is %d\n",__LINE__,op);
            #endif
        }
    } else {
        #ifdef LOONGARCH_DEBUG
        printf("tcg_out_brcond at Line %d, \t opc is %d\n",__LINE__,op);
        #endif
        tcg_out_reloc(s, s->code_ptr, R_LARCH_SOP_POP_32_S_10_16_S2, l, 0);
        tcg_out_opc_branch(s, op, arg1, arg2, 0);
    }
}

/*
 * Refer to Mips insn to implement below code
 * I don't wanna consider REG width.
 */
/* Bit 0 set if inversion required; bit 1 set if swapping required.  */
#define LOONGARCH_CMP_INV  1
#define LOONGARCH_CMP_SWAP 2

static const uint8_t loongarch_cmp_map[16] = {
    [TCG_COND_LT]  = 0,
    [TCG_COND_LTU] = 0,
    [TCG_COND_GE]  = LOONGARCH_CMP_INV,
    [TCG_COND_GEU] = LOONGARCH_CMP_INV,
    [TCG_COND_LE]  = LOONGARCH_CMP_INV | LOONGARCH_CMP_SWAP,
    [TCG_COND_LEU] = LOONGARCH_CMP_INV | LOONGARCH_CMP_SWAP,
    [TCG_COND_GT]  = LOONGARCH_CMP_SWAP,
    [TCG_COND_GTU] = LOONGARCH_CMP_SWAP,
};

static void tcg_out_setcond(TCGContext *s, TCGCond cond, TCGReg ret,
                            TCGReg arg1, TCGReg arg2)
{
    LoongarchInsn s_opc = OPC_SLTU;
    int cmp_map;

    switch (cond) {
    case TCG_COND_EQ:
        if (arg2 != 0) {
            tcg_out_opc_reg(s, OPC_XOR, ret, arg1, arg2);
            arg1 = ret;
        }
        tcg_out_opc_imm(s, OPC_SLTIU, ret, arg1, 1);
        break;

    case TCG_COND_NE:
        if (arg2 != 0) {
            tcg_out_opc_reg(s, OPC_XOR, ret, arg1, arg2);
            arg1 = ret;
        }
        tcg_out_opc_reg(s, OPC_SLTU, ret, TCG_REG_ZERO, arg1);
        break;

    case TCG_COND_LT:
    case TCG_COND_GE:
    case TCG_COND_LE:
    case TCG_COND_GT:
        s_opc = OPC_SLT;
        /* FALLTHRU */

    case TCG_COND_LTU:
    case TCG_COND_GEU:
    case TCG_COND_LEU:
    case TCG_COND_GTU:
        cmp_map = loongarch_cmp_map[cond];
        if (cmp_map & LOONGARCH_CMP_SWAP) {
            TCGReg t = arg1;
            arg1 = arg2;
            arg2 = t;
        }
        tcg_out_opc_reg(s, s_opc, ret, arg1, arg2);
        if (cmp_map & LOONGARCH_CMP_INV) {
            tcg_out_opc_imm(s, OPC_XORI, ret, ret, 1);
        }
        break;

     default:
         tcg_abort();
         break;
     }
}

/*
 * Port from mips based on r6 implemtation.
 * dest = (c1 cond c2 ? v1 : v2)
 */
static void tcg_out_movcond(TCGContext *s, TCGCond cond, TCGReg ret,
                            TCGReg c1, TCGReg c2, TCGReg v1, TCGReg v2)
{
    bool eqz = false;

    /* If one of the values is zero, put it last to match SEL*Z instructions */
    if (v1 == 0) {
        v1 = v2;
        v2 = 0;
        cond = tcg_invert_cond(cond);
    }

    switch (cond) {
    case TCG_COND_EQ:
        eqz = true;
        /* FALLTHRU */
    case TCG_COND_NE:
        if (c2 != 0) {
            tcg_out_opc_reg(s, OPC_XOR, TCG_REG_TMP0, c1, c2);
            c1 = TCG_REG_TMP0;
        }
        break;

    default:
        /* Minimize code size by preferring a compare not requiring INV.  */
        if (loongarch_cmp_map[cond] & LOONGARCH_CMP_INV) {
            cond = tcg_invert_cond(cond);
            eqz = true;
        }
        tcg_out_setcond(s, cond, TCG_REG_TMP0, c1, c2);
        c1 = TCG_REG_TMP0;
        break;
    }

    LoongarchInsn m_opc_f = eqz ? OPC_MASKEQZ : OPC_MASKNEZ;
    LoongarchInsn m_opc_t = eqz ? OPC_MASKNEZ : OPC_MASKEQZ;

    if (v2 != 0) {
        tcg_out_opc_reg(s, m_opc_f, TCG_REG_TMP1, v2, c1);
    }
    tcg_out_opc_reg(s, m_opc_t, ret, v1, c1);
    if (v2 != 0) {
        tcg_out_opc_reg(s, OPC_OR, ret, ret, TCG_REG_TMP1);
    }
}

static void tcg_out_cltz(TCGContext *s, LoongarchInsn opc,
                        int width, TCGReg a0, TCGReg a1, TCGArg a2)
{
    if (a2 == width) {
        tcg_out_opc_2reg(s, opc, a0, a1);
    } else {
        tcg_out_opc_2reg(s, opc, TCG_REG_TMP0, a1);
        tcg_out_movcond(s, TCG_COND_EQ, a0, a1, 0, a2, TCG_REG_TMP0);
    }
}

static void tcg_out_brcond2(TCGContext *s, TCGCond cond, TCGReg al, TCGReg ah,
                            TCGReg bl, TCGReg bh, TCGLabel *l)
{
    /* todo */
    g_assert_not_reached();
}

static void tcg_out_setcond2(TCGContext *s, TCGCond cond, TCGReg ret,
                             TCGReg al, TCGReg ah, TCGReg bl, TCGReg bh)
{
    /* todo */
    g_assert_not_reached();
}

__attribute__((unused))
static void tcg_out_goto(TCGContext *s, tcg_insn_unit *target)
{
    ptrdiff_t offset = tcg_pcrel_diff(s, target);
    tcg_debug_assert(offset == sextreg(offset, 1, 20) << 1);
    //tcg_out_opc_jump(s, OPC_JAL, TCG_REG_ZERO, offset);
    #ifdef LOONGARCH_DEBUG
    printf("[Fix me!!!!] OPs at Line %d\n",__LINE__);
    #endif
    tcg_debug_assert(0);
}

/*
 * Type jump.
 * Returns true if the branch was in range and the insn was emitted.
 * Only OPC_B & OPC_BL could be used.
 * LoongArch has different B/BL design with mips
 * Refer to Risc-V implementation.
 */
static void tcg_out_opc_jmp(TCGContext *s, LoongarchInsn opc, uint32_t imm)
{
    #ifdef LOONGARCH_DEBUG
    printf ("[%s] insn: 0x%x,opc=0x%x, imm=0x%x\n",__func__,encode_uj(opc, imm),opc,imm);
    #endif
    tcg_out32(s, encode_uj(opc, imm));
}

/*
 * LoongArch inherit Mips branch insn, keep leverage Mips insn.
 */
static void tcg_out_call_int(TCGContext *s, tcg_insn_unit *arg, bool tail)
{
    /*
     * Cause B/BL has different with J JAL.
     * Refer to Riscv offset calculation.
     */
    LoongarchInsn opc  = tail ? OPC_B : OPC_BL;
    ptrdiff_t offset = tcg_pcrel_diff(s, arg);
    /*
     * B BL 
     */
    if (offset == sextreg(offset, 2, 26) << 2) {
        #ifdef LOONGARCH_DEBUG
        printf("[%s]offset is %ld\n",__func__,offset);
        #endif
        /* 28 bit jump */
        tcg_out_opc_jmp(s, opc, offset);
    } else if (TCG_TARGET_REG_BITS == 32 ||
        offset == sextreg(offset, 2, 31) << 2) {
        tcg_debug_assert(0);
    } else if (TCG_TARGET_REG_BITS == 64) {
        tcg_debug_assert(0);
    } else {
        g_assert_not_reached();
    }
}

static void tcg_out_call(TCGContext *s, tcg_insn_unit *arg)
{
    tcg_out_call_int(s, arg, false);
}

static void tcg_out_mb(TCGContext *s, TCGArg a0)
{
    int32_t insn = 0;
    insn = 0x38720000;
    tcg_out32(s, insn);
}

/*
 * Load/store and TLB
 */

#if defined(CONFIG_SOFTMMU)
#include "tcg-ldst.inc.c"

/* helper signature: helper_ret_ld_mmu(CPUState *env, target_ulong addr,
 *                                     TCGMemOpIdx oi, uintptr_t ra)
 */
static void * const qemu_ld_helpers[16] = {
    [MO_UB]   = helper_ret_ldub_mmu,
    [MO_SB]   = helper_ret_ldsb_mmu,
    [MO_LEUW] = helper_le_lduw_mmu,
    [MO_LESW] = helper_le_ldsw_mmu,
    [MO_LEUL] = helper_le_ldul_mmu,
#if TCG_TARGET_REG_BITS == 64
    [MO_LESL] = helper_le_ldsl_mmu,
#endif
    [MO_LEQ]  = helper_le_ldq_mmu,
    [MO_BEUW] = helper_be_lduw_mmu,
    [MO_BESW] = helper_be_ldsw_mmu,
    [MO_BEUL] = helper_be_ldul_mmu,
#if TCG_TARGET_REG_BITS == 64
    [MO_BESL] = helper_be_ldsl_mmu,
#endif
    [MO_BEQ]  = helper_be_ldq_mmu,
};

/* helper signature: helper_ret_st_mmu(CPUState *env, target_ulong addr,
 *                                     uintxx_t val, TCGMemOpIdx oi,
 *                                     uintptr_t ra)
 */
static void * const qemu_st_helpers[16] = {
    [MO_UB]   = helper_ret_stb_mmu,
    [MO_LEUW] = helper_le_stw_mmu,
    [MO_LEUL] = helper_le_stl_mmu,
    [MO_LEQ]  = helper_le_stq_mmu,
    [MO_BEUW] = helper_be_stw_mmu,
    [MO_BEUL] = helper_be_stl_mmu,
    [MO_BEQ]  = helper_be_stq_mmu,
};

/* We don't support oversize guests */
QEMU_BUILD_BUG_ON(TCG_TARGET_REG_BITS < TARGET_LONG_BITS);

/* We expect to use a 12-bit negative offset from ENV.  */
QEMU_BUILD_BUG_ON(TLB_MASK_TABLE_OFS(0) > 0);
QEMU_BUILD_BUG_ON(TLB_MASK_TABLE_OFS(0) < -(1 << 11));

static void tcg_out_tlb_load(TCGContext *s, TCGReg addrl,
                             TCGReg addrh, TCGMemOpIdx oi,
                             tcg_insn_unit **label_ptr, bool is_load)
{
    MemOp opc = get_memop(oi);
    unsigned s_bits = opc & MO_SIZE;
    unsigned a_bits = get_alignment_bits(opc);
    tcg_target_long compare_mask;
    int mem_index = get_mmuidx(oi);
    int fast_ofs = TLB_MASK_TABLE_OFS(mem_index);
    int mask_ofs = fast_ofs + offsetof(CPUTLBDescFast, mask);
    int table_ofs = fast_ofs + offsetof(CPUTLBDescFast, table);
    TCGReg mask_base = TCG_AREG0, table_base = TCG_AREG0;

    tcg_out_ld(s, TCG_TYPE_PTR, TCG_REG_TMP0, mask_base, mask_ofs);
    tcg_out_ld(s, TCG_TYPE_PTR, TCG_REG_TMP1, table_base, table_ofs);

    tcg_out_opc_imm(s, OPC_SRLI, TCG_REG_TMP2, addrl,
                    TARGET_PAGE_BITS - CPU_TLB_ENTRY_BITS);
    tcg_out_opc_reg(s, OPC_AND, TCG_REG_TMP2, TCG_REG_TMP2, TCG_REG_TMP0);
    tcg_out_opc_reg(s, OPC_ADD, TCG_REG_TMP2, TCG_REG_TMP2, TCG_REG_TMP1);

    /* Load the tlb comparator and the addend.  */
    tcg_out_ld(s, TCG_TYPE_TL, TCG_REG_TMP0, TCG_REG_TMP2,
               is_load ? offsetof(CPUTLBEntry, addr_read)
               : offsetof(CPUTLBEntry, addr_write));
    tcg_out_ld(s, TCG_TYPE_PTR, TCG_REG_TMP2, TCG_REG_TMP2,
               offsetof(CPUTLBEntry, addend));

    /* We don't support unaligned accesses. */
    if (a_bits < s_bits) {
        a_bits = s_bits;
    }
    /* Clear the non-page, non-alignment bits from the address.  */
    compare_mask = (tcg_target_long)TARGET_PAGE_MASK | ((1 << a_bits) - 1);
    if (compare_mask == sextreg(compare_mask, 0, 12)) {
        tcg_out_opc_imm(s, OPC_ANDI, TCG_REG_TMP1, addrl, compare_mask);
    } else {
        tcg_out_movi(s, TCG_TYPE_TL, TCG_REG_TMP1, compare_mask);
        tcg_out_opc_reg(s, OPC_AND, TCG_REG_TMP1, TCG_REG_TMP1, addrl);
    }

    /* Compare masked address with the TLB entry. */
    label_ptr[0] = s->code_ptr;
    tcg_out_opc_branch(s, OPC_BNE, TCG_REG_TMP0, TCG_REG_TMP1, 0);
    /* NOP to allow patching later */
    tcg_out_opc_imm(s, OPC_ANDI, TCG_REG_ZERO, TCG_REG_ZERO, 0);

    /* TLB Hit - translate address using addend.  */
    if (TCG_TARGET_REG_BITS > TARGET_LONG_BITS) {
        tcg_out_ext32u(s, TCG_REG_TMP0, addrl);
        addrl = TCG_REG_TMP0;
    }
    tcg_out_opc_reg(s, OPC_ADD, TCG_REG_TMP0, TCG_REG_TMP2, addrl);
}

static void add_qemu_ldst_label(TCGContext *s, int is_ld, TCGMemOpIdx oi,
                                TCGType ext,
                                TCGReg datalo, TCGReg datahi,
                                TCGReg addrlo, TCGReg addrhi,
                                void *raddr, tcg_insn_unit **label_ptr)
{
    #ifdef LOONGARCH_DEBUG
    printf("%s at Line %d\n",__func__,__LINE__);
    #endif
    TCGLabelQemuLdst *label = new_ldst_label(s);

    label->is_ld = is_ld;
    label->oi = oi;
    label->type = ext;
    label->datalo_reg = datalo;
    label->datahi_reg = datahi;
    label->addrlo_reg = addrlo;
    label->addrhi_reg = addrhi;
    label->raddr = raddr;
    label->label_ptr[0] = label_ptr[0];
}

static bool tcg_out_qemu_ld_slow_path(TCGContext *s, TCGLabelQemuLdst *l)
{
    #ifdef LOONGARCH_DEBUG
    printf("%s at Line %d\n",__func__,__LINE__);
    #endif
    TCGMemOpIdx oi = l->oi;
    MemOp opc = get_memop(oi);
    TCGReg a0 = tcg_target_call_iarg_regs[0];
    TCGReg a1 = tcg_target_call_iarg_regs[1];
    TCGReg a2 = tcg_target_call_iarg_regs[2];
    TCGReg a3 = tcg_target_call_iarg_regs[3];

    /* We don't support oversize guests */
    if (TCG_TARGET_REG_BITS < TARGET_LONG_BITS) {
        g_assert_not_reached();
    }

    /* resolve label address */
    if (!patch_reloc(l->label_ptr[0], R_RISCV_BRANCH,
                     (intptr_t) s->code_ptr, 0)) {
        return false;
    }

    /* call load helper */
    tcg_out_mov(s, TCG_TYPE_PTR, a0, TCG_AREG0);
    tcg_out_mov(s, TCG_TYPE_PTR, a1, l->addrlo_reg);
    tcg_out_movi(s, TCG_TYPE_PTR, a2, oi);
    tcg_out_movi(s, TCG_TYPE_PTR, a3, (tcg_target_long)l->raddr);

    tcg_out_call(s, qemu_ld_helpers[opc & (MO_BSWAP | MO_SSIZE)]);
    tcg_out_mov(s, (opc & MO_SIZE) == MO_64, l->datalo_reg, a0);

    tcg_out_goto(s, l->raddr);
    return true;
}

static bool tcg_out_qemu_st_slow_path(TCGContext *s, TCGLabelQemuLdst *l)
{
    #ifdef LOONGARCH_DEBUG
    printf("%s at Line %d\n",__func__,__LINE__);
    #endif
    TCGMemOpIdx oi = l->oi;
    MemOp opc = get_memop(oi);
    MemOp s_bits = opc & MO_SIZE;
    TCGReg a0 = tcg_target_call_iarg_regs[0];
    TCGReg a1 = tcg_target_call_iarg_regs[1];
    TCGReg a2 = tcg_target_call_iarg_regs[2];
    TCGReg a3 = tcg_target_call_iarg_regs[3];
    TCGReg a4 = tcg_target_call_iarg_regs[4];

    /* We don't support oversize guests */
    if (TCG_TARGET_REG_BITS < TARGET_LONG_BITS) {
        g_assert_not_reached();
    }

    /* resolve label address */
    if (!patch_reloc(l->label_ptr[0], R_RISCV_BRANCH,
                     (intptr_t) s->code_ptr, 0)) {
        return false;
    }

    /* call store helper */
    tcg_out_mov(s, TCG_TYPE_PTR, a0, TCG_AREG0);
    tcg_out_mov(s, TCG_TYPE_PTR, a1, l->addrlo_reg);
    tcg_out_mov(s, TCG_TYPE_PTR, a2, l->datalo_reg);
    switch (s_bits) {
    case MO_8:
        tcg_out_ext8u(s, a2, a2);
        break;
    case MO_16:
        tcg_out_ext16u(s, a2, a2);
        break;
    default:
        break;
    }
    tcg_out_movi(s, TCG_TYPE_PTR, a3, oi);
    tcg_out_movi(s, TCG_TYPE_PTR, a4, (tcg_target_long)l->raddr);

    tcg_out_call(s, qemu_st_helpers[opc & (MO_BSWAP | MO_SSIZE)]);

    tcg_out_goto(s, l->raddr);
    return true;
}
#endif /* CONFIG_SOFTMMU */

static void tcg_out_qemu_ld_direct(TCGContext *s, TCGReg lo, TCGReg hi,
                                   TCGReg base, MemOp opc, bool is_64)
{
    const MemOp bswap = opc & MO_BSWAP;

    /* We don't yet handle byteswapping, assert */
    g_assert(!bswap);

    switch (opc & (MO_SSIZE)) {
    case MO_UB:
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d\n",__func__,__LINE__);
        #endif
        tcg_out_opc_imm(s, OPC_LBU, lo, base, 0);
        break;
    case MO_SB:
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d\n",__func__,__LINE__);
        #endif
        tcg_out_opc_imm(s, OPC_LB, lo, base, 0);
        break;
    case MO_UW:
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d\n",__func__,__LINE__);
        #endif
        tcg_out_opc_imm(s, OPC_LHU, lo, base, 0);
        break;
    case MO_SW:
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d\n",__func__,__LINE__);
        #endif
        tcg_out_opc_imm(s, OPC_LH, lo, base, 0);
        break;
    case MO_UL:
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d\n",__func__,__LINE__);
        #endif
        if (TCG_TARGET_REG_BITS == 64 && is_64) {
            tcg_out_opc_imm(s, OPC_LWU, lo, base, 0);
            break;
        }
        /* FALLTHRU */
    case MO_SL:
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d\n",__func__,__LINE__);
        #endif
        tcg_out_opc_imm(s, OPC_LW, lo, base, 0);
        break;
    case MO_Q:
        /* Prefer to load from offset 0 first, but allow for overlap.  */
        if (TCG_TARGET_REG_BITS == 64) {
            #ifdef LOONGARCH_DEBUG
            printf("[%s] at Line %d\n",__func__,__LINE__);
            #endif
            tcg_out_opc_imm(s, OPC_LD, lo, base, 0);
        } else if (lo != base) {
            #ifdef LOONGARCH_DEBUG
            printf("[%s] at Line %d\n",__func__,__LINE__);
            #endif
            tcg_debug_assert(0);
            tcg_out_opc_imm(s, OPC_LW, lo, base, 0);
            tcg_out_opc_imm(s, OPC_LW, hi, base, 4);
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("[%s] at Line %d\n",__func__,__LINE__);
            #endif
            tcg_debug_assert(0);
            tcg_out_opc_imm(s, OPC_LW, hi, base, 4);
            tcg_out_opc_imm(s, OPC_LW, lo, base, 0);
        }
        break;
    default:
        g_assert_not_reached();
    }
}

static void tcg_out_qemu_ld(TCGContext *s, const TCGArg *args, bool is_64)
{
    TCGReg addr_regl, addr_regh __attribute__((unused));
    TCGReg data_regl, data_regh;
    TCGMemOpIdx oi;
    MemOp opc;
#if defined(CONFIG_SOFTMMU)
    tcg_insn_unit *label_ptr[1];
#endif
    TCGReg base = TCG_REG_A0;

    data_regl = *args++;
    data_regh = (TCG_TARGET_REG_BITS == 32 && is_64 ? *args++ : 0);
    addr_regl = *args++;
    addr_regh = (TCG_TARGET_REG_BITS < TARGET_LONG_BITS ? *args++ : 0);
    oi = *args++;
    opc = get_memop(oi);

#if defined(CONFIG_SOFTMMU)
    tcg_out_tlb_load(s, addr_regl, addr_regh, oi, label_ptr, 1);
    tcg_out_qemu_ld_direct(s, data_regl, data_regh, base, opc, is_64);
    add_qemu_ldst_label(s, 1, oi,
                        (is_64 ? TCG_TYPE_I64 : TCG_TYPE_I32),
                        data_regl, data_regh, addr_regl, addr_regh,
                        s->code_ptr, label_ptr);
#else
    /*
     * 32bit X86 binary
     * (gdb) p data_regl
     * $9 = TCG_REG_S3
     * (gdb) p data_regh
     * $10 = TCG_REG_ZERO
     * (gdb) p addr_regh
     * $11 = TCG_REG_ZERO
     * (gdb) p addr_regl
     * $12 = TCG_REG_S2
     */
    if (TCG_TARGET_REG_BITS > TARGET_LONG_BITS) {
        tcg_out_ext32u(s, base, addr_regl);
        addr_regl = base;
    }

    if (guest_base == 0) {
        tcg_out_opc_reg(s, ALIAS_PADD, base, addr_regl, TCG_REG_ZERO);
    } else if (guest_base == (int16_t)guest_base) {
        tcg_debug_assert(0);
    } else {
        tcg_out_opc_reg(s, ALIAS_PADD, base, TCG_GUEST_BASE_REG, addr_regl);
    }
    tcg_out_qemu_ld_direct(s, data_regl, data_regh, base, opc, is_64);
#endif
}

static void tcg_out_qemu_st_direct(TCGContext *s, TCGReg lo, TCGReg hi,
                                   TCGReg base, MemOp opc)
{
    const MemOp bswap = opc & MO_BSWAP;

    /* We don't yet handle byteswapping, assert */
    g_assert(!bswap);

    switch (opc & (MO_SSIZE)) {
    case MO_8:
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d\n",__func__,__LINE__);
        #endif
        tcg_out_opc_store(s, OPC_SB, base, lo, 0);
        break;
    case MO_16:
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d\n",__func__,__LINE__);
        #endif
        tcg_out_opc_store(s, OPC_SH, base, lo, 0);
        break;
    case MO_32:
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d\n",__func__,__LINE__);
        #endif
        tcg_out_opc_store(s, OPC_SW, base, lo, 0);
        break;
    case MO_64:
        #ifdef LOONGARCH_DEBUG
        printf("[%s] at Line %d\n",__func__,__LINE__);
        #endif
        if (TCG_TARGET_REG_BITS == 64) {
            tcg_out_opc_store(s, OPC_SD, base, lo, 0);
        } else {
            tcg_debug_assert(0);
            tcg_out_opc_store(s, OPC_SW, base, lo, 0);
            tcg_out_opc_store(s, OPC_SW, base, hi, 4);
        }
        break;
    default:
        g_assert_not_reached();
    }
}

static void tcg_out_qemu_st(TCGContext *s, const TCGArg *args, bool is_64)
{
    TCGReg addr_regl, addr_regh __attribute__((unused));
    TCGReg data_regl, data_regh;
    TCGMemOpIdx oi;
    MemOp opc;
#if defined(CONFIG_SOFTMMU)
    tcg_insn_unit *label_ptr[1];
#endif
    TCGReg base = TCG_REG_A0;

    data_regl = *args++;
    data_regh = (TCG_TARGET_REG_BITS == 32 && is_64 ? *args++ : 0);
    addr_regl = *args++;
    addr_regh = (TCG_TARGET_REG_BITS < TARGET_LONG_BITS ? *args++ : 0);
    oi = *args++;
    opc = get_memop(oi);

#if defined(CONFIG_SOFTMMU)
    tcg_out_tlb_load(s, addr_regl, addr_regh, oi, label_ptr, 0);
    tcg_out_qemu_st_direct(s, data_regl, data_regh, base, opc);
    add_qemu_ldst_label(s, 0, oi,
                        (is_64 ? TCG_TYPE_I64 : TCG_TYPE_I32),
                        data_regl, data_regh, addr_regl, addr_regh,
                        s->code_ptr, label_ptr);
#else
    if (TCG_TARGET_REG_BITS > TARGET_LONG_BITS) {
        tcg_out_ext32u(s, base, addr_regl);
        addr_regl = base;
    }

    if (guest_base == 0) {
        tcg_out_opc_reg(s, ALIAS_PADD, base, addr_regl, TCG_REG_ZERO);
    } else if (guest_base == (int16_t)guest_base) {
        tcg_debug_assert(0);
    } else {
        tcg_out_opc_reg(s, ALIAS_PADD, base, TCG_GUEST_BASE_REG, addr_regl);
    }
    tcg_out_qemu_st_direct(s, data_regl, data_regh, base, opc);
#endif
}

static tcg_insn_unit *tb_ret_addr;

static void tcg_out_op(TCGContext *s, TCGOpcode opc,
                       const TCGArg *args, const int *const_args)
{
    TCGArg a0 = args[0];
    TCGArg a1 = args[1];
    TCGArg a2 = args[2];
    int c2 = const_args[2];

    switch (opc) {
    case INDEX_op_exit_tb:
        /* Reuse the zeroing that exists for goto_ptr.  */
        if (a0 == 0) {
            tcg_out_call_int(s, s->code_gen_epilogue, true);
        } else {
            tcg_out_movi(s, TCG_TYPE_PTR, TCG_REG_A0, a0);
            tcg_out_call_int(s, tb_ret_addr, true);
        }
        break;
    case INDEX_op_goto_tb:
        if (s->tb_jmp_insn_offset) {
            /* direct jump method */
            s->tb_jmp_insn_offset[a0] = tcg_current_code_size(s);
            tcg_out_opc_jmp(s, OPC_B, (*(uint32_t *)s->code_ptr & 0xFFFFFFF));
        } else {
            assert(s->tb_jmp_insn_offset == 0);
            /* indirect jump method */
            tcg_out_ld(s, TCG_TYPE_PTR, TCG_REG_TMP0, TCG_REG_ZERO,
                       (uintptr_t)(s->tb_jmp_target_addr + a0));
            tcg_out_opc_jirl(s, TCG_REG_ZERO, TCG_REG_TMP0);
        }
        set_jmp_reset_offset(s, a0);
        break;
    case INDEX_op_goto_ptr:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_jirl(s, TCG_REG_ZERO, a0);
        break;

    case INDEX_op_br:
        #ifdef LOONGARCH_DEBUG
        printf("[Fix me!!!!] OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_brcond(s, TCG_COND_EQ, TCG_REG_ZERO, TCG_REG_ZERO,
                       arg_label(a0));
        break;
    case INDEX_op_ld8u_i32:
    case INDEX_op_ld8u_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_ldst(s, OPC_LBU, a0, a1, a2);
        break;
    case INDEX_op_ld8s_i32:
    case INDEX_op_ld8s_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_ldst(s, OPC_LB, a0, a1, a2);
        break;
    case INDEX_op_ld16u_i32:
    case INDEX_op_ld16u_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_ldst(s, OPC_LHU, a0, a1, a2);
        break;
    case INDEX_op_ld16s_i32:
    case INDEX_op_ld16s_i64:
        tcg_debug_assert(0);
        tcg_out_ldst(s, OPC_LH, a0, a1, a2);
        break;
    case INDEX_op_ld32u_i64:
        tcg_out_ldst(s, OPC_LWU, a0, a1, a2);
        break;
    case INDEX_op_ld_i32:
    case INDEX_op_ld32s_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_ldst(s, OPC_LW, a0, a1, a2);
        break;
    case INDEX_op_ld_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_ldst(s, OPC_LD, a0, a1, a2);
        break;

    case INDEX_op_st8_i32:
    case INDEX_op_st8_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_ldst(s, OPC_SB, a0, a1, a2);
        break;
    case INDEX_op_st16_i32:
    case INDEX_op_st16_i64:
        tcg_out_ldst(s, OPC_SH, a0, a1, a2);
        tcg_debug_assert(0);
        break;
    case INDEX_op_st_i32:
    case INDEX_op_st32_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_ldst(s, OPC_SW, a0, a1, a2);
        break;
    case INDEX_op_st_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_ldst(s, OPC_SD, a0, a1, a2);
        break;
    case INDEX_op_add_i32:
        if (c2) {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_imm(s, OPC_ADDIW, a0, a1, a2);
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_reg(s, OPC_ADDW, a0, a1, a2);
        }
        break;
    case INDEX_op_add_i64:
        if (c2) {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_imm(s, OPC_ADDI, a0, a1, a2);
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_reg(s, OPC_ADD, a0, a1, a2);
        }
        break;
    case INDEX_op_sub_i32:
        if (c2) {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_imm(s, OPC_ADDIW, a0, a1, -a2);
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_reg(s, OPC_SUBW, a0, a1, a2);
        }
        break;
    case INDEX_op_sub_i64:
        if (c2) {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_imm(s, OPC_ADDI, a0, a1, -a2);
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_reg(s, OPC_SUB, a0, a1, a2);
        }
        break;
    case INDEX_op_and_i32:
        /*
         * Todo: using EXT/DEXT to handle ops AND
         * Coz LA ANDI support ui12 only as well as risc-v is si12
         * insn is incorrect if a2 is neg.
         * Need to implement like Mips
         * constraints: 2^n - 1.
         */
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t c2 is %d, a2 is 0x%x\n",__LINE__,c2,a2);
        #endif
        if (c2 && a2 != ((uint16_t)a2 & 0xFFF)) {
            int msb = ctz32(~a2) - 1;
            tcg_debug_assert(is_p2m1(a2));
            tcg_out_opc_bf(s, OPC_EXT, a0, a1, msb, 0);
            break;
        }
        if (c2) {
            tcg_out_opc_imm(s, OPC_ANDI, a0, a1, a2);
        } else {
            tcg_out_opc_reg(s, OPC_AND, a0, a1, a2);
        }
        break;
    case INDEX_op_and_i64:
        if (c2 && a2 != ((uint16_t)a2 & 0xFFF)) {
            int msb = ctz64(~a2) - 1;
            tcg_debug_assert(is_p2m1(a2));
            tcg_out_opc_bf64(s, OPC_DEXT, a0, a1, msb, 0);
            break;
        }
        if (c2) {
            tcg_out_opc_imm(s, OPC_ANDI, a0, a1, a2);
        } else {
            tcg_out_opc_reg(s, OPC_AND, a0, a1, a2);
        }
        break;
    case INDEX_op_or_i32:
    case INDEX_op_or_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t c2 is %d\n",__LINE__,c2);
        #endif
        if (c2) {
            tcg_out_opc_imm(s, OPC_ORI, a0, a1, a2);
        } else {
            tcg_out_opc_reg(s, OPC_OR, a0, a1, a2);
        }
        break;
    case INDEX_op_xor_i32:
    case INDEX_op_xor_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t c2 is %d\n",__LINE__,c2);
        #endif
        if (c2) {
            tcg_out_opc_imm(s, OPC_XORI, a0, a1, a2);
        } else {
            tcg_out_opc_reg(s, OPC_XOR, a0, a1, a2);
        }
        break;
    case INDEX_op_nor_i32:
    case INDEX_op_nor_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t c2 is %d\n",__LINE__,c2);
        #endif
        tcg_out_opc_reg(s, OPC_NOR, a0, a1, a2);
        break;
    case INDEX_op_not_i32:
    case INDEX_op_not_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_NOR, a0, TCG_REG_ZERO, a1);
        break;

    case INDEX_op_neg_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_SUBW, a0, TCG_REG_ZERO, a1);
        break;
    case INDEX_op_neg_i64:
        tcg_out_opc_reg(s, OPC_SUB, a0, TCG_REG_ZERO, a1);
        break;
    case INDEX_op_andc_i32:
    case INDEX_op_andc_i64:
        if (c2) {
        tcg_debug_assert(0);
        } else {
            tcg_out_opc_reg(s, OPC_ANDN, a0, a1, a2);
        }
        break;
    case INDEX_op_orc_i32:
    case INDEX_op_orc_i64:
        if (c2) {
            tcg_debug_assert(0);
        } else {
            tcg_debug_assert(0);
            tcg_out_opc_reg(s, OPC_ORN, a0, a1, a2);
        }
        break;
    case INDEX_op_mul_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_MULW, a0, a1, a2);
        break;
    case INDEX_op_mul_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_MUL, a0, a1, a2);
        break;
    case INDEX_op_div_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        //tcg_out_opc_reg(s, OPC_DIVW, a0, a1, a2);
        break;
    case INDEX_op_div_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_DIVD, a0, a1, a2);
        break;
    case INDEX_op_divu_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        //tcg_out_opc_reg(s, OPC_DIVUW, a0, a1, a2);
        break;
    case INDEX_op_divu_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_DIVDU, a0, a1, a2);
        break;
    case INDEX_op_rem_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        //tcg_out_opc_reg(s, OPC_REMW, a0, a1, a2);
        break;
    case INDEX_op_rem_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_MODD, a0, a1, a2);
        break;
    case INDEX_op_remu_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        //tcg_out_opc_reg(s, OPC_REMUW, a0, a1, a2);
        break;
    case INDEX_op_remu_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_MODDU, a0, a1, a2);
        break;
    case INDEX_op_shl_i32:
        /*
         * Refer to mips, SLLI.W and SLL.W will be used.
         */
        if (c2) {
            tcg_out_opc_sa(s, OPC_SLLIW, a0, a1, a2);
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_reg(s, OPC_SLLW, a0, a1, a2);
        }
        break;
    case INDEX_op_shl_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        if (c2) {
            tcg_out_dsll(s, a0, a1, a2);
        } else {
            tcg_out_opc_reg(s, OPC_SLL, a0, a1, a2);
        }
        break;
    case INDEX_op_shr_i32:
        /*
         * Refer to mips, SRLI.W and SRL.W will be used.
         */
        if (c2) {
            tcg_out_opc_sa(s, OPC_SRLIW, a0, a1, a2);
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_reg(s, OPC_SRLW, a0, a1, a2);
        }
        break;
    case INDEX_op_shr_i64:
        if (c2) {
            tcg_out_opc_sa64(s, OPC_SRLID, a0, a1, a2);
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_reg(s, OPC_SRL, a0, a1, a2);
        }
        break;
    case INDEX_op_sar_i32:
        /*
         * Refer to mips, SRLI.W and SRL.W will be used.
         */
        if (c2) {
            tcg_out_opc_sa(s, OPC_SRAIW, a0, a1, a2);
        } else {
            #ifdef LOONGARCH_DEBUG
            printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
            #endif
            tcg_out_opc_reg(s, OPC_SRAW, a0, a1, a2);
        }
        break;
    case INDEX_op_sar_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        if (c2) {
            tcg_out_opc_sa64(s, OPC_SRAI, a0, a1, a2);
        } else {
            tcg_out_opc_reg(s, OPC_SRA, a0, a1, a2);
        }
        break;
    case INDEX_op_add2_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_addsub2(s, a0, a1, a2, args[3], args[4], args[5],
                        const_args[4], const_args[5], false, true);
        break;
    case INDEX_op_add2_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_addsub2(s, a0, a1, a2, args[3], args[4], args[5],
                        const_args[4], const_args[5], false, false);
        break;
    case INDEX_op_sub2_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_addsub2(s, a0, a1, a2, args[3], args[4], args[5],
                        const_args[4], const_args[5], true, true);
        break;
    case INDEX_op_sub2_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_addsub2(s, a0, a1, a2, args[3], args[4], args[5],
                        const_args[4], const_args[5], true, false);
        break;
    case INDEX_op_brcond_i32:
    case INDEX_op_brcond_i64:
        #ifdef LOONGARCH_DEBUG
        printf("[Fix me] brcond at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_brcond(s, a2, a0, a1, arg_label(args[3]));
        break;
    case INDEX_op_movcond_i32:
    case INDEX_op_movcond_i64:
        #ifdef LOONGARCH_DEBUG
        printf("[Fix me] brcond at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_movcond(s, args[5], a0, a1, a2, args[3], args[4]);
        break;
    case INDEX_op_brcond2_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_brcond2(s, args[4], a0, a1, a2, args[3], arg_label(args[5]));
        break;
    case INDEX_op_setcond_i32:
    case INDEX_op_setcond_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_setcond(s, args[3], a0, a1, a2);
        break;
    case INDEX_op_setcond2_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_setcond2(s, args[5], a0, a1, a2, args[3], args[4]);
        break;
    case INDEX_op_qemu_ld_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_qemu_ld(s, args, false);
        break;
    case INDEX_op_qemu_ld_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_qemu_ld(s, args, true);
        break;
    case INDEX_op_qemu_st_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_qemu_st(s, args, false);
        break;
    case INDEX_op_qemu_st_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_qemu_st(s, args, true);
        break;
    case INDEX_op_ext8u_i32:
    case INDEX_op_ext8u_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_ext8u(s, a0, a1);
        break;
    case INDEX_op_ext16u_i32:
    case INDEX_op_ext16u_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_ext16u(s, a0, a1);
        break;
    case INDEX_op_ext32u_i64:
    case INDEX_op_extu_i32_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_ext32u(s, a0, a1);
        break;
    case INDEX_op_ext8s_i32:
    case INDEX_op_ext8s_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_ext8s(s, a0, a1);
        break;
    case INDEX_op_ext16s_i32:
    case INDEX_op_ext16s_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_ext16s(s, a0, a1);
        break;
    case INDEX_op_ext32s_i64:
    case INDEX_op_extrl_i64_i32:
    case INDEX_op_ext_i32_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_ext32s(s, a0, a1);
        break;
    case INDEX_op_extrh_i64_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_debug_assert(0);
        tcg_out_opc_imm(s, OPC_SRAI, a0, a1, 32);
        break;
    case INDEX_op_deposit_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d, pos=%ld,len=%ld\n",__LINE__,opc,args[3],args[4]);
        #endif
        tcg_out_opc_bf(s, OPC_INS, a0, a2, args[3] + args[4] - 1, args[3]);
        break;
    case INDEX_op_deposit_i64:
        tcg_out_opc_bf64(s, OPC_DINS, a0, a2,
                         args[3] + args[4] - 1, args[3]);
        break;
    case INDEX_op_mulsh_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_MULHW, a0, a1, a2);
        break;
    case INDEX_op_mulsh_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_DMULH, a0, a1, a2);
	break;
    case INDEX_op_muluh_i32:
        tcg_out_opc_reg(s, OPC_MULHWU, a0, a1, a2);
        break;
    case INDEX_op_muluh_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_opc_reg(s, OPC_MULHDU, a0, a1, a2);
        break;
    case INDEX_op_extract_i32:
        tcg_out_opc_bf(s, OPC_EXT, a0, a1, args[3] - 1 + a2, a2);
        break;
    case INDEX_op_extract_i64:
        /*
         * DEXT [msdb:lsdb]
         */
        tcg_out_opc_bf64(s, OPC_DEXT, a0, a1, args[3] - 1 + a2, a2);
        break;
    case INDEX_op_clz_i32:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_cltz(s, OPC_CLZW, 32 ,a0, a1, a2);
        break;
    case INDEX_op_clz_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_cltz(s, OPC_CLZD, 64, a0, a1, a2);
        break;
    case INDEX_op_ctz_i32:
        tcg_out_cltz(s, OPC_CTZW, 32 ,a0, a1, a2);
        break;
    case INDEX_op_ctz_i64:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_cltz(s, OPC_CTZD, 64, a0, a1, a2);
        break;
    case INDEX_op_rotr_i32:
        if (c2) {
            tcg_debug_assert(0);
            tcg_out_opc_sa(s, OPC_ROTRIW, a0, a1, a2);
            break;
        }
        tcg_debug_assert(0);
        tcg_out_opc_reg(s, OPC_ROTRW, a0, a2, a1);
        break;
    case INDEX_op_rotr_i64:
        if (c2) {
            tcg_debug_assert(0);
            tcg_out_opc_sa64(s, OPC_ROTRID, a0, a1, a2);
            break;
        }
        tcg_debug_assert(0);
        tcg_out_opc_reg(s, OPC_ROTRD, a0, a2, a1);
        break;
    case INDEX_op_rotl_i32:
        if (c2) {
            tcg_out_opc_sa(s, OPC_ROTRIW, a0, a1, 32 - a2);
        } else {
            tcg_debug_assert(0);
        }
        break;
    case INDEX_op_rotl_i64:
        if (c2) {
            tcg_out_opc_sa64(s, OPC_ROTRID, a0, a1, 64 - a2);
        } else {
            tcg_out_opc_reg(s, OPC_SUB, TCG_REG_TMP0, TCG_REG_ZERO, a2);
            tcg_out_opc_reg(s, OPC_ROTRD, a0, a1, TCG_REG_TMP0);
        }
        break;
    case INDEX_op_mb:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        tcg_out_mb(s, a0);
        break;
    case INDEX_op_mov_i32:  /* Always emitted via tcg_out_mov.  */
    case INDEX_op_mov_i64:
    case INDEX_op_movi_i32: /* Always emitted via tcg_out_movi.  */
    case INDEX_op_movi_i64:
    case INDEX_op_call:     /* Always emitted via tcg_out_call.  */
    default:
        #ifdef LOONGARCH_DEBUG
        printf("OPs at Line %d, \t opc is %d\n",__LINE__,opc);
        #endif
        g_assert_not_reached();
    }
}

static const TCGTargetOpDef *tcg_target_op_def(TCGOpcode op)
{
    static const TCGTargetOpDef r
        = { .args_ct_str = { "r" } };
    static const TCGTargetOpDef r_r
        = { .args_ct_str = { "r", "r" } };
    static const TCGTargetOpDef rZ_r
        = { .args_ct_str = { "rZ", "r" } };
    static const TCGTargetOpDef rZ_rZ
        = { .args_ct_str = { "rZ", "rZ" } };
    static const TCGTargetOpDef rZ_rZ_rZ_rZ
        = { .args_ct_str = { "rZ", "rZ", "rZ", "rZ" } };
    static const TCGTargetOpDef r_r_ri
        = { .args_ct_str = { "r", "r", "ri" } };
    static const TCGTargetOpDef r_r_rI
        = { .args_ct_str = { "r", "r", "rI" } };
    static const TCGTargetOpDef r_r_rUK
        = { .args_ct_str = { "r", "r", "rUK" } };
    static const TCGTargetOpDef r_r_rU
        = { .args_ct_str = { "r", "r", "rU" } };
    static const TCGTargetOpDef r_rZ_rN
        = { .args_ct_str = { "r", "rZ", "rN" } };
    static const TCGTargetOpDef r_rZ_rZ
        = { .args_ct_str = { "r", "rZ", "rZ" } };
    static const TCGTargetOpDef r_rZ_rZ_rZ_rZ
        = { .args_ct_str = { "r", "rZ", "rZ", "rZ", "rZ" } };
    static const TCGTargetOpDef r_L
        = { .args_ct_str = { "r", "L" } };
    static const TCGTargetOpDef r_r_L
        = { .args_ct_str = { "r", "r", "L" } };
    static const TCGTargetOpDef r_L_L
        = { .args_ct_str = { "r", "L", "L" } };
    static const TCGTargetOpDef r_r_L_L
        = { .args_ct_str = { "r", "r", "L", "L" } };
    static const TCGTargetOpDef LZ_L
        = { .args_ct_str = { "LZ", "L" } };
    static const TCGTargetOpDef LZ_L_L
        = { .args_ct_str = { "LZ", "L", "L" } };
    static const TCGTargetOpDef LZ_LZ_L
        = { .args_ct_str = { "LZ", "LZ", "L" } };
    static const TCGTargetOpDef LZ_LZ_L_L
        = { .args_ct_str = { "LZ", "LZ", "L", "L" } };
    static const TCGTargetOpDef r_r_rZ_rZ_rM_rM
        = { .args_ct_str = { "r", "r", "rZ", "rZ", "rM", "rM" } };
    static const TCGTargetOpDef dep
        = { .args_ct_str = { "r", "0", "rZ" } };

    switch (op) {
    case INDEX_op_goto_ptr:
        return &r;

    case INDEX_op_ld8u_i32:
    case INDEX_op_ld8s_i32:
    case INDEX_op_ld16u_i32:
    case INDEX_op_ld16s_i32:
    case INDEX_op_ld_i32:
    case INDEX_op_not_i32:
    case INDEX_op_bswap16_i32:
    case INDEX_op_bswap32_i32:
    case INDEX_op_neg_i32:
    case INDEX_op_ld8u_i64:
    case INDEX_op_ld8s_i64:
    case INDEX_op_ld16u_i64:
    case INDEX_op_ld16s_i64:
    case INDEX_op_ld32s_i64:
    case INDEX_op_ld32u_i64:
    case INDEX_op_ld_i64:
    case INDEX_op_not_i64:
    case INDEX_op_bswap16_i64:
    case INDEX_op_bswap32_i64:
    case INDEX_op_bswap64_i64:
    case INDEX_op_neg_i64:
    case INDEX_op_ext8u_i32:
    case INDEX_op_ext8u_i64:
    case INDEX_op_ext16u_i32:
    case INDEX_op_ext16u_i64:
    case INDEX_op_ext32u_i64:
    case INDEX_op_extu_i32_i64:
    case INDEX_op_ext8s_i32:
    case INDEX_op_ext8s_i64:
    case INDEX_op_ext16s_i32:
    case INDEX_op_ext16s_i64:
    case INDEX_op_ext32s_i64:
    case INDEX_op_extrl_i64_i32:
    case INDEX_op_extrh_i64_i32:
    case INDEX_op_ext_i32_i64:
    case INDEX_op_extract_i32:
    case INDEX_op_extract_i64:
        return &r_r;

    case INDEX_op_st8_i32:
    case INDEX_op_st16_i32:
    case INDEX_op_st_i32:
    case INDEX_op_st8_i64:
    case INDEX_op_st16_i64:
    case INDEX_op_st32_i64:
    case INDEX_op_st_i64:
        return &rZ_r;

    case INDEX_op_add_i32:
    case INDEX_op_add_i64:
        return &r_r_rI;
    case INDEX_op_or_i32:
    case INDEX_op_xor_i32:
    case INDEX_op_or_i64:
    case INDEX_op_xor_i64:
        return &r_r_rU;

    case INDEX_op_and_i32:
    case INDEX_op_and_i64:
        return &r_r_rUK;

    case INDEX_op_andc_i32:
    case INDEX_op_andc_i64:
        return &r_r_rU;

    case INDEX_op_orc_i32:
    case INDEX_op_orc_i64:
        return &r_r_rU;

    case INDEX_op_sub_i32:
    case INDEX_op_sub_i64:
        return &r_rZ_rN;

    case INDEX_op_mul_i32:
    case INDEX_op_mulsh_i32:
    case INDEX_op_muluh_i32:
    case INDEX_op_div_i32:
    case INDEX_op_divu_i32:
    case INDEX_op_rem_i32:
    case INDEX_op_remu_i32:
    case INDEX_op_nor_i32:
    case INDEX_op_setcond_i32:
    case INDEX_op_mul_i64:
    case INDEX_op_mulsh_i64:
    case INDEX_op_muluh_i64:
    case INDEX_op_div_i64:
    case INDEX_op_divu_i64:
    case INDEX_op_rem_i64:
    case INDEX_op_remu_i64:
    case INDEX_op_nor_i64:
    case INDEX_op_setcond_i64:
        return &r_rZ_rZ;

    case INDEX_op_shl_i32:
    case INDEX_op_shr_i32:
    case INDEX_op_sar_i32:
    case INDEX_op_rotr_i32:
    case INDEX_op_rotl_i32:
    case INDEX_op_shl_i64:
    case INDEX_op_shr_i64:
    case INDEX_op_sar_i64:
    case INDEX_op_rotr_i64:
    case INDEX_op_rotl_i64:
    case INDEX_op_clz_i32:
    case INDEX_op_ctz_i32:
    case INDEX_op_clz_i64:
    case INDEX_op_ctz_i64:
        return &r_r_ri;

    case INDEX_op_deposit_i32:
    case INDEX_op_deposit_i64:
        return &dep;

    case INDEX_op_brcond_i32:
    case INDEX_op_brcond_i64:
        return &rZ_rZ;

    case INDEX_op_add2_i32:
    case INDEX_op_add2_i64:
    case INDEX_op_sub2_i32:
    case INDEX_op_sub2_i64:
        return &r_r_rZ_rZ_rM_rM;

    case INDEX_op_brcond2_i32:
        return &rZ_rZ_rZ_rZ;

    case INDEX_op_movcond_i32:
    case INDEX_op_setcond2_i32:
    case INDEX_op_movcond_i64:
        return &r_rZ_rZ_rZ_rZ;

    case INDEX_op_qemu_ld_i32:
        return TARGET_LONG_BITS <= TCG_TARGET_REG_BITS ? &r_L : &r_L_L;
    case INDEX_op_qemu_st_i32:
        return TARGET_LONG_BITS <= TCG_TARGET_REG_BITS ? &LZ_L : &LZ_L_L;
    case INDEX_op_qemu_ld_i64:
        return TCG_TARGET_REG_BITS == 64 ? &r_L
               : TARGET_LONG_BITS <= TCG_TARGET_REG_BITS ? &r_r_L
               : &r_r_L_L;
    case INDEX_op_qemu_st_i64:
        return TCG_TARGET_REG_BITS == 64 ? &LZ_L
               : TARGET_LONG_BITS <= TCG_TARGET_REG_BITS ? &LZ_LZ_L
               : &LZ_LZ_L_L;

    default:
        return NULL;
    }
}

static const int tcg_target_callee_save_regs[] = {
    TCG_REG_S0,       /* used for the global env (TCG_AREG0) */
    TCG_REG_S1,
    TCG_REG_S2,
    TCG_REG_S3,
    TCG_REG_S4,
    TCG_REG_S5,
    TCG_REG_S6,
    TCG_REG_S7,
    TCG_REG_S8,
    TCG_REG_RA,       /* should be last for ABI compliance */
};

/* Stack frame parameters.  */
#define REG_SIZE   (TCG_TARGET_REG_BITS / 8)
#define SAVE_SIZE  ((int)ARRAY_SIZE(tcg_target_callee_save_regs) * REG_SIZE)
#define TEMP_SIZE  (CPU_TEMP_BUF_NLONGS * (int)sizeof(long))
#define FRAME_SIZE ((TCG_STATIC_CALL_ARGS_SIZE + TEMP_SIZE + SAVE_SIZE \
                     + TCG_TARGET_STACK_ALIGN - 1) \
                    & -TCG_TARGET_STACK_ALIGN)
#define SAVE_OFS   (TCG_STATIC_CALL_ARGS_SIZE + TEMP_SIZE)

/* We're expecting to be able to use an immediate for frame allocation.  */
QEMU_BUILD_BUG_ON(FRAME_SIZE > 0x7ff);

/* Generate global QEMU prologue and epilogue code */
static void tcg_target_qemu_prologue(TCGContext *s)
{
    int i;

    #ifdef LOONGARCH_DEBUG
    printf("Start tcg_target_qemu_prologue.\n");
    #endif

    tcg_set_frame(s, TCG_REG_SP, TCG_STATIC_CALL_ARGS_SIZE, TEMP_SIZE);

    /* TB prologue */
    tcg_out_opc_imm(s, ALIAS_PADDI, TCG_REG_SP, TCG_REG_SP, -FRAME_SIZE);
    for (i = 0; i < ARRAY_SIZE(tcg_target_callee_save_regs); i++) {
        tcg_out_st(s, TCG_TYPE_REG, tcg_target_callee_save_regs[i],
                   TCG_REG_SP, SAVE_OFS + i * REG_SIZE);
    }

#ifndef CONFIG_SOFTMMU
    if (guest_base) {
        tcg_out_movi(s, TCG_TYPE_PTR, TCG_GUEST_BASE_REG, guest_base);
        tcg_regset_set_reg(s->reserved_regs, TCG_GUEST_BASE_REG);
    }
#endif

    /* Call generated code */
    tcg_out_mov(s, TCG_TYPE_PTR, TCG_AREG0, tcg_target_call_iarg_regs[0]);
    //LoongArch
    //tcg_out_opc_imm(s, OPC_JALR, TCG_REG_ZERO, tcg_target_call_iarg_regs[1], 0);
    tcg_out_opc_jirl(s, TCG_REG_ZERO, tcg_target_call_iarg_regs[1]);
    /* Return path for goto_ptr. Set return value to 0 */
    s->code_gen_epilogue = s->code_ptr;
    tcg_out_mov(s, TCG_TYPE_REG, TCG_REG_A0, TCG_REG_ZERO);

    /* TB epilogue */
    tb_ret_addr = s->code_ptr;
    for (i = 0; i < ARRAY_SIZE(tcg_target_callee_save_regs); i++) {
        tcg_out_ld(s, TCG_TYPE_REG, tcg_target_callee_save_regs[i],
                   TCG_REG_SP, SAVE_OFS + i * REG_SIZE);
    }

    tcg_out_opc_imm(s, ALIAS_PADDI, TCG_REG_SP, TCG_REG_SP, FRAME_SIZE);
    //LoongArch
    //tcg_out_opc_imm(s, OPC_JALR, TCG_REG_ZERO, TCG_REG_RA, 0);
    tcg_out_opc_jirl(s, TCG_REG_ZERO, TCG_REG_RA);

    #ifdef LOONGARCH_DEBUG
    printf("End tcg_target_qemu_prologue.\n");
    #endif
}

static void tcg_target_init(TCGContext *s)
{
    tcg_target_available_regs[TCG_TYPE_I32] = 0xffffffff;
    if (TCG_TARGET_REG_BITS == 64) {
        tcg_target_available_regs[TCG_TYPE_I64] = 0xffffffff;
    }

    tcg_target_call_clobber_regs = -1u;
    tcg_regset_reset_reg(tcg_target_call_clobber_regs, TCG_REG_S0);
    tcg_regset_reset_reg(tcg_target_call_clobber_regs, TCG_REG_S1);
    tcg_regset_reset_reg(tcg_target_call_clobber_regs, TCG_REG_S2);
    tcg_regset_reset_reg(tcg_target_call_clobber_regs, TCG_REG_S3);
    tcg_regset_reset_reg(tcg_target_call_clobber_regs, TCG_REG_S4);
    tcg_regset_reset_reg(tcg_target_call_clobber_regs, TCG_REG_S5);
    tcg_regset_reset_reg(tcg_target_call_clobber_regs, TCG_REG_S6);
    tcg_regset_reset_reg(tcg_target_call_clobber_regs, TCG_REG_S7);
    tcg_regset_reset_reg(tcg_target_call_clobber_regs, TCG_REG_S8);

    s->reserved_regs = 0;
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_ZERO);
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_TMP0);
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_TMP1);
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_TMP2);
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_TP);
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_SP);
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_X);
    tcg_regset_set_reg(s->reserved_regs, TCG_REG_FP);
}

void tb_target_set_jmp_target(uintptr_t tc_ptr, uintptr_t jmp_addr,
                              uintptr_t addr)
{
    ptrdiff_t offset = addr - jmp_addr;
    tcg_insn_unit insn;
    if (offset == sextract64(offset, 0, 26)) {
        insn = encode_uj(OPC_B, offset&0xFFFFFFF);
    } else {
        tcg_debug_assert(0);
    } 
    /*
     * Update insn with new address.
     */
    atomic_set((uint32_t *)jmp_addr, insn);
    flush_icache_range(jmp_addr, jmp_addr + 4);
}

typedef struct {
    DebugFrameHeader h;
    uint8_t fde_def_cfa[4];
    uint8_t fde_reg_ofs[ARRAY_SIZE(tcg_target_callee_save_regs) * 2];
} DebugFrame;

#define ELF_HOST_MACHINE EM_LOONGARCH

static const DebugFrame debug_frame = {
    .h.cie.len = sizeof(DebugFrameCIE) - 4, /* length after .len member */
    .h.cie.id = -1,
    .h.cie.version = 1,
    .h.cie.code_align = 1,
    .h.cie.data_align = -(TCG_TARGET_REG_BITS / 8) & 0x7f, /* sleb128 */
    .h.cie.return_column = TCG_REG_RA,

    /* Total FDE size does not include the "len" member.  */
    .h.fde.len = sizeof(DebugFrame) - offsetof(DebugFrame, h.fde.cie_offset),

    .fde_def_cfa = {
        12, TCG_REG_SP,                 /* DW_CFA_def_cfa sp, ... */
        (FRAME_SIZE & 0x7f) | 0x80,     /* ... uleb128 FRAME_SIZE */
        (FRAME_SIZE >> 7)
    },
    .fde_reg_ofs = {
        0x80 + 24, 9,                   /* DW_CFA_offset, s1,  -72 */
        0x80 + 25, 8,                   /* DW_CFA_offset, s2,  -64 */
        0x80 + 26, 7,                   /* DW_CFA_offset, s3,  -56 */
        0x80 + 27, 6,                   /* DW_CFA_offset, s4,  -48 */
        0x80 + 28, 5,                   /* DW_CFA_offset, s5,  -40 */
        0x80 + 29, 4,                   /* DW_CFA_offset, s6,  -32 */
        0x80 + 30, 3,                   /* DW_CFA_offset, s7,  -24 */
        0x80 + 31, 2,                   /* DW_CFA_offset, s8,  -16 */
        0x80 + 1 , 1,                   /* DW_CFA_offset, ra,  -8 */
    }
};

void tcg_register_jit(void *buf, size_t buf_size)
{
    tcg_register_jit_int(buf, buf_size, &debug_frame, sizeof(debug_frame));
}
