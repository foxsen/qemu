/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * QEMU LoongArch LSX helper functions.
 *
 * Copyright (c) 2022 Loongson Technology Corporation Limited
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"

#define DO_VVV(NAME, BIT, FUNC, ...)                          \
    void helper_##NAME(CPULoongArchState *env,                \
                       uint32_t vd, uint32_t vj, uint32_t vk) \
    { FUNC(env, vd, vj, vk, BIT, __VA_ARGS__); }

static void do_vvv(CPULoongArchState *env,
                   uint32_t vd, uint32_t vj, uint32_t vk, int bit,
                   void (*func)(VReg*, VReg*, VReg*, int, int))
{
    int i;
    VReg *Vd = &(env->fpr[vd].vreg);
    VReg *Vj = &(env->fpr[vj].vreg);
    VReg *Vk = &(env->fpr[vk].vreg);

    for (i = 0; i < LSX_LEN/bit; i++) {
        func(Vd, Vj, Vk, bit, i);
    }
}

void helper_vadd_q(CPULoongArchState *env,
                   uint32_t vd, uint32_t vj, uint32_t vk)
{
    VReg *Vd = &(env->fpr[vd].vreg);
    VReg *Vj = &(env->fpr[vj].vreg);
    VReg *Vk = &(env->fpr[vk].vreg);

    Vd->Q[0] = int128_add(Vj->Q[0], Vk->Q[0]);    
}

void helper_vsub_q(CPULoongArchState *env,
                   uint32_t vd, uint32_t vj, uint32_t vk)
{
    VReg *Vd = &(env->fpr[vd].vreg);
    VReg *Vj = &(env->fpr[vj].vreg);
    VReg *Vk = &(env->fpr[vk].vreg);

    Vd->Q[0] = int128_sub(Vj->Q[0], Vk->Q[0]);
}

static void do_vhaddw_s(VReg *Vd, VReg *Vj, VReg *Vk, int bit, int i)
{
    int odd = 2 * i + 1;
    int even = 2 * i;
    switch (bit) {
    case 16:
        Vd->H[i] = (int16_t)Vj->B[odd] + (int16_t)Vk->B[even];
        break;
    case 32:
        Vd->W[i] = (int32_t)Vj->H[odd] + (int32_t)Vk->H[even];
        break;
    case 64:
        Vd->D[i] = (int64_t)Vj->W[odd] + (int64_t)Vk->W[even];
        break;
    case 128:
        Vd->Q[i] = int128_add(Vj->D[odd], Vk->D[even]);
        break;
    default:
        g_assert_not_reached();
    }
}

static void do_vhaddw_u(VReg *Vd, VReg *Vj, VReg *Vk, int bit, int i)
{
    int odd = 2 * i + 1;
    int even = 2 * i;
    switch (bit) {
    case 16:
        Vd->H[i] = (uint16_t)(uint8_t)Vj->B[odd] + (uint16_t)(uint8_t)Vk->B[even];
        break;
    case 32:
        Vd->W[i] = (uint32_t)(uint16_t)Vj->H[odd] + (uint32_t)(uint16_t)Vk->H[even];
        break;
    case 64:
        Vd->D[i] = (uint64_t)(uint32_t)Vj->W[odd] + (uint64_t)(uint32_t)Vk->W[even];
        break;
    case 128:
        Vd->Q[i] = int128_add((uint64_t)Vj->D[odd],  (uint64_t)Vk->D[even]);
        break;
    default:
        g_assert_not_reached();
    }
}

static void do_vhsubw_s(VReg *Vd, VReg *Vj, VReg *Vk, int bit, int i)
{
    int odd = 2 * i + 1;
    int even = 2 * i;
    switch (bit) {
    case 16:
        Vd->H[i] = (int16_t)Vj->B[odd] - (int16_t)Vk->B[even];
        break;
    case 32:
        Vd->W[i] = (int32_t)Vj->H[odd] - (int32_t)Vk->H[even];
        break;
    case 64:
        Vd->D[i] = (int64_t)Vj->W[odd] - (int64_t)Vk->W[even];
        break;
    case 128:
        Vd->Q[i] = int128_sub(Vj->D[odd],  Vk->D[even]);
        break;
    default:
        g_assert_not_reached();
    }
}

static void do_vhsubw_u(VReg *Vd, VReg *Vj, VReg *Vk, int bit, int i)
{
    int odd = 2 * i + 1;
    int even = 2 * i;
    switch (bit) {
    case 16:
        Vd->H[i] = (uint16_t)(uint8_t)Vj->B[odd] - (uint16_t)(uint8_t)Vk->B[even];
        break;
    case 32:
        Vd->W[i] = (uint32_t)(uint16_t)Vj->H[odd] - (uint32_t)(uint16_t)Vk->H[even];
        break;
    case 64:
        Vd->D[i] = (uint64_t)(uint32_t)Vj->W[odd] - (uint64_t)(uint32_t)Vk->W[even];
        break;
    case 128:
        Vd->Q[i] = int128_sub((uint64_t)Vj->D[odd], (uint64_t)Vk->D[even]);
        break;
    default:
        g_assert_not_reached();
    }
}

DO_VVV(vhaddw_h_b, 16, do_vvv, do_vhaddw_s)
DO_VVV(vhaddw_w_h, 32, do_vvv, do_vhaddw_s)
DO_VVV(vhaddw_d_w, 64, do_vvv, do_vhaddw_s)
DO_VVV(vhaddw_q_d, 128, do_vvv, do_vhaddw_s)
DO_VVV(vhaddw_hu_bu, 16, do_vvv, do_vhaddw_u)
DO_VVV(vhaddw_wu_hu, 32, do_vvv, do_vhaddw_u)
DO_VVV(vhaddw_du_wu, 64, do_vvv, do_vhaddw_u)
DO_VVV(vhaddw_qu_du, 128, do_vvv, do_vhaddw_u)
DO_VVV(vhsubw_h_b, 16, do_vvv, do_vhsubw_s)
DO_VVV(vhsubw_w_h, 32, do_vvv, do_vhsubw_s)
DO_VVV(vhsubw_d_w, 64, do_vvv, do_vhsubw_s)
DO_VVV(vhsubw_q_d, 128, do_vvv, do_vhsubw_s)
DO_VVV(vhsubw_hu_bu, 16, do_vvv, do_vhsubw_u)
DO_VVV(vhsubw_wu_hu, 32, do_vvv, do_vhsubw_u)
DO_VVV(vhsubw_du_wu, 64, do_vvv, do_vhsubw_u)
DO_VVV(vhsubw_qu_du, 128, do_vvv, do_vhsubw_u)
