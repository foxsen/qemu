/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2021 Loongson Technology Corporation Limited
 *
 * Helpers for IOCSR reads/writes
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "qemu/log.h"
#include "cpu.h"
#include "qemu/host-utils.h"
#include "exec/helper-proto.h"
#include "exec/exec-all.h"

/* treat word/dword access as the same, please choose right version to use */
uint64_t helper_safecfgr_w(CPULoongArchState *env, target_ulong addr)
{
    uint64_t val;
    if (addr == 0) {
	    val = env->ss_en;
    } else if (addr == 0x20) {
	    val = env->ssbuf_base;
    } else if (addr == 0x28) {
	    val = env->ssbuf_top;
    } else if (addr >= 0xff800 && addr <= 0xffff8) {
        int index = (0xffff8 - addr) / 8;
        val = env->ssbuf[index];
    } else {
	    qemu_log_mask(CPU_LOG_PCALL, "ERROR: illegal safecfgr address " TARGET_FMT_lx, addr);
	    return -1;
    }
    qemu_log_mask(CPU_LOG_PCALL, "safecfgr [%lx] = %lx\n", addr, val);
    return val;
}

uint64_t helper_safecfgr_d(CPULoongArchState *env, target_ulong addr)
{
    return helper_safecfgr_w(env, addr);
}

void helper_safecfgw_w(CPULoongArchState *env, target_ulong addr, target_ulong val)
{
    if (addr == 0) {
	    env->ss_en = val;
    } else if (addr == 0x20) {
	    env->ssbuf_base = val;
    } else if (addr == 0x28) {
	    env->ssbuf_top = val;
    } else if (addr >= 0xff800 && addr <= 0xffff8) {
        int index = (0xffff8 - addr) / 8;
        env->ssbuf[index] = val;
    } else {
	    qemu_log_mask(CPU_LOG_PCALL, "ERROR: illegal safecfgw address " TARGET_FMT_lx, addr);
	    return;
    }
    qemu_log_mask(CPU_LOG_PCALL, "safecfgw [%lx] = %lx\n", addr, val);
}

void helper_safecfgw_d(CPULoongArchState *env, target_ulong addr, target_ulong val)
{
    helper_safecfgw_w(env, addr, val);
}
