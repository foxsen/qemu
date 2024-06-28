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
#include "internals.h"
#include "qemu/host-utils.h"
#include "exec/helper-proto.h"
#include "exec/exec-all.h"

static inline bool check_auth(CPULoongArchState *env, hwaddr phy_pc)
{
    int i;
    hwaddr blk_num, blk_off, tab_num, tab_size, sub_blk_mask;

    // extract blk number and offset from physical pc
    blk_num = phy_pc >> 8;
    blk_off = phy_pc & 0xff;
    qemu_log_mask(CPU_LOG_PCALL, "blk_begin: %lx, blk_off: %lx\n", blk_num, blk_off);

    // 0x1c000000 to 0x1c000fff (4KB) is the default safe instrution area
    if (0x1c000000 <= phy_pc && phy_pc <= 0x1c000fff) {
        return true;
    } else if (env->si_table_en) {
    // or we should check the instrction table
        for (i = 0; i <= 64; i++) {
            if (!env->si_valid[i]) continue;
            tab_num = env->si_cfg[i] >> 8;
            tab_size = 1 << ((env->si_cfg[i] & 0x1f) + 8);
            sub_blk_mask = 1 << (blk_off / (tab_size >> 6));
            if (
                blk_num == tab_num &&
                blk_off < tab_size &&
                (sub_blk_mask & env->si_bitmap[i])
            ) {
                return true;
            }
        }
    }

    return false;
}

/* treat word/dword access as the same, please choose right version to use */
uint64_t helper_safecfgr_w(CPULoongArchState *env, target_ulong addr)
{
    uint64_t val;
    int n;
    //! Is is a proper way to get the physical address?
    hwaddr phy_pc = loongarch_cpu_get_phys_addr(env, env->pc);

    if (!check_auth(env, phy_pc)) {
        do_raise_exception(env, EXCCODE_SINST, env->pc);
        qemu_log_mask(CPU_LOG_PCALL, "safety authorization check failed pc: %lx\n", env->pc);
        return -1;
    }

    if (addr == 0) {
	    val = env->ss_en;
    } else if (addr == 8) {
	    val = env->ssbuf_size;
    } else if (addr == 0x10) {
        val = env->si_table_en;
    } else if (addr == 0x20) {
	    val = env->ssbuf_base;
    } else if (addr == 0x28) {
	    val = env->ssbuf_top;
    }  else if (0x40000 <= addr && addr <= 0x44000) {
        n = (addr - 0x40000) / 0x20;
        switch (addr & 0x1f) {
        case 0x0:
            val = env->si_valid[n];
            break;
        case 0x8:
            val = env->si_cfg[n];
            break;
        case 0x10:
            val = env->si_bitmap[n];
            break;
        default:
            qemu_log_mask(CPU_LOG_PCALL, "ERROR: illegal safecfgw address " TARGET_FMT_lx, addr);
	        return -1;
        }
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
    //! Is is a proper way to get the physical address?
    int n;
    hwaddr phy_pc = loongarch_cpu_get_phys_addr(env, env->pc);

    if (!check_auth(env, phy_pc)) {
        do_raise_exception(env, EXCCODE_SINST, env->pc);
        qemu_log_mask(CPU_LOG_PCALL, "safety authorization check failed pc: %lx\n", env->pc);
        return;
    }

    if (addr == 0) {
	    env->ss_en = val;
    } else if (addr == 0x10) {
        env->si_table_en = val;
    } else if (addr == 0x20) {
	    env->ssbuf_base = val;
    } else if (addr == 0x28) {
	    env->ssbuf_top = val;
    } else if (0x40000 <= addr && addr <= 0x44000) {
        n = (addr - 0x40000) / 0x20;
        switch (addr & 0x1f) {
        case 0x0:
            env->si_valid[n] = val & 0x1;
            break;
        case 0x8:
            env->si_cfg[n] = val;
            break;
        case 0x10:
            env->si_bitmap[n] = val;
            break;
        default:
            qemu_log_mask(CPU_LOG_PCALL, "ERROR: illegal safecfgw address " TARGET_FMT_lx, addr);
            return;
        }
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
