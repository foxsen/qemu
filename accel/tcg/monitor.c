/*
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  QEMU TCG monitor
 *
 *  Copyright (c) 2003-2005 Fabrice Bellard
 */

#include "qemu/osdep.h"
#include "qemu/accel.h"
#include "qemu/qht.h"
#include "qapi/error.h"
#include "qapi/type-helpers.h"
#include "qapi/qapi-commands-machine.h"
#include "monitor/monitor.h"
#include "sysemu/cpus.h"
#include "sysemu/cpu-timers.h"
#include "sysemu/tcg.h"
#include "tcg/tcg.h"
#include "internal-common.h"
#include "tb-context.h"


static void dump_drift_info(GString *buf)
{
    if (!icount_enabled()) {
        return;
    }

    g_string_append_printf(buf, "Host - Guest clock  %"PRIi64" ms\n",
                           (cpu_get_clock() - icount_get()) / SCALE_MS);
    if (icount_align_option) {
        g_string_append_printf(buf, "Max guest delay     %"PRIi64" ms\n",
                               -max_delay / SCALE_MS);
        g_string_append_printf(buf, "Max guest advance   %"PRIi64" ms\n",
                               max_advance / SCALE_MS);
    } else {
        g_string_append_printf(buf, "Max guest delay     NA\n");
        g_string_append_printf(buf, "Max guest advance   NA\n");
    }
}

static void dump_accel_info(GString *buf)
{
    AccelState *accel = current_accel();
    bool one_insn_per_tb = object_property_get_bool(OBJECT(accel),
                                                    "one-insn-per-tb",
                                                    &error_fatal);

    g_string_append_printf(buf, "Accelerator settings:\n");
    g_string_append_printf(buf, "one-insn-per-tb: %s\n\n",
                           one_insn_per_tb ? "on" : "off");
}

static void print_qht_statistics(struct qht_stats hst, GString *buf)
{
    uint32_t hgram_opts;
    size_t hgram_bins;
    char *hgram;

    if (!hst.head_buckets) {
        return;
    }
    g_string_append_printf(buf, "TB hash buckets     %zu/%zu "
                           "(%0.2f%% head buckets used)\n",
                           hst.used_head_buckets, hst.head_buckets,
                           (double)hst.used_head_buckets /
                           hst.head_buckets * 100);

    hgram_opts =  QDIST_PR_BORDER | QDIST_PR_LABELS;
    hgram_opts |= QDIST_PR_100X   | QDIST_PR_PERCENT;
    if (qdist_xmax(&hst.occupancy) - qdist_xmin(&hst.occupancy) == 1) {
        hgram_opts |= QDIST_PR_NODECIMAL;
    }
    hgram = qdist_pr(&hst.occupancy, 10, hgram_opts);
    g_string_append_printf(buf, "TB hash occupancy   %0.2f%% avg chain occ. "
                           "Histogram: %s\n",
                           qdist_avg(&hst.occupancy) * 100, hgram);
    g_free(hgram);

    hgram_opts = QDIST_PR_BORDER | QDIST_PR_LABELS;
    hgram_bins = qdist_xmax(&hst.chain) - qdist_xmin(&hst.chain);
    if (hgram_bins > 10) {
        hgram_bins = 10;
    } else {
        hgram_bins = 0;
        hgram_opts |= QDIST_PR_NODECIMAL | QDIST_PR_NOBINRANGE;
    }
    hgram = qdist_pr(&hst.chain, hgram_bins, hgram_opts);
    g_string_append_printf(buf, "TB hash avg chain   %0.3f buckets. "
                           "Histogram: %s\n",
                           qdist_avg(&hst.chain), hgram);
    g_free(hgram);
}

struct tb_tree_stats {
    size_t nb_tbs;
    size_t host_size;
    size_t target_size;
    size_t max_target_size;
    size_t direct_jmp_count;
    size_t direct_jmp2_count;
    size_t cross_page;
};

static gboolean tb_tree_stats_iter(gpointer key, gpointer value, gpointer data)
{
    const TranslationBlock *tb = value;
    struct tb_tree_stats *tst = data;

    tst->nb_tbs++;
    tst->host_size += tb->tc.size;
    tst->target_size += tb->size;
    if (tb->size > tst->max_target_size) {
        tst->max_target_size = tb->size;
    }
    if (tb->page_addr[1] != -1) {
        tst->cross_page++;
    }
    if (tb->jmp_reset_offset[0] != TB_JMP_OFFSET_INVALID) {
        tst->direct_jmp_count++;
        if (tb->jmp_reset_offset[1] != TB_JMP_OFFSET_INVALID) {
            tst->direct_jmp2_count++;
        }
    }
    return false;
}

static void tlb_flush_counts(size_t *pfull, size_t *ppart, size_t *pelide)
{
    CPUState *cpu;
    size_t full = 0, part = 0, elide = 0;

    CPU_FOREACH(cpu) {
        full += qatomic_read(&cpu->neg.tlb.c.full_flush_count);
        part += qatomic_read(&cpu->neg.tlb.c.part_flush_count);
        elide += qatomic_read(&cpu->neg.tlb.c.elide_flush_count);
    }
    *pfull = full;
    *ppart = part;
    *pelide = elide;
}

static void dump_tlb_config(GString *buf)
{
    size_t fixed_entries = 0;
    size_t current_min = SIZE_MAX;
    size_t current_max = 0;
    bool victim_enabled = true;
    bool found = false;
    CPUState *cpu;
    int mmu_idx;

    CPU_FOREACH(cpu) {
        CPUTLBCommon *common = &cpu->neg.tlb.c;

        found = true;
        if (common->fixed_tlb_bits) {
            fixed_entries = (size_t)1 << common->fixed_tlb_bits;
        }
        victim_enabled &= common->victim_tlb_enabled;
        for (mmu_idx = 0; mmu_idx < NB_MMU_MODES; mmu_idx++) {
            CPUTLBDescFast *fast = &cpu->neg.tlb.f[mmu_idx];
            size_t entries = (fast->mask >> CPU_TLB_ENTRY_BITS) + 1;

            current_min = MIN(current_min, entries);
            current_max = MAX(current_max, entries);
        }
    }
    if (!found) {
        current_min = 0;
    }
    g_string_append_printf(buf,
        "SoftMMU TLB config fixed_entries=%zu victim=%s "
        "current_min=%zu current_max=%zu\n",
        fixed_entries, victim_enabled ? "on" : "off",
        current_min, current_max);
}

static void dump_lp_tlb(GString *buf)
{
    static const char * const mode_name[] = {
        "off", "on", "probe", "adaptive",
    };
    size_t lookup = 0, match = 0, hit = 0, insert = 0;
    size_t evict = 0, flush = 0;
    size_t sample = 0, active = 0, bypass = 0, bypass_lookups = 0;
    size_t bypass_cpus = 0;
    unsigned mode = 0;
    CPUState *cpu;

    CPU_FOREACH(cpu) {
        mode = MAX(mode, cpu->neg.tlb.c.lp_tlb_mode);
        lookup += qatomic_read(&cpu->neg.tlb.c.lp_tlb_lookup_count);
        match += qatomic_read(&cpu->neg.tlb.c.lp_tlb_match_count);
        hit += qatomic_read(&cpu->neg.tlb.c.lp_tlb_hit_count);
        insert += qatomic_read(&cpu->neg.tlb.c.lp_tlb_insert_count);
        evict += qatomic_read(&cpu->neg.tlb.c.lp_tlb_evict_count);
        flush += qatomic_read(&cpu->neg.tlb.c.lp_tlb_flush_count);
        sample += qatomic_read(
            &cpu->neg.tlb.c.lp_tlb_adaptive.sample_window_count);
        active += qatomic_read(
            &cpu->neg.tlb.c.lp_tlb_adaptive.active_window_count);
        bypass += qatomic_read(
            &cpu->neg.tlb.c.lp_tlb_adaptive.bypass_window_count);
        bypass_cpus += cpu->neg.tlb.c.lp_tlb_adaptive.phase ==
                       CPU_TLB_ADAPTIVE_BYPASS;
#ifdef QEMU_TLB_PROFILE
        bypass_lookups += qatomic_read(
            &cpu->neg.tlb.c.lp_tlb_adaptive.bypass_lookup_count);
#endif
    }
    g_string_append_printf(buf,
        "Large-page cache mode=%s lookup=%zu match=%zu hit=%zu "
        "insert=%zu eviction=%zu flush=%zu\n",
        mode < ARRAY_SIZE(mode_name) ? mode_name[mode] : "invalid",
        lookup, match, hit, insert, evict, flush);
    g_string_append_printf(buf,
        "Large-page adaptive sample_windows=%zu active_windows=%zu "
        "bypass_windows=%zu bypassed_lookups=%zu bypass_cpus=%zu\n",
        sample, active, bypass, bypass_lookups, bypass_cpus);
}

static void dump_ptw_cache(GString *buf)
{
    static const char * const mode_name[] = {
        "off", "on", "probe", "adaptive",
    };
    size_t lookup[5] = {}, match[5] = {}, hit[5] = {};
    size_t insert[5] = {}, evict[5] = {}, flush = 0;
    size_t sample = 0, active = 0, bypass = 0, bypass_lookups = 0;
    size_t bypass_cpus = 0;
    unsigned mode = 0, level;
    CPUState *cpu;

    CPU_FOREACH(cpu) {
        mode = MAX(mode, cpu->neg.tlb.c.ptw_cache_mode);
        flush += qatomic_read(&cpu->neg.tlb.c.ptw_cache_flush_count);
        sample += qatomic_read(
            &cpu->neg.tlb.c.ptw_cache_adaptive.sample_window_count);
        active += qatomic_read(
            &cpu->neg.tlb.c.ptw_cache_adaptive.active_window_count);
        bypass += qatomic_read(
            &cpu->neg.tlb.c.ptw_cache_adaptive.bypass_window_count);
        bypass_cpus += cpu->neg.tlb.c.ptw_cache_adaptive.phase ==
                       CPU_TLB_ADAPTIVE_BYPASS;
#ifdef QEMU_TLB_PROFILE
        bypass_lookups += qatomic_read(
            &cpu->neg.tlb.c.ptw_cache_adaptive.bypass_lookup_count);
#endif
        for (level = 2; level <= 4; level++) {
            lookup[level] += qatomic_read(
                &cpu->neg.tlb.c.ptw_cache_lookup_count[level]);
            match[level] += qatomic_read(
                &cpu->neg.tlb.c.ptw_cache_match_count[level]);
            hit[level] += qatomic_read(
                &cpu->neg.tlb.c.ptw_cache_hit_count[level]);
            insert[level] += qatomic_read(
                &cpu->neg.tlb.c.ptw_cache_insert_count[level]);
            evict[level] += qatomic_read(
                &cpu->neg.tlb.c.ptw_cache_evict_count[level]);
        }
    }
    g_string_append_printf(buf, "PTW cache mode=%s flush=%zu\n",
        mode < ARRAY_SIZE(mode_name) ? mode_name[mode] : "invalid", flush);
    g_string_append_printf(buf,
        "PTW adaptive sample_windows=%zu active_windows=%zu "
        "bypass_windows=%zu bypassed_lookups=%zu bypass_cpus=%zu\n",
        sample, active, bypass, bypass_lookups, bypass_cpus);
    for (level = 2; level <= 4; level++) {
        g_string_append_printf(buf,
            "PTW cache level=%u lookup=%zu match=%zu hit=%zu "
            "insert=%zu eviction=%zu\n",
            level, lookup[level], match[level], hit[level],
            insert[level], evict[level]);
    }
}

#ifdef QEMU_TLB_PROFILE
static void dump_tlb_profile(GString *buf)
{
    const size_t default_entries = 256;
    static const char * const origin_name[3] = {
        "helper", "probe", "atomic",
    };
    static const char * const ptw_stage_name[2] = {
        "primary", "nested",
    };
    static const char * const access_name[MMU_ACCESS_COUNT] = {
        [MMU_DATA_LOAD] = "load",
        [MMU_DATA_STORE] = "store",
        [MMU_INST_FETCH] = "fetch",
    };
    size_t l1_miss[MMU_ACCESS_COUNT] = {};
    size_t victim_hit[MMU_ACCESS_COUNT] = {};
    size_t fill_call[MMU_ACCESS_COUNT] = {};
    size_t origin_l1_miss[3][MMU_ACCESS_COUNT] = {};
    size_t origin_victim_hit[3][MMU_ACCESS_COUNT] = {};
    size_t origin_fill_call[3][MMU_ACCESS_COUNT] = {};
    size_t origin_lp_tlb_hit[3][MMU_ACCESS_COUNT] = {};
    size_t fill_page_bits[64] = {};
    size_t ptw_walk[2][MMU_ACCESS_COUNT] = {};
    size_t ptw_full_restart[2][MMU_ACCESS_COUNT] = {};
    size_t ptw_level[2][MMU_ACCESS_COUNT][6] = {};
    size_t fill_install = 0;
    CPUState *cpu;
    int access_type, bits, level, mmu_idx, origin, stage;

    CPU_FOREACH(cpu) {
        for (access_type = 0; access_type < MMU_ACCESS_COUNT; access_type++) {
            l1_miss[access_type] += qatomic_read(
                &cpu->neg.tlb.c.l1_miss_count[access_type]);
            victim_hit[access_type] += qatomic_read(
                &cpu->neg.tlb.c.victim_hit_count[access_type]);
            fill_call[access_type] += qatomic_read(
                &cpu->neg.tlb.c.fill_call_count[access_type]);
            for (stage = 0; stage < ARRAY_SIZE(ptw_stage_name); stage++) {
                ptw_walk[stage][access_type] += qatomic_read(
                    &cpu->neg.tlb.c.ptw_walk_count[stage][access_type]);
                ptw_full_restart[stage][access_type] += qatomic_read(
                    &cpu->neg.tlb.c.ptw_full_restart_count
                    [stage][access_type]);
                for (level = 1; level <= 5; level++) {
                    ptw_level[stage][access_type][level] += qatomic_read(
                        &cpu->neg.tlb.c.ptw_level_count
                        [stage][access_type][level]);
                }
            }
            for (origin = 0; origin < ARRAY_SIZE(origin_name); origin++) {
                origin_l1_miss[origin][access_type] += qatomic_read(
                    &cpu->neg.tlb.c.origin_l1_miss_count[origin][access_type]);
                origin_victim_hit[origin][access_type] += qatomic_read(
                    &cpu->neg.tlb.c.origin_victim_hit_count
                    [origin][access_type]);
                origin_fill_call[origin][access_type] += qatomic_read(
                    &cpu->neg.tlb.c.origin_fill_call_count
                    [origin][access_type]);
                origin_lp_tlb_hit[origin][access_type] += qatomic_read(
                    &cpu->neg.tlb.c.origin_lp_tlb_hit_count
                    [origin][access_type]);
            }
        }
        fill_install += qatomic_read(&cpu->neg.tlb.c.fill_install_count);
        for (bits = 0; bits < ARRAY_SIZE(fill_page_bits); bits++) {
            fill_page_bits[bits] += qatomic_read(
                &cpu->neg.tlb.c.fill_page_bits[bits]);
        }
    }

    g_string_append_printf(buf, "\nSoftMMU TLB profile (cumulative):\n");
    for (access_type = 0; access_type < MMU_ACCESS_COUNT; access_type++) {
        g_string_append_printf(buf,
            "TLB %-5s l1_miss=%zu victim_hit=%zu fill_call=%zu\n",
            access_name[access_type], l1_miss[access_type],
            victim_hit[access_type], fill_call[access_type]);
    }
    for (origin = 0; origin < ARRAY_SIZE(origin_name); origin++) {
        for (access_type = 0; access_type < MMU_ACCESS_COUNT; access_type++) {
            if (origin_l1_miss[origin][access_type] ||
                origin_victim_hit[origin][access_type] ||
                origin_fill_call[origin][access_type] ||
                origin_lp_tlb_hit[origin][access_type]) {
                g_string_append_printf(buf,
                    "TLB origin=%s access=%s l1_miss=%zu "
                    "victim_hit=%zu fill_call=%zu\n",
                    origin_name[origin], access_name[access_type],
                    origin_l1_miss[origin][access_type],
                    origin_victim_hit[origin][access_type],
                    origin_fill_call[origin][access_type]);
                if (origin_lp_tlb_hit[origin][access_type]) {
                    g_string_append_printf(buf,
                        "TLB large-page origin=%s access=%s hit=%zu\n",
                        origin_name[origin], access_name[access_type],
                        origin_lp_tlb_hit[origin][access_type]);
                }
            }
        }
    }
    g_string_append_printf(buf, "TLB fill installs   %zu\n", fill_install);
    g_string_append_printf(buf, "TLB fill page bits ");
    for (bits = 0; bits < ARRAY_SIZE(fill_page_bits); bits++) {
        if (fill_page_bits[bits]) {
            g_string_append_printf(buf, " %d:%zu", bits,
                                   fill_page_bits[bits]);
        }
    }
    g_string_append_c(buf, '\n');

    for (stage = 0; stage < ARRAY_SIZE(ptw_stage_name); stage++) {
        for (access_type = 0; access_type < MMU_ACCESS_COUNT; access_type++) {
            if (ptw_walk[stage][access_type] ||
                ptw_full_restart[stage][access_type]) {
                g_string_append_printf(buf,
                    "PTW stage=%s access=%s walks=%zu full_restarts=%zu "
                    "levels=1:%zu,2:%zu,3:%zu,4:%zu,5:%zu\n",
                    ptw_stage_name[stage], access_name[access_type],
                    ptw_walk[stage][access_type],
                    ptw_full_restart[stage][access_type],
                    ptw_level[stage][access_type][1],
                    ptw_level[stage][access_type][2],
                    ptw_level[stage][access_type][3],
                    ptw_level[stage][access_type][4],
                    ptw_level[stage][access_type][5]);
            }
        }
    }

    CPU_FOREACH(cpu) {
        for (mmu_idx = 0; mmu_idx < NB_MMU_MODES; mmu_idx++) {
            CPUTLBDesc *desc = &cpu->neg.tlb.d[mmu_idx];
            CPUTLBDescFast *fast = &cpu->neg.tlb.f[mmu_idx];
            size_t entries = (fast->mask >> CPU_TLB_ENTRY_BITS) + 1;

            if (desc->n_used_entries || desc->window_max_entries ||
                entries != default_entries) {
                g_string_append_printf(buf,
                    "TLB cpu=%d mmu=%d entries=%zu used=%zu window_max=%zu\n",
                    cpu->cpu_index, mmu_idx, entries,
                    desc->n_used_entries, desc->window_max_entries);
            }
        }
    }
}
#endif

static void tcg_dump_info(GString *buf)
{
    g_string_append_printf(buf, "[TCG profiler not compiled]\n");
}

static void dump_exec_info(GString *buf)
{
    struct tb_tree_stats tst = {};
    struct qht_stats hst;
    size_t nb_tbs, flush_full, flush_part, flush_elide;

    tcg_tb_foreach(tb_tree_stats_iter, &tst);
    nb_tbs = tst.nb_tbs;
    /* XXX: avoid using doubles ? */
    g_string_append_printf(buf, "Translation buffer state:\n");
    /*
     * Report total code size including the padding and TB structs;
     * otherwise users might think "-accel tcg,tb-size" is not honoured.
     * For avg host size we use the precise numbers from tb_tree_stats though.
     */
    g_string_append_printf(buf, "gen code size       %zu/%zu\n",
                           tcg_code_size(), tcg_code_capacity());
    g_string_append_printf(buf, "TB count            %zu\n", nb_tbs);
    g_string_append_printf(buf, "TB avg target size  %zu max=%zu bytes\n",
                           nb_tbs ? tst.target_size / nb_tbs : 0,
                           tst.max_target_size);
    g_string_append_printf(buf, "TB avg host size    %zu bytes "
                           "(expansion ratio: %0.1f)\n",
                           nb_tbs ? tst.host_size / nb_tbs : 0,
                           tst.target_size ?
                           (double)tst.host_size / tst.target_size : 0);
    g_string_append_printf(buf, "cross page TB count %zu (%zu%%)\n",
                           tst.cross_page,
                           nb_tbs ? (tst.cross_page * 100) / nb_tbs : 0);
    g_string_append_printf(buf, "direct jump count   %zu (%zu%%) "
                           "(2 jumps=%zu %zu%%)\n",
                           tst.direct_jmp_count,
                           nb_tbs ? (tst.direct_jmp_count * 100) / nb_tbs : 0,
                           tst.direct_jmp2_count,
                           nb_tbs ? (tst.direct_jmp2_count * 100) / nb_tbs : 0);

    qht_statistics_init(&tb_ctx.htable, &hst);
    print_qht_statistics(hst, buf);
    qht_statistics_destroy(&hst);

    g_string_append_printf(buf, "\nStatistics:\n");
    g_string_append_printf(buf, "TB flush count      %u\n",
                           qatomic_read(&tb_ctx.tb_flush_count));
    g_string_append_printf(buf, "TB invalidate count %u\n",
                           qatomic_read(&tb_ctx.tb_phys_invalidate_count));

    tlb_flush_counts(&flush_full, &flush_part, &flush_elide);
    g_string_append_printf(buf, "TLB full flushes    %zu\n", flush_full);
    g_string_append_printf(buf, "TLB partial flushes %zu\n", flush_part);
    g_string_append_printf(buf, "TLB elided flushes  %zu\n", flush_elide);
    dump_tlb_config(buf);
    dump_lp_tlb(buf);
    dump_ptw_cache(buf);
#ifdef QEMU_TLB_PROFILE
    dump_tlb_profile(buf);
#endif
    tcg_dump_info(buf);
}

HumanReadableText *qmp_x_query_jit(Error **errp)
{
    g_autoptr(GString) buf = g_string_new("");

    if (!tcg_enabled()) {
        error_setg(errp, "JIT information is only available with accel=tcg");
        return NULL;
    }

    dump_accel_info(buf);
    dump_exec_info(buf);
    dump_drift_info(buf);

    return human_readable_text_from_str(buf);
}

static void tcg_dump_op_count(GString *buf)
{
    g_string_append_printf(buf, "[TCG profiler not compiled]\n");
}

HumanReadableText *qmp_x_query_opcount(Error **errp)
{
    g_autoptr(GString) buf = g_string_new("");

    if (!tcg_enabled()) {
        error_setg(errp,
                   "Opcode count information is only available with accel=tcg");
        return NULL;
    }

    tcg_dump_op_count(buf);

    return human_readable_text_from_str(buf);
}

static void hmp_tcg_register(void)
{
    monitor_register_hmp_info_hrt("jit", qmp_x_query_jit);
    monitor_register_hmp_info_hrt("opcount", qmp_x_query_opcount);
}

type_init(hmp_tcg_register);
