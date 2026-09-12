/*
 *  x86 exception helpers - sysemu code
 *
 *  Copyright (c) 2003 Fabrice Bellard
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/cpu_ldst.h"
#include "exec/exec-all.h"
#include "tcg/helper-tcg.h"

typedef struct TranslateParams {
    target_ulong addr;
    target_ulong cr3;
    int pg_mode;
    int mmu_idx;
    int ptw_idx;
    MMUAccessType access_type;
#ifdef QEMU_TLB_PROFILE
    int profile_stage;
#endif
} TranslateParams;

typedef struct TranslateResult {
    hwaddr paddr;
    int prot;
    int page_size;
    int translation_size;
} TranslateResult;

typedef enum TranslateFaultStage2 {
    S2_NONE,
    S2_GPA,
    S2_GPT,
} TranslateFaultStage2;

typedef struct TranslateFault {
    int exception_index;
    int error_code;
    target_ulong cr2;
    TranslateFaultStage2 stage2;
} TranslateFault;

typedef struct PTETranslate {
    CPUX86State *env;
    TranslateFault *err;
    int ptw_idx;
#ifdef QEMU_TLB_PROFILE
    MMUAccessType access_type;
    int profile_stage;
#endif
    void *haddr;
    hwaddr gaddr;
} PTETranslate;

#ifdef QEMU_TLB_PROFILE
static bool ptw_translate(PTETranslate *inout, hwaddr addr, int level,
                          uint64_t ra)
#else
static bool ptw_translate(PTETranslate *inout, hwaddr addr, uint64_t ra)
#endif
{
    int flags;

#ifdef QEMU_TLB_PROFILE
    qatomic_inc(&env_cpu(inout->env)->neg.tlb.c.ptw_level_count
                [inout->profile_stage][inout->access_type][level]);
#endif

    inout->gaddr = addr;
    flags = probe_access_full_mmu(inout->env, addr, 0, MMU_DATA_STORE,
                                  inout->ptw_idx, &inout->haddr, NULL);

    if (unlikely(flags & TLB_INVALID_MASK)) {
        TranslateFault *err = inout->err;

        assert(inout->ptw_idx == MMU_NESTED_IDX);
        *err = (TranslateFault){
            .error_code = inout->env->error_code,
            .cr2 = addr,
            .stage2 = S2_GPT,
        };
        return false;
    }
    return true;
}

#ifdef QEMU_TLB_PROFILE
#define PTW_TRANSLATE(inout, addr, level, ra) \
    ptw_translate(inout, addr, level, ra)
#else
#define PTW_TRANSLATE(inout, addr, level, ra) ptw_translate(inout, addr, ra)
#endif

static inline uint32_t ptw_ldl(const PTETranslate *in, uint64_t ra)
{
    if (likely(in->haddr)) {
        return ldl_p(in->haddr);
    }
    return cpu_ldl_mmuidx_ra(in->env, in->gaddr, in->ptw_idx, ra);
}

static inline uint64_t ptw_ldq(const PTETranslate *in, uint64_t ra)
{
    if (likely(in->haddr)) {
        return ldq_p(in->haddr);
    }
    return cpu_ldq_mmuidx_ra(in->env, in->gaddr, in->ptw_idx, ra);
}

/*
 * Note that we can use a 32-bit cmpxchg for all page table entries,
 * even 64-bit ones, because PG_PRESENT_MASK, PG_ACCESSED_MASK and
 * PG_DIRTY_MASK are all in the low 32 bits.
 */
static bool ptw_setl_slow(const PTETranslate *in, uint32_t old, uint32_t new)
{
    uint32_t cmp;

    CPUState *cpu = env_cpu(in->env);
    /* We are in cpu_exec, and start_exclusive can't be called directly.*/
    g_assert(cpu->running);
    cpu_exec_end(cpu);
    /* Does x86 really perform a rmw cycle on mmio for ptw? */
    start_exclusive();
    cmp = cpu_ldl_mmuidx_ra(in->env, in->gaddr, in->ptw_idx, 0);
    if (cmp == old) {
        cpu_stl_mmuidx_ra(in->env, in->gaddr, new, in->ptw_idx, 0);
    }
    end_exclusive();
    cpu_exec_start(cpu);
    return cmp == old;
}

static inline bool ptw_setl(const PTETranslate *in, uint32_t old, uint32_t set)
{
    if (set & ~old) {
        uint32_t new = old | set;
        if (likely(in->haddr)) {
            old = cpu_to_le32(old);
            new = cpu_to_le32(new);
            return qatomic_cmpxchg((uint32_t *)in->haddr, old, new) == old;
        }
        return ptw_setl_slow(in, old, new);
    }
    return true;
}

static unsigned ptw_cache_shift(unsigned level)
{
    static const uint8_t shift[5] = { [2] = 21, [3] = 30, [4] = 39 };

    return shift[level];
}

static unsigned ptw_cache_hash(const TranslateParams *in, unsigned level)
{
    uint64_t key = in->cr3 ^ (in->addr >> ptw_cache_shift(level));

    key ^= (uint64_t)in->pg_mode << 7;
    key ^= (uint64_t)in->mmu_idx << 3;
    key ^= (uint64_t)in->ptw_idx << 11;
    key ^= level;
    key ^= key >> 17;
    key ^= key >> 9;
    return key & (X86_PTW_CACHE_SETS - 1);
}

static bool ptw_cache_entry_matches(const X86PTWCacheEntry *entry,
                                    const TranslateParams *in,
                                    uint64_t generation, unsigned level)
{
    return entry->generation == generation && entry->level == level &&
           entry->cr3 == in->cr3 && entry->pg_mode == in->pg_mode &&
           entry->mmu_idx == in->mmu_idx && entry->ptw_idx == in->ptw_idx &&
           entry->vaddr_prefix == in->addr >> ptw_cache_shift(level);
}

static unsigned ptw_cache_lookup(CPUX86State *env, const TranslateParams *in,
                                 hwaddr *next_table, uint64_t *ptep)
{
    CPUTLBCommon *common = &env_cpu(env)->neg.tlb.c;
    uint64_t generation = common->ptw_cache_generation;
    unsigned level;

    if (common->ptw_cache_mode == CPU_TLB_CACHE_OFF) {
        return 0;
    }
    if (common->ptw_cache_mode == CPU_TLB_CACHE_ADAPTIVE &&
        !cpu_tlb_cache_adaptive_begin(&common->ptw_cache_adaptive)) {
        return 0;
    }
    for (level = 2; level <= 4; level++) {
        unsigned set = ptw_cache_hash(in, level);
        unsigned way;

#ifdef QEMU_TLB_PROFILE
        qatomic_inc(&common->ptw_cache_lookup_count[level]);
#endif
        for (way = 0; way < X86_PTW_CACHE_WAYS; way++) {
            X86PTWCacheEntry *entry = &env->ptw_cache[set][way];

            if (!ptw_cache_entry_matches(entry, in, generation, level)) {
                continue;
            }
#ifdef QEMU_TLB_PROFILE
            qatomic_inc(&common->ptw_cache_match_count[level]);
#endif
            if (common->ptw_cache_mode == CPU_TLB_CACHE_PROBE) {
                return 0;
            }
            if (common->ptw_cache_mode == CPU_TLB_CACHE_ADAPTIVE) {
                /* A level-N hit avoids 5 - N non-leaf walk levels. */
                cpu_tlb_cache_adaptive_complete(
                    &common->ptw_cache_adaptive, 5 - level, 3);
            }
            *next_table = entry->next_table;
            *ptep = entry->ptep;
#ifdef QEMU_TLB_PROFILE
            qatomic_inc(&common->ptw_cache_hit_count[level]);
#endif
            return level;
        }
    }
    if (common->ptw_cache_mode == CPU_TLB_CACHE_ADAPTIVE) {
        /* Require one avoided walk level per eight lookups. */
        cpu_tlb_cache_adaptive_complete(
            &common->ptw_cache_adaptive, 0, 3);
    }
    return 0;
}

static void ptw_cache_insert(CPUX86State *env, const TranslateParams *in,
                             unsigned level, hwaddr next_table, uint64_t ptep)
{
    CPUTLBCommon *common = &env_cpu(env)->neg.tlb.c;
    uint64_t generation = common->ptw_cache_generation;
    X86PTWCacheEntry *entry = NULL;
    unsigned set, way;

    if (common->ptw_cache_mode == CPU_TLB_CACHE_OFF ||
        (common->ptw_cache_mode == CPU_TLB_CACHE_ADAPTIVE &&
         common->ptw_cache_adaptive.phase == CPU_TLB_ADAPTIVE_BYPASS)) {
        return;
    }
    set = ptw_cache_hash(in, level);
    for (way = 0; way < X86_PTW_CACHE_WAYS; way++) {
        X86PTWCacheEntry *candidate = &env->ptw_cache[set][way];

        if (ptw_cache_entry_matches(candidate, in, generation, level)) {
            entry = candidate;
            break;
        }
        if (!entry && candidate->generation != generation) {
            entry = candidate;
        }
    }
    if (!entry) {
        way = env->ptw_cache_next[set]++ % X86_PTW_CACHE_WAYS;
        entry = &env->ptw_cache[set][way];
#ifdef QEMU_TLB_PROFILE
        qatomic_inc(&common->ptw_cache_evict_count[level]);
#endif
    }
    *entry = (X86PTWCacheEntry) {
        .generation = generation,
        .cr3 = in->cr3,
        .pg_mode = in->pg_mode,
        .ptep = ptep,
        .vaddr_prefix = in->addr >> ptw_cache_shift(level),
        .next_table = next_table,
        .mmu_idx = in->mmu_idx,
        .ptw_idx = in->ptw_idx,
        .level = level,
    };
#ifdef QEMU_TLB_PROFILE
    qatomic_inc(&common->ptw_cache_insert_count[level]);
#endif
}

static bool mmu_translate(CPUX86State *env, const TranslateParams *in,
                          TranslateResult *out, TranslateFault *err,
                          uint64_t ra)
{
    const target_ulong addr = in->addr;
    const int pg_mode = in->pg_mode;
    const bool is_user = is_mmu_index_user(in->mmu_idx);
    const MMUAccessType access_type = in->access_type;
    uint64_t ptep, pte, rsvd_mask;
    PTETranslate pte_trans = {
        .env = env,
        .err = err,
        .ptw_idx = in->ptw_idx,
#ifdef QEMU_TLB_PROFILE
        .access_type = access_type,
        .profile_stage = in->profile_stage,
#endif
    };
    hwaddr pte_addr, paddr;
    uint32_t pkr;
    unsigned cached_level = 0;
    hwaddr cached_next_table = 0;
    uint64_t cached_ptep = 0;
    int page_size, translation_size;
    int error_code;
    int prot;
#ifdef QEMU_TLB_PROFILE
    bool restarted = false;
    int profile_stage = in->profile_stage;

    /*
     * nested_pg_mode contains only PG_MODE_SVM_MASK and therefore omits
     * PG_MODE_PG, even though MMU_NESTED_IDX always performs an NPT walk.
     */
    if ((pg_mode & PG_MODE_PG) || profile_stage) {
        qatomic_inc(&env_cpu(env)->neg.tlb.c.ptw_walk_count
                    [profile_stage][access_type]);
    }
#endif

 restart_all:
#ifdef QEMU_TLB_PROFILE
    if (restarted) {
        qatomic_inc(&env_cpu(env)->neg.tlb.c.ptw_full_restart_count
                    [profile_stage][access_type]);
    }
    restarted = true;
#endif
    rsvd_mask = ~MAKE_64BIT_MASK(0, env_archcpu(env)->phys_bits);
    rsvd_mask &= PG_ADDRESS_MASK;
    if (!(pg_mode & PG_MODE_NXE)) {
        rsvd_mask |= PG_NX_MASK;
    }

    if (pg_mode & PG_MODE_PAE) {
#ifdef TARGET_X86_64
        if (pg_mode & PG_MODE_LMA) {
            if (pg_mode & PG_MODE_LA57) {
                /*
                 * Page table level 5
                 */
                pte_addr = (in->cr3 & ~0xfff) + (((addr >> 48) & 0x1ff) << 3);
                if (!PTW_TRANSLATE(&pte_trans, pte_addr, 5, ra)) {
                    return false;
                }
            restart_5:
                pte = ptw_ldq(&pte_trans, ra);
                if (!(pte & PG_PRESENT_MASK)) {
                    goto do_fault;
                }
                if (pte & (rsvd_mask | PG_PSE_MASK)) {
                    goto do_fault_rsvd;
                }
                if (!ptw_setl(&pte_trans, pte, PG_ACCESSED_MASK)) {
                    goto restart_5;
                }
                ptep = pte ^ PG_NX_MASK;
            } else {
                pte = in->cr3;
                ptep = PG_NX_MASK | PG_USER_MASK | PG_RW_MASK;
            }

            cached_level = ptw_cache_lookup(env, in, &cached_next_table,
                                            &cached_ptep);
            if (cached_level) {
                pte = cached_next_table;
                /* Retain the freshly loaded level-5 permissions under LA57. */
                ptep &= cached_ptep;
                if (cached_level == 2) {
                    goto walk_level_1;
                } else if (cached_level == 3) {
                    goto walk_level_2;
                }
                goto walk_level_3;
            }

            /*
             * Page table level 4
             */
            pte_addr = (pte & PG_ADDRESS_MASK) + (((addr >> 39) & 0x1ff) << 3);
            if (!PTW_TRANSLATE(&pte_trans, pte_addr, 4, ra)) {
                return false;
            }
        restart_4:
            pte = ptw_ldq(&pte_trans, ra);
            if (!(pte & PG_PRESENT_MASK)) {
                goto do_fault;
            }
            if (pte & (rsvd_mask | PG_PSE_MASK)) {
                goto do_fault_rsvd;
            }
            if (!ptw_setl(&pte_trans, pte, PG_ACCESSED_MASK)) {
                goto restart_4;
            }
            ptep &= pte ^ PG_NX_MASK;
            ptw_cache_insert(env, in, 4, pte & PG_ADDRESS_MASK, ptep);

        walk_level_3:
            /*
             * Page table level 3
             */
            pte_addr = (pte & PG_ADDRESS_MASK) + (((addr >> 30) & 0x1ff) << 3);
            if (!PTW_TRANSLATE(&pte_trans, pte_addr, 3, ra)) {
                return false;
            }
        restart_3_lma:
            pte = ptw_ldq(&pte_trans, ra);
            if (!(pte & PG_PRESENT_MASK)) {
                goto do_fault;
            }
            if (pte & rsvd_mask) {
                goto do_fault_rsvd;
            }
            if (!ptw_setl(&pte_trans, pte, PG_ACCESSED_MASK)) {
                goto restart_3_lma;
            }
            ptep &= pte ^ PG_NX_MASK;
            if (pte & PG_PSE_MASK) {
                /* 1 GB page */
                page_size = 1024 * 1024 * 1024;
                goto do_check_protect;
            }
            ptw_cache_insert(env, in, 3, pte & PG_ADDRESS_MASK, ptep);
        } else
#endif
        {
            /*
             * Page table level 3
             */
            pte_addr = (in->cr3 & 0xffffffe0ULL) + ((addr >> 27) & 0x18);
            if (!PTW_TRANSLATE(&pte_trans, pte_addr, 3, ra)) {
                return false;
            }
            rsvd_mask |= PG_HI_USER_MASK;
        restart_3_nolma:
            pte = ptw_ldq(&pte_trans, ra);
            if (!(pte & PG_PRESENT_MASK)) {
                goto do_fault;
            }
            if (pte & (rsvd_mask | PG_NX_MASK)) {
                goto do_fault_rsvd;
            }
            if (!ptw_setl(&pte_trans, pte, PG_ACCESSED_MASK)) {
                goto restart_3_nolma;
            }
            ptep = PG_NX_MASK | PG_USER_MASK | PG_RW_MASK;
        }

    walk_level_2:
        /*
         * Page table level 2
         */
        pte_addr = (pte & PG_ADDRESS_MASK) + (((addr >> 21) & 0x1ff) << 3);
        if (!PTW_TRANSLATE(&pte_trans, pte_addr, 2, ra)) {
            return false;
        }
    restart_2_pae:
        pte = ptw_ldq(&pte_trans, ra);
        if (!(pte & PG_PRESENT_MASK)) {
            goto do_fault;
        }
        if (pte & rsvd_mask) {
            goto do_fault_rsvd;
        }
        if (pte & PG_PSE_MASK) {
            /* 2 MB page */
            page_size = 2048 * 1024;
            ptep &= pte ^ PG_NX_MASK;
            goto do_check_protect;
        }
        if (!ptw_setl(&pte_trans, pte, PG_ACCESSED_MASK)) {
            goto restart_2_pae;
        }
        ptep &= pte ^ PG_NX_MASK;
        if (pg_mode & PG_MODE_LMA) {
            ptw_cache_insert(env, in, 2, pte & PG_ADDRESS_MASK, ptep);
        }

    walk_level_1:
        /*
         * Page table level 1
         */
        pte_addr = (pte & PG_ADDRESS_MASK) + (((addr >> 12) & 0x1ff) << 3);
        if (!PTW_TRANSLATE(&pte_trans, pte_addr, 1, ra)) {
            return false;
        }
        pte = ptw_ldq(&pte_trans, ra);
        if (!(pte & PG_PRESENT_MASK)) {
            goto do_fault;
        }
        if (pte & rsvd_mask) {
            goto do_fault_rsvd;
        }
        /* combine pde and pte nx, user and rw protections */
        ptep &= pte ^ PG_NX_MASK;
        page_size = 4096;
    } else if (pg_mode & PG_MODE_PG) {
        /*
         * Page table level 2
         */
        pte_addr = (in->cr3 & 0xfffff000ULL) + ((addr >> 20) & 0xffc);
        if (!PTW_TRANSLATE(&pte_trans, pte_addr, 2, ra)) {
            return false;
        }
    restart_2_nopae:
        pte = ptw_ldl(&pte_trans, ra);
        if (!(pte & PG_PRESENT_MASK)) {
            goto do_fault;
        }
        ptep = pte | PG_NX_MASK;

        /* if PSE bit is set, then we use a 4MB page */
        if ((pte & PG_PSE_MASK) && (pg_mode & PG_MODE_PSE)) {
            page_size = 4096 * 1024;
            /*
             * Bits 20-13 provide bits 39-32 of the address, bit 21 is reserved.
             * Leave bits 20-13 in place for setting accessed/dirty bits below.
             */
            pte = (uint32_t)pte | ((pte & 0x1fe000LL) << (32 - 13));
            rsvd_mask = 0x200000;
            goto do_check_protect_pse36;
        }
        if (!ptw_setl(&pte_trans, pte, PG_ACCESSED_MASK)) {
            goto restart_2_nopae;
        }

        /*
         * Page table level 1
         */
        pte_addr = (pte & ~0xfffu) + ((addr >> 10) & 0xffc);
        if (!PTW_TRANSLATE(&pte_trans, pte_addr, 1, ra)) {
            return false;
        }
        pte = ptw_ldl(&pte_trans, ra);
        if (!(pte & PG_PRESENT_MASK)) {
            goto do_fault;
        }
        /* combine pde and pte user and rw protections */
        ptep &= pte | PG_NX_MASK;
        page_size = 4096;
        rsvd_mask = 0;
    } else {
        /*
         * No paging (real mode), let's tentatively resolve the address as 1:1
         * here, but conditionally still perform an NPT walk on it later.
         */
        page_size = 0x40000000;
        translation_size = TARGET_PAGE_SIZE;
        paddr = in->addr;
        prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
        goto stage2;
    }

do_check_protect:
    rsvd_mask |= (page_size - 1) & PG_ADDRESS_MASK & ~PG_PSE_PAT_MASK;
do_check_protect_pse36:
    if (pte & rsvd_mask) {
        goto do_fault_rsvd;
    }
    ptep ^= PG_NX_MASK;

    /* can the page can be put in the TLB?  prot will tell us */
    if (is_user && !(ptep & PG_USER_MASK)) {
        goto do_fault_protect;
    }

    prot = 0;
    if (!is_mmu_index_smap(in->mmu_idx) || !(ptep & PG_USER_MASK)) {
        prot |= PAGE_READ;
        if ((ptep & PG_RW_MASK) || !(is_user || (pg_mode & PG_MODE_WP))) {
            prot |= PAGE_WRITE;
        }
    }
    if (!(ptep & PG_NX_MASK) &&
        (is_user ||
         !((pg_mode & PG_MODE_SMEP) && (ptep & PG_USER_MASK)))) {
        prot |= PAGE_EXEC;
    }

    if (ptep & PG_USER_MASK) {
        pkr = pg_mode & PG_MODE_PKE ? env->pkru : 0;
    } else {
        pkr = pg_mode & PG_MODE_PKS ? env->pkrs : 0;
    }
    if (pkr) {
        uint32_t pk = (pte & PG_PKRU_MASK) >> PG_PKRU_BIT;
        uint32_t pkr_ad = (pkr >> pk * 2) & 1;
        uint32_t pkr_wd = (pkr >> pk * 2) & 2;
        uint32_t pkr_prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;

        if (pkr_ad) {
            pkr_prot &= ~(PAGE_READ | PAGE_WRITE);
        } else if (pkr_wd && (is_user || (pg_mode & PG_MODE_WP))) {
            pkr_prot &= ~PAGE_WRITE;
        }
        if ((pkr_prot & (1 << access_type)) == 0) {
            goto do_fault_pk_protect;
        }
        prot &= pkr_prot;
    }

    if ((prot & (1 << access_type)) == 0) {
        goto do_fault_protect;
    }

    /* yes, it can! */
    {
        uint32_t set = PG_ACCESSED_MASK;
        if (access_type == MMU_DATA_STORE) {
            set |= PG_DIRTY_MASK;
        } else if (!(pte & PG_DIRTY_MASK)) {
            /*
             * Only set write access if already dirty...
             * otherwise wait for dirty access.
             */
            prot &= ~PAGE_WRITE;
        }
        if (!ptw_setl(&pte_trans, pte, set)) {
            /*
             * We can arrive here from any of 3 levels and 2 formats.
             * The only safe thing is to restart the entire lookup.
             */
            goto restart_all;
        }
    }

    /* merge offset within page */
    paddr = (pte & PG_ADDRESS_MASK & ~(page_size - 1)) | (addr & (page_size - 1));
    translation_size = page_size;
 stage2:

    /*
     * Note that NPT is walked (for both paging structures and final guest
     * addresses) using the address with the A20 bit set.
     */
    if (in->ptw_idx == MMU_NESTED_IDX) {
        CPUTLBEntryFull *full;
        int flags, nested_page_size, nested_translation_size;

        flags = probe_access_full_mmu(env, paddr, 0, access_type,
                                      MMU_NESTED_IDX, &pte_trans.haddr, &full);
        if (unlikely(flags & TLB_INVALID_MASK)) {
            *err = (TranslateFault){
                .error_code = env->error_code,
                .cr2 = paddr,
                .stage2 = S2_GPA,
            };
            return false;
        }

        /* Merge stage1 & stage2 protection bits. */
        prot &= full->prot;

        /* Re-verify resulting protection. */
        if ((prot & (1 << access_type)) == 0) {
            goto do_fault_protect;
        }

        /* Merge stage1 & stage2 addresses to final physical address. */
        nested_page_size = 1 << full->lg_page_size;
        nested_translation_size = 1 << full->lg_translation_size;
        paddr = (full->phys_addr & ~(nested_page_size - 1))
              | (paddr & (nested_page_size - 1));

        /* Only the intersection of both linear mappings is reusable. */
        translation_size = MIN(translation_size, nested_translation_size);

        /*
         * Use the larger of stage1 & stage2 page sizes, so that
         * invalidation works.
         */
        if (nested_page_size > page_size) {
            page_size = nested_page_size;
        }
    }

    out->paddr = paddr & x86_get_a20_mask(env);
    out->prot = prot;
    out->page_size = page_size;
    out->translation_size = translation_size;
    return true;

 do_fault_rsvd:
    error_code = PG_ERROR_RSVD_MASK;
    goto do_fault_cont;
 do_fault_protect:
    error_code = PG_ERROR_P_MASK;
    goto do_fault_cont;
 do_fault_pk_protect:
    assert(access_type != MMU_INST_FETCH);
    error_code = PG_ERROR_PK_MASK | PG_ERROR_P_MASK;
    goto do_fault_cont;
 do_fault:
    error_code = 0;
 do_fault_cont:
    if (is_user) {
        error_code |= PG_ERROR_U_MASK;
    }
    switch (access_type) {
    case MMU_DATA_LOAD:
        break;
    case MMU_DATA_STORE:
        error_code |= PG_ERROR_W_MASK;
        break;
    case MMU_INST_FETCH:
        if (pg_mode & (PG_MODE_NXE | PG_MODE_SMEP)) {
            error_code |= PG_ERROR_I_D_MASK;
        }
        break;
    }
    *err = (TranslateFault){
        .exception_index = EXCP0E_PAGE,
        .error_code = error_code,
        .cr2 = addr,
    };
    return false;
}

static G_NORETURN void raise_stage2(CPUX86State *env, TranslateFault *err,
                                    uintptr_t retaddr)
{
    uint64_t exit_info_1 = err->error_code;

    switch (err->stage2) {
    case S2_GPT:
        exit_info_1 |= SVM_NPTEXIT_GPT;
        break;
    case S2_GPA:
        exit_info_1 |= SVM_NPTEXIT_GPA;
        break;
    default:
        g_assert_not_reached();
    }

    x86_stq_phys(env_cpu(env),
                 env->vm_vmcb + offsetof(struct vmcb, control.exit_info_2),
                 err->cr2);
    cpu_vmexit(env, SVM_EXIT_NPF, exit_info_1, retaddr);
}

static bool get_physical_address(CPUX86State *env, vaddr addr,
                                 MMUAccessType access_type, int mmu_idx,
                                 TranslateResult *out, TranslateFault *err,
                                 uint64_t ra)
{
    TranslateParams in;
    bool use_stage2 = env->hflags2 & HF2_NPT_MASK;

    in.addr = addr;
    in.access_type = access_type;
#ifdef QEMU_TLB_PROFILE
    /* Preserve the requested translation stage before mmu_idx is rewritten. */
    in.profile_stage = mmu_idx == MMU_NESTED_IDX;
#endif

    switch (mmu_idx) {
    case MMU_PHYS_IDX:
        break;

    case MMU_NESTED_IDX:
        if (likely(use_stage2)) {
            in.cr3 = env->nested_cr3;
            in.pg_mode = env->nested_pg_mode;
            in.mmu_idx =
                env->nested_pg_mode & PG_MODE_LMA ? MMU_USER64_IDX : MMU_USER32_IDX;
            in.ptw_idx = MMU_PHYS_IDX;

            if (!mmu_translate(env, &in, out, err, ra)) {
                err->stage2 = S2_GPA;
                return false;
            }
            return true;
        }
        break;

    default:
        if (is_mmu_index_32(mmu_idx)) {
            addr = (uint32_t)addr;
        }

        if (likely(env->cr[0] & CR0_PG_MASK || use_stage2)) {
            in.cr3 = env->cr[3];
            in.mmu_idx = mmu_idx;
            in.ptw_idx = use_stage2 ? MMU_NESTED_IDX : MMU_PHYS_IDX;
            in.pg_mode = get_pg_mode(env);

            if (in.pg_mode & PG_MODE_LMA) {
                /* test virtual address sign extension */
                int shift = in.pg_mode & PG_MODE_LA57 ? 56 : 47;
                int64_t sext = (int64_t)addr >> shift;
                if (sext != 0 && sext != -1) {
                    *err = (TranslateFault){
                        .exception_index = EXCP0D_GPF,
                        .cr2 = addr,
                    };
                    return false;
                }
            }
            return mmu_translate(env, &in, out, err, ra);
        }
        break;
    }

    /* No translation needed. */
    out->paddr = addr & x86_get_a20_mask(env);
    out->prot = PAGE_READ | PAGE_WRITE | PAGE_EXEC;
    out->page_size = TARGET_PAGE_SIZE;
    out->translation_size = TARGET_PAGE_SIZE;
    return true;
}

bool x86_cpu_tlb_fill(CPUState *cs, vaddr addr, int size,
                      MMUAccessType access_type, int mmu_idx,
                      bool probe, uintptr_t retaddr)
{
    CPUX86State *env = cpu_env(cs);
    TranslateResult out;
    TranslateFault err;

    if (get_physical_address(env, addr, access_type, mmu_idx, &out, &err,
                             retaddr)) {
        CPUTLBEntryFull full = {
            .phys_addr = out.paddr & TARGET_PAGE_MASK,
            .attrs = cpu_get_mem_attrs(env),
            .prot = out.prot,
            .lg_page_size = ctz64(out.page_size),
            .lg_translation_size = ctz64(out.translation_size),
        };

        /*
         * Even if 4MB pages, we map only one 4KB page in the cache to
         * avoid filling it too fast.
         */
        assert(out.prot & (1 << access_type));
        tlb_set_page_full(cs, mmu_idx, addr & TARGET_PAGE_MASK, &full);
        return true;
    }

    if (probe) {
        /* This will be used if recursing for stage2 translation. */
        env->error_code = err.error_code;
        return false;
    }

    if (err.stage2 != S2_NONE) {
        raise_stage2(env, &err, retaddr);
    }

    if (env->intercept_exceptions & (1 << err.exception_index)) {
        /* cr2 is not modified in case of exceptions */
        x86_stq_phys(cs, env->vm_vmcb +
                     offsetof(struct vmcb, control.exit_info_2),
                     err.cr2);
    } else {
        env->cr[2] = err.cr2;
    }
    raise_exception_err_ra(env, err.exception_index, err.error_code, retaddr);
}

G_NORETURN void x86_cpu_do_unaligned_access(CPUState *cs, vaddr vaddr,
                                            MMUAccessType access_type,
                                            int mmu_idx, uintptr_t retaddr)
{
    X86CPU *cpu = X86_CPU(cs);
    handle_unaligned_access(&cpu->env, vaddr, access_type, retaddr);
}
