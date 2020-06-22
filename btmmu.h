#ifndef QEMU_SPT_H
#define QEMU_SPT_H

#include "qemu/osdep.h"
#include "qapi/error.h"

#ifdef CONFIG_BTMMU
extern int btmmu_allowed;
#define btmmu_enabled() btmmu_allowed
#else
#define btmmu_enabled() (0)
#endif

//#define BTMMU_USER_ONLY 1

extern unsigned long faulting_page;

/* btmmu profiling */
extern int btmmu_qemu_tlb_flush;
extern int btmmu_flush_all_count;
extern int btmmu_flush_page_count;
extern int btmmu_segv_count;
extern int btmmu_map_page_count;
extern int btmmu_unmap_page_count;

extern void btmmu_info_dump(CPUState *cs, const char *name);

struct CPUState;
extern void btmmu_init(void);
extern int btmmu_flush_all(struct CPUState *);
extern int btmmu_flush_page(struct CPUState *, uint64_t page, int mmu_idx, int rw);
extern int btmmu_map_page(struct CPUState *, uint64_t target_addr, uint64_t host_addr, int mmu_idx, int rw);
extern int btmmu_update_tlb(unsigned long tlb_base, int mmu_idx, unsigned long mask);
extern void btmmu_sigsegv_handler(int host_signum, siginfo_t *info, void *puc);

/*** IOCTL structures and commands ***/
struct btmmu_op_arg {
    unsigned long x86_addr; 
    unsigned long host_addr;
    int mid;
    int rw;
};

struct btmmu_update_arg {
    unsigned long base;
    int mmu_idx;
    unsigned long mask;
};

/*
 * ioctl calls that are permitted to the /dev/btmmu interface
 */
#define BTMMU_SET _IOW('b', 0x01, struct btmmu_op_arg) /* Set ftlb entry */
#define BTMMU_DEL _IOW('b', 0x02, struct btmmu_op_arg) /* Del etlb entry */
#define BTMMU_FLUSH _IO('b', 0x03) /* Flush ftlb */
#define BTMMU_UPDATE _IOW('b', 0x04, struct btmmu_update_arg) /* Update qemu TLB info */

#endif
