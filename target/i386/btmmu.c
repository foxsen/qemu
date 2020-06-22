#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include <stdarg.h>
#include <errno.h>
#include <assert.h>
#include <sys/ucontext.h>
#include <sys/resource.h>
#include "btmmu.h"
#include "cpu.h"
#include "tcg/tcg.h"
#include "qemu-common.h"
#include "qemu/qemu-print.h"
#include "exec/cpu_ldst.h"
#include "exec/cpu-all.h"

int btmmu_allowed = false;
volatile int btmmu_debug = 0;
unsigned long faulting_page = 0;

/* btmmu profiling */
int btmmu_qemu_tlb_flush       = 0;
int btmmu_flush_all_count      = 0;
int btmmu_flush_page_count     = 0;
int btmmu_segv_count           = 0;
int btmmu_map_page_count       = 0;
int btmmu_unmap_page_count     = 0;

static int btmmu_fd = -1;

#define CHECK_FD(fd)	\
	do {		\
		if (fd <= 0)	\
			return -1;	\
	} while(0)

static int btmmu_open(void)
{
	/* check /dev/btmmu */
	btmmu_fd = open("/dev/btmmu", O_RDWR);
	if (btmmu_fd < 0)
	{
		qemu_printf("[qemu] /dev/btmmu Open Error!\n");
		return -1;
	}
    qemu_printf("[qemu] /dev/btmmu Opened!\n");
	return btmmu_fd;
}

static int btmmu_ftlb_flush(void)
{
    int ret;
	CHECK_FD(btmmu_fd);
    ret = ioctl(btmmu_fd, BTMMU_FLUSH);
    return ret;
}

static int btmmu_ftlb_set(uint64_t target_addr, uint64_t host_addr, int mid, int rw)
{
    int ret;
    struct btmmu_op_arg arg;

	CHECK_FD(btmmu_fd);

    btmmu_map_page_count++;

    if (btmmu_debug == 1) {
        qemu_printf("%lx %lx %x %x\n", target_addr, host_addr, mid, rw);
    }

    arg.x86_addr = target_addr;
    arg.host_addr = host_addr;
    arg.mid = mid;
    arg.rw = rw;
	
    ret = ioctl(btmmu_fd, BTMMU_SET, (void*)&arg);
	return ret;
}

static int btmmu_ftlb_del(uint64_t target_addr, int mid, int rw)
{
    int ret;
    struct btmmu_op_arg arg;

	CHECK_FD(btmmu_fd);

    btmmu_unmap_page_count++;

    arg.x86_addr = target_addr;
    arg.mid = mid;
    arg.rw = rw;
    ret = ioctl(btmmu_fd, BTMMU_DEL, (void*)&arg);
    return ret;
}

/*
static void btmmu_close(void)
{
	CHECK_FD(btmmu_fd);
	close(btmmu_fd);
}
*/

void btmmu_init(void)
{	
    btmmu_open();
}

int btmmu_flush_all(CPUState *cpu) 
{
    /* btmmu profiling */
    btmmu_flush_all_count++;

    if (btmmu_debug == 1 ) 
        qemu_printf("btmmu flush\n");

    return btmmu_ftlb_flush();
}

int btmmu_flush_page(CPUState *cpu, uint64_t page, int mmu_idx, int rw) 
{
#ifdef BTMMU_USER_ONLY
    if (mmu_idx != MMU_USER_IDX) return;
#endif
    if (btmmu_debug == 1 ) 
        qemu_printf("btmmu flush page %lx\n", page);

    /* btmmu profiling */
    btmmu_flush_page_count++;

    /* todo. We assume no ksmap now */
    if (mmu_idx == MMU_KSMAP_IDX) {
        return -1;
    }

    return btmmu_ftlb_del(page, mmu_idx == MMU_USER_IDX ? 2 : 3, rw);
}

int btmmu_map_page(CPUState *cpu, uint64_t target_addr, uint64_t host_addr, int mmu_idx, int rw)
{   
#ifdef BTMMU_USER_ONLY
    if (mmu_idx != MMU_USER_IDX) return;
#endif

    if (btmmu_debug == 1 ) 
        qemu_printf("btmmu map page %lx %lx\n", target_addr, host_addr);

    /* btmmu profiling */
    btmmu_map_page_count++;

    /* todo. We assume no ksmap now */
    if (mmu_idx == MMU_KSMAP_IDX) {
        qemu_printf("No ksmap support now!\n");
        assert(0);
    }

    return btmmu_ftlb_set(target_addr, host_addr, mmu_idx == MMU_USER_IDX ? 2 : 3, rw);
}

#if defined(HOST_MIPS)

#define PC_sig(context)       ((context)->uc_mcontext.pc)
#define TRAP_sig(info)        ((info)->si_signo)
#define ERROR_sig(info)       ((info)->si_errno)
#define MASK_sig(context)     ((context)->uc_sigmask)
#define PC_skip(context)      (PC_sig(context) += 16)

#else

#error "not supported host"

#endif

static bool in_native_code(unsigned long pc) 
{
    if( pc >= (unsigned long)tcg_ctx->code_gen_buffer && 
        pc < (unsigned long)tcg_ctx->code_gen_ptr)
        return true;
    else {
        return false;
    }
}

void btmmu_sigsegv_handler(int host_signum, siginfo_t *info, void *puc)
{
    /* btmmu profiling */
    btmmu_segv_count++;

    //handle SIGSEGV
    ucontext_t *uc = puc;

    unsigned long address = (unsigned long)info->si_addr;
    unsigned long pc = PC_sig(uc);

	unsigned int *pre_pc = (unsigned int *)(long)(uc->uc_mcontext.pc);
	unsigned int pre_inst = *pre_pc;

    // Analysis SpaceID
	int is_setmem = (pre_inst & 0xfff00000) == ((0x12 << 26) | (0xf << 21));
	int mid = (pre_inst >> 18) & 0x3;

	if (!(is_setmem && (mid > 1))) {
		printf("[handler] Illegal mid in SIGSEGV, abort\n");
        assert(0);
	}

    if(!in_native_code(pc)) {
        fprintf(stderr, "\nSEGV out of CODE CACHE!! pc: 0x%lx, mem: 0x%lx!\n",
                pc, address);
		assert(0);
    }

    int cause = (ERROR_sig(info) >> 2) & 0x1f;
    if (btmmu_debug == 1)
        qemu_printf("x86_addr %lx cause %x\n", address, cause);

    int mmu_idx = mid - 1;
    uint32_t target_addr = (uint32_t)address;
    if (cause >=1 && cause <= 3) {
        /* TLBMOD/TLBL/TLBS */
        CPUArchState *env = (CPUArchState*)current_cpu->env_ptr;
        CPUTLBEntry *entry = tlb_entry(env, mmu_idx, address);
        target_ulong tlb_addr =  cause == 2 ? entry->addr_read : entry->addr_write;
        if (((tlb_addr & 0xfff) == 0) && tlb_hit(tlb_addr, target_addr)) {
            // hit, refill the ftlb
            if (btmmu_debug == 1)
                qemu_printf("x86_addr %x, mips_addr %lx\n", target_addr, target_addr + entry->addend); 
            //btmmu_ftlb_set(address, target_addr + entry->addend, mid, entry->addr_write == (target_addr & ~0xfff));
            btmmu_ftlb_set(tlb_addr, tlb_addr + entry->addend, mid, entry->addr_write == (target_addr & ~0xfff));
        }
    }

    //cannot refill, skip the next few instructions to fall back to softmmu code
    PC_skip(uc);
}

void btmmu_info_dump(CPUState *cs, const char *name)
{
    if (strcmp(name, "stat") == 0) {
        qemu_printf("tlb flush count = %d\n", btmmu_qemu_tlb_flush);
        qemu_printf("tlb flush all count = %d\n", btmmu_flush_all_count);
        qemu_printf("tlb flush page count = %d\n", btmmu_flush_page_count);
        qemu_printf("tlb segv count = %d\n", btmmu_segv_count);
        qemu_printf("tlb map page count = %d\n", btmmu_map_page_count);
        qemu_printf("tlb unmap page count = %d\n", btmmu_unmap_page_count);
    } else {
       qemu_printf("unknown sub command %s\n", name);
    }
}

int btmmu_update_tlb(unsigned long tlb_base, int mmu_idx, unsigned long mask)
{
    int ret;
    struct btmmu_update_arg arg;

	CHECK_FD(btmmu_fd);

    arg.base = tlb_base;
    arg.mmu_idx = mmu_idx;
    arg.mask = mask;

    ret = ioctl(btmmu_fd, BTMMU_UPDATE, (void*)&arg);
    return ret;
}
