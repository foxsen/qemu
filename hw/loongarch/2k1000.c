/*
 * QEMU loongson 2k1000 develop board emulation
 *
 * Copyright (c) 2013 qiaochong@loongson.cn
 * Copyright (c) 2024 fxzhang@ict.ac.cn
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
#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/host-utils.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/loongarch/2k1000.h"
#include "sysemu/sysemu.h"
#include "sysemu/qtest.h"
#include "sysemu/runstate.h"
#include "sysemu/reset.h"
#include "sysemu/rtc.h"
#include "hw/i2c/smbus.h"
#include "hw/char/serial.h"
#include "hw/block/flash.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/ide.h"
#include "hw/loader.h"
#include "hw/timer/mc146818rtc.h"
#include "hw/timer/i8254.h"
#include "hw/nvram/fw_cfg.h"
#include "hw/smbios/smbios.h"
#include "exec/address-spaces.h"
#include "block/block.h"
#include "hw/sysbus.h"             /* SysBusDevice */
#include "hw/empty_slot.h"
#include "hw/ssi/ssi.h"
#include "hw/ide/pci.h"
#include "hw/ide/ahci_internal.h"
#include "hw/pci/pcie_host.h"
#include "hw/pci/pcie_port.h"
#include "hw/timer/hpet.h"
#include "elf.h"
#include "2k1000_rom.h"

#define APBBASE 0x1fe20000
#define CFGBASE 0x1fe00000
#define GPUBASE 0xd0000000
#define CACHE_BASE_VIRT 0x9000000000000000ULL

#define PHYS_TO_VIRT(x) ((x) | CACHE_BASE_VIRT)

#define MAX_IDE_BUS 2
#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK << 2)

#define ALIAS_REGION_FROM_RAS_TO_RA(REGIONA, ADDR, SIZE, REGIONB, ALIAS)      \
        ({                                                                    \
                MemoryRegion *alias_mr = g_new(MemoryRegion, 1);              \
                memory_region_init_alias(alias_mr, NULL, NULL, REGIONA, ADDR, \
                                         SIZE);                               \
                memory_region_add_subregion(REGIONB, ALIAS, alias_mr);        \
                alias_mr; })

#define _str(x) #x
#define str(x) _str(x)
#define SIMPLE_OPS(ADDR, SIZE)                                               \
        ({                                                                   \
                MemoryRegion *iomem = g_new(MemoryRegion, 1);                \
                memory_region_init_io(iomem, NULL, &loongarch_qemu_ops,      \
                                      (void *)ADDR, str(ADDR) , SIZE);       \
                memory_region_add_subregion_overlap(address_space_mem, ADDR, \
                                                    iomem, 1);               \
                iomem;                                                       \
        })

static struct _loaderparams {
        uint64_t ram_size;
        const char *kernel_filename;
        const char *kernel_cmdline;
        const char *initrd_filename;
        target_ulong a0, a1, a2;
} loaderparams;

static PCIBus *ls2k_pci_bus;
__attribute__((weak)) struct hpet_fw_config hpet_cfg = {.count = UINT8_MAX};

static qemu_irq *ls2k_irq, *ls2k_irq1;
static int pci_ls2k_map_irq(PCIDevice *d, int irq_num);

static MemoryRegion *ddrcfg_iomem;
static int reg424 = 0x100;

static uint64_t gpio_ov;
static SysBusDevice *liodev[2];
static MemoryRegion *liomr;

static char ddr_level_reg[0x1000];

#define MAX_CPUS 2
static CPULOONGARCHState *mycpu[MAX_CPUS];

static int ddr2config;
#define BOOTPARAM_PHYADDR ((240 << 20))
#define BOOTPARAM_ADDR (CACHE_BASE_VIRT + BOOTPARAM_PHYADDR)
struct boot_params;

#define CORE0_STATUS_OFF       0x000
#define CORE0_EN_OFF           0x004
#define CORE0_SET_OFF          0x008
#define CORE0_CLEAR_OFF        0x00c
#define CORE0_BUF_20           0x020
#define CORE0_BUF_28           0x028
#define CORE0_BUF_30           0x030
#define CORE0_BUF_38           0x038


#define MYID 0xa0
/* #define DEBUG_LS2K */
#ifdef DEBUG_LS2K
        #define IPI_DPRINTF printf
#else
        #define IPI_DPRINTF(...)
#endif

typedef struct gipi_single {
        uint32_t status;
        uint32_t en;
        uint32_t set;
        uint32_t clear;
        uint32_t buf[8];
        qemu_irq irq;
} gipi_single;

typedef struct gipiStates  gipiStates;
struct gipiStates {
        gipi_single core[8];
} ;

static void loongarch_qemu_writel(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
        hwaddr offset = addr;
        addr = ((hwaddr)(long)opaque) + addr;
        switch (addr) {
        case 0x0ff00000 ... 0x0ff01000 - 1:
                memcpy(ddr_level_reg + offset, &val, size);
                break;
        case CFGBASE + 0x0510 ... CFGBASE + 0x0517:
                memcpy((void *)&gpio_ov + addr - CFGBASE + 0x0510, (void *)&val,
                size);
                {
                        int i = (gpio_ov >> 48) & 1;
                        if (liomr->alias != liodev[i]->mmio[0].memory ||
                        liomr->alias_offset != (((gpio_ov >> 44) & 7) *
                        0x2000000)) {
                                memory_region_del_subregion(get_system_memory(),
                                liomr);
                                g_free(liomr);
                                liomr = ALIAS_REGION_FROM_RAS_TO_RA(
                                liodev[i]->mmio[0].memory, ((gpio_ov >> 44) & 7)
                                * 0x2000000 + 0x100000, 0x2000000 - 0x100000,
                                get_system_memory(), 0x1c000000 + 0x100000);
                        }
                }
                break;
        case CFGBASE + 0x0c10:
                //ls_sdio_set_dmaaddr(val);
                break;
        case CFGBASE + 0x0424:
                reg424 = val;
                memory_region_transaction_begin();
                if (ddrcfg_iomem->container == get_system_memory())
                        memory_region_del_subregion(get_system_memory(),
                        ddrcfg_iomem);

                if ((val & 0x100) == 0) {
                        memory_region_add_subregion_overlap(get_system_memory(),
                        0x0ff00000, ddrcfg_iomem, 1);
                }

                memory_region_transaction_commit();
                break;
#ifdef DEBUG_PCIEDMA
        case 0x1fef0000:
                dma_memory_write(ls2k_pci_as, ls2k_pci_addr, &val, 4);
                break;
        case 0x1fef0004:
                ls2k_pci_addr = val;
                break;
#endif
        }
}

static uint64_t loongarch_qemu_readl(void *opaque, hwaddr addr, unsigned size)
{
        uint64_t val = 0;
        hwaddr offset = addr;
        int i;

        addr = ((hwaddr)(long)opaque) + addr;
        switch (addr) {
        case CFGBASE + 0x0510 ... CFGBASE + 0x0517:
                memcpy((void *)&val, (void *)&gpio_ov + addr - CFGBASE + 0x0510,
                size);
                break;
#ifdef DEBUG_PCIEDMA
        case 0x1fef0000:
                dma_memory_read(ls2k_pci_as, ls2k_pci_addr, &val, 4);
                return val;
                break;
        case 0x1fef0004:
                return ls2k_pci_addr;
                break;
#endif
        case GPUBASE + 4:
        case GPUBASE + 0:
        case GPUBASE + 0x100:
                return random();
        case CFGBASE + 0x04b0:
                return 0x10000;
        case CFGBASE + 0x04c0:
                return 0x10000;
        case CFGBASE + 0x0480:
                return 0x50010c85;
        case CFGBASE + 0x0484:
                return 0x00000450;
        case CFGBASE + 0x0488:
                return 0x00000002;
        case CFGBASE + 0x048c:
                return 0;
        case CFGBASE + 0x0490:
                return 0x10010c87;
        case CFGBASE + 0x0494:
                return 0x00000440;
        case CFGBASE + 0x0498:
                return 0x01c00004;
        case CFGBASE + 0x049c:
                return 00064000;
        case CFGBASE + 0x04a0 ... CFGBASE + 0x04af:
        {
                uint64_t data[2] = { 0x45010010c87, 0x4000008 };
                memcpy(&val, (char *)data + (addr - CFGBASE - 0x04a0), size);
        }
                return val;
        case CFGBASE + 0x0424:
                return reg424;
        case 0x0ff00000 ... 0x0ff01000 - 1:
                if (offset >= 0x160 && offset <= 0x167) {
                        uint64_t data = 0x000000000f000101;
                        val = 0;
                        memcpy(&val, (char *)&data + (offset - 0x160), size);
                        return val;
                } else if (offset >= 0x180 && offset <= 0x187) {
                        union {
                                uint8_t b[8];
                                uint64_t l;
                        } d;
                        memcpy(d.b, ddr_level_reg + 0x180, 8);
                        d.b[7] = 1;
                        d.b[6] = 1;
                        d.b[5] = 1;
                        if (ddr_level_reg[0x180] == 1) {
                                d.b[7] = ddr_level_reg[0x3a + 0 * 0x20] >= 1
                                        && ddr_level_reg[0x3a + 0 * 0x20] < 20 ? 0x1 : 0;
                        /*glvl*/
                        } else if (ddr_level_reg[0x180] == 2) {
                                d.b[7] = ddr_level_reg[0x38 + 0 * 0x20] >= 1
                                        && ddr_level_reg[0x38 + 0 * 0x20] < 20 ? 0x3 : 0;
                        }
                        val = 0;
                        memcpy(&val, d.b + offset - 0x180, size);
                        return val;
                } else if (offset >= 0x188 && offset <= 0x18f) {
                        union {
                                uint8_t b[8];
                                uint64_t l;
                        } d;
                        memcpy(d.b, ddr_level_reg + 0x188, 8);
                        for (i = 0; i <= 7; i++) {
                                if (ddr_level_reg[0x180] == 1) {
                                        d.b[i] = ddr_level_reg[0x3a + (i + 1) * 0x20] >= 1
                                        && ddr_level_reg[0x3a + (i + 1) * 0x20] < 20 ? 0x1
                                                : 0;
                                        /*glvl */
                                } else if (ddr_level_reg[0x180] == 2) {
                                        d.b[i] = ddr_level_reg[0x38 + (i + 1) * 0x20] >= 1
                                        && ddr_level_reg[0x38 + (i + 1) * 0x20] < 20 ? 0x3
                                                : 0;
                                }
                        }
                        val = 0;
                        memcpy(&val, d.b + offset - 0x188, size);
                        return val;
                } else if (offset >= 0x168 && offset < 0x170) {
                        val = 0;
                        memcpy(&val, ddr_level_reg + offset, size);
                        ((char *)&val)[4] = random();
                    return  val;
                } else {
                        val = 0;
                        memcpy(&val, ddr_level_reg + offset, size);
                        return val;
                }
        default:
                return  random();
                break;
        }
        return val;
}

static const MemoryRegionOps loongarch_qemu_ops = {
        .read = loongarch_qemu_readl,
        .write = loongarch_qemu_writel,
        .endianness = DEVICE_NATIVE_ENDIAN,
        .impl = {
            .max_access_size = 8,
        },
};

static int set_bootparam(ram_addr_t initrd_offset, long initrd_size, char *dtb)
{
        char memenv[32];
        char highmemenv[32];
        long params_size, dtboffs;
        void *params_buf;
        unsigned long long *parg_env;
        int ret;

        /* Store command line.  */
#undef PBUF_SIZE
#define PBUF_SIZE 0x100000
        params_size = PBUF_SIZE;
        params_buf = g_malloc(params_size);

        parg_env = (void *)params_buf;

        /*
         * pram buf like this:
         *argv[0] argv[1] 0 env[0] env[1] ...env[i] ,0, argv[0]'s data ,
         *argv[1]'s data ,env[0]'data,...,env[i]'s dat,0
         */

        /* jump over argv and env area */
        ret = (3 + 1) * 8;
        /* argv0 */
        *parg_env++ = BOOTPARAM_ADDR + ret;
        ret += 1 + snprintf(params_buf + ret, PBUF_SIZE - ret, "g");
        /* argv1 */
        *parg_env++ = BOOTPARAM_ADDR + ret;
        if (initrd_size > 0) {
                ret += 1 + snprintf(params_buf + ret, PBUF_SIZE - ret,
                                    "rd_start=0x%llx rd_size=%li %s",
                                    PHYS_TO_VIRT(initrd_offset),
                                    initrd_size, loaderparams.kernel_cmdline);
        } else {
                ret += 1 + snprintf(params_buf + ret, PBUF_SIZE - ret, "%s",
                                    loaderparams.kernel_cmdline);
        }
        /* argv2 */
        *parg_env++ = 0;

        /* env */
        sprintf(memenv, "%d", (int)(loaderparams.ram_size > 0xf000000 ? 240 :
                                    (loaderparams.ram_size >> 20)));
        sprintf(highmemenv, "%d",
                (int)(loaderparams.ram_size > 0x10000000 ?
                (loaderparams.ram_size >> 20) - 256 : 0));
        setenv("memsize", memenv, 1);
        setenv("highmemsize", highmemenv, 1);

        ret = QEMU_ALIGN_UP(ret , 32);
        dtboffs = ret;


        if (dtb) {
                int size;
                void *fdt;
                fdt = load_device_tree(dtb, &size);
                printf("fdt %x %p\n", BOOTPARAM_PHYADDR + ret, fdt);
                params_size = QEMU_ALIGN_UP(ret + size, 8);
                params_buf = g_realloc(params_buf, params_size);
                memcpy(params_buf + ret, fdt, size);
                ret = params_size;

                qemu_fdt_dumpdtb(fdt, size);
        }
        params_size = ret;

        init_boot_param(params_buf, BOOTPARAM_PHYADDR, ret, dtboffs);

        loaderparams.a0 = 2;
        loaderparams.a1 = (target_ulong) CACHE_BASE_VIRT + BOOTPARAM_PHYADDR;
        loaderparams.a2 = (target_ulong) CACHE_BASE_VIRT + BOOTPARAM_PHYADDR + ret;


        return 0;
}

static uint64_t cpu_loongarch_virt_to_phys(void *opaque, uint64_t addr)
{
/*map to unused area, here we use kernel symbol only*/
        return addr & 0x1fffffffll;
}

static int64_t load_kernel(char *dtb)
{
        int64_t entry, kernel_low, kernel_high;
        long kernel_size, initrd_size;
        ram_addr_t initrd_offset;

        if (getenv("BOOTROM")) {
                qemu_strtoi64(getenv("BOOTROM"), NULL, 0, &kernel_high);
                kernel_size = load_image_targphys(loaderparams.kernel_filename,
                kernel_high, ram_size);
                kernel_high += kernel_size;
                entry = 0;
        } else {
                ddr2config = 0;
                kernel_size = load_elf(loaderparams.kernel_filename,
                                       cpu_loongarch_virt_to_phys, NULL, 
                                       (uint64_t *)&entry, 
                                       (uint64_t*) &kernel_low, 
                                       (uint64_t *)&kernel_high, 0, 
                                       EM_LOONGARCH, 1, 0);
                if (kernel_size < 0) {
                        fprintf(stderr, "qemu: could not load kernel '%s'\n",
                                loaderparams.kernel_filename);
                        exit(1);
                }
        }

        /* load initrd */
        initrd_size = 0;
        initrd_offset = 0;
        if (loaderparams.initrd_filename) {
                initrd_size = get_image_size(loaderparams.initrd_filename);
                if (initrd_size > 0) {
                        initrd_offset = (kernel_high + ~TARGET_REALPAGE_MASK) &
                        TARGET_REALPAGE_MASK;
                        if (getenv("INITRD_OFFSET")) {
                                qemu_strtou64(getenv("INITRD_OFFSET"), NULL, 0,
                                &initrd_offset);
                        }
                        initrd_size = load_image_targphys(
                                          loaderparams.initrd_filename,
                                          initrd_offset, ram_size - initrd_offset);
                }
                if (initrd_size == (target_ulong)-1) {
                        fprintf(stderr, "qemu: could not load initial ram"
                                " disk '%s'\n", loaderparams.initrd_filename);
                        exit(1);
                }
        }
        set_bootparam(initrd_offset, initrd_size, dtb);

        return entry;
}

static void main_cpu_reset(void *opaque)
{
        ResetData *s = (ResetData *)opaque;
        CPULOONGARCHState *env = &s->cpu->env;

        cpu_reset(CPU(s->cpu));
        env->CSR_MCSR0 = (env->CSR_MCSR0 & ~0xffffffULL) | 0x14a000;
        env->CSR_ASID = 0x80000;
        env->CSR_MCSR1 &= ~(7ULL << 6);
        env->active_tc.PC = s->vector;
        env->active_tc.gpr[4] = loaderparams.a0;
        env->active_tc.gpr[5] = loaderparams.a1;
        env->active_tc.gpr[6] = loaderparams.a2;
}

static void gipi_writel(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
        gipiStates *s = opaque;
        int no = (addr >> 8) & 3;


        if (size != 4) {
                hw_error("size not 4");
        }

        addr &= 0xff;
        switch (addr) {
        case CORE0_STATUS_OFF:
                hw_error("CORE0_SET_OFF Can't be write\n");
                break;
        case CORE0_EN_OFF:
                s->core[no].en = val;
                break;
        case CORE0_SET_OFF:
                s->core[no].status |= val;
                qemu_irq_raise(s->core[no].irq);
                break;
        case CORE0_CLEAR_OFF:
                s->core[no].status ^= val;
                qemu_irq_lower(s->core[no].irq);
                break;
        case 0x20 ... 0x3c:
                s->core[no].buf[(addr - 0x20) / 4] = val;
                break;
        default:
                break;
        }
        IPI_DPRINTF("gipi_write: addr=0x%02x val=0x%02llx cpu=%d pc=%llx\n",
        (int)addr, (long long)val, (int)cpu->cpu_index, (long long)mypc);
}

static uint32_t ls2k_cpu_irqsts(int cpu, int i);
static uint64_t gipi_readl(void *opaque, hwaddr addr, unsigned size)
{
        gipiStates *s = opaque;
#ifdef DEBUG_LS2K
        CPUState *cpu = current_cpu;
#endif
        uint64_t ret = 0;
        int no = (addr >> 8) & 3;

        addr &= 0xff;

        switch (addr) {
        case CORE0_STATUS_OFF:
                ret =  s->core[no].status;
                break;

        case CORE0_EN_OFF:
                ret =  s->core[no].en;
                break;
        case CORE0_SET_OFF:
                ret = 0;
                break;
        case CORE0_CLEAR_OFF:
                ret = 0;
        case 0x20 ... 0x3c:
                ret = s->core[no].buf[(addr - 0x20) / 4];
                break;
        case 0x40:
                ret = ls2k_cpu_irqsts(no, 0);
                break;
        case 0x48:
                ret = ls2k_cpu_irqsts(no, 1);
                break;
        default:
                break;
        }

        IPI_DPRINTF("gipi_read: addr=0x%02x val=0x%02llx cpu=%d pc=%llx\n",
        (int)addr, (long long)ret, (int)cpu->cpu_index, (long long)mypc);
        return ret;
}

static const MemoryRegionOps gipi_ops = {
        .read = gipi_readl,
        .write = gipi_writel,
        .endianness = DEVICE_NATIVE_ENDIAN,
};

static int godson_ipi_init(qemu_irq parent_irq, unsigned long index,
                           gipiStates *s)
{
        s->core[index].irq = parent_irq;
        return 0;
}





static void *ls2k_intctl_init(MemoryRegion *mr, hwaddr addr,
                              qemu_irq *parent_irq, MemoryRegion *pcimr);

static PCIBus **pcibus_ls2k_init(int busno, qemu_irq *pic,
                                 int (*board_map_irq)(PCIDevice *d, int
                                 irq_num), MemoryRegion *ram, MemoryRegion
                                 *ram1, MemoryRegion *ram4);


const char *lookup_symbol(target_ulong orig_addr);
static CPUUnassignedAccess real_do_unassigned_access;
static void loongarch_ls2k_do_unassigned_access(CPUState *cpu, hwaddr addr,
                bool is_write, bool is_exec,
                int opaque, unsigned size)
{
        qemu_log("pc=%s/" TARGET_FMT_plx "\n", lookup_symbol(mypc), mypc);
        if (!is_exec) {
                /* ignore invalid access (ie do not raise exception) */
                return;
        }
        (*real_do_unassigned_access)(cpu, addr, is_write, is_exec, opaque,
        size);
}

static int ls2k_cpu_irq;
static int ls2k_cpu_irq1;
static int ls2k_cpu_irq_old;

static void ls2k_set_cpuirq(void *opaque, int irq, int level)
{
        int *p = opaque ? &ls2k_cpu_irq1 : &ls2k_cpu_irq;
        int ls2k_cpu_irq_new;

        if (level) {
                *p |= 1 << irq;
        } else {
                *p &= ~(1 << irq);
        }

        ls2k_cpu_irq_new =  ls2k_cpu_irq | ls2k_cpu_irq1;

        if (ls2k_cpu_irq_new != ls2k_cpu_irq_old) {
                if (ls2k_cpu_irq_new & (1 << irq)) {
                        qemu_irq_raise(mycpu[irq / 4]->irq[2 + (irq % 4)]);
                } else {
                        qemu_irq_lower(mycpu[irq / 4]->irq[2 + (irq % 4)]);
                }

                ls2k_cpu_irq_old = ls2k_cpu_irq | ls2k_cpu_irq1;
        }
}

static target_ulong confbus_addr(CPULOONGARCHState *env, int cpuid,
                                 target_ulong csr_addr)
{
        return 0x800000001fe00000UL + csr_addr;
}

static void loongarch_ls2k_init(MachineState *machine)
{
        ram_addr_t ram_size = machine->ram_size;
        const char *kernel_filename = machine->kernel_filename;
        const char *kernel_cmdline = machine->kernel_cmdline;
        const char *initrd_filename = machine->initrd_filename;
        char *filename;
        MemoryRegion *address_space_mem = get_system_memory();
        MemoryRegion *ram = g_new(MemoryRegion, 1);
        MemoryRegion *bios;
        int bios_size;
        LOONGARCHCPU *cpu;
        CPULOONGARCHState *env;
        ResetData *reset_info[2];
        PCIBus **pci_bus;
        DriveInfo *flash_dinfo = NULL;
        CPUClass *cc;
        MemoryRegion *iomem_root = g_new(MemoryRegion, 1);
        AddressSpace *as = g_new(AddressSpace, 1);
        int i;
        qemu_irq *cpu_irq;
        qemu_irq *cpu_irq1;
        cpu_irq = qemu_allocate_irqs(ls2k_set_cpuirq, (void *)0, 8);
        cpu_irq1 = qemu_allocate_irqs(ls2k_set_cpuirq, (void *)1, 8);

        /* init CPUs */

        gipiStates *gipis = g_malloc0(sizeof(gipiState));

        int bootcore = 0;
        if (getenv("BOOTCORE")) {
                bootcore = atoi(getenv("BOOTCORE"));
        }

        for (i = 0; i < smp_cpus; i++) {
                /* init CPUs */
                Object *obj = NULL;
                Error *local_err = NULL;

                obj = object_new(machine->cpu_type);

                object_property_set_uint(obj, i, "id", &local_err);
                object_property_set_bool(obj, true, "realized", &local_err);

                object_unref(obj);
                error_propagate(&error_fatal, local_err);

                cpu = LOONGARCH_CPU(CPU(obj));
                if (cpu == NULL) {
                        fprintf(stderr, "Unable to find CPU definition\n");
                        exit(1);
                }

                cpu->confbus_addr = confbus_addr;
                env = &cpu->env;
                mycpu[i] = env;

                cc = CPU_GET_CLASS(cpu);
                real_do_unassigned_access = cc->do_unassigned_access;
                cc->do_unassigned_access = loongarch_ls2k_do_unassigned_access;
                if (getenv("DEBUG_UNALIGN")) {
                        cc->do_unaligned_access = loongarch_cpu_real_do_unaligned_access;
                }

                reset_info[i] = g_malloc0(sizeof(ResetData));
                reset_info[i]->cpu = cpu;
                reset_info[i]->vector = env->active_tc.PC;
                qemu_register_reset(main_cpu_reset, reset_info[i]);

                /* Init CPU internal devices */
                cpu_init_irq(cpu);
                cpu_loongarch_clock_init(cpu);
                godson_ipi_init(env->irq[12], i, gipis);
        }

        env = mycpu[0];

        /* allocate RAM */
        if (mem_path) {
                memory_region_init_ram_from_file(ram, NULL, "MIPS_R4K_RAM",
                ram_size, 0, true, mem_path, &error_fatal);
        } else {
                memory_region_init_ram(ram, NULL, "loongarch_r4k.ram", ram_size,
                &error_fatal);
        }

        MemoryRegion *ram1 = g_new(MemoryRegion, 1);
        memory_region_init_alias(ram1, NULL, "lowmem", ram, 0, 0x10000000);
        memory_region_add_subregion(address_space_mem, 0, ram1);
        MemoryRegion *ram3 = g_new(MemoryRegion, 1);
        memory_region_init_alias(ram3, NULL, "lowmem2G", ram, 0, 0x80000000ULL);
        memory_region_add_subregion(address_space_mem, 0x80000000ULL, ram3);
        MemoryRegion *ram0 =  g_new(MemoryRegion, 1);
        memory_region_init_alias(ram0, NULL, "lowmem", ram, 0, ram_size);
        memory_region_add_subregion(address_space_mem, 0x100000000ULL, ram0);
        if (ram_size >= 0x40000000) {
                MemoryRegion *ram4 = g_new(MemoryRegion, 1);
                memory_region_init_alias(ram4, NULL, "lowmem2G", ram, ram_size -
                0x20000000, 0x20000000);
                memory_region_add_subregion(address_space_mem, 0x20000000,
                ram4);
        }

        MemoryRegion *ram_pciram = g_new(MemoryRegion, 1);
        MemoryRegion *ram_pciram1 = g_new(MemoryRegion, 1);
        MemoryRegion *ram_pciram2 = NULL;
        if (ram_size >= 0x40000000) {
                ram_pciram2 = g_new(MemoryRegion, 1);
                memory_region_init_alias(ram_pciram2, NULL, "gpumem", ram,
                                         ram_size - 0x20000000, 0x20000000);
        }
        memory_region_init_alias(ram_pciram1, NULL, "ddrlowmem", ram, 0,
        0x10000000);
        memory_region_init_alias(ram_pciram, NULL, "ddrmem", ram, 0,
                                 memory_region_size(ram));


        memory_region_init(iomem_root, NULL,  "ls2k axi", UINT64_MAX);
        address_space_init(as, iomem_root, "ls2k axi memory");

        MemoryRegion *ram2 = g_new(MemoryRegion, 1);
        memory_region_init_alias(ram2, NULL, "lowmem", ram, 0, ram_size);
        memory_region_add_subregion(iomem_root, 0, ram2);

        if (kernel_filename) {
                loaderparams.ram_size = ram_size;
                loaderparams.kernel_filename = kernel_filename;
                loaderparams.kernel_cmdline = kernel_cmdline;
                loaderparams.initrd_filename = initrd_filename;
                ((int64_t *)aui_boot_code)[1] = load_kernel(machine->dtb);
        }
        ((int64_t *)aui_boot_code)[2] = bootcore;

        /* Try to load a BIOS image. If this fails, we continue regardless,
           but initialize the hardware ourselves. When a kernel gets
           preloaded we also initialize the hardware, since the BIOS wasn't
           run. */
        bios_size = -1;
        if (bios_name) {
                filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
                if (filename) {
                        bios_size = get_image_size(filename);
                } else {
                        bios_size = -1;
                }
        }

        flash_dinfo = drive_get(IF_PFLASH, 0, 0);
        if (flash_dinfo) {
                ddr2config = 1;
        } else if ((bios_size > 0) && (bios_size <= BIOS_SIZE)) {
                bios = g_new(MemoryRegion, 1);
                memory_region_init_ram(bios, NULL, "loongarch_r4k.bios", BIOS_SIZE,
                &error_fatal);
                memory_region_set_readonly(bios, true);
                memory_region_add_subregion(get_system_memory(), 0x1c000000,
                bios);
                load_image_targphys(filename, 0x1c000000, BIOS_SIZE);
                ddr2config = 1;
        } else {
                bios = g_new(MemoryRegion, 1);
                memory_region_init_ram(bios, NULL, "loongarch_r4k.bios", BIOS_SIZE,
                &error_fatal);
                memory_region_set_readonly(bios, true);
                memory_region_add_subregion(get_system_memory(), 0x1c000000,
                bios);
                bios_size = sizeof(aui_boot_code);
                rom_add_blob_fixed("bios", aui_boot_code, bios_size,
                0x1c000000);
        }
        g_free(filename);


        /* Register 64 KB of IO space at 0x1f000000 */
        pci_bus = pcibus_ls2k_init(0, NULL, pci_ls2k_map_irq, ram_pciram,
        ram_pciram1, ram_pciram2);
        ls2k_irq = ls2k_intctl_init(get_system_memory(), CFGBASE + 0x1400,
        cpu_irq, ls2k_pci_bus->address_space_mem);
        ls2k_irq1 = ls2k_intctl_init(get_system_memory(), CFGBASE + 0x1440,
        cpu_irq1, ls2k_pci_bus->address_space_mem);

        if (serial_hd(0))
                serial_mm_init(address_space_mem, APBBASE + 0x0000, 0,
                ls2k_irq[0], 115200, serial_hd(0), DEVICE_NATIVE_ENDIAN);

        if (serial_hd(1))
                serial_mm_init(address_space_mem, APBBASE + 0x0100, 0,
                ls2k_irq[0], 115200, serial_hd(1), DEVICE_NATIVE_ENDIAN);

        if (serial_hd(2))
                serial_mm_init(address_space_mem, APBBASE + 0x0200, 0,
                ls2k_irq[0], 115200, serial_hd(2), DEVICE_NATIVE_ENDIAN);

        if (serial_hd(3))
                serial_mm_init(address_space_mem, APBBASE + 0x0300, 0,
                ls2k_irq[0], 115200, serial_hd(3), DEVICE_NATIVE_ENDIAN);

        if (serial_hd(4))
                serial_mm_init(address_space_mem, APBBASE + 0x0400, 0,
                ls2k_irq[1], 115200, serial_hd(4), DEVICE_NATIVE_ENDIAN);

        if (serial_hd(5))
                serial_mm_init(address_space_mem, APBBASE + 0x0500, 0,
                ls2k_irq[1], 115200, serial_hd(5), DEVICE_NATIVE_ENDIAN);

        sysbus_create_simple("ls2h_acpi", APBBASE + 0x7000, NULL);

        {
                PCIDevice *pci_dev = pci_create_multifunction(pci_bus[1], -1,
                false, "e1000e");
                DeviceState *dev = DEVICE(pci_dev);

                if (nd_table[2].used) {
                        qdev_set_nic_properties(dev, &nd_table[2]);
                }
                DeviceClass *dc = DEVICE_GET_CLASS(dev);
                PCIDeviceClass *k = PCI_DEVICE_CLASS(DEVICE_CLASS(dc));
                k->romfile = NULL;
                dc->vmsd = NULL;
                qdev_init_nofail(dev);
        }
        {
                char *mempath;
                struct stat buf;
                mempath = getenv("PCI8619");
                if (mempath && (!stat(mempath, &buf))) {
                        PCIDevice *dev = pci_create_multifunction(pci_bus[0],
                        PCI_DEVFN(0, 0), true, "pci8619");
                        qdev_prop_set_uint16(&dev->qdev, "vendor", 0x10b5);
                        qdev_prop_set_uint16(&dev->qdev, "device", 0x8619);
                        qdev_prop_set_uint32(&dev->qdev, "offset", 0x0f000000);
                        qdev_prop_set_uint32(&dev->qdev, "len", 0x01000000);
                        qdev_prop_set_string(&dev->qdev, "mempath", mempath);
                        qdev_init_nofail(&dev->qdev);
                        dev = pci_create_multifunction(pci_bus[0], PCI_DEVFN(0,
                        1), true, "pciram");
                        qdev_prop_set_uint32(&dev->qdev, "bar0", ~(0x00001000 -
                        1));
                        qdev_init_nofail(&dev->qdev);
                }
        }
        pci_create_simple(pci_bus[2], -1, "nec-usb-xhci");

        {
                DeviceState *dev, *dev1;
                void *bus;
                qemu_irq cs_line;
                dev = sysbus_create_simple("ls_spi", 0x1fff0220, ls2k_irq[8]);
                bus = qdev_get_child_bus(dev, "ssi");
                if (flash_dinfo) {
                        dev1 = ssi_create_slave_no_init(bus, "spi-flash");
                        qdev_prop_set_drive(dev1, "drive",
                        blk_by_legacy_dinfo(flash_dinfo), &error_fatal);
                        qdev_prop_set_uint32(dev1, "size", 0x100000);
                        qdev_prop_set_uint64(dev1, "addr", 0x1c000000);
                        qdev_init_nofail(dev1);
                        cs_line = qdev_get_gpio_in_named(dev1, "ssi-gpio-cs",
                        0);
                        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1, cs_line);
                }

                DriveInfo *spinand = drive_get(IF_MTD, 0, 1);
                if (spinand) {
                        dev1 = ssi_create_slave_no_init(bus, "spi-nand");
                        qdev_prop_set_int32(dev1, "ftype", 0x2);
                        qdev_init_nofail(dev1);
                        cs_line = qdev_get_gpio_in_named(dev1, "ssi-gpio-cs",
                        0);
                        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 2, cs_line);
                }
        }

        {
                size_t flash_sector_size        = 128 * KiB;
                size_t flash_size               = 64 * MiB;
                DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 1);
                /* Spansion S29NS128P */
                liodev[0] = SYS_BUS_DEVICE(pflash_cfi02_register(0, NULL,
                "lioflash", flash_size, dinfo ? blk_by_legacy_dinfo(dinfo) :
                NULL, flash_sector_size, flash_size / flash_sector_size, 1, 2,
                0x89, 0x227e, 0x2248, 0x2201, 0x555, 0x2aa, 0));

                memory_region_del_subregion(get_system_memory(),
                liodev[0]->mmio[0].memory);
                flash_size = 128 * MiB;

                dinfo = drive_get(IF_PFLASH, 0, 2);
                liodev[1] = SYS_BUS_DEVICE(pflash_cfi02_register(0, NULL,
                "lioflash1", flash_size, dinfo ? blk_by_legacy_dinfo(dinfo) :
                NULL, flash_sector_size, flash_size / flash_sector_size, 1, 2,
                0x1, 0x227e, 0x2248, 0x2201, 0x555, 0x2aa, 0));
                memory_region_del_subregion(get_system_memory(),
                liodev[1]->mmio[0].memory);

                liomr = ALIAS_REGION_FROM_RAS_TO_RA(liodev[0]->mmio[0].memory,
                ((gpio_ov >> 44) & 7) * 0x2000000 + 0x100000, 0x2000000 -
                0x100000, get_system_memory(), 0x1c000000 + 0x100000);
                loongarch_qemu_writel((void *)CFGBASE + 0x0510, 0, 0, 8);
        }

        {
                DeviceState *dev;
                SysBusDevice *s;
                dev = qdev_create(NULL, "ls_dma");
                qdev_prop_set_uint8(dev, "mode", 0x1);
                qdev_init_nofail(dev);
                s = SYS_BUS_DEVICE(dev);
                sysbus_mmio_map(s, 0, CFGBASE + 0x0c00);
        }
        {
                DeviceState *dev;
                SysBusDevice *s;
                const char *str;
                dev = qdev_create(NULL, "ls_nand");
                qdev_prop_set_uint8(dev, "cs", 0x2);
                str = getenv("DEBUG_MYNAND");
                while (str && *str) {
                        unsigned long cs, id;
                        if (!strncmp(str, "cs=", 3)) {
                                qemu_strtoul(str + 3, &str, 0, &cs);
                                qdev_prop_set_uint8(dev, "cs", cs);
                        } else if (!strncmp(str, "id=", 3)) {
                                qemu_strtoul(str + 3, &str, 0, &id);
                                qdev_prop_set_uint8(dev, "chip_id", id & 0xff);
                                if (id & 0xff00) {
                                        qdev_prop_set_uint8(dev,
                                        "manufacturer_id", (id >> 8) & 0xff);
                                }
                        } else
                                break;
                        if (*str) {
                                str++;
                        }
                }
                qdev_init_nofail(dev);
                s = SYS_BUS_DEVICE(dev);
                sysbus_mmio_map(s, 0, APBBASE + 0x6000);
                sysbus_connect_irq(s, 0, ls2k_irq1[12]);
        }

#if 0
        {
                DriveInfo *dinfo;
                dinfo = drive_get(IF_SD, 0, 0);
                if (!dinfo) {
                        fprintf(stderr, "qemu: missing SecureDigital device\n");
                        exit(1);
                }

                ls_mmci_init(address_space_mem, APBBASE + 0xc000,
                                 blk_by_legacy_dinfo(dinfo),
                                 ls2k_irq[31]
                                );
        }
#end

#if 1
        {
                DeviceState *dev;
                SysBusDevice *s;
                dev = qdev_create(NULL, "ls_rtc");
                qdev_init_nofail(dev);
                s = SYS_BUS_DEVICE(dev);
                sysbus_mmio_map(s, 0, APBBASE + 0x7820);
                sysbus_connect_irq(s, 0, ls2k_irq[14]);
                sysbus_connect_irq(s, 1, ls2k_irq[15]);
                sysbus_connect_irq(s, 2, ls2k_irq[16]);
        }
#endif

        {
                DeviceState *dev;
                void *bus;
                dev = sysbus_create_simple("ls_i2c", APBBASE + 0x1000,
                ls2k_irq[7]);
                bus = qdev_get_child_bus(dev, "i2c");
                i2c_create_slave(bus, "ds1338", 0x68);
                i2c_create_slave(bus, "ds1338", 0x22);
                i2c_create_slave(bus, "ds1338", 0x24);
                i2c_create_slave(bus, "ds1338", 0x30);
        }

        {
                DeviceState *dev;
                void *bus;
                dev = sysbus_create_simple("ls_i2c", APBBASE + 0x1800,
                ls2k_irq[8]);
                bus = qdev_get_child_bus(dev, "i2c");
                i2c_create_slave(bus, "ds1338", 0x68);
                i2c_create_slave(bus, "ds1338", 0x22);
                i2c_create_slave(bus, "ds1338", 0x24);
                i2c_create_slave(bus, "ds1338", 0x30);
        }
        {
                DeviceState *dev;
                SysBusDevice *s;
                Object *obj;
                Error *err = NULL;


                dev = qdev_create(NULL, "ls2k_can");
                obj = object_new("can-bus");
                object_property_add_child(object_get_objects_root(), "canbus0",
                obj, &err);
                object_property_parse(OBJECT(dev), "canbus0", "canbus", &err);
                s = SYS_BUS_DEVICE(dev);
                qdev_init_nofail(dev);
                sysbus_connect_irq(s, 0, ls2k_irq[16]);
                sysbus_mmio_map(s, 0, APBBASE + 0x0c00);
        }
        {
                DeviceState *dev;
                SysBusDevice *s;
                Object *obj;
                Error *err = NULL;


                dev = qdev_create(NULL, "ls2k_can");
                obj = object_new("can-bus");
                object_property_add_child(object_get_objects_root(), "canbus1",
                obj, &err);
                object_property_parse(OBJECT(dev), "canbus1", "canbus", &err);
                s = SYS_BUS_DEVICE(dev);
                qdev_init_nofail(dev);
                sysbus_connect_irq(s, 0, ls2k_irq[17]);
                sysbus_mmio_map(s, 0, APBBASE + 0x0d00);
        }

        {
                DeviceState *hpet = qdev_try_create(NULL, TYPE_HPET);
                if (hpet) {
                        /* For pc-piix-*, hpet's intcap is always IRQ2. For
                        pc-q35-1.7
                         * and earlier, use IRQ2 for compat. Otherwise, use
                         * IRQ16~23,
                         * IRQ8 and IRQ2.
                         */
                        uint8_t compat = object_property_get_uint(OBJECT(hpet),
                                         HPET_INTCAP, NULL);
                        if (!compat) {
                                qdev_prop_set_uint32(hpet, HPET_INTCAP, 1);
                        }
                        qdev_init_nofail(hpet);
                        sysbus_mmio_map(SYS_BUS_DEVICE(hpet), 0, APBBASE +
                        0x4000);
                        sysbus_connect_irq(SYS_BUS_DEVICE(hpet), 0,
                        ls2k_irq[21]);
                }
        }



        SIMPLE_OPS(CFGBASE + 0x4b0, 8);
        SIMPLE_OPS(CFGBASE + 0x4c0, 8);
        SIMPLE_OPS(CFGBASE + 0xc10, 4);
        SIMPLE_OPS(CFGBASE + 0x480, 0x20);
        SIMPLE_OPS(CFGBASE + 0x490, 0x10);
        SIMPLE_OPS(CFGBASE + 0x4a0, 16);
        SIMPLE_OPS(CFGBASE + 0x424, 4);
#ifdef DEBUG_PCIEDMA
        SIMPLE_OPS(0x1fef0000, 8);
#endif


        {
                ddrcfg_iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(ddrcfg_iomem, NULL, &loongarch_qemu_ops,
                                      (void *)0x0ff00000, "ddr", 0x001000);

                if (ddr2config) {
                        loongarch_qemu_writel(0, CFGBASE + 0x0424, 0x000, 4);
                }
        }


        {
                static MemoryRegion *gipi_iomem;
                gipi_iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(gipi_iomem, NULL, &gipi_ops,
                                      (void *)gipis, "gipi", 0x200);
                memory_region_add_subregion(get_system_memory(), CFGBASE +
                0x1000, gipi_iomem);
        }

        {
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_ram(iomem, NULL, "ls2k.gpio", 0x40,
                &error_fatal);
                memory_region_add_subregion(get_system_memory(), CFGBASE +
                0x0500, iomem);
                SIMPLE_OPS(CFGBASE + 0x0510, 8);
        }
        SIMPLE_OPS(CFGBASE + 0x0590, 8);

}


static void loongarch_machine_init(MachineClass *mc)
{
        mc->desc = "loongson 2k1000 platform";
        mc->init = loongarch_ls2k_init;
        mc->max_cpus = 2;
        mc->block_default_type = IF_IDE;
        mc->default_cpu_type = LOONGARCH_CPU_TYPE_NAME("la264");
}

DEFINE_MACHINE("ls2k", loongarch_machine_init)

