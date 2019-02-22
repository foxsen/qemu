/*
 * QEMU Loongson 3A board(Lemote LX-1507) support
 *
 * Copyright (c) 2016 Fuxin Zhang (zhangfx@lemote.com)
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

/*
 * Lemote LX-1507 pc board, using Loongson 3A 2000 cpu.
 *
 */

#include "hw/hw.h"
#include "hw/char/serial.h"
#include "net/net.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/devices.h"
#include "hw/isa/isa.h"
#include "hw/i386/pc.h"
#include "hw/ssi.h"
#include "hw/i2c/i2c.h"
#include "sysemu/block-backend.h"
#include "hw/block/flash.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "sysemu/char.h"
#include "sysemu/sysemu.h"
#include "audio/audio.h"
#include "qemu/log.h"
#include "hw/loader.h"
#include "hw/mips/bios.h"
#include "hw/ide.h"
#include "hw/isa/isa.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"
#include "elf.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"
#include "sysemu/qtest.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "net/net.h"

extern NetClientState *net_hub_add_port(int hub_id, const char *name);

#include "ls3a.h"

//#define DEBUG_LM1507

#if defined (DEBUG_LM1507)
#  define DPRINTF(fmt, ...) \
    do { fprintf(stderr, "LM1507: " fmt, ## __VA_ARGS__); } while (0)
#else
static inline GCC_FMT_ATTR(1, 2) int DPRINTF(const char *fmt, ...)
{
    return 0;
}
#endif

#define ENVP_ADDR       0x80002000l
#define ENVP_NB_ENTRIES	 	16
#define ENVP_ENTRY_SIZE	 	256

#define LM1507_BIOSNAME "pmon_lm1507.bin"

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;

static void GCC_FMT_ATTR(3, 4) prom_set(uint32_t* prom_buf, int index,
                                        const char *string, ...)
{
    va_list ap;
    int32_t table_addr;

    if (index >= ENVP_NB_ENTRIES)
        return;

    if (string == NULL) {
        prom_buf[index] = 0;
        return;
    }

    table_addr = sizeof(int32_t) * ENVP_NB_ENTRIES + index * ENVP_ENTRY_SIZE;
    prom_buf[index] = tswap32(ENVP_ADDR + table_addr);

    va_start(ap, string);
    vsnprintf((char *)prom_buf + table_addr, ENVP_ENTRY_SIZE, string, ap);
    va_end(ap);
}

static int64_t load_kernel (CPUMIPSState *env)
{
    int64_t kernel_entry, kernel_low, kernel_high;
    int index = 0;
    long initrd_size;
    ram_addr_t initrd_offset;
    uint32_t *prom_buf;
    long prom_size;

    if (load_elf(loaderparams.kernel_filename, cpu_mips_kseg0_to_phys, NULL,
                 (uint64_t *)&kernel_entry, (uint64_t *)&kernel_low,
                 (uint64_t *)&kernel_high, 0, EM_MIPS, 1) < 0) {
        DPRINTF("qemu: could not load kernel '%s'\n", \
                loaderparams.kernel_filename);
        exit(1);
    }

    /* load initrd */
    initrd_size = 0;
    initrd_offset = 0;
    if (loaderparams.initrd_filename) {
        initrd_size = get_image_size (loaderparams.initrd_filename);
        if (initrd_size > 0) {
            initrd_offset = (kernel_high + ~INITRD_PAGE_MASK) & INITRD_PAGE_MASK;
            if (initrd_offset + initrd_size > ram_size) {
                DPRINTF("qemu: memory too small for initial ram disk '%s'\n",\
                loaderparams.initrd_filename);
                exit(1);
            }
            initrd_size = load_image_targphys(loaderparams.initrd_filename,
                                     initrd_offset, ram_size - initrd_offset);
        }
        if (initrd_size == (target_ulong) -1) {
            DPRINTF("qemu: could not load initial ram disk '%s'\n", \
                    loaderparams.initrd_filename);
            exit(1);
        }
    }

    /* Setup prom parameters. */
    prom_size = ENVP_NB_ENTRIES * (sizeof(int32_t) + ENVP_ENTRY_SIZE);
    prom_buf = g_malloc(prom_size);

    prom_set(prom_buf, index++, "%s", loaderparams.kernel_filename);
    if (initrd_size > 0) {
        prom_set(prom_buf, index++, "rd_start=0x%" PRIx64 " rd_size=%li %s",
                 cpu_mips_phys_to_kseg0(NULL, initrd_offset), initrd_size,
                 loaderparams.kernel_cmdline);
    } else {
        prom_set(prom_buf, index++, "%s", loaderparams.kernel_cmdline);
    }

    /* Setup minimum environment variables */
    prom_set(prom_buf, index++, "busclock=33000000");
    prom_set(prom_buf, index++, "cpuclock=100000000");
    prom_set(prom_buf, index++, "memsize=%i", loaderparams.ram_size/1024/1024);
    prom_set(prom_buf, index++, "modetty0=38400n8r");
    prom_set(prom_buf, index++, NULL);

    rom_add_blob_fixed("prom", prom_buf, prom_size,
                       cpu_mips_kseg0_to_phys(NULL, ENVP_ADDR));

    g_free(prom_buf);
    return kernel_entry;
}

static void write_bootloader (CPUMIPSState *env, uint8_t *base, int64_t kernel_addr)
{
    uint32_t *p;

    /* Small bootloader */
    p = (uint32_t *) base;

    stl_p(p++, 0x0bf00010);                                      /* j 0x1fc00040 */
    stl_p(p++, 0x00000000);                                      /* nop */

    /* Second part of the bootloader */
    p = (uint32_t *) (base + 0x040);

    stl_p(p++, 0x3c040000);                                      /* lui a0, 0 */
    stl_p(p++, 0x34840002);                                      /* ori a0, a0, 2 */
    stl_p(p++, 0x3c050000 | ((ENVP_ADDR >> 16) & 0xffff));       /* lui a1, high(ENVP_ADDR) */
    stl_p(p++, 0x34a50000 | (ENVP_ADDR & 0xffff));               /* ori a1, a0, low(ENVP_ADDR) */
    stl_p(p++, 0x3c060000 | (((ENVP_ADDR + 8) >> 16) & 0xffff)); /* lui a2, high(ENVP_ADDR + 8) */
    stl_p(p++, 0x34c60000 | ((ENVP_ADDR + 8) & 0xffff));         /* ori a2, a2, low(ENVP_ADDR + 8) */
    stl_p(p++, 0x3c070000 | (loaderparams.ram_size >> 16));      /* lui a3, high(env->ram_size) */
    stl_p(p++, 0x34e70000 | (loaderparams.ram_size & 0xffff));   /* ori a3, a3, low(env->ram_size) */
    stl_p(p++, 0x3c1f0000 | ((kernel_addr >> 16) & 0xffff));     /* lui ra, high(kernel_addr) */;
    stl_p(p++, 0x37ff0000 | (kernel_addr & 0xffff));             /* ori ra, ra, low(kernel_addr) */
    stl_p(p++, 0x03e00008);                                      /* jr ra */
    stl_p(p++, 0x00000000);                                      /* nop */
}


static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    CPUMIPSState *env = &cpu->env;

    cpu_reset(CPU(cpu));
    /* TODO: 3A reset stuff */
    env->CP0_Status = 0x30c000e4;
    if (loaderparams.kernel_filename) {
        env->CP0_Status &= ~((1 << CP0St_BEV) | (1 << CP0St_ERL));
    }
}

static void ls3a_serial_set_irq(void *opaque, int irq, int level)
{
    int i;
    LM1507State *s = (LM1507State *)opaque;
    for(i = 0; i < smp_cpus; i++)
        qemu_set_irq(s->mycpu[i]->irq[2], level);
}

static LM1507State s;

/* memory controller io */
static uint64_t mc0_values[180];
static void mc0_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("mc0 write addr %lx with val %lx size %d\n", addr, val, size);
    if (addr < 180 * 16) 
        mc0_values[addr/16] = val;
}

static uint64_t mc0_read(void *opaque, hwaddr addr, unsigned size)
{
    //LM1507State *s = (LM1507State *)opaque;
    uint64_t val = -1;
    if (addr == 0x10) {
        /* bit 0 is for dll lock status */
        val = mc0_values[1] | 0x1;
    } else if (addr == 0x960) {
        /* not clear from doc */
        val = mc0_values[0x96] | 0x100;
    } else if (addr < 180 * 16) {
        val = mc0_values[addr / 16];
    }

    DPRINTF("mc0 read addr %lx size %d,val=%lx\n", addr, size, val);
    return val;
}

static const MemoryRegionOps mc0_io_ops = {
    .read = mc0_read,
    .write = mc0_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint64_t mc1_values[180];
static void mc1_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("mc1 write addr %lx with val %lx size %d\n", addr, val, size);
    if (addr < 180 * 16) 
        mc1_values[addr/16] = val;
}

static uint64_t mc1_read(void *opaque, hwaddr addr, unsigned size)
{
    //LM1507State *s = (LM1507State *)opaque;
    uint64_t val = -1;
    if (addr == 0x10) {
        /* bit 0 is for dll lock status */
        val = mc1_values[1] | 0x1;
    } else if (addr == 0x960) {
        /* not clear from doc */
        val = mc1_values[0x96] | 0x100;
    } else if (addr < 180 * 16) {
        val = mc1_values[addr / 16];
    }

    DPRINTF("mc1 read addr %lx size %d,val=%lx\n", addr, size, val);
    return val;
}

static const MemoryRegionOps mc1_io_ops = {
    .read = mc1_read,
    .write = mc1_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* chip misc io */
static void misc_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("misc write addr %lx with val %lx size %d\n", addr, val, size);

    /* chip config0 */
    if (addr == 0x180) {
        /* bit 4 is MC0 ddr confspace disable */
        if (val & (1<<4)) {
            memory_region_set_enabled(&s.mc0_mem, 0);

            DPRINTF("MC0 memory space disabled\n");
        } else {
            memory_region_set_enabled(&s.mc0_mem, 1);
            DPRINTF("MC0 memory space enabled\n");
        }
        /* bit 9 is MC1 ddr confspace disable */
        if (val & (1<<9)) {
            memory_region_set_enabled(&s.mc1_mem, 0);

            DPRINTF("MC1 memory space disabled\n");
        } else {
            memory_region_set_enabled(&s.mc1_mem, 1);
            DPRINTF("MC1 memory space enabled\n");
        }
    } 
}

static uint64_t misc_read(void *opaque, hwaddr addr, unsigned size)
{
    //LM1507State *s = (LM1507State *)opaque;
    uint64_t val = -1;

    DPRINTF("misc read addr %lx size %d,val=%lx\n", addr, size, val);
    return val;
}

static const MemoryRegionOps misc_io_ops = {
    .read = misc_read,
    .write = misc_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void ht_update_irq(void *opaque,int disable)
{
    LM1507State *s = opaque;

    uint32_t isr,ier,irtr,core,irq_nr;

    isr = s->ht_ctlconf_reg[HT_IRQ_VECTOR_REG0/4];
    ier = s->ht_ctlconf_reg[HT_IRQ_ENABLE_REG0/4];

    irtr = s->intc.route[INT_ROUTER_REGS_HT1_INT0];

    core = irtr & 0xf;
    irq_nr = ((irtr >> 4) & 0xf);

    if(core > 0 && irq_nr > 0)
    {
        if((isr & ier) && !disable)
            qemu_irq_raise(s->mycpu[ffs(core) - 1]->irq[ffs(irq_nr) + 1]);
        else
            qemu_irq_lower(s->mycpu[ffs(core) - 1]->irq[ffs(irq_nr) + 1]);
    }
}

static void ht_set_irq(void *opaque, int irq, int level)
{
    LM1507State *s = opaque;
    uint32_t isr = 0;

    if (irq == 0) {
        if (level)
            isr = (cpu_inb(0x20)&~cpu_inb(0x21)) | ((cpu_inb(0xa0)&~cpu_inb(0xa1)) << 8);
        else 
            isr = 0;
        s->ht_ctlconf_reg[HT_IRQ_VECTOR_REG0/4] = isr; 
    }

    ht_update_irq(opaque,0);
}

static void *ls3a_intctl_init(ISABus *isa_bus, void *opache)
{
    qemu_irq *ht_irq;
    qemu_irq *i8259;

	ht_irq = qemu_allocate_irqs(ht_set_irq, opache, 8);

	i8259 = i8259_init(isa_bus, ht_irq[0]); 

	cpu_outb(0x4d0, 0xff);
	cpu_outb(0x4d1, 0xff);

    return i8259;
}

/* chip config io */
static void creg_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    LM1507State *s = (LM1507State *)opaque;
    int node = (addr >> 44) & 3;
    int coreno = (addr >> 8) & 3;
    int no = coreno + node * 4;

    DPRINTF("creg write addr %lx with val %lx size %d\n", addr, val, size);


    /* don't support multi node yet */
    addr &= ~(3ULL << 44);

    if (addr == SLOCK0_ADDR) {
        /* lock scache control */
        if (val != 0) {
            uint64_t size = (~s->scache0_mask & ((1LL<< PA_BITS) - 1)) + 1;
            s->scache0_addr = val & ( (1LL << PA_BITS) - 1);
            fprintf(stderr, "enable scache access %lx %lx\n", s->scache0_addr, size);

            memory_region_init_ram(&s->scache0_ram, NULL, "lm1507.scache0", 
                    size, &error_fatal);

            memory_region_add_subregion_overlap(get_system_memory(), 
                    s->scache0_addr, &s->scache0_ram, 1);
        } else {
            DPRINTF("disable scache0 locked access\n");
            memory_region_del_subregion(get_system_memory(), &s->scache0_ram);
            memory_region_unref(&s->scache0_ram);
        }
    } else if (addr == SLOCK1_ADDR) {
        /* lock scache control */
        if (val != 0) {
            uint64_t size = (~s->scache1_mask & ((1LL<< PA_BITS) - 1)) + 1;
            s->scache1_addr = val & ( (1LL << PA_BITS) - 1);
            fprintf(stderr, "enable scache access %lx %lx\n", s->scache1_addr, size);

            memory_region_init_ram(&s->scache1_ram, NULL, "lm1507.scache1", 
                    size, &error_fatal);

            memory_region_add_subregion_overlap(get_system_memory(), 
                    s->scache1_addr, &s->scache1_ram, 1);
        } else {
            DPRINTF("disable scache1 locked access\n");
            memory_region_del_subregion(get_system_memory(), &s->scache1_ram);
            memory_region_unref(&s->scache1_ram);
        }
    } else if (addr == SLOCK0_MASK) {
        s->scache0_mask = val;
    } else if (addr == SLOCK1_MASK) {
        s->scache1_mask = val;
    } else if (addr >= 0x1000 && addr < 0x1400) {
        CPUState *cpu = current_cpu;

        if (size != 4) hw_error("size not 4");

        /* interrupt control
         * 0x1000 - 0x1400: coren IPI status/int/set/clr
         */
        addr &= 0xff;
        switch(addr){
            case CORE0_STATUS_OFF: 
                hw_error("CORE0_STATUS_OFF Can't be write\n");
                break;
            case CORE0_EN_OFF:
                if ((cpu->mem_io_vaddr & 0xff) != addr) break;
                s->intc.core[no].en = val;
                break;
            case CORE0_SET_OFF:
                s->intc.core[no].status |= val;
                qemu_irq_raise(s->intc.core[no].irq);
                break;
            case CORE0_CLEAR_OFF:
                if ((cpu->mem_io_vaddr & 0xff) != addr) break;
                s->intc.core[no].status ^= val;
                qemu_irq_lower(s->intc.core[no].irq);
                break;
            case 0x20 ... 0x3c:
                s->intc.core[no].buf[(addr-0x20)/4] = val;
                break;
            default:
                break;
        }
    } else if (addr >= 0x1400 && addr < 0x1460) {
        addr = addr - 0x1400;

        if (size != 4) hw_error("size not 4");

        if (addr == INT_ROUTER_REGS_HT1_INT0) {
            uint32_t old;
            old = s->intc.route[addr/4];
            if (old != val)	
                ht_update_irq(s,1);
            s->intc.route[addr/4] = val;
            ht_update_irq(s,0);
        }
        *(uint32*)((void*)&s->intc.route + addr) = val;
    }
}

static uint64_t creg_read(void *opaque, hwaddr addr, unsigned size)
{
    LM1507State *s = (LM1507State *)opaque;
    int node = (addr >> 44) & 3;
    int coreno = (addr >> 8) & 3;
    int no = coreno + node * 4;

    uint64_t val = -1LL;

    DPRINTF("creg read addr %lx size %d val %lx\n", addr, size, val);

    addr &= ~(3ULL << 44);

    if (addr == SLOCK0_ADDR) {
        val = s->scache0_addr;
    } else if (addr == SLOCK0_MASK) {
        val = s->scache0_mask;
    } else if (addr >= 0x1000 && addr < 0x1400) {
        addr &= 0xff;
        if (size != 4) hw_error("size not 4");
        switch(addr){
            case CORE0_STATUS_OFF: 
                val =  s->intc.core[no].status;
                break;
            case CORE0_EN_OFF:
                val =  s->intc.core[no].en;
                break;
            case CORE0_SET_OFF:
                val = 0; //hw_error("CORE0_SET_OFF Can't be Read\n");
                break;
            case CORE0_CLEAR_OFF:
                val = 0; //hw_error("CORE0_CLEAR_OFF Can't be Read\n");
            case 0x20 ... 0x3c:
                val = s->intc.core[no].buf[(addr-0x20)/4];
                break;
            default:
                break;
        }
    } else if (addr >= 0x1400 && addr < 0x1460) {
        addr = addr - 0x1400;
        if (size != 4) hw_error("size not 4");

        val = *(uint32*)((void*)&s->intc.route + addr);
    }

    return val;
}

static const MemoryRegionOps creg_io_ops = {
    .read = creg_read,
    .write = creg_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

#if 0
/* GPU reg io */
static void gpu_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("gpu write addr %lx with val %lx size %d\n", addr, val, size);
}

static uint64_t gpu_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = -1LL;

    DPRINTF("gpu read addr %lx size %d val %lx\n", addr, size, val);
    return val;
}

static const MemoryRegionOps gpu_io_ops = {
    .read = gpu_read,
    .write = gpu_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* nand reg io */
static void nand_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("nand write addr %lx with val %lx size %d\n", addr, val, size);
}

static uint64_t nand_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = -1LL;

    DPRINTF("nand read addr %lx size %d val %lx\n", addr, size, val);
    return val;
}

static const MemoryRegionOps nand_io_ops = {
    .read = nand_read,
    .write = nand_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};
#endif

/* ht controller config reg io */
static int64_t timer_count = 10;
static void ht_ctlconf_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    LM1507State *s = (LM1507State *)opaque;

    DPRINTF("HT controller config write addr %lx with val %lx size %d\n", addr, val, size);

    if (addr >= 0x120) return;

    if (addr == 0x44) {
        /* don't set crc error bits */
        val = val & ~(0x300);
    } 

    s->ht_ctlconf_reg[addr / 4] = val;
}

static uint64_t ht_ctlconf_read(void *opaque, hwaddr addr, unsigned size)
{
    LM1507State *s = (LM1507State *)opaque;
    uint64_t val = -1LL;

    if (addr >= 0x120) return val;

    if (addr == HT_LINK_CONFIG_REG) {
        val = s->ht_ctlconf_reg[addr / 4];
        /* set init complete after each read to emulate system reset */
        if (timer_count > 0) timer_count--;
        if (timer_count == 0) 
            s->ht_ctlconf_reg[HT_LINK_CONFIG_REG / 4] |= (1ULL<<5);
    } else if ( addr == HT_IRQ_VECTOR_REG0) {
        /* connect i8259 to ht0 */
        val = (cpu_inb(0x20) & ~cpu_inb(0x21)) | ((cpu_inb(0xa0) & ~cpu_inb(0xa1)) << 8);
    } else {
        val = s->ht_ctlconf_reg[addr / 4];
    }

    DPRINTF("HT controller config read addr %lx size %d val %lx\n", addr, size, val);
    return val;
}

static const MemoryRegionOps ht_ctlconf_ops = {
    .read = ht_ctlconf_read,
    .write = ht_ctlconf_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* ht io */
static void unassigned_ht_io_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    fprintf(stderr, "unassigned HT IO write addr %lx with val %lx size %d\n",
            addr, val, size);
}

static uint64_t unassigned_ht_io_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = -1LL;

    fprintf(stderr, "unassigned HT IO read addr %lx size %d val %lx\n", 
            addr, size, val);
    return val;
}

static const MemoryRegionOps unassigned_ht_io_ops = {
    .read = unassigned_ht_io_read,
    .write = unassigned_ht_io_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* pci memory space */
static void unassigned_pci_mem_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    LM1507State *s = (LM1507State *)opaque;

    DPRINTF("PCI memory write addr %lx with val %lx size %d\n", addr, val, size);
    if (addr == 0x10000) {
        /* watchdog control, bios will use watchdog to reset system, 
         * clear the init complete bit
         */
        if (val == 0x81) {
            s->ht_ctlconf_reg[HT_LINK_CONFIG_REG / 4] &= ~(1ULL << 5);
            timer_count = 10;
        }
    } 
}

static uint64_t unassigned_pci_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = -1LL;

    DPRINTF("PCI memory read addr %lx size %d val %lx\n", addr, size, val);

    /* south brdige PCIE_VC0_RESOURCE_STATUS, bit 1 is negotiation pending */
    if (addr == (0x60000000 | (8 << 15) | 0x12a)) {
            val = 0;
    }
    return val;
}

static const MemoryRegionOps unassigned_pci_mem_ops = {
    .read = unassigned_pci_mem_read,
    .write = unassigned_pci_mem_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* ht bus config reg io */
#define DEVFN(bus, dev, fn) (((bus) << 16) | ((dev) << 11) | ((fn) << 8))
static uint64_t read_data(void *addr, unsigned size)
{
    uint64_t val = -1UL;
    if (((uint64_t)addr) & (size - 1)) {
        printf("unaligned access!\n"); 
        return val;
    }
    switch (size) {
        case 1:
            val = *(unsigned char*) addr;
            break;
        case 2:
            val = *(unsigned short*) addr;
            break;
        case 4:
            val = *(unsigned int*) addr;
            break;
        case 8:
            val = *(unsigned long*) addr;
            break;
        default:
            printf("unknown size %d\n", size);
    }
    return val;
}

static void write_data(void *addr, uint64_t val, unsigned size)
{
    if (((uint64_t)addr) & (size - 1)) {
        printf("unaligned access!\n"); 
        return;
    }
    switch (size) {
        case 1:
            *(unsigned char*) addr = val;
            break;
        case 2:
            *(unsigned short*) addr = val;
            break;
        case 4:
            *(unsigned int*) addr = val;
            break;
        case 8:
            *(unsigned long*) addr = val;
            break;
        default:
            printf("unknown size %d\n", size);
    }
}

//00:00.0 Host bridge: Advanced Micro Devices, Inc. [AMD] RS780 Host Bridge
static unsigned char pci_confdata[][256] = {
{
0x22,0x10,0x00,0x96,0x06,0x00,0x30,0x22,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0x10,0x00,0x96,
0x00,0x00,0x00,0x00,0xc4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x08,0x9c,0x00,0xc0,0x40,0x00,0x00,0x00,0x43,0x00,0x00,0x00,0x42,0x20,0x05,0x00,
0x22,0x10,0x00,0x96,0x08,0x40,0x00,0x90,0x08,0x10,0x05,0x00,0x00,0x00,0x00,0x00,
0x8c,0x00,0x00,0x00,0xe8,0x37,0x00,0x00,0x00,0x02,0x20,0x00,0x0d,0x80,0x8e,0x81,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x20,
0x00,0x00,0x00,0x00,0x95,0x00,0x00,0x03,0x20,0x36,0x00,0x00,0x30,0x21,0x00,0x10,
0x00,0x00,0x80,0xff,0x32,0x01,0x00,0x00,0x00,0x00,0x00,0x80,0x08,0xf8,0x7c,0xd0,
0x3a,0x00,0x74,0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x71,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x80,0x08,0x54,0x80,0x01,0x20,0x00,0x11,0x00,0xd0,0x00,0x00,0x00,
0x60,0x05,0x75,0x1c,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x1c,0x00,0x01,0x00,0x09,0x01,0x00,0x00,0xac,0x02,0x00,0x00,0x0b,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x00,0x08,0x00,0x00,0xe0,0x00,0x00,0x00,0x00
},
//00:02.0 PCI bridge: Advanced Micro Devices, Inc. [AMD] RS780 PCI to PCI bridge (ext gfx port 0)
{
0x22,0x10,0x03,0x96,0x47,0x01,0x10,0x00,0x00,0x00,0x04,0x06,0x08,0x00,0x01,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x41,0x41,0x00,0x00,
0x00,0x58,0x00,0x58,0x01,0x40,0xf1,0x57,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0x58,0x03,0xc8,0x00,0x00,0x00,0x00,0x10,0xa0,0x42,0x00,0x20,0x80,0x00,0x00,
0x10,0x08,0x00,0x00,0x02,0x0d,0x30,0x00,0x00,0x00,0x02,0xb1,0x00,0x00,0x04,0x00,
0x00,0x00,0x40,0x01,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x42,0x00,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x05,0xb0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x0d,0xb8,0x00,0x00,0x22,0x10,0x00,0x96,0x08,0x00,0x03,0xa8,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x70,0x00,0x00,0x00,0xf7,0x5f,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
},
//00:04.0 PCI bridge: Advanced Micro Devices, Inc. [AMD] RS780/RS880 PCI to PCI bridge (PCIE port 0)
{
0x22,0x10,0x04,0x96,0x47,0x01,0x10,0x00,0x00,0x00,0x04,0x06,0x08,0x00,0x01,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x02,0x00,0x51,0x51,0x00,0x00,
0x10,0x58,0x10,0x58,0x21,0x58,0x21,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0x58,0x03,0xc8,0x00,0x00,0x00,0x00,0x10,0xa0,0x42,0x00,0x20,0x80,0x00,0x00,
0x10,0x08,0x00,0x00,0x42,0x0c,0x30,0x01,0x00,0x00,0x11,0x70,0x00,0x00,0x04,0x00,
0x00,0x00,0x40,0x01,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x42,0x00,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x05,0xb0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x0d,0xb8,0x00,0x00,0x22,0x10,0x00,0x96,0x08,0x00,0x03,0xa8,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xa2,0x00,0x00,0x00,0x16,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
},
//00:09.0 PCI bridge: Advanced Micro Devices, Inc. [AMD] RS780/RS880 PCI to PCI bridge (PCIE port 4)
{
0x22,0x10,0x08,0x96,0x47,0x01,0x10,0x00,0x00,0x00,0x04,0x06,0x08,0x00,0x01,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x00,0xf1,0x01,0x00,0x00,
0xf0,0xff,0x00,0x00,0xf1,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0x58,0x03,0xc8,0x00,0x00,0x00,0x00,0x10,0xa0,0x42,0x00,0x20,0x80,0x00,0x00,
0x10,0x08,0x00,0x00,0x12,0x0c,0x30,0xf7,0x00,0x00,0x00,0x19,0x00,0x00,0x04,0x00,
0x00,0x00,0x40,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x42,0x00,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x05,0xb0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x0d,0xb8,0x00,0x00,0x22,0x10,0x00,0x96,0x08,0x00,0x03,0xa8,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xa2,0x00,0x00,0x00,0x16,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
},
//00:0a.0 PCI bridge: Advanced Micro Devices, Inc. [AMD] RS780/RS880 PCI to PCI bridge (PCIE port 5)
{
0x22,0x10,0x09,0x96,0x47,0x01,0x10,0x00,0x00,0x00,0x04,0x06,0x08,0x00,0x01,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x00,0xf1,0x01,0x00,0x00,
0xf0,0xff,0x00,0x00,0xf1,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0x58,0x03,0xc8,0x00,0x00,0x00,0x00,0x10,0xa0,0x42,0x00,0x20,0x80,0x00,0x00,
0x10,0x08,0x00,0x00,0x12,0x0c,0x30,0xf7,0x00,0x00,0x00,0x19,0x00,0x00,0x04,0x00,
0x00,0x00,0x40,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x05,0xb0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x0d,0xb8,0x00,0x00,0x22,0x10,0x00,0x96,0x08,0x00,0x03,0xa8,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xa2,0x00,0x00,0x00,0x16,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
},
#if 0
//00:11.0 SATA controller: Advanced Micro Devices, Inc. [AMD/ATI] SB7x0/SB8x0/SB9x0 SATA Controller [IDE mode]
{
0x02,0x10,0x90,0x43,0x47,0x01,0x30,0x02,0x00,0x01,0x06,0x01,0x00,0xf8,0x00,0x00,
0x21,0x60,0x00,0x00,0x31,0x60,0x00,0x00,0x29,0x60,0x00,0x00,0x35,0x60,0x00,0x00,
0x01,0x60,0x00,0x00,0x00,0x90,0x30,0x58,0x00,0x00,0x00,0x00,0x02,0x10,0x80,0x43,
0x00,0x00,0x00,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x01,0x00,0x00,
0x14,0x00,0x00,0x00,0x01,0x00,0x10,0x00,0x80,0xbf,0x00,0x00,0x00,0x00,0x00,0x00,
0x05,0x70,0x84,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0x50,0x22,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x12,0x00,0x10,0x00,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x2c,0xd6,0x80,0xb4,0x01,0xd6,0x80,0xb4,0x01,
0xd6,0x80,0xb4,0x01,0xd6,0x80,0xb4,0x01,0x16,0x80,0xb4,0x01,0x16,0x80,0xb4,0x01,
0xfa,0xa0,0xfa,0xa0,0xfa,0xa0,0xfa,0xa0,0x7a,0xa0,0x7a,0xa0,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x00,
0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
},
#endif
//00:14.0 SMBus: Advanced Micro Devices, Inc. [AMD/ATI] SBx00 SMBus Controller (rev 3c)
{
0x02,0x10,0x85,0x43,0x43,0x05,0x30,0x02,0x3c,0x00,0x05,0x0c,0x00,0x00,0x80,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xb0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xd4,0xbb,0x00,0x0c,0x00,0x00,0x00,0x00,0x0f,0xff,0x00,0x00,0x00,0x00,0x00,0x80,
0xf0,0x05,0xf0,0x0e,0xf0,0x0f,0xf0,0x0f,0xfe,0xff,0xf0,0x0f,0x00,0x00,0x00,0x00,
0x01,0x00,0x2c,0x24,0x37,0x7d,0x9e,0x82,0xff,0x90,0x00,0x00,0x20,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x08,0x00,0xc0,0xfe,0xff,0x6f,0x00,0x00,0x00,0x00,0xf0,0x04,
0x08,0x02,0xf0,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0x10,0x00,0x00,0xfb,0xde,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xff,0xff,0x7f,0xff,0xf0,0x09,0x05,0xfa,0x0d,0x02,0x06,0x69,0x00,0x1c,
0x08,0x00,0x02,0xa8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0x0f,0x12,0x12,
0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x20,0x0b,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xd8,0x0c,0x00,0x00,0x00,0x00,0x44,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x36,0x00,
},
//00:14.1 IDE interface: Advanced Micro Devices, Inc. [AMD/ATI] SB7x0/SB8x0/SB9x0 IDE Controller
{
0x02,0x10,0x9c,0x43,0x45,0x01,0x20,0x02,0x00,0x8a,0x01,0x01,0x00,0xf8,0x00,0x00,
0x39,0xa0,0x00,0x00,0x31,0xa0,0x00,0x00,0x29,0xa0,0x00,0x00,0x21,0xa0,0x00,0x00,
0x11,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,
0x5d,0x5d,0x5d,0x5d,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x40,0x06,0x10,0x2c,0x01,0x07,0x01,0x00,0x00,0x00,0xff,0xff,0x0f,0x00,
0x05,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
},
//00:14.2 Audio device: Advanced Micro Devices, Inc. [AMD/ATI] SBx00 Azalia (Intel HDA)
{
0x02,0x10,0x83,0x43,0x06,0x00,0x10,0x04,0x00,0x00,0x03,0x04,0x08,0xf0,0x00,0x00,
0x04,0x00,0x30,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x01,0x00,0x00,
0x00,0x00,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x01,0x00,0x42,0xc8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x05,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
},
//00:14.3 ISA bridge: Advanced Micro Devices, Inc. [AMD/ATI] SB7x0/SB8x0/SB9x0 LPC host controller
{
0x02,0x10,0x9d,0x43,0x4f,0x01,0x20,0x02,0x00,0x00,0x01,0x06,0x00,0x00,0x80,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x04,0x00,0x00,0x00,0xff,0xff,0xc3,0xff,0xff,0xff,0x62,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xd0,0xfe,0xd0,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x00,0xf0,0xff,0xff,0xff,
0x67,0x45,0x23,0x00,0x00,0x00,0x00,0x00,0x1d,0x00,0x00,0x00,0x07,0x0a,0x00,0x00,
0x08,0x00,0x03,0xa8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,
0x02,0x00,0x00,0x00,0x2f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf2,0xff,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0xf7,0xff,0xff,0xff,0x00,0x00,0x00,0x78,
0x00,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x0c,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
},
//00:14.4 PCI bridge: Advanced Micro Devices, Inc. [AMD/ATI] SBx00 PCI to PCI Bridge
{
0x02,0x10,0x84,0x43,0x67,0x01,0xa0,0x02,0x00,0x01,0x04,0x06,0x00,0xff,0x81,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x40,0xf0,0x00,0x80,0x22,
0xf0,0xff,0x00,0x00,0xf0,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,
0x26,0x00,0x3c,0xff,0x00,0x00,0x00,0x00,0x0c,0x0f,0x3f,0xd1,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x08,0x00,0x03,0xa8,0x00,0x00,0x00,0x00,0x85,0x00,0xff,0xff,
0xca,0x0e,0x17,0x00,0xba,0x99,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x02,0x06,
0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
},
//00:14.5 USB controller: Advanced Micro Devices, Inc. [AMD/ATI] SB7x0/SB8x0/SB9x0 USB OHCI2 Controller,
{
0x02,0x10,0x99,0x43,0x47,0x01,0xa0,0x02,0x00,0x10,0x03,0x0c,0x08,0xfc,0x00,0x00,
0x00,0x80,0x30,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x03,0x00,0x00,
0x80,0x03,0x00,0x00,0x11,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x40,0x13,0x1f,0xf0,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xff,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
}
};

static void ht_busconf_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    LM1507State *s = (LM1507State *)opaque;
    //DPRINTF("HT bus config write addr %lx with val %lx size %d\n", addr, val, size);
    //int type1 = 0;
    uint64_t devfn, reg;

    //if (addr & 0x1000000) type1 = 1;

    //printf("HT bus config write addr %lx with val %lx size %d\n", addr, val, size);

    addr &= 0xffffffUL;
    devfn = addr & ~0xff;
    reg = addr & 0xff;

    //printf("bus=%ld, dev=%ld, fn=%ld\n", devfn >> 16, (devfn >> 11) & 0x1f, (devfn >> 8) & 0x7);

    if (devfn == DEVFN(0, 0, 0)) {
        write_data(&pci_confdata[0][reg], val, size);
#if 0
    } else if ( devfn == DEVFN(0, 2, 0) ) {
        write_data(&pci_confdata[1][reg], val, size);
    } else if ( devfn == DEVFN(0, 4, 0) ) {
        write_data(&pci_confdata[2][reg], val, size);
    } else if ( devfn == DEVFN(0, 9, 0) ) {
        write_data(&pci_confdata[3][reg], val, size);
    } else if ( devfn == DEVFN(0, 0xa, 0) ) {
        write_data(&pci_confdata[4][reg], val, size);
#endif
    } else if ( devfn == DEVFN(0, 0x11, 0) ||
                devfn == DEVFN(0, 0x12, 0) ) {
        pci_data_write(s->pci_bus, addr, val, size);
#if 0
    } else if ( devfn == DEVFN(0, 0x14, 0) ) {
        write_data(&pci_confdata[5][reg], val, size);
    } else if ( devfn == DEVFN(0, 0x14, 1) ) {
        write_data(&pci_confdata[6][reg], val, size);
    } else if ( devfn == DEVFN(0, 0x14, 2) ) {
        write_data(&pci_confdata[7][reg], val, size);
    } else if ( devfn == DEVFN(0, 0x14, 3) ) {
        write_data(&pci_confdata[8][reg], val, size);
    } else if ( devfn == DEVFN(0, 0x14, 4) ) {
        write_data(&pci_confdata[9][reg], val, size);
    } else if ( devfn == DEVFN(0, 0x14, 5) ) {
        write_data(&pci_confdata[10][reg], val, size);
#endif
    }

    //pci_data_write(opaque, addr, val, size);
}

static uint64_t ht_busconf_read(void *opaque, hwaddr addr, unsigned size)
{
    LM1507State *s = (LM1507State *)opaque;
    uint64_t val = -1LL, devfn, reg;

    DPRINTF("HT bus config read addr %lx size %d val %lx\n", addr, size, val);

    /* link_error */
    if (addr == 0x48) 
        return 0x80250023;

    /* 0:8:0 is south bridge, 0xe0 is NBPCIE_INDEX, its 0xa5 is PCIE_LC_STATE0, its bit 0-5 must be 0x10 */
    if (addr == ((8<<11) | 0xe4)) {
        return 0x10;
    }

    //val = pci_data_read(opaque, addr, size);
    
    addr &= 0xffffffUL;
    devfn = addr & ~0xff;
    reg = addr & 0xff;

    //printf("bus=%ld, dev=%ld, fn=%ld\n", devfn >> 16, (devfn >> 11) & 0x1f, (devfn >> 8) & 0x7);

    if (devfn == DEVFN(0, 0, 0)) {
        val = read_data(&pci_confdata[0][reg], size);
        if (reg >= 0x10 && reg < 0x28) 
            val = 0;
#if 0
    } else if ( devfn == DEVFN(0, 2, 0) ) {
        val = read_data(&pci_confdata[1][reg], size);
    } else if ( devfn == DEVFN(0, 4, 0) ) {
        val = read_data(&pci_confdata[2][reg], size);
    } else if ( devfn == DEVFN(0, 9, 0) ) {
        val = read_data(&pci_confdata[3][reg], size);
    } else if ( devfn == DEVFN(0, 0xa, 0) ) {
        val = read_data(&pci_confdata[4][reg], size);
#endif
    } else if ( devfn == DEVFN(0, 0x11, 0) ||
                devfn == DEVFN(0, 0x12, 0) ) {
        val = pci_data_read(s->pci_bus, addr, size);
#if 0
    } else if ( devfn == DEVFN(0, 0x14, 0) ) {
        val = read_data(&pci_confdata[5][reg], size);
    } else if ( devfn == DEVFN(0, 0x14, 1) ) {
        val = read_data(&pci_confdata[6][reg], size);
    } else if ( devfn == DEVFN(0, 0x14, 2) ) {
        val = read_data(&pci_confdata[7][reg], size);
    } else if ( devfn == DEVFN(0, 0x14, 3) ) {
        val = read_data(&pci_confdata[8][reg], size);
    } else if ( devfn == DEVFN(0, 0x14, 4) ) {
        val = read_data(&pci_confdata[9][reg], size);
    } else if ( devfn == DEVFN(0, 0x14, 5) ) {
        val = read_data(&pci_confdata[10][reg], size);
#endif
    } 

    //printf("HT bus config read addr %lx size %d val %lx\n", addr, size, val);

    return val;
}

static const MemoryRegionOps ht_busconf_ops = {
    .read = ht_busconf_read,
    .write = ht_busconf_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void smbus_io_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("smbus io write addr %lx with val %lx size %d\n", addr, val, size);
}

static uint64_t smbus_io_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = -1ULL;

    if (addr == 0) {
        /* bit 0 is smbus busy bit, return 0 to avoid waiting */
        return 0;
    }
    return val;
}

static const MemoryRegionOps smbus_io_ops = {
    .read = smbus_io_read,
    .write = smbus_io_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

#if 0

/* sysbus LPC implementation */
#define TYPE_LM1507_LPC "lm1507-lpc"
#define LM1507_LPC(obj) OBJECT_CHECK(LM1507LPCState, (obj), TYPE_LM1507_LPC)

typedef struct {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    qemu_irq parent_irq;

    int nirqs;

    /* cfg0: SIRQ en; cfg1: int_en; cfg2: int_src; cfg3: int_clr */
    uint32_t cfg[4];
} LM1507LPCState;

static inline void lm1507_lpc_inth_update(LM1507LPCState *s)
{
    if (s->cfg[1] & s->cfg[2]) {
        qemu_set_irq(s->parent_irq, 1);
    } else {
        qemu_set_irq(s->parent_irq, 0);
    }
}

static void lm1507_lpc_set_intr(void *opaque, int irq, int req)
{
    LM1507LPCState *s = LM1507_LPC(opaque);

    int mask = 1 << irq;

    if (req) {
        s->cfg[2] |= mask;
    } else {
        s->cfg[2] &= ~mask;
    }

    lm1507_lpc_inth_update(s);
}

static uint64_t lm1507_lpc_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    LM1507LPCState *s = LM1507_LPC(opaque);
    uint64_t value = s->cfg[(addr & 0xf) >> 2];
    return value;
}

static void lm1507_lpc_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    LM1507LPCState *s = LM1507_LPC(opaque);

    s->cfg[(addr & 0xf) >> 2] = value;

    lm1507_lpc_inth_update(s);
}

static const MemoryRegionOps lm1507_lpc_mem_ops = {
    .read = lm1507_lpc_read,
    .write = lm1507_lpc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void lm1507_lpc_reset(DeviceState *dev)
{
    LM1507LPCState *s = LM1507_LPC(dev);
    int i;

    for (i = 0; i < 4; i++)
        s->cfg[i] = 0;
}

static int lm1507_lpc_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    LM1507LPCState *s = LM1507_LPC(dev);

    s->nirqs = 18;
    sysbus_init_irq(sbd, &s->parent_irq);
    qdev_init_gpio_in(dev, lm1507_lpc_set_intr, s->nirqs);
    memory_region_init_io(&s->mmio, OBJECT(s), &lm1507_lpc_mem_ops, s,
                          "lm1507-lpc", 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);

    return 0;
}

static Property lm1507_lpc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void lm1507_lpc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = lm1507_lpc_init;
    dc->reset = lm1507_lpc_reset;
    dc->props = lm1507_lpc_properties;
}

static const TypeInfo lm1507_lpc_info = {
    .name          = "lm1507-lpc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .class_init    = lm1507_lpc_class_init,
    .instance_size = sizeof(LM1507LPCState),
};

/** I2C slave implementations **/
typedef enum i2c_state { I2C_STOP, I2C_SEND, I2C_RECV } i2c_state_t;

/* I2C slave implementation of memory SPD eeprom */
#define TYPE_LM1507_SPD "lm1507-spd"
#define LM1507_SPD(obj) OBJECT_CHECK(LM1507SPDState, (obj), TYPE_LM1507_SPD)

/* byte 3 == 0x2, udimm
   byte 4 == 0x2, 1Gb chip, 8 banks 
   byte 5 == 0x11, row 14bit, col 10bit
   byte 6 == 0x6, 1.2/1.35/1.5v ok
   byte 7 == 0x1, 8bit device width, 1rank
   byte 8 == 0x3, bus width 64
   Total capacity = sdram capicity / 8 * buswidth / device width * 
   ranks = 1Gb / 8 * 64 / 8 * 1 = 1GB
   or (row + col)  + 3 (8 banks) + 3 ( 64/8 width) + 0(1rank) = 2^30

   Only the first row is correctly set for now
 */
static const uint8_t eeprom_spd[0x80] = {
    0x00,0x11,0x0b,0x02,0x02,0x11,0x06,0x01,0x03,0x70,
    0x70,0x00,0x82,0x10,0x00,0x01,0x0e,0x04,0x0c,0x01,
    0x02,0x20,0x80,0x75,0x70,0x00,0x00,0x50,0x3c,0x50,
    0x2d,0x20,0xb0,0xb0,0x50,0x50,0x00,0x00,0x00,0x00,
    0x00,0x41,0x48,0x3c,0x32,0x75,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x9c,0x7b,0x07,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x48,0x42,0x35,0x34,0x41,0x32,
    0x35,0x36,0x38,0x4b,0x4e,0x2d,0x41,0x37,0x35,0x42,
    0x20,0x30,0x20
};

typedef struct {
    I2CSlave parent_obj;

    i2c_state_t state;
    uint8_t offset;

    const uint8_t *eeprom;
} LM1507SPDState;

static int lm1507_spd_send(I2CSlave *i2c, uint8_t data)
{
    LM1507SPDState *s = LM1507_SPD(i2c);
    if (s->state == I2C_SEND) 
        s->offset = data;
    else
        fprintf(stderr, "data sent in wrong state %d\n", s->state);
    return 0;
}

static void lm1507_spd_event(I2CSlave *i2c, enum i2c_event event)
{
    LM1507SPDState *s = LM1507_SPD(i2c);
    switch (event) {
        case I2C_START_SEND:
            s->state = I2C_SEND;
            break;
        case I2C_START_RECV:
            s->state = I2C_RECV;
            break;
        case I2C_FINISH:
            s->state = I2C_STOP;
            s->offset = 0;
            break;
        case I2C_NACK:
            break;
    }
}

static int lm1507_spd_recv(I2CSlave *i2c)
{
    int ret;
    LM1507SPDState *s = LM1507_SPD(i2c);

    if (s->state != I2C_RECV) {
        fprintf(stderr, "i2c receive in wrong state %d\n", s->state);
        ret = -1;
    } else 
        ret = s->eeprom[s->offset & 0x7f];
    DPRINTF("spd recv %d at %d\n", ret, s->offset);
    return ret;
}

static int lm1507_spd_init(I2CSlave *i2c)
{
    LM1507SPDState *s = LM1507_SPD(i2c);

    s->offset = 0;
    s->state = I2C_STOP;
    s->eeprom = eeprom_spd;
    return 0;
}

static void lm1507_spd_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = lm1507_spd_init;
    k->event = lm1507_spd_event;
    k->recv = lm1507_spd_recv;
    k->send = lm1507_spd_send;
}

static const TypeInfo lm1507_spd_info = {
    .name          = TYPE_LM1507_SPD,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LM1507SPDState),
    .class_init    = lm1507_spd_class_init,
};

/* I2C slave implementation for gmac eeprom */
#define TYPE_LM1507_MACROM "lm1507-macrom"
#define LM1507_MACROM(obj) OBJECT_CHECK(LM1507MacromState, (obj),TYPE_LM1507_MACROM)

/* mac0/mac1 eeprom, only the first 12 bytes emulated */
static const unsigned char eeprom_mac[] = 
{ 0x00, 0x23, 0x9e, 0x00, 0x01, 0x02, 0x00, 0x23, 0x9e, 0x00, 0x01, 0x03 };

typedef struct {
    I2CSlave parent_obj;

    i2c_state_t state;
    uint8_t offset;
    int len;

    const uint8_t *eeprom;
} LM1507MacromState;

static int lm1507_macrom_send(I2CSlave *i2c, uint8_t data)
{
    LM1507MacromState *s = LM1507_MACROM(i2c);
    if (s->state == I2C_SEND) { 
        if (s->len == 0) {
            s->offset = data << 8;
            s->len = 1;
        } else if (s->len == 1) {
            s->offset |= data;
            s->len = 2;
        } else {
          fprintf(stderr, "addr sent with wrong len %d\n", s->len);
        }
    } else {
        fprintf(stderr, "data sent in wrong state %d\n", s->state);
    }
    return 0;
}

static void lm1507_macrom_event(I2CSlave *i2c, enum i2c_event event)
{
    LM1507MacromState *s = LM1507_MACROM(i2c);
    switch (event) {
        case I2C_START_SEND:
            s->state = I2C_SEND;
            break;
        case I2C_START_RECV:
            s->state = I2C_RECV;
            break;
        case I2C_FINISH:
            s->state = I2C_STOP;
            s->len = 0;
            s->offset = 0;
            break;
        case I2C_NACK:
            break;
    }
}

static int lm1507_macrom_recv(I2CSlave *i2c)
{
    int ret;
    LM1507MacromState *s = LM1507_MACROM(i2c);

    if (s->state != I2C_RECV) {
        fprintf(stderr, "i2c receive in wrong state %d\n", s->state);
        ret = -1;
    } else {
        ret = (s->offset > 11) ? 0 : s->eeprom[s->offset];
    }
    DPRINTF("macrom recv %d at %d\n", ret, s->offset);
    /* auto increase offset */
    s->offset ++;
    return ret;
}

static int lm1507_macrom_init(I2CSlave *i2c)
{
    LM1507MacromState *s = LM1507_MACROM(i2c);

    s->offset = 0;
    s->state = I2C_STOP;
    s->len = 0;
    s->eeprom = eeprom_mac;

    return 0;
}

static void lm1507_macrom_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = lm1507_macrom_init;
    k->event = lm1507_macrom_event;
    k->recv = lm1507_macrom_recv;
    k->send = lm1507_macrom_send;
}

static const TypeInfo lm1507_macrom_info = {
    .name          = TYPE_LM1507_MACROM,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LM1507MacromState),
    .class_init    = lm1507_macrom_class_init,
};

/* I2C slave implementation for EDID data */
#define TYPE_LM1507_EDIDROM "lm1507-edidrom"
#define LM1507_EDIDROM(obj) OBJECT_CHECK(LM1507EDIDromState, (obj),TYPE_LM1507_EDIDROM)

/* 800x600 EDID, see Documentation/EDID/ in kernel tree for more information */
static const unsigned char eeprom_edid[] = { 
0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x31,0xD8,0x00,0x00,0x00,0x00,0x00,0x00,
0x05,0x16,0x01,0x03,0x6D,0x1B,0x14,0x78,0xEA,0x5E,0xC0,0xA4,0x59,0x4A,0x98,0x25,
0x20,0x50,0x54,0x01,0x00,0x00,0x45,0x40,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0xA0,0x0F,0x20,0x00,0x31,0x58,0x1C,0x20,0x28,0x80,
0x14,0x00,0x15,0xD0,0x10,0x00,0x00,0x1E,0x00,0x00,0x00,0xFF,0x00,0x4C,0x69,0x6E,
0x75,0x78,0x20,0x23,0x30,0x0A,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0xFD,0x00,0x3B,
0x3D,0x24,0x26,0x05,0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0xFC,
0x00,0x4C,0x69,0x6E,0x75,0x78,0x20,0x53,0x56,0x47,0x41,0x0A,0x20,0x20,0x00,0xC2
};

typedef struct {
    I2CSlave parent_obj;

    i2c_state_t state;
    uint8_t offset;
    int len;

    const uint8_t *eeprom;
} LM1507EDIDromState;

static int lm1507_edidrom_send(I2CSlave *i2c, uint8_t data)
{
    LM1507EDIDromState *s = LM1507_EDIDROM(i2c);
    if (s->state == I2C_SEND) { 
        if (s->len == 0) {
            s->offset = data << 8;
            s->len = 1;
        } else if (s->len == 1) {
            s->offset |= data;
            s->len = 2;
        } else {
          fprintf(stderr, "addr sent with wrong len %d\n", s->len);
        }
    } else {
        fprintf(stderr, "data sent in wrong state %d\n", s->state);
    }
    return 0;
}

static void lm1507_edidrom_event(I2CSlave *i2c, enum i2c_event event)
{
    LM1507EDIDromState *s = LM1507_EDIDROM(i2c);
    switch (event) {
        case I2C_START_SEND:
            s->state = I2C_SEND;
            break;
        case I2C_START_RECV:
            s->state = I2C_RECV;
            break;
        case I2C_FINISH:
            s->state = I2C_STOP;
            s->len = 0;
            s->offset = 0;
            break;
        case I2C_NACK:
            break;
    }
}

static int lm1507_edidrom_recv(I2CSlave *i2c)
{
    int ret;
    LM1507EDIDromState *s = LM1507_EDIDROM(i2c);

    if (s->state != I2C_RECV) {
        fprintf(stderr, "i2c receive in wrong state %d\n", s->state);
        ret = -1;
    } else {
        ret = (s->offset > 11) ? 0 : s->eeprom[s->offset];
    }
    DPRINTF("edidrom recv %d at %d\n", ret, s->offset);
    /* auto increase offset */
    s->offset ++;
    return ret;
}

static int lm1507_edidrom_init(I2CSlave *i2c)
{
    LM1507EDIDromState *s = LM1507_EDIDROM(i2c);

    s->offset = 0;
    s->state = I2C_STOP;
    s->len = 0;
    s->eeprom = eeprom_edid;

    return 0;
}

static void lm1507_edidrom_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = lm1507_edidrom_init;
    k->event = lm1507_edidrom_event;
    k->recv = lm1507_edidrom_recv;
    k->send = lm1507_edidrom_send;
}

static const TypeInfo lm1507_edidrom_info = {
    .name          = TYPE_LM1507_EDIDROM,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LM1507EDIDromState),
    .class_init    = lm1507_edidrom_class_init,
};

/* acpi */
static void acpi_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("acpi write addr %lx with val %lx size %d\n", addr, val, size);
#if 0
    addr += LM1507_ACPI_REG_BASE;
    switch (addr) {
        case LM1507_PM1_STS_REG:
            break;
        case LM1507_PM1_CNT_REG:
            if (val == 0x3c00)
                qemu_system_shutdown_request();
            break;
        case LM1507_RST_CNT_REG:
            if (val & 1)
                qemu_system_reset_request();
            break;
    }
#endif
}

static uint64_t acpi_read(void *opaque, hwaddr addr, unsigned size)
{
    DPRINTF("acpi read addr %lx size %d\n", addr, size);
    return 0;
}

static MemoryRegionOps acpi_io_ops = {
    .read = acpi_read,
    .write = acpi_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* sysbus RTC implementation */
#define TYPE_LM1507_RTC "lm1507-rtc"
#define LM1507_RTC(obj) OBJECT_CHECK(LM1507RTCState, (obj), TYPE_LM1507_RTC)

typedef struct {
    SysBusDevice parent_obj;

    qemu_irq irq[8];

    MemoryRegion io;

    uint32_t toy_trim, toy_lo, toy_hi, toy_match0, toy_match1, toy_match2;
    uint32_t rtc_trim, rtc_count, rtc_match0, rtc_match1, rtc_match2, rtc_ctrl;
} LM1507RTCState;

/* todo: interrupt */
static void rtc_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    LM1507RTCState *s = LM1507_RTC(opaque);

    uint64_t vaddr = addr + LM1507_RTC_REG_BASE;
    DPRINTF("rtc write addr %lx with val %lx size %d\n", addr, val, size);
    switch (vaddr) {
        case LM1507_TOY_TRIM_REG:
            s->toy_trim = (uint32_t) val;
            break;
        case LM1507_TOY_WRITE0_REG:
            s->toy_lo = (uint32_t) val;
            break;
        case LM1507_TOY_WRITE1_REG:
            s->toy_hi = (uint32_t) val;
            break;
        case LM1507_TOY_MATCH0_REG:
            s->toy_match0 = (uint32_t) val;
            break;
        case LM1507_TOY_MATCH1_REG:
            s->toy_match1 = (uint32_t) val;
            break;
        case LM1507_TOY_MATCH2_REG:
            s->toy_match2 = (uint32_t) val;
            break;
        case LM1507_RTC_CTRL_REG:
            s->rtc_ctrl = (uint32_t) val;
            break;
        case LM1507_RTC_TRIM_REG:
            s->rtc_trim = (uint32_t) val;
            break;
        case LM1507_RTC_WRITE0_REG:
            s->rtc_count = (uint32_t) val;
            break;
        case LM1507_RTC_MATCH0_REG:
            s->rtc_match0 = (uint32_t) val;
            break;
        case LM1507_RTC_MATCH1_REG:
            s->rtc_match1 = (uint32_t) val;
            break;
        case LM1507_RTC_MATCH2_REG:
            s->rtc_match2 = (uint32_t) val;
            break;
        default:
            printf("invalid rtc address %lx\n", vaddr);
    }
}

static uint64_t rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    LM1507RTCState *s = LM1507_RTC(opaque);
    uint64_t vaddr = addr + LM1507_RTC_REG_BASE;
    uint64_t val = -1;
    struct tm tm;

    qemu_get_timedate(&tm, 0);

    switch (vaddr) {
        case LM1507_TOY_TRIM_REG:
            val = s->toy_trim; 
            break;
        case LM1507_TOY_READ0_REG:
            val = ( (((tm.tm_mon + 1) & 0x3f) << 26) |
                    ((tm.tm_mday & 0x1f) << 21) |
                    ((tm.tm_hour & 0x1f) << 16) |
                    ((tm.tm_min & 0x3f) << 10)  | 
                    ((tm.tm_sec & 0x3f) << 4) ); 
            DPRINTF("mon %d day %d\n", tm.tm_mon, tm.tm_mday);
            break;
        case LM1507_TOY_READ1_REG:
            val = tm.tm_year;
            DPRINTF("year %d\n", tm.tm_year);
            break;
        case LM1507_TOY_MATCH0_REG:
            val = s->toy_match0;
            break;
        case LM1507_TOY_MATCH1_REG:
            val = s->toy_match1;
            break;
        case LM1507_TOY_MATCH2_REG:
            val = s->toy_match2;
            break;
        case LM1507_RTC_TRIM_REG:
            val = s->rtc_trim;
            break;
        case LM1507_RTC_READ0_REG:
            val = s->rtc_count;
            break;
        case LM1507_RTC_MATCH0_REG:
            val = s->rtc_match0;
            break;
        case LM1507_RTC_MATCH1_REG:
            val = s->rtc_match1;
            break;
        case LM1507_RTC_MATCH2_REG:
            val = s->rtc_match2;
            break;
        default:
            printf("invalid rtc address %lx\n", vaddr);
    }

    DPRINTF("RTC read addr %lx size %d,val=%lx\n", addr, size, val);
    return val;
}

static MemoryRegionOps lm1507_rtc_ops = {
    .read = rtc_read,
    .write = rtc_write,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void lm1507_rtc_reset(DeviceState *dev)
{
    //LM1507RTCState *s = LM1507_RTC(dev);
}

static int lm1507_rtc_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    LM1507RTCState *s = LM1507_RTC(dev);
    int i;

    for (i = 0; i < 8; i++)
        sysbus_init_irq(sbd, &s->irq[i]);
    memory_region_init_io(&s->io, OBJECT(s), &lm1507_rtc_ops, s,
                          "lm1507-rtc", 0x8000);
    sysbus_init_mmio(sbd, &s->io);

    return 0;
}

static Property lm1507_rtc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void lm1507_rtc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = lm1507_rtc_init;
    dc->reset = lm1507_rtc_reset;
    dc->props = lm1507_rtc_properties;
}

static TypeInfo lm1507_rtc_info = {
    .name          = "lm1507-rtc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .class_init    = lm1507_rtc_class_init,
    .instance_size = sizeof(LM1507RTCState),
};
#endif //if 0

//-------------------------
// pci bridge
//-----------------

/* PCI declarations */
#define TYPE_RS780_PCI_HOST_BRIDGE "ls3a-pcihost"
typedef struct Rs780State Rs780State;

#define RS780_PCI_HOST_BRIDGE(obj) \
    OBJECT_CHECK(Rs780State, (obj), TYPE_RS780_PCI_HOST_BRIDGE)

struct Rs780State {
    PCIHostState parent_obj;
};

static int board_map_irq(int bus,int dev,int func,int pin)
{
    return pin;
}

static int pci_ls3a_map_irq(PCIDevice *d, int irq_num)
{
    int dev = (d->devfn >> 3) & 0x1f;
    int func = d->devfn & 7;

    return board_map_irq(0,dev,func,irq_num);
}

static void pci_ls3a_set_irq(void *opaque, int irq_num, int level)
{
    qemu_irq *pic = opaque;
    qemu_set_irq(pic[irq_num], level);
}

#if 0
static void rs780_pcihost_class_init(ObjectClass *klass, void *data)
{
    //DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = rs780_pcihost_initfn;
}
#endif

static const TypeInfo rs780_pcihost_info = {
    .name          = TYPE_RS780_PCI_HOST_BRIDGE,
    .parent        = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(Rs780State),
    //    .class_init    = rs780_pcihost_class_init,
};

static PCIBus *pci_ls3a_init(DeviceState *dev, qemu_irq *pic)
{
    PCIBus *b;
    PCIHostState *phb;

    /* map PCI memory into low part of HT1 Lo memory space,
     * To check: how about 64 bit PCI?
     */
    memory_region_init_io(&s.pci_mem, NULL, &unassigned_pci_mem_ops, &s, 
                          "PCI memory space", 1ULL << 32);
    address_space_init(&s.pci_mem_as, &s.pci_mem, "PCI memory address space");

    memory_region_init_alias(&s.pci_mem_alias, NULL, "PCI MEM space alias", 
                             &s.pci_mem, 0x0, 1ULL << 32);
    memory_region_add_subregion(&s.ht1_lo_mem, 0x0, &s.pci_mem_alias);

    /* use system_io as pci io, map it into ht1_lo_io */
    s.pci_io_p = get_system_io();
    memory_region_init_alias(&s.pci_io_alias, NULL, "PCI IO space alias", 
                             get_system_io(), 0x0, 
                             memory_region_size(get_system_io()));
    memory_region_add_subregion(&s.ht1_lo_io, 0x0, &s.pci_io_alias);

    s.pci_irq = pic;

    phb = PCI_HOST_BRIDGE(dev);

    phb->bus = b = pci_register_bus(dev, "pci", pci_ls3a_set_irq, 
            pci_ls3a_map_irq, pic, &s.pci_mem, s.pci_io_p, 
            PCI_DEVFN(0, 0), 4, TYPE_PCI_BUS);

    return b;
}

/* 780e devices */

/* host bridge */
#if 0
static int rs780_init(PCIDevice *dev)
{
    return 0;
}
#endif

static void rs780_pci_realize(PCIDevice *d, Error **errp)
{
    pci_set_word(d->config + PCI_COMMAND, 0);
    pci_set_word(d->config + PCI_STATUS,
                 PCI_STATUS_FAST_BACK | PCI_STATUS_DEVSEL_MEDIUM);
    pci_config_set_prog_interface(d->config, 0);
}

static void rs780_pci_class_init(ObjectClass *klass, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->realize = rs780_pci_realize;
    k->vendor_id = PCI_VENDOR_ID_AMD;
    k->device_id = PCI_DEVICE_ID_AMD_780E;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_BRIDGE_HOST;
    /*
     * PCI-facing part of the host bridge, not usable without the
     * host-facing part, which can't be device_add'ed, yet.
     */
    dc->cannot_instantiate_with_device_add_yet = true;
}

static const TypeInfo rs780_pci_info = {
    .name          = "rs780_pci",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCIDevice),
    .class_init    = rs780_pci_class_init,
};

/* machine init entrance */
static void mips_lm1507_init(MachineState *machine)
{
    ram_addr_t ram_size = machine->ram_size;
    const char *cpu_model = machine->cpu_model;
    const char *kernel_filename = machine->kernel_filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    const char *initrd_filename = machine->initrd_filename;
    char *filename;
    MemoryRegion *address_space_mem = get_system_memory();
    long bios_size;
    DriveInfo *dinfo;
    int64_t kernel_entry;
    CPUMIPSState *env;
    MIPSCPU *cpu;
    //I2CBus *i2cbus;
    qemu_irq *i8259;
    DeviceState *dev;
    //MemoryRegion *mr;
    qemu_irq irq;
    int i;

    /* init CPUs */
    if (cpu_model == NULL) {
        cpu_model = "Loongson-3A2000";
    }

    for(i = 0; i < smp_cpus; i++) {
        printf("==== init smp_cpus=%d ====\n", i);
        cpu = cpu_mips_init(cpu_model);
        if (cpu == NULL) {
            fprintf(stderr, "Unable to find CPU definition\n");
            exit(1);
        }

        env = &cpu->env;
        s.mycpu[i] = env;

        env->CP0_EBase |= i;

        qemu_register_reset(main_cpu_reset, cpu);

        /* Init CPU internal devices */
        cpu_mips_irq_init_cpu(env);
        cpu_mips_clock_init(env);

        s.intc.core[i].irq = env->irq[6];
    }
    env = s.mycpu[0];

    /* hard coded now, TODO */
    ram_size = 1024 * 1024 * 1024;
    s.ram_size = ram_size;

    /* allocate RAM */
    /* system dram is not accessible till inited 
       Mem controller's address space is overlapped with memory
    */
    s.memory_initialized = 0;
    memory_region_allocate_system_memory(&s.ram, NULL, "lm1507.ram",
            s.ram_size);
    memory_region_add_subregion(get_system_memory(), 0, &s.ram);

    /* memory above 256M mapped to 0x400000000 */
    memory_region_init_alias(&s.ram_hi, NULL, "lm1507.ram_hi", 
            &s.ram, 0x10000000, s.ram_size - 0x10000000);
    memory_region_add_subregion(get_system_memory(), 0x400000000ULL, 
            &s.ram_hi);
    /* ??? to check */
    memory_region_init_alias(&s.ram_hi2, NULL, "lm1507.ram_hi2", 
            &s.ram, 0, s.ram_size);
    memory_region_add_subregion_overlap(get_system_memory(), 0x1000000000ULL, 
            &s.ram_hi2, 0);

    memory_region_init_io(&s.mc0_io, NULL, &mc0_io_ops, &s, 
                          "memory controller0 I/O", 0x1000);
    memory_region_add_subregion_overlap(get_system_memory(), 
                                LM1507_MC0_REG_BASE - KSEG0_BASE, 
                                &s.mc0_io, 1);

    memory_region_init_io(&s.mc1_io, NULL, &mc1_io_ops, &s, 
                          "memory controller1 I/O", 0x1000);
    memory_region_add_subregion_overlap(get_system_memory(), 
                                LM1507_MC1_REG_BASE - KSEG0_BASE, 
                                &s.mc1_io, 1);

    /* Try to load a BIOS image. If this fails, we continue regardless,
       but initialize the hardware ourselves. When a kernel gets
       preloaded we also initialize the hardware, since the BIOS wasn't
       run. */
    if (bios_name == NULL)
        bios_name = LM1507_BIOSNAME;
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        bios_size = get_image_size(filename);
    } else {
        bios_size = -1;
    }
    if (bios_size != -1) {
        memory_region_init_ram(&s.flash, NULL, "lm1507.flash", bios_size,
                &error_fatal);
        vmstate_register_ram_global(&s.flash);
        memory_region_set_readonly(&s.flash, true);

        memory_region_add_subregion(address_space_mem, 0x1c000000LL, &s.flash);
        load_image_targphys(filename, 0x1c000000, bios_size);
    } else if ((dinfo = drive_get(IF_PFLASH, 0, 0)) != NULL) {
        /* spi rom */
#if 1
        SSIBus *spi;
        SysBusDevice *busdev;

        s.spidev = qdev_create(NULL, "ls2h-spi");
        qdev_init_nofail(s.spidev);
        busdev = SYS_BUS_DEVICE(s.spidev);
        sysbus_mmio_map(busdev, 0, LM1507_SPI_REG_BASE - KSEG0_BASE);
        sysbus_mmio_map(busdev, 1, 0xbc000000 - KSEG0_BASE);
        /* delay it till interrupt controller inited */
        //sysbus_connect_irq(busdev, 0, irq[SPI_IRQ]);

        spi = (SSIBus *)qdev_get_child_bus(s.spidev, "spi");

        /* 2MB flash */
        dev = ssi_create_slave(spi, "sst25vf016b");
        irq = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        sysbus_connect_irq(busdev, 1, irq);

        /* tell slave to use our rom memory as internal storage */
        ssi_set_storage(spi, 
                memory_region_get_ram_ptr(busdev->mmio[1].memory));
#else
        /* if bios is connected via LPC flash, use this */
        uint32_t mips_rom = 0x00100000;
        if (!pflash_cfi01_register(0x1c000000, NULL, "lm1507.flash", mips_rom,
                                   blk_by_legacy_dinfo(dinfo),
                                   sector_len, mips_rom / sector_len,
                                   4, 0, 0, 0, 0, 0)) {
            fprintf(stderr, "qemu: Error registering flash memory.\n");
        }
#endif
    } else if (!qtest_enabled()) {
	/* not fatal */
        fprintf(stderr, "qemu: Warning, could not load MIPS bios '%s'\n",
		bios_name);
    }

    /* map [bc000000,bc100000) to [bfc00000, bfd00000)
     * we have only 1MB here due to architecture limit
     */
    memory_region_init_alias(&s.bios, NULL, "lm1507.bios", 
            get_system_memory(), 0x1c000000, 0x100000);
    memory_region_add_subregion(get_system_memory(), 0x1fc00000ULL, 
            &s.bios);

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        kernel_entry = load_kernel (env);
        write_bootloader(env, memory_region_get_ram_ptr(&s.bios), kernel_entry);
    }

#if 0
    /* set ssi irq */
    irq = qdev_get_gpio_in(s.intc_dev, 6);
    sysbus_connect_irq(SYS_BUS_DEVICE(s.spidev), 0, irq);
#endif

    /* init other devices */
    //DMA_init(0);

    /* SERIAL IO */
    if (serial_hds[0]) {
        qemu_irq *pirq;

        pirq = qemu_allocate_irqs(ls3a_serial_set_irq, &s, 1);

        serial_mm_init(get_system_memory(), LM1507_UART0_REG_BASE - KSEG0_BASE, 0,
                pirq[0], 115200, serial_hds[0], DEVICE_NATIVE_ENDIAN);
    }

    /* misc on chip IO 
     *   bfe00000-bfe00100: PCI/PCI-X conf
     *   bfe00100-bfe001e0: chip_config/chip_sample etc.
     */
    memory_region_init_io(&s.misc_io, NULL, &misc_io_ops, (void*)&s, 
                          "chip config io", 0x1e0);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_MISC_IO_REG_BASE - KSEG0_BASE, 
                                &s.misc_io);

    /* chip configuration
     *   3ff00000-3ff10000: window setting etc.
     *   map to system ram space [0x900000003ff00000, 0x900000003ff10000)
     *   To handle IPI, the range of 3ff01000 - 0x3ff02000 might overlap
     */
    memory_region_init_io(&s.creg_io, NULL, &creg_io_ops, (void*)&s, 
                          "chip config io", 0x10000);
    memory_region_add_subregion_overlap(get_system_memory(), 
                                LM1507_CHIP_CFG_REG_BASE - UNCACHED_BASE64, 
                                &s.creg_io, 0);


    /* HT1 Lo address space
     *   [0x90000e0000000000, 0x90000f0000000000)
     */
    memory_region_init(&s.ht1_lo, NULL, "ht1_lo", 1ULL << 40);
    address_space_init(&s.ht1_lo_as, &s.ht1_lo, "HT1 Lo address space");

    /* HT1 Lo mem
     * [0x90000e0000000000, 0x90000efd00000)
     */

    /* add to HT space */
    memory_region_init(&s.ht1_lo_mem, NULL, "HT1_Lo memory space", 0xfd00000000);
    memory_region_add_subregion(&s.ht1_lo, 0, &s.ht1_lo_mem);

    /* map to system memory space */
    memory_region_init_alias(&s.ht1_lo_mem_alias, NULL, "HT1_Lo memory space", 
                             &s.ht1_lo_mem, 0,  0xfd00000000);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_HT_MEM_REG_BASE - UNCACHED_BASE64, 
                                &s.ht1_lo_mem_alias);
    /* also mapped to [10000000, 11000000) for legacy devices */
    memory_region_init_alias(&s.isa_mem, NULL, "ISA memory", &s.ht1_lo_mem, 
                          0, 0x1000000);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_ISA_MEM_REG_BASE - KSEG0_BASE, 
                                &s.isa_mem);

    /* HT1 Lo controller config
     *   [0x90000efdfb000000, 0x90000efdfc000000)
     */
    memory_region_init_io(&s.ht1_lo_ctlconf, NULL, &ht_ctlconf_ops, (void*)&s, 
                          "HT1 Lo controller config io", 0x1000000);
    /* add to HT space */
    memory_region_add_subregion(&s.ht1_lo, 0xfdfb000000, &s.ht1_lo_ctlconf);
    /* add to system memory space */
    memory_region_init_alias(&s.ht1_lo_ctlconf_alias, NULL, "HT1_Lo control config space", 
                             &s.ht1_lo_ctlconf, 0x0, 0x1000000);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_HT_CTLCONF_REG_BASE - UNCACHED_BASE64, 
                                &s.ht1_lo_ctlconf_alias);

    /* HT1 Lo IO
     *   [0x90000efdfc000000, 0x90000efdfd000000)
     */
    memory_region_init_io(&s.ht1_lo_io, NULL, &unassigned_ht_io_ops, (void*)&s, 
                          "HT1 Lo IO", 0x1000000);
   

    /* add to HT space */
    memory_region_add_subregion(&s.ht1_lo, 0xfdfc000000, &s.ht1_lo_io);
    /* map to system memory space */
    memory_region_init_alias(&s.ht1_lo_io_alias, NULL, "HT1_Lo IO space", 
                             &s.ht1_lo_io, 0x0, 0x1000000);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_HT_IO_REG_BASE - UNCACHED_BASE64, 
                                &s.ht1_lo_io_alias);

    /* also map to [0xb8000000, 0xb9000000) */
    memory_region_init_alias(&s.ht1_lo_io_alias32, NULL, "HT IO 32", &s.ht1_lo_io, 
                          0, 0x1000000);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_HT_IO_REG_BASE32 - KSEG0_BASE, 
                                &s.ht1_lo_io_alias32);

    /* HT1 Lo bus config
     *   [0x90000efdfe000000, 0x90000efe00000000)
     */
    memory_region_init_io(&s.ht1_lo_busconf, NULL, &ht_busconf_ops, (void*)&s, 
                          "HT bus config io", 0x2000000);
    /* add to HT space */
    memory_region_add_subregion(&s.ht1_lo, 0xfdfe000000, &s.ht1_lo_busconf);

    /* add to system memory space */
    memory_region_init_alias(&s.ht1_lo_busconf_alias, NULL, "HT1_Lo bus config space", 
                             &s.ht1_lo_busconf, 0x0, 0x2000000);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_HT_BUSCONF_REG_BASE - UNCACHED_BASE64, 
                                &s.ht1_lo_busconf_alias);
    /* also mapped to [ba000000, bc000000) */
    memory_region_init_alias(&s.ht1_lo_busconf_alias32, NULL, "HT busconf 32", &s.ht1_lo_busconf, 
                          0, 0x2000000);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_HT_BUSCONF_REG_BASE32 - KSEG0_BASE, 
                                &s.ht1_lo_busconf_alias32);

    s.isa_bus = isa_bus_new(NULL, &s.isa_mem, get_system_io(), &error_abort);
    i8259 = ls3a_intctl_init(s.isa_bus, &s);
    /* The PIC is attached to the MIPS CPU INT0 pin */
    isa_bus_irqs(s.isa_bus, i8259);

    dev = qdev_create(NULL, TYPE_RS780_PCI_HOST_BRIDGE);
    qdev_init_nofail(dev);

    s.pci_bus = pci_ls3a_init(dev, &i8259[5]);

    //pci_create_simple(s.pci_bus, PCI_DEVFN(0x11, 0), "ich9-ahci");
    pci_create_simple(s.pci_bus, PCI_DEVFN(0x12, 0), "pci-ohci");

    /* Temp hack, should move to pci emulation. 
       Handle 0xeee0 smbus access */
    memory_region_init_io(&s.smbus_io, NULL, &smbus_io_ops, &s, 
                          "SMBus I/O", 0x8);
    memory_region_add_subregion(get_system_io(), 
                                LM1507_SMBUS_REG_BASE - LM1507_IO_REG_BASE, 
                                &s.smbus_io);

#if 0
    /* SATA IO */
    dev = qdev_create(NULL, "sysbus-ahci");
    qdev_prop_set_uint32(dev, "num-ports", 2);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0,
            LM1507_SATA_REG_BASE - KSEG0_BASE);
    irq = qdev_get_gpio_in(s.intc_dev, 37);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

    /* GPU REG IO */
    memory_region_init_alias(&s.gpu_mem, NULL, "GPU reg io mem",
                             get_system_io(), 
                             LM1507_GPU_REG_BASE - LM1507_IO_REG_BASE, 
                             0x800);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_GPU_REG_BASE - KSEG0_BASE, 
                                &s.gpu_mem);
    memory_region_init_io(&s.gpu_io, NULL, &gpu_io_ops, (void*)&s, 
                          "GPU io", 0x800);
    memory_region_add_subregion(get_system_io(), 
				LM1507_GPU_REG_BASE - LM1507_IO_REG_BASE, &s.gpu_io);

    /* nand REG IO */
    memory_region_init_alias(&s.nand_mem, NULL, "nand reg io mem",
                             get_system_io(), 
                             LM1507_NAND_REG_BASE - LM1507_IO_REG_BASE, 
                             0x800);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_NAND_REG_BASE - KSEG0_BASE, 
                                &s.nand_mem);
    memory_region_init_io(&s.nand_io, NULL, &nand_io_ops, (void*)&s, 
                          "nand io", 0x800);
    memory_region_add_subregion(get_system_io(), 
				LM1507_NAND_REG_BASE - LM1507_IO_REG_BASE, &s.nand_io);

    /* I2C0 IO */
    irq = qdev_get_gpio_in(s.intc_dev, 7);
    s.i2c0_dev = sysbus_create_simple("lm1507-i2c", 
                                       LM1507_I2C0_REG_BASE - KSEG0_BASE, irq);
    i2cbus = (I2CBus *)qdev_get_child_bus(s.i2c0_dev, "i2c");
    i2c_create_slave(i2cbus, "lm1507-spd", 0xa8);

    /* I2C1 IO */
    irq = qdev_get_gpio_in(s.intc_dev, 8);
    s.i2c1_dev = sysbus_create_simple("lm1507-i2c", 
                                       LM1507_I2C1_REG_BASE - KSEG0_BASE, irq);
    i2cbus = (I2CBus *)qdev_get_child_bus(s.i2c1_dev, "i2c");
    i2c_create_slave(i2cbus, "lm1507-macrom", 0xa0);
    i2c_create_slave(i2cbus, "lm1507-edidrom", 0x50);

    /* ACPI */
    memory_region_init_alias(&s.acpi_mem, NULL, "ACPI I/O mem",
                             get_system_io(), 
                             LM1507_ACPI_REG_BASE - LM1507_IO_REG_BASE, 
                             0x8000);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_ACPI_REG_BASE - KSEG0_BASE, 
                                &s.acpi_mem);

    memory_region_init_io(&s.acpi_io, NULL, &acpi_io_ops, &s, 
                          "ACPI I/O", 0x8000);
    memory_region_add_subregion(get_system_io(), 
				LM1507_ACPI_REG_BASE - LM1507_IO_REG_BASE,
				&s.acpi_io);

    /* RTC */
    {
        qemu_irq irq[8];
        int i;
        for (i = 0; i < 8; i++)
            irq[i] = qdev_get_gpio_in(s.intc_dev, 14 + i);
        sysbus_create_varargs("lm1507-rtc", LM1507_RTC_REG_BASE - KSEG0_BASE, 
           irq[0], irq[1], irq[2], irq[3], irq[4], irq[5], irq[6], irq[7]
           , NULL);
    }

    /* ohci */
    dev = qdev_create(NULL, "sysbus-ohci");
    qdev_prop_set_uint32(dev, "num-ports", 6);
    qdev_prop_set_uint64(dev, "dma-offset", 0);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0,
            LM1507_OHCI_REG_BASE - KSEG0_BASE);
    irq = qdev_get_gpio_in(s.intc_dev, 33);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

    /* ehci */
    irq = qdev_get_gpio_in(s.intc_dev, 32);
    sysbus_create_simple("lm1507-ehci-usb", LM1507_EHCI_REG_BASE - KSEG0_BASE, irq);

    /* gmac */
    if (nd_table[0].used) {
        /* replace default nic */
        qemu_check_nic_model(&nd_table[0], "stmmac");
        dev = qdev_create(NULL, "stmmac");
        qdev_set_nic_properties(dev, &nd_table[0]);
        qdev_init_nofail(dev);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 
                        LM1507_GMAC0_REG_BASE - KSEG0_BASE);
        irq = qdev_get_gpio_in(s.intc_dev, 35);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

        qemu_check_nic_model(&nd_table[1], "stmmac");
        dev = qdev_create(NULL, "stmmac");
        qdev_set_nic_properties(dev, &nd_table[1]);
        qdev_init_nofail(dev);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 
                        LM1507_GMAC1_REG_BASE - KSEG0_BASE);
        irq = qdev_get_gpio_in(s.intc_dev, 36);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    } else {
        nd_table[0].used = true;
        /* hardcoded hack, should parse the options */
        //nd_table[0].netdev = qemu_find_netdev("net0");
        nd_table[0].netdev = net_hub_add_port(0, NULL);
        qemu_check_nic_model(&nd_table[0], "stmmac");
        dev = qdev_create(NULL, "stmmac");
        qdev_prop_set_uint32(dev, "phyaddr", 1);
        qdev_set_nic_properties(dev, &nd_table[0]);
        qdev_init_nofail(dev);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 
                        LM1507_GMAC0_REG_BASE - KSEG0_BASE);
        irq = qdev_get_gpio_in(s.intc_dev, 35);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

        nd_table[1].used = true;
        /* hardcoded hack, should parse the options */
        //nd_table[0].netdev = qemu_find_netdev("net0");
        nd_table[1].netdev = net_hub_add_port(0, NULL);
        qemu_check_nic_model(&nd_table[1], "stmmac");
        dev = qdev_create(NULL, "stmmac");
        qdev_prop_set_uint32(dev, "phyaddr", 2);
        qdev_set_nic_properties(dev, &nd_table[1]);
        qdev_init_nofail(dev);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 
                        LM1507_GMAC1_REG_BASE - KSEG0_BASE);
        irq = qdev_get_gpio_in(s.intc_dev, 36);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    }

    /* display controller */
    irq = qdev_get_gpio_in(s.intc_dev, 39);
    sysbus_create_simple("lm1507-dc", LM1507_DC_REG_BASE - KSEG0_BASE, irq);

    /* LPC; keyboard/mouse etc. */
    irq = qdev_get_gpio_in(s.intc_dev, 13);
    s.lpc_dev = sysbus_create_simple("lm1507-lpc", 
                                     LM1507_LPC_REG_BASE - KSEG0_BASE, irq);
    for (i = 0; i < 16; i++) {
        s.isa_irqs[i] = qdev_get_gpio_in(s.lpc_dev, i);
    }

    memory_region_init_alias(&s.lpc_mem, NULL, "LPC I/O mem",
                             get_system_io(), 
                             0, /* note: offset 0 for isa */
                             0x1000);
    memory_region_add_subregion(get_system_memory(), 
                                LM1507_LPC_IO_BASE - KSEG0_BASE, 
                                &s.lpc_mem);

    /* we don't really has isabus; fake one to use pckeyboard emulation */
    isa_bus = isa_bus_new(NULL, &s.lpc_mem, get_system_io(), NULL);
    isa_bus_irqs(isa_bus, (qemu_irq *)&s.isa_irqs);
    isa_create_simple(isa_bus, "i8042");
#endif
}

static void mips_lm1507_machine_init(MachineClass *mc)
{
    mc->desc = "Lemote LX-1507 board";
    mc->init = mips_lm1507_init;
}

DEFINE_MACHINE("lm1507", mips_lm1507_machine_init)

/* register devices */
static void lm1507_register_types(void)
{
#if 0
    type_register_static(&lm1507_spd_info);
    type_register_static(&lm1507_macrom_info);
    type_register_static(&lm1507_edidrom_info);
    type_register_static(&lm1507_lpc_info);
    type_register_static(&lm1507_rtc_info);
#endif
    type_register_static(&rs780_pcihost_info);
    type_register_static(&rs780_pci_info);
}

type_init(lm1507_register_types)
