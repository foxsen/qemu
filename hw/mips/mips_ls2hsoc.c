/*
 * QEMU Loongson 2H reference board support
 *
 * Copyright (c) 2015 Fuxin Zhang (zhangfx@lemote.com)
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

/*
 * Loongson 2HSoC reference board, information(pmon/manuals etc.) refer to:
 *  http://wiki.loongnix.org/index.php/2HSoc%E5%BC%80%E5%8F%91%E6%9D%BF
 *
 */

#include "hw/hw.h"
#include "hw/char/serial.h"
#include "net/net.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/devices.h"
#include "hw/isa/isa.h"
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
#include "elf.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"
#include "sysemu/qtest.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "net/net.h"

extern NetClientState *net_hub_add_port(int hub_id, const char *name);

#include "ls2h.h"

//#define DEBUG_LS2HSOC

#if defined (DEBUG_LS2HSOC)
#  define DPRINTF(fmt, ...) \
    do { fprintf(stderr, "LS2HSoC: " fmt, ## __VA_ARGS__); } while (0)
#else
static inline GCC_FMT_ATTR(1, 2) int DPRINTF(const char *fmt, ...)
{
    return 0;
}
#endif

#define ENVP_ADDR       0x80002000l
#define ENVP_NB_ENTRIES	 	16
#define ENVP_ENTRY_SIZE	 	256

#define LS2H_BIOSNAME "pmon_ls2h.bin"

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
    /* TODO: 2H reset stuff */
    if (loaderparams.kernel_filename) {
        env->CP0_Status &= ~((1 << CP0St_BEV) | (1 << CP0St_ERL));
    }
}


static LS2hState s;

/* memory controller io */
static uint64_t mc_values[180];
static void mc_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("mc write addr %lx with val %lx size %d\n", addr, val, size);
    if (addr < 180 * 16) 
        mc_values[addr/16] = val;
}

static uint64_t mc_read(void *opaque, hwaddr addr, unsigned size)
{
    LS2hState *s = (LS2hState *)opaque;
    uint64_t val = -1;
    if (addr == 0x10) {
        /* bit 0 is for dll lock status */
        val = mc_values[1] | 0x1;
    } else if (addr == 0x960) {
        /* not clear from doc */
        val = mc_values[0x96] | 0x100;
    } else if (addr < 180 * 16) {
        val = mc_values[addr / 16];
    }

    DPRINTF("mc read addr %lx size %d,val=%lx,pc=%lx\n", addr, size, val,
            s->cpu->env.active_tc.PC);
    return val;
}

static const MemoryRegionOps mc_io_ops = {
    .read = mc_read,
    .write = mc_write,
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

/* chip config io */
static void creg_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("creg write addr %lx with val %lx size %d\n", addr, val, size);
    /* chip config0 */
    if (addr == 0x200) {
        /* bit 13 set is to close memory controller space */
        if (val & (1<<13)) {
            memory_region_set_enabled(&s.mc_mem, 0);

            DPRINTF("memory space enabled\n");
        }else {
            memory_region_set_enabled(&s.mc_mem, 1);
            DPRINTF("memory space disabled\n");
        }
    } else if (addr == 0x84200) {
        if (val != 0) {
            uint64_t size = (~s.scache0_mask & ((1LL<< PA_BITS) - 1)) + 1;
            s.scache0_addr = val & ( (1LL << PA_BITS) - 1);
            fprintf(stderr, "enable scache access %lx %lx\n", s.scache0_addr, size);

            memory_region_init_ram(&s.scache0_ram, NULL, "ls2h.scache", 
                    size, &error_fatal);

            memory_region_add_subregion_overlap(get_system_memory(), 
                    s.scache0_addr, &s.scache0_ram, 1);
        } else {
            DPRINTF("disable scache locked access\n");
            memory_region_del_subregion(get_system_memory(), &s.scache0_ram);
            memory_region_unref(&s.scache0_ram);
        }

    } else if (addr == 0x84240) {
        s.scache0_mask = val;
    }
}

static uint64_t creg_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = -1LL;

    addr += LS2H_CHIP_CFG_REG_BASE;
    switch (addr) {
        case LS2H_GPIO_IN_REG: //board ver num
            val = 0xf00LL;
            break;
    }
    DPRINTF("creg read addr %lx size %d val %lx\n", addr, size, val);
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

/* GPU reg io */
static void gpu_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("gpu write addr %lx with val %lx size %d\n", addr, val, size);
}

static uint64_t gpu_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = -1LL;

    addr += LS2H_GPU_REG_BASE;
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

    addr += LS2H_NAND_REG_BASE;
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

/* sysbus LPC implementation */
#define TYPE_LS2H_LPC "ls2h-lpc"
#define LS2H_LPC(obj) OBJECT_CHECK(Ls2hLPCState, (obj), TYPE_LS2H_LPC)

typedef struct {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    qemu_irq parent_irq;

    int nirqs;

    /* cfg0: SIRQ en; cfg1: int_en; cfg2: int_src; cfg3: int_clr */
    uint32_t cfg[4];
} Ls2hLPCState;

static inline void ls2h_lpc_inth_update(Ls2hLPCState *s)
{
    if (s->cfg[1] & s->cfg[2]) {
        qemu_set_irq(s->parent_irq, 1);
    } else {
        qemu_set_irq(s->parent_irq, 0);
    }
}

static void ls2h_lpc_set_intr(void *opaque, int irq, int req)
{
    Ls2hLPCState *s = LS2H_LPC(opaque);

    int mask = 1 << irq;

    if (req) {
        s->cfg[2] |= mask;
    } else {
        s->cfg[2] &= ~mask;
    }

    ls2h_lpc_inth_update(s);
}

static uint64_t ls2h_lpc_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    Ls2hLPCState *s = LS2H_LPC(opaque);
    uint64_t value = s->cfg[(addr & 0xf) >> 2];
    return value;
}

static void ls2h_lpc_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    Ls2hLPCState *s = LS2H_LPC(opaque);

    s->cfg[(addr & 0xf) >> 2] = value;

    ls2h_lpc_inth_update(s);
}

static const MemoryRegionOps ls2h_lpc_mem_ops = {
    .read = ls2h_lpc_read,
    .write = ls2h_lpc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ls2h_lpc_reset(DeviceState *dev)
{
    Ls2hLPCState *s = LS2H_LPC(dev);
    int i;

    for (i = 0; i < 4; i++)
        s->cfg[i] = 0;
}

static int ls2h_lpc_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    Ls2hLPCState *s = LS2H_LPC(dev);

    s->nirqs = 18;
    sysbus_init_irq(sbd, &s->parent_irq);
    qdev_init_gpio_in(dev, ls2h_lpc_set_intr, s->nirqs);
    memory_region_init_io(&s->mmio, OBJECT(s), &ls2h_lpc_mem_ops, s,
                          "ls2h-lpc", 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);

    return 0;
}

static Property ls2h_lpc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void ls2h_lpc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls2h_lpc_init;
    dc->reset = ls2h_lpc_reset;
    dc->props = ls2h_lpc_properties;
}

static const TypeInfo ls2h_lpc_info = {
    .name          = "ls2h-lpc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .class_init    = ls2h_lpc_class_init,
    .instance_size = sizeof(Ls2hLPCState),
};

/** I2C slave implementations **/
typedef enum i2c_state { I2C_STOP, I2C_SEND, I2C_RECV } i2c_state_t;

/* I2C slave implementation of memory SPD eeprom */
#define TYPE_LS2H_SPD "ls2h-spd"
#define LS2H_SPD(obj) OBJECT_CHECK(Ls2hSPDState, (obj), TYPE_LS2H_SPD)

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
} Ls2hSPDState;

static int ls2h_spd_send(I2CSlave *i2c, uint8_t data)
{
    Ls2hSPDState *s = LS2H_SPD(i2c);
    if (s->state == I2C_SEND) 
        s->offset = data;
    else
        fprintf(stderr, "data sent in wrong state %d\n", s->state);
    return 0;
}

static void ls2h_spd_event(I2CSlave *i2c, enum i2c_event event)
{
    Ls2hSPDState *s = LS2H_SPD(i2c);
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

static int ls2h_spd_recv(I2CSlave *i2c)
{
    int ret;
    Ls2hSPDState *s = LS2H_SPD(i2c);

    if (s->state != I2C_RECV) {
        fprintf(stderr, "i2c receive in wrong state %d\n", s->state);
        ret = -1;
    } else 
        ret = s->eeprom[s->offset & 0x7f];
    DPRINTF("spd recv %d at %d\n", ret, s->offset);
    return ret;
}

static int ls2h_spd_init(I2CSlave *i2c)
{
    Ls2hSPDState *s = LS2H_SPD(i2c);

    s->offset = 0;
    s->state = I2C_STOP;
    s->eeprom = eeprom_spd;
    return 0;
}

static void ls2h_spd_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = ls2h_spd_init;
    k->event = ls2h_spd_event;
    k->recv = ls2h_spd_recv;
    k->send = ls2h_spd_send;
}

static const TypeInfo ls2h_spd_info = {
    .name          = TYPE_LS2H_SPD,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Ls2hSPDState),
    .class_init    = ls2h_spd_class_init,
};

/* I2C slave implementation for gmac eeprom */
#define TYPE_LS2H_MACROM "ls2h-macrom"
#define LS2H_MACROM(obj) OBJECT_CHECK(Ls2hMacromState, (obj),TYPE_LS2H_MACROM)

/* mac0/mac1 eeprom, only the first 12 bytes emulated */
static const unsigned char eeprom_mac[] = 
{ 0x00, 0x23, 0x9e, 0x00, 0x01, 0x02, 0x00, 0x23, 0x9e, 0x00, 0x01, 0x03 };

typedef struct {
    I2CSlave parent_obj;

    i2c_state_t state;
    uint8_t offset;
    int len;

    const uint8_t *eeprom;
} Ls2hMacromState;

static int ls2h_macrom_send(I2CSlave *i2c, uint8_t data)
{
    Ls2hMacromState *s = LS2H_MACROM(i2c);
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

static void ls2h_macrom_event(I2CSlave *i2c, enum i2c_event event)
{
    Ls2hMacromState *s = LS2H_MACROM(i2c);
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

static int ls2h_macrom_recv(I2CSlave *i2c)
{
    int ret;
    Ls2hMacromState *s = LS2H_MACROM(i2c);

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

static int ls2h_macrom_init(I2CSlave *i2c)
{
    Ls2hMacromState *s = LS2H_MACROM(i2c);

    s->offset = 0;
    s->state = I2C_STOP;
    s->len = 0;
    s->eeprom = eeprom_mac;

    return 0;
}

static void ls2h_macrom_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = ls2h_macrom_init;
    k->event = ls2h_macrom_event;
    k->recv = ls2h_macrom_recv;
    k->send = ls2h_macrom_send;
}

static const TypeInfo ls2h_macrom_info = {
    .name          = TYPE_LS2H_MACROM,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Ls2hMacromState),
    .class_init    = ls2h_macrom_class_init,
};

/* I2C slave implementation for EDID data */
#define TYPE_LS2H_EDIDROM "ls2h-edidrom"
#define LS2H_EDIDROM(obj) OBJECT_CHECK(Ls2hEDIDromState, (obj),TYPE_LS2H_EDIDROM)

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
} Ls2hEDIDromState;

static int ls2h_edidrom_send(I2CSlave *i2c, uint8_t data)
{
    Ls2hEDIDromState *s = LS2H_EDIDROM(i2c);
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

static void ls2h_edidrom_event(I2CSlave *i2c, enum i2c_event event)
{
    Ls2hEDIDromState *s = LS2H_EDIDROM(i2c);
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

static int ls2h_edidrom_recv(I2CSlave *i2c)
{
    int ret;
    Ls2hEDIDromState *s = LS2H_EDIDROM(i2c);

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

static int ls2h_edidrom_init(I2CSlave *i2c)
{
    Ls2hEDIDromState *s = LS2H_EDIDROM(i2c);

    s->offset = 0;
    s->state = I2C_STOP;
    s->len = 0;
    s->eeprom = eeprom_edid;

    return 0;
}

static void ls2h_edidrom_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = ls2h_edidrom_init;
    k->event = ls2h_edidrom_event;
    k->recv = ls2h_edidrom_recv;
    k->send = ls2h_edidrom_send;
}

static const TypeInfo ls2h_edidrom_info = {
    .name          = TYPE_LS2H_EDIDROM,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Ls2hEDIDromState),
    .class_init    = ls2h_edidrom_class_init,
};

/* acpi */
static void acpi_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("acpi write addr %lx with val %lx size %d\n", addr, val, size);
    addr += LS2H_ACPI_REG_BASE;
    switch (addr) {
        case LS2H_PM1_STS_REG:
            break;
        case LS2H_PM1_CNT_REG:
            if (val == 0x3c00)
                qemu_system_shutdown_request();
            break;
        case LS2H_RST_CNT_REG:
            if (val & 1)
                qemu_system_reset_request();
            break;
    }
}

static uint64_t acpi_read(void *opaque, hwaddr addr, unsigned size)
{
    DPRINTF("acpi read addr %lx size %d\n", addr, size);
    return 0;
}

static const MemoryRegionOps acpi_io_ops = {
    .read = acpi_read,
    .write = acpi_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* sysbus RTC implementation */
#define TYPE_LS2H_RTC "ls2h-rtc"
#define LS2H_RTC(obj) OBJECT_CHECK(Ls2hRTCState, (obj), TYPE_LS2H_RTC)

typedef struct {
    SysBusDevice parent_obj;

    qemu_irq irq[8];

    MemoryRegion io;

    uint32_t toy_trim, toy_lo, toy_hi, toy_match0, toy_match1, toy_match2;
    uint32_t rtc_trim, rtc_count, rtc_match0, rtc_match1, rtc_match2, rtc_ctrl;
} Ls2hRTCState;

/* todo: interrupt */
static void rtc_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    Ls2hRTCState *s = LS2H_RTC(opaque);

    uint64_t vaddr = addr + LS2H_RTC_REG_BASE;
    DPRINTF("rtc write addr %lx with val %lx size %d\n", addr, val, size);
    switch (vaddr) {
        case LS2H_TOY_TRIM_REG:
            s->toy_trim = (uint32_t) val;
            break;
        case LS2H_TOY_WRITE0_REG:
            s->toy_lo = (uint32_t) val;
            break;
        case LS2H_TOY_WRITE1_REG:
            s->toy_hi = (uint32_t) val;
            break;
        case LS2H_TOY_MATCH0_REG:
            s->toy_match0 = (uint32_t) val;
            break;
        case LS2H_TOY_MATCH1_REG:
            s->toy_match1 = (uint32_t) val;
            break;
        case LS2H_TOY_MATCH2_REG:
            s->toy_match2 = (uint32_t) val;
            break;
        case LS2H_RTC_CTRL_REG:
            s->rtc_ctrl = (uint32_t) val;
            break;
        case LS2H_RTC_TRIM_REG:
            s->rtc_trim = (uint32_t) val;
            break;
        case LS2H_RTC_WRITE0_REG:
            s->rtc_count = (uint32_t) val;
            break;
        case LS2H_RTC_MATCH0_REG:
            s->rtc_match0 = (uint32_t) val;
            break;
        case LS2H_RTC_MATCH1_REG:
            s->rtc_match1 = (uint32_t) val;
            break;
        case LS2H_RTC_MATCH2_REG:
            s->rtc_match2 = (uint32_t) val;
            break;
        default:
            printf("invalid rtc address %lx\n", vaddr);
    }
}

static uint64_t rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    Ls2hRTCState *s = LS2H_RTC(opaque);
    uint64_t vaddr = addr + LS2H_RTC_REG_BASE;
    uint64_t val = -1;
    struct tm tm;

    qemu_get_timedate(&tm, 0);

    switch (vaddr) {
        case LS2H_TOY_TRIM_REG:
            val = s->toy_trim; 
            break;
        case LS2H_TOY_READ0_REG:
            val = ( (((tm.tm_mon + 1) & 0x3f) << 26) |
                    ((tm.tm_mday & 0x1f) << 21) |
                    ((tm.tm_hour & 0x1f) << 16) |
                    ((tm.tm_min & 0x3f) << 10)  | 
                    ((tm.tm_sec & 0x3f) << 4) ); 
            DPRINTF("mon %d day %d\n", tm.tm_mon, tm.tm_mday);
            break;
        case LS2H_TOY_READ1_REG:
            val = tm.tm_year;
            DPRINTF("year %d\n", tm.tm_year);
            break;
        case LS2H_TOY_MATCH0_REG:
            val = s->toy_match0;
            break;
        case LS2H_TOY_MATCH1_REG:
            val = s->toy_match1;
            break;
        case LS2H_TOY_MATCH2_REG:
            val = s->toy_match2;
            break;
        case LS2H_RTC_TRIM_REG:
            val = s->rtc_trim;
            break;
        case LS2H_RTC_READ0_REG:
            val = s->rtc_count;
            break;
        case LS2H_RTC_MATCH0_REG:
            val = s->rtc_match0;
            break;
        case LS2H_RTC_MATCH1_REG:
            val = s->rtc_match1;
            break;
        case LS2H_RTC_MATCH2_REG:
            val = s->rtc_match2;
            break;
        default:
            printf("invalid rtc address %lx\n", vaddr);
    }

    DPRINTF("RTC read addr %lx size %d,val=%lx\n", addr, size, val);
    return val;
}

static const MemoryRegionOps ls2h_rtc_ops = {
    .read = rtc_read,
    .write = rtc_write,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void ls2h_rtc_reset(DeviceState *dev)
{
    //Ls2hRTCState *s = LS2H_RTC(dev);
}

static int ls2h_rtc_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    Ls2hRTCState *s = LS2H_RTC(dev);
    int i;

    for (i = 0; i < 8; i++)
        sysbus_init_irq(sbd, &s->irq[i]);
    memory_region_init_io(&s->io, OBJECT(s), &ls2h_rtc_ops, s,
                          "ls2h-rtc", 0x8000);
    sysbus_init_mmio(sbd, &s->io);

    return 0;
}

static Property ls2h_rtc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void ls2h_rtc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls2h_rtc_init;
    dc->reset = ls2h_rtc_reset;
    dc->props = ls2h_rtc_properties;
}

static const TypeInfo ls2h_rtc_info = {
    .name          = "ls2h-rtc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .class_init    = ls2h_rtc_class_init,
    .instance_size = sizeof(Ls2hRTCState),
};

/* machine init entrance */
static void mips_ls2h_init(MachineState *machine)
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
    I2CBus *i2cbus;
    ISABus *isabus;
    DeviceState *dev;
    MemoryRegion *mr;
    qemu_irq irq;
    int i;

    /* init CPUs */
    if (cpu_model == NULL) {
        cpu_model = "Loongson-2H";
    }
    s.cpu = cpu_mips_init(cpu_model);
    if (s.cpu == NULL) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    env = &(s.cpu->env);

    qemu_register_reset(main_cpu_reset, s.cpu);

    /* hard coded now, TODO */
    ram_size = 1024 * 1024 * 1024;
    s.ram_size = ram_size;

    /* allocate RAM */
    /* system dram is not accessible till inited 
       Mem controller's address space is overlapped with memory
    */
    s.memory_initialized = 0;
    memory_region_allocate_system_memory(&s.ram, NULL, "ls2h.ram",
            s.ram_size);
    memory_region_add_subregion(get_system_memory(), 0, &s.ram);

    /* memory above 256M mapped to 0x110000000 */
    memory_region_init_alias(&s.ram_hi, NULL, "ls2h.ram_hi", 
            &s.ram, 0x10000000, s.ram_size - 0x10000000);
    memory_region_add_subregion(get_system_memory(), 0x110000000ULL, 
            &s.ram_hi);
    memory_region_init_alias(&s.ram_hi2, NULL, "ls2h.ram_hi2", 
            &s.ram, 0, s.ram_size);
    memory_region_add_subregion_overlap(get_system_memory(), 0x1000000000ULL, 
            &s.ram_hi2, 0);

    memory_region_init_io(&s.mc_io, NULL, &mc_io_ops, &s, 
                          "memory controller I/O", 0x1000);
    /* this doesn't belongs to system_io */
    memory_region_init_alias(&s.mc_mem, NULL, "memory controller I/O mem",
                             &s.mc_io, 0, 0x1000);
    memory_region_add_subregion_overlap(get_system_memory(), 
                                LS2H_MC_REG_BASE - KSEG0_BASE, 
                                &s.mc_mem, 1);

    /* Try to load a BIOS image. If this fails, we continue regardless,
       but initialize the hardware ourselves. When a kernel gets
       preloaded we also initialize the hardware, since the BIOS wasn't
       run. */
    if (bios_name == NULL)
        bios_name = LS2H_BIOSNAME;
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        bios_size = get_image_size(filename);
    } else {
        bios_size = -1;
    }
    if (bios_size != -1) {
        memory_region_init_ram(&s.bios, NULL, "ls2h.bios", bios_size,
                &error_fatal);
        vmstate_register_ram_global(&s.bios);
        memory_region_set_readonly(&s.bios, true);

        memory_region_add_subregion(address_space_mem, 0x1fc00000LL, &s.bios);
        load_image_targphys(filename, 0x1fc00000, bios_size);
    } else if ((dinfo = drive_get(IF_PFLASH, 0, 0)) != NULL) {
        /* spi rom */
#if 1
        SSIBus *spi;
        SysBusDevice *busdev;

        s.spidev = qdev_create(NULL, "ls2h-spi");
        qdev_init_nofail(s.spidev);
        busdev = SYS_BUS_DEVICE(s.spidev);
        sysbus_mmio_map(busdev, 0, LS2H_SPI_REG_BASE - KSEG0_BASE);
        sysbus_mmio_map(busdev, 1, 0xbfc00000 - KSEG0_BASE);
        /* delay it till interrupt controller inited */
        //sysbus_connect_irq(busdev, 0, irq[SPI_IRQ]);

        spi = (SSIBus *)qdev_get_child_bus(s.spidev, "spi");

        dev = ssi_create_slave(spi, "sst25vf080b");
        irq = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        sysbus_connect_irq(busdev, 1, irq);

        /* tell slave to use our rom memory as internal storage */
        ssi_set_storage(spi, 
                memory_region_get_ram_ptr(busdev->mmio[1].memory));
#else
        /* if bios is connected via LPC flash, use this */
        uint32_t mips_rom = 0x00100000;
        if (!pflash_cfi01_register(0x1fc00000, NULL, "ls2h.bios", mips_rom,
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

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        kernel_entry = load_kernel (env);
        write_bootloader(env, memory_region_get_ram_ptr(&s.bios), kernel_entry);
    }

    /* Init internal devices */
    cpu_mips_irq_init_cpu(env);
    cpu_mips_clock_init(env);

    /* Interrupt controller */
    /* must end with a NULL; we take care of msr too 
     */
    s.intc_dev = sysbus_create_varargs("ls2h-intc", LS2H_MSI_PORT_REG - 
            KSEG0_BASE, env->irq[2], env->irq[3], env->irq[4], 
            env->irq[5], env->irq[6], NULL);
    /* set higher priority then general chip reg io */
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(s.intc_dev), 0);
    mr->may_overlap = true;
    mr->priority = 1;

    /* set ssi irq */
    irq = qdev_get_gpio_in(s.intc_dev, 6);
    sysbus_connect_irq(SYS_BUS_DEVICE(s.spidev), 0, irq);

    /* init other devices */
    //DMA_init(0);

    /* enlarge the default system IO size to include 
      [0x1f000000,0x1fffffff] */
    memory_region_set_size(get_system_io(), 0x1000000);

    /* SERIAL IO */
    memory_region_init_alias(&s.uart0_mem, NULL, "uart0_mmio",
                             get_system_io(),
                             LS2H_UART0_REG_BASE - LS2H_IO_REG_BASE,
                             0x00001000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_UART0_REG_BASE - KSEG0_BASE, 
                                &s.uart0_mem);
    if (serial_hds[0]) {
        irq = qdev_get_gpio_in(s.intc_dev, 2);
        serial_init(LS2H_UART0_REG_BASE - LS2H_IO_REG_BASE, irq, 
                    115200, serial_hds[0], get_system_io());
    }

    /* uart1 */
    memory_region_init_alias(&s.uart1_mem, NULL, "uart1_mmio",
                             get_system_io(),
                             LS2H_UART1_REG_BASE - LS2H_IO_REG_BASE,
                             0x00001000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_UART1_REG_BASE - KSEG0_BASE, 
                                &s.uart1_mem);
    if (serial_hds[1]) {
        irq = qdev_get_gpio_in(s.intc_dev, 3);
        serial_init(LS2H_UART1_REG_BASE - LS2H_IO_REG_BASE, irq, 
                    115200, serial_hds[1], get_system_io());
    }

    /* uart2 */
    memory_region_init_alias(&s.uart2_mem, NULL, "uart2_mmio",
                             get_system_io(),
                             LS2H_UART2_REG_BASE - LS2H_IO_REG_BASE,
                             0x00001000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_UART2_REG_BASE - KSEG0_BASE, 
                                &s.uart2_mem);
    if (serial_hds[2]) {
        irq = qdev_get_gpio_in(s.intc_dev, 4);
        serial_init(LS2H_UART2_REG_BASE - LS2H_IO_REG_BASE, irq, 
                    115200, serial_hds[2], get_system_io());
    }

    /* uart3 */
    memory_region_init_alias(&s.uart3_mem, NULL, "uart3_mmio",
                             get_system_io(),
                             LS2H_UART3_REG_BASE - LS2H_IO_REG_BASE,
                             0x00001000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_UART3_REG_BASE - KSEG0_BASE, 
                                &s.uart3_mem);
    if (serial_hds[3]) {
        irq = qdev_get_gpio_in(s.intc_dev, 5);
        serial_init(LS2H_UART3_REG_BASE - LS2H_IO_REG_BASE, irq, 
                    115200, serial_hds[3], get_system_io());
    }

    /* Chip config IO */
    memory_region_init_alias(&s.creg_mem, NULL, "chip config io mem",
                             get_system_io(), 
                             LS2H_CHIP_CFG_REG_BASE - LS2H_IO_REG_BASE, 
                             0x00100000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_CHIP_CFG_REG_BASE - KSEG0_BASE, 
                                &s.creg_mem);
    memory_region_init_io(&s.creg_io, NULL, &creg_io_ops, (void*)&s, 
                          "chip config io", 0x100000);
    memory_region_add_subregion_overlap(get_system_io(), 
				LS2H_CHIP_CFG_REG_BASE - LS2H_IO_REG_BASE, &s.creg_io, 0);

    /* SATA IO */
    dev = qdev_create(NULL, "sysbus-ahci");
    qdev_prop_set_uint32(dev, "num-ports", 2);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0,
            LS2H_SATA_REG_BASE - KSEG0_BASE);
    irq = qdev_get_gpio_in(s.intc_dev, 37);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

    /* GPU REG IO */
    memory_region_init_alias(&s.gpu_mem, NULL, "GPU reg io mem",
                             get_system_io(), 
                             LS2H_GPU_REG_BASE - LS2H_IO_REG_BASE, 
                             0x800);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_GPU_REG_BASE - KSEG0_BASE, 
                                &s.gpu_mem);
    memory_region_init_io(&s.gpu_io, NULL, &gpu_io_ops, (void*)&s, 
                          "GPU io", 0x800);
    memory_region_add_subregion(get_system_io(), 
				LS2H_GPU_REG_BASE - LS2H_IO_REG_BASE, &s.gpu_io);

    /* nand REG IO */
    memory_region_init_alias(&s.nand_mem, NULL, "nand reg io mem",
                             get_system_io(), 
                             LS2H_NAND_REG_BASE - LS2H_IO_REG_BASE, 
                             0x800);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_NAND_REG_BASE - KSEG0_BASE, 
                                &s.nand_mem);
    memory_region_init_io(&s.nand_io, NULL, &nand_io_ops, (void*)&s, 
                          "nand io", 0x800);
    memory_region_add_subregion(get_system_io(), 
				LS2H_NAND_REG_BASE - LS2H_IO_REG_BASE, &s.nand_io);

    /* I2C0 IO */
    irq = qdev_get_gpio_in(s.intc_dev, 7);
    s.i2c0_dev = sysbus_create_simple("ls2h-i2c", 
                                       LS2H_I2C0_REG_BASE - KSEG0_BASE, irq);
    i2cbus = (I2CBus *)qdev_get_child_bus(s.i2c0_dev, "i2c");
    i2c_create_slave(i2cbus, "ls2h-spd", 0xa8);

    /* I2C1 IO */
    irq = qdev_get_gpio_in(s.intc_dev, 8);
    s.i2c1_dev = sysbus_create_simple("ls2h-i2c", 
                                       LS2H_I2C1_REG_BASE - KSEG0_BASE, irq);
    i2cbus = (I2CBus *)qdev_get_child_bus(s.i2c1_dev, "i2c");
    i2c_create_slave(i2cbus, "ls2h-macrom", 0xa0);
    i2c_create_slave(i2cbus, "ls2h-edidrom", 0x50);

    /* ACPI */
    memory_region_init_alias(&s.acpi_mem, NULL, "ACPI I/O mem",
                             get_system_io(), 
                             LS2H_ACPI_REG_BASE - LS2H_IO_REG_BASE, 
                             0x8000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_ACPI_REG_BASE - KSEG0_BASE, 
                                &s.acpi_mem);

    memory_region_init_io(&s.acpi_io, NULL, &acpi_io_ops, &s, 
                          "ACPI I/O", 0x8000);
    memory_region_add_subregion(get_system_io(), 
				LS2H_ACPI_REG_BASE - LS2H_IO_REG_BASE,
				&s.acpi_io);

    /* RTC */
    {
        qemu_irq irq[8];
        int i;
        for (i = 0; i < 8; i++)
            irq[i] = qdev_get_gpio_in(s.intc_dev, 14 + i);
        sysbus_create_varargs("ls2h-rtc", LS2H_RTC_REG_BASE - KSEG0_BASE, 
           irq[0], irq[1], irq[2], irq[3], irq[4], irq[5], irq[6], irq[7]
           , NULL);
    }

    /* ohci */
    dev = qdev_create(NULL, "sysbus-ohci");
    qdev_prop_set_uint32(dev, "num-ports", 6);
    qdev_prop_set_uint64(dev, "dma-offset", 0);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0,
            LS2H_OHCI_REG_BASE - KSEG0_BASE);
    irq = qdev_get_gpio_in(s.intc_dev, 33);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

    /* ehci */
    irq = qdev_get_gpio_in(s.intc_dev, 32);
    sysbus_create_simple("ls2h-ehci-usb", LS2H_EHCI_REG_BASE - KSEG0_BASE, irq);

    /* gmac */
    if (nd_table[0].used) {
        /* replace default nic */
        qemu_check_nic_model(&nd_table[0], "stmmac");
        dev = qdev_create(NULL, "stmmac");
        qdev_set_nic_properties(dev, &nd_table[0]);
        qdev_init_nofail(dev);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 
                        LS2H_GMAC0_REG_BASE - KSEG0_BASE);
        irq = qdev_get_gpio_in(s.intc_dev, 35);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

        qemu_check_nic_model(&nd_table[1], "stmmac");
        dev = qdev_create(NULL, "stmmac");
        qdev_set_nic_properties(dev, &nd_table[1]);
        qdev_init_nofail(dev);
        sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 
                        LS2H_GMAC1_REG_BASE - KSEG0_BASE);
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
                        LS2H_GMAC0_REG_BASE - KSEG0_BASE);
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
                        LS2H_GMAC1_REG_BASE - KSEG0_BASE);
        irq = qdev_get_gpio_in(s.intc_dev, 36);
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    }

    /* display controller */
    irq = qdev_get_gpio_in(s.intc_dev, 39);
    sysbus_create_simple("ls2h-dc", LS2H_DC_REG_BASE - KSEG0_BASE, irq);

    /* LPC; keyboard/mouse etc. */
    irq = qdev_get_gpio_in(s.intc_dev, 13);
    s.lpc_dev = sysbus_create_simple("ls2h-lpc", 
                                     LS2H_LPC_REG_BASE - KSEG0_BASE, irq);
    for (i = 0; i < 16; i++) {
        s.isa_irqs[i] = qdev_get_gpio_in(s.lpc_dev, i);
    }

    memory_region_init_alias(&s.lpc_mem, NULL, "LPC I/O mem",
                             get_system_io(), 
                             0, /* note: offset 0 for isa */
                             0x1000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_LPC_IO_BASE - KSEG0_BASE, 
                                &s.lpc_mem);

    /* we don't really has isabus; fake one to use pckeyboard emulation */
    isabus = isa_bus_new(NULL, &s.lpc_mem, get_system_io(), NULL);
    isa_bus_irqs(isabus, (qemu_irq *)&s.isa_irqs);
    isa_create_simple(isabus, "i8042");


}

static void mips_ls2h_machine_init(MachineClass *mc)
{
    mc->desc = "Loongson 2H SoC reference board";
    mc->init = mips_ls2h_init;
}

DEFINE_MACHINE("ls2h", mips_ls2h_machine_init)

/* register devices */
static void ls2h_register_types(void)
{
    type_register_static(&ls2h_spd_info);
    type_register_static(&ls2h_macrom_info);
    type_register_static(&ls2h_edidrom_info);
    type_register_static(&ls2h_lpc_info);
    type_register_static(&ls2h_rtc_info);
}

type_init(ls2h_register_types)
