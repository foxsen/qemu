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

#define MAX_IDE_BUS 2

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

/* byte 3 == 0x2, udimm
   byte 4 == 0x2, 1Gb chip, 8 banks 
   byte 5 == 0x11, row 14bit, col 10bit
   byte 6 == 0x6, 1.2/1.35/1.5v ok
   byte 7 == 0x1, 8bit device width, 1rank
   byte 8 == 0x3, bus width 64
   Total capacity = sdram capicity / 8 * buswidth / device width * ranks 
                  = 1Gb / 8 * 64 / 8 * 1 = 1GB
        or (row + col)  + 3 (8 banks) + 3 ( 64/8 width) + 0(1rank) = 2^30
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
            DPRINTF("enable scache access %lx %lx\n", s.scache0_addr, size);

            memory_region_init_alias(&s.scache0_ram, NULL, "ls2h.scache", 
                    &s.ram, 0, s.ram_size);
            memory_region_add_subregion(get_system_memory(), s.scache0_addr, 
                    &s.scache0_ram);
        } else {
            DPRINTF("disable scache locked access\n");
            memory_region_del_subregion(get_system_memory(), &s.scache0_ram);
            memory_region_set_enabled(&s.scache0_ram, 0);
        }

#if 0
        memory_region_init_ram(&s.scache0_ram, NULL, "ls2h.scache", 
                size, &error_fatal);
        memory_region_add_subregion(get_system_memory(), s.scache0_addr, 
                &s.scache0_ram);
#endif
    } else if (addr == 0x84240) {
        s.scache0_mask = val;
    }
}

static uint64_t creg_read(void *opaque, hwaddr addr, unsigned size)
{
    DPRINTF("creg read addr %lx size %d\n", addr, size);
    uint64_t val = -1LL;

    addr += LS2H_CHIP_CFG_REG_BASE;
    switch (addr) {
        case LS2H_GPIO_IN_REG: //board ver num
            val = 0xf00LL;
            break;
    }
    return val;
}

static const MemoryRegionOps creg_io_ops = {
    .read = creg_read,
    .write = creg_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};


/* a minimal implentation just to keep current pmon going */
static void i2c0_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    LS2hState *s = (LS2hState *)opaque;
    DPRINTF("i2c0 write addr %lx with val %lx size %d\n", addr, val, size);
    if (addr == LS2H_I2C0_CR_REG - LS2H_I2C0_REG_BASE) {
        s->i2c0_status = val;
        if (val == (CR_START | CR_WRITE)) {
            s->i2c0_addr = s->i2c0_data;
        } else if (val == CR_WRITE) {
            s->i2c0_offset = s->i2c0_data;
        } else if (val == (CR_READ | CR_ACK)) {
            if ((s->i2c0_addr & 0xfe) == 0xa8) 
                s->i2c0_data = eeprom_spd[s->i2c0_offset & 0x7f];
            else
                s->i2c0_data = 0; //no dimm1 
            DPRINTF("read at offset %lx\n", s->i2c0_offset);
        } else if (val == CR_STOP) {
            s->i2c0_status = 0;
            s->i2c0_offset = 0;
            s->i2c0_data = 0;
        } else {
            fprintf(stderr, "invalid i2c cr value %lx ignored\n", val);
        }
    }else if (addr == LS2H_I2C0_TXR_REG - LS2H_I2C0_REG_BASE) {
        s->i2c0_data = val;
    }else if (addr > 5) {
        fprintf(stderr, "i2c write invalid\n");
    }
}

static uint64_t i2c0_read(void *opaque, hwaddr addr, unsigned size)
{
    LS2hState *s = (LS2hState *)opaque;
    uint64_t val = -1;
    if (addr == LS2H_I2C0_SR_REG - LS2H_I2C0_REG_BASE) {
        val = 0; // ~SR_TIP, always ready
    } else if (addr == LS2H_I2C0_RXR_REG - LS2H_I2C0_REG_BASE) {
        val = s->i2c0_data;
    } 
    DPRINTF("i2c read addr %lx size %d val=%lx\n", addr, size, val);
    return val;
}

static const MemoryRegionOps i2c0_io_ops = {
    .read = i2c0_read,
    .write = i2c0_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static unsigned char eeprom_gmac[8192];
enum i2c_state { I2C_START_R, I2C_START_W, I2C_WRITE1, I2C_WRITE2, 
    I2C_WRITEN, I2C_STOP }  state;

static void i2c1_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    LS2hState *s = (LS2hState *)opaque;
    DPRINTF("i2c1 write addr %lx with val %lx size %d\n", addr, val, size);
    if (addr == LS2H_I2C1_CR_REG - LS2H_I2C1_REG_BASE) {
        s->i2c1_status = val;
        if (val == (CR_START | CR_WRITE)) {
            s->i2c1_addr = s->i2c1_data;
            state = (s->i2c1_addr == 0xa0) ? I2C_START_W : I2C_START_R;
        } else if (val == CR_WRITE) {
            if (state == I2C_START_W) {
                state = I2C_WRITE1;
                s->i2c1_offset = s->i2c1_data << 8;
            } else if (state == I2C_WRITE1) {
                state = I2C_WRITE2;
                s->i2c1_offset |= s->i2c1_data;
            } else if (state == I2C_WRITE2) {
                eeprom_gmac[s->i2c1_offset & 0x1fff] = s->i2c1_data;
                s->i2c1_offset ++;
            } else {
                fprintf(stderr, "Invalid CR_WRITE seen at %lx\n",
                        s->cpu->env.active_tc.PC);
            }
        } else if (val & CR_READ) {
            if (state == I2C_START_R) {
                s->i2c1_data = eeprom_gmac[s->i2c1_offset & 0x1fff];
                s->i2c1_offset ++;
            } else {
                s->i2c1_data = 0; 
                fprintf(stderr, "Invalid CR_READ seen at %lx\n",
                        s->cpu->env.active_tc.PC);
            }
            DPRINTF("read at offset %lx\n", s->i2c1_offset);
        } else if (val == CR_STOP) {
            s->i2c1_addr = 0;
            s->i2c1_status = 0;
            s->i2c1_offset = 0;
            s->i2c1_data = 0;
            state = I2C_STOP;
        } else {
            fprintf(stderr, "invalid i2c cmd value\n");
        }
    }else if (addr == LS2H_I2C1_TXR_REG - LS2H_I2C1_REG_BASE) {
        s->i2c1_data = val;
    }else if (addr > 5) {
        fprintf(stderr, "i2c write invalid\n");
    }
}

static uint64_t i2c1_read(void *opaque, hwaddr addr, unsigned size)
{
    LS2hState *s = (LS2hState *)opaque;
    uint64_t val = -1;
    if (addr == LS2H_I2C1_SR_REG - LS2H_I2C1_REG_BASE) {
        val = 0; // ~SR_TIP, always ready
    } else if (addr == LS2H_I2C1_RXR_REG - LS2H_I2C1_REG_BASE) {
        val = s->i2c1_data;
    } 
    DPRINTF("i2c1 read addr %lx size %d val=%lx\n", addr, size, val);
    return val;
}

static const MemoryRegionOps i2c1_io_ops = {
    .read = i2c1_read,
    .write = i2c1_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};


/* acpi */
static void acpi_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    DPRINTF("acpi write addr %lx with val %lx size %d\n", addr, val, size);
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

/* rtc io */
static void rtc_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned int size)
{
    uint64_t vaddr = addr + LS2H_RTC_REG_BASE;
    DPRINTF("rtc write addr %lx with val %lx size %d\n", addr, val, size);
    switch (vaddr) {
        case LS2H_TOY_TRIM_REG:
            s.toy_trim = (unsigned int) val;
            break;
        case LS2H_TOY_WRITE0_REG:
            s.toy_lo = (unsigned int) val;
            break;
        case LS2H_TOY_WRITE1_REG:
            s.toy_hi = (unsigned int) val;
            break;
        case LS2H_TOY_MATCH0_REG:
            s.toy_match0 = (unsigned int) val;
            break;
        case LS2H_TOY_MATCH1_REG:
            s.toy_match1 = (unsigned int) val;
            break;
        case LS2H_TOY_MATCH2_REG:
            s.toy_match2 = (unsigned int) val;
            break;
        case LS2H_RTC_CTRL_REG:
            s.rtc_ctrl = (unsigned int) val;
            break;
        case LS2H_RTC_TRIM_REG:
            s.rtc_trim = (unsigned int) val;
            break;
        case LS2H_RTC_WRITE0_REG:
            s.rtc_count = (unsigned int) val;
            break;
        case LS2H_RTC_MATCH0_REG:
            s.rtc_match0 = (unsigned int) val;
            break;
        case LS2H_RTC_MATCH1_REG:
            s.rtc_match1 = (unsigned int) val;
            break;
        case LS2H_RTC_MATCH2_REG:
            s.rtc_match2 = (unsigned int) val;
            break;
        default:
            printf("invalid rtc address %lx\n", vaddr);
    }
}

static uint64_t rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t vaddr = addr + LS2H_RTC_REG_BASE;
    uint64_t val;
    struct tm tm;
    qemu_get_timedate(&tm, 0);
    switch (vaddr) {
        case LS2H_TOY_TRIM_REG:
            val = s.toy_trim; 
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
            val = s.toy_match0;
            break;
        case LS2H_TOY_MATCH1_REG:
            val = s.toy_match1;
            break;
        case LS2H_TOY_MATCH2_REG:
            val = s.toy_match2;
            break;
        case LS2H_RTC_TRIM_REG:
            val = s.rtc_trim;
            break;
        case LS2H_RTC_READ0_REG:
            val = s.rtc_count;
            break;
        case LS2H_RTC_MATCH0_REG:
            val = s.rtc_match0;
            break;
        case LS2H_RTC_MATCH1_REG:
            val = s.rtc_match1;
            break;
        case LS2H_RTC_MATCH2_REG:
            val = s.rtc_match2;
            break;
        default:
            printf("invalid rtc address %lx\n", vaddr);
    }

    DPRINTF("mc read addr %lx size %d,val=%lx,pc=%lx\n", addr, size, val,
            s.cpu->env.active_tc.PC);
    return val;
}

static const MemoryRegionOps rtc_io_ops = {
    .read = rtc_read,
    .write = rtc_write,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};


static const int sector_len = 64 * 1024;
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
    ISABus *isabus;
    DeviceState *dev;

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
        qemu_irq cs_line;

        dev = qdev_create(NULL, "ls2h-spi");
        qdev_init_nofail(dev);
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, LS2H_SPI_REG_BASE - KSEG0_BASE);
        sysbus_mmio_map(busdev, 1, 0xbfc00000 - KSEG0_BASE);
        //sysbus_connect_irq(busdev, 0, irq[SPI_IRQ]);

        spi = (SSIBus *)qdev_get_child_bus(dev, "spi");

        dev = ssi_create_slave(spi, "sst25vf080b");

        cs_line = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        sysbus_connect_irq(busdev, 1, cs_line);

        /* tell salve to use our rom memory as internal storage */
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

    /* init other devices */
    //DMA_init(0);

    /* enlarge the default system IO size to include 
      [0x1f000000,0x1fffffff] */
    memory_region_set_size(get_system_io(), 0x1000000);

    /* Register of SERIAL IO space at 0x1fe80000. */
    memory_region_init_alias(&s.uart0_mem, NULL, "uart0_mmio",
                             get_system_io(),
                             LS2H_UART0_REG_BASE - LS2H_IO_REG_BASE,
                             0x00001000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_UART0_REG_BASE - KSEG0_BASE, 
                                &s.uart0_mem);

    /* A single 16450 sits at offset 0x1fe8000. It is attached to
       MIPS CPU INT2, which is interrupt 4. */
    if (serial_hds[0]) {
        serial_init(LS2H_UART0_REG_BASE - LS2H_IO_REG_BASE, env->irq[4], 
                    115200, serial_hds[0], get_system_io());
    }

    /* Chip config IO */
    memory_region_init_alias(&s.creg_mem, NULL, "chip config io mem",
                             get_system_io(), 
                             LS2H_CHIP_CFG_REG_BASE - LS2H_IO_REG_BASE, 
                             0x00100000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_CHIP_CFG_REG_BASE - KSEG0_BASE, 
                                &s.creg_mem);
    memory_region_init_io(&s.creg_io, NULL, &creg_io_ops, (void*)env, 
                          "chip config io", 0x100000);
    memory_region_add_subregion(get_system_io(), 
				LS2H_CHIP_CFG_REG_BASE - LS2H_IO_REG_BASE,
				&s.creg_io);

    /* SATA IO */
    sysbus_create_simple("sysbus-ahci", LS2H_SATA_REG_BASE - KSEG0_BASE,
                         env->irq[3]);


    /* I2C0 IO */
    memory_region_init_alias(&s.i2c0_mem, NULL, "I2C0 I/O mem",
                             get_system_io(), 
                             LS2H_I2C0_REG_BASE - LS2H_IO_REG_BASE, 
                             0x1000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_I2C0_REG_BASE - KSEG0_BASE, 
                                &s.i2c0_mem);

    memory_region_init_io(&s.i2c0_io, NULL, &i2c0_io_ops, &s, 
                          "I2C I/O", 0x1000);
    memory_region_add_subregion(get_system_io(), 
				LS2H_I2C0_REG_BASE - LS2H_IO_REG_BASE,
				&s.i2c0_io);

    /* I2C1 IO */
    memory_region_init_alias(&s.i2c1_mem, NULL, "I2C1 I/O mem",
                             get_system_io(), 
                             LS2H_I2C1_REG_BASE - LS2H_IO_REG_BASE, 
                             0x1000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_I2C1_REG_BASE - KSEG0_BASE, 
                                &s.i2c1_mem);

    memory_region_init_io(&s.i2c1_io, NULL, &i2c1_io_ops, &s, 
                          "I2C1 I/O", 0x1000);
    memory_region_add_subregion(get_system_io(), 
				LS2H_I2C1_REG_BASE - LS2H_IO_REG_BASE,
				&s.i2c1_io);

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
    memory_region_init_alias(&s.rtc_mem, NULL, "RTC I/O mem",
                             get_system_io(), 
                             LS2H_RTC_REG_BASE - LS2H_IO_REG_BASE, 
                             0x8000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_RTC_REG_BASE - KSEG0_BASE, 
                                &s.rtc_mem);

    memory_region_init_io(&s.rtc_io, NULL, &rtc_io_ops, &s, 
                          "RTC I/O", 0x8000);
    memory_region_add_subregion(get_system_io(), 
				LS2H_RTC_REG_BASE - LS2H_IO_REG_BASE,
				&s.rtc_io);

#if 1
    /* ohci */
    dev = qdev_create(NULL, "sysbus-ohci");
    qdev_prop_set_uint32(dev, "num-ports", 2);
    qdev_prop_set_uint64(dev, "dma-offset", 0);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0,
            LS2H_OHCI_REG_BASE - KSEG0_BASE);
    //sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
#endif

    /* gmac */
    sysbus_create_simple("xgmac", LS2H_GMAC0_REG_BASE - KSEG0_BASE, NULL);

    sysbus_create_simple("xgmac", LS2H_GMAC1_REG_BASE - KSEG0_BASE, NULL);

    /* display controller */
    sysbus_create_simple("ls2h-dc", LS2H_DC_REG_BASE - KSEG0_BASE, NULL);

    /* keyboard/mouse */
    memory_region_init_alias(&s.lpc_mem, NULL, "LPC I/O mem",
                             get_system_io(), 
                             0, /* note: offset 0 for isa */
                             0x1000);
    memory_region_add_subregion(get_system_memory(), 
                                LS2H_LPC_IO_BASE - KSEG0_BASE, 
                                &s.lpc_mem);

    isabus = isa_bus_new(NULL, &s.lpc_mem, get_system_io());
    isa_bus_irqs(isabus, (qemu_irq *)&s.isa_irqs);
    isa_create_simple(isabus, "i8042");

}

static void mips_ls2h_machine_init(MachineClass *mc)
{
    mc->desc = "Loongson 2H SoC reference board";
    mc->init = mips_ls2h_init;
}

DEFINE_MACHINE("ls2h", mips_ls2h_machine_init)
