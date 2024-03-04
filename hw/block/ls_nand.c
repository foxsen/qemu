#include "qemu/osdep.h"
#include "hw/hw.h"
#include "sysemu/block-backend.h"
#include "hw/qdev.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "sysemu/blockdev.h"
#include "exec/memory.h"
#include "sysemu/dma.h"
#include "sysemu/sysemu.h"
#include "hw/loader.h"
#define DPRINTF(a...)           /*printf(a)*/
#define HW_FLASH_H
#include "ls_nand.h"
#include "nand.c"

struct dma_desc {
        uint32_t orderad;
        uint32_t saddr;
        uint32_t daddr;
        uint32_t length;
        uint32_t step_length;
        uint32_t step_times;
        uint32_t cmd;
        uint32_t dummy;
        uint32_t order_addr_hi;
        uint32_t saddr_hi;
        /*used by logic only */
        uint32_t left;
        uint32_t active;
        uint64_t nextaddr;
};

enum {
        DMA_INT_MASK = 1,
        DMA_INT = 2,
        DMA_SINGLE_TRANS_OVER = 4,
        DMA_TRANS_OVER = 8,
        DMA_ORDER_EN = 1,
        DMA_READ_DDR = 0x1000,
};

struct ls_nand_regs {
        uint32_t cmd;
        uint32_t addr_l;
        uint32_t addr_h;
        uint32_t nand_timing;
        uint32_t id_l;
        uint32_t status_id_h;
        uint32_t paramter;
        uint32_t op_num;
        uint32_t cs_rdy_map;    /*0x20 */
        uint32_t dma_address;   /*0x40 */
};

enum {
        NAND_CMD = 0,
        NAND_ADDR_L = 4,
        NAND_ADDR_H = 8,
        NAND_NAND_TIMING = 0xc,
        NAND_ID_L = 0x10,
        NAND_STATUS_ID_H = 0x14,
        NAND_NAND_PARAMETER = 0x18,
        NAND_NAND_OP_NUM = 0x1c,
        NAND_CS_RDY_MAP = 0x20,
        PLL_8030 = 0x30,        /* not nand reg */
        PLL_8034 = 0x34,        /* not nand reg */
        NAND_DMA_ADDRESS = 0x40,
};

/*irq is placed on each module now */
typedef struct NandState {
        MemoryRegion iomem;
        MemoryRegion iomem1;
        struct ls_nand_regs regs;
        NANDFlashState *chip;
        struct dma_desc dma_desc;
        int dma_int_status;
        qemu_irq irq;
        union {
                AddressSpace *as;
                void *as_ptr;
        };
        uint8_t manf_id, chip_id;
        uint8_t cs;
} NandState;

typedef struct nand_sysbus_state {
        SysBusDevice busdev;
        NandState nand;
        /*bellow for nand boot */
        uint64_t addr;
        uint32_t size;
        MemoryRegion mem;
        uint8_t *buf;
} nand_sysbus_state;

enum {
        CMD_STOP = 0,
        CMD_VALID = 1,
        CMD_READ = 2,
        CMD_WRITE = 4,
        CMD_ERASE = 8,
        CMD_BLKERASE = 16,
        CMD_READID = 32,
        CMD_RESET = 64,
        CMD_READSTATUS = 128,
        CMD_MAIN = 256,
        CMD_SPARE = 512,
        CMD_DONE = 1024,
};

static void nand_check_irq(NandState *s)
{
        /* edge interrupt */
        if (s->dma_int_status & 1) {
                qemu_irq_raise(s->irq);
                qemu_irq_lower(s->irq);
        }

        s->dma_int_status = 0;
}

/*
  dma transfer size should not address len,but can less len.
*/
#define min(x, y) (((x) < (y)) ? (x) : (y))

static int dma_next(NandState *s)
{

        if (!s->dma_desc.step_times) {
                if (s->dma_desc.cmd & DMA_INT_MASK) {
                        s->dma_desc.cmd |= DMA_INT;
                        s->dma_int_status = 1;
                        nand_check_irq(s);
                }

                if (s->dma_desc.orderad & DMA_ORDER_EN) {
                        dma_memory_read(s->as,
                                        (s->dma_desc.
                                         orderad & ~DMA_ORDER_EN) | ((uint64_t)
                                                         s->
                                                         dma_desc.
                                                         order_addr_hi
                                                         << 32),
                                        (uint8_t *) &s->dma_desc, 4 * 10);
                        s->dma_desc.left = s->dma_desc.length * 4;
                        s->dma_desc.step_times--;
                } else if (s->dma_desc.nextaddr) {
                        dma_memory_read(s->as, s->dma_desc.nextaddr,
                                        (uint8_t *) &s->dma_desc, 4 * 10);
                        s->dma_desc.nextaddr = 0;
                        s->dma_desc.left = s->dma_desc.length * 4;
                } else {
                        s->dma_desc.cmd |= DMA_TRANS_OVER;
                        s->dma_desc.active = 0;
                }
        } else {
                s->dma_desc.step_times--;
                s->dma_desc.saddr += s->dma_desc.step_length * 4;
                s->dma_desc.left = s->dma_desc.length * 4;
        }
        return s->dma_desc.active;
}

#define  CHIP_CAP_SHIFT 8
static const char cap2cs[16] = {
 [0] = 16, [1] = 17, [2] = 18, [3] = 19, [4] = 19,
 [5] = 19, [6] = 20, [7] = 21, [9] = 14, [10] = 15, [11] = 16,
 [12] = 17, [13] = 18
};
#define addr_cs_h(cs) ((cs) * (1ULL << cap2cs[(s->regs.paramter >> \
 CHIP_CAP_SHIFT) & 0xf]))

#define PAGE_ADDR(s) ((uint64_t)(s->regs.addr_h - addr_cs_h(s->cs)))
#define COLUM_ADDR(s) (s->regs.addr_l)
#define ACCESS_ME(s) (s->regs.addr_h >= addr_cs_h(s->cs) && s->regs.addr_h < \
 addr_cs_h(s->cs + 1))

static int nand_load_next(NandState *s)
{
        uint64_t offset, addr, iolen, oplen;
        int cmd;
        cmd = s->regs.cmd;

        /*nand addr maybe not page aligned */
        if ((cmd & (CMD_SPARE | CMD_MAIN)) == CMD_SPARE) {
                addr = PAGE_ADDR(s) << s->chip->addr_shift;
                offset = COLUM_ADDR(s);
                iolen = (1 << s->chip->page_shift) + s->chip->oob_size - offset;
        } else if ((cmd & (CMD_SPARE | CMD_MAIN)) == (CMD_SPARE | CMD_MAIN)) {
                addr = PAGE_ADDR(s) << s->chip->addr_shift;
                offset = COLUM_ADDR(s);
                iolen = (1 << s->chip->page_shift) + s->chip->oob_size - offset;
        } else {
                addr = PAGE_ADDR(s) << s->chip->addr_shift;
                offset = COLUM_ADDR(s);
                iolen = (1 << s->chip->page_shift) - offset;
        }

        oplen = min(s->regs.op_num, iolen);

        s->chip->addr = addr;
        if (ACCESS_ME(s)) {
                s->chip->blk_load(s->chip, addr, offset);
        }
        s->chip->iolen = oplen;
        if (s->chip->iolen >= iolen) {
                s->regs.addr_h += 0x1;
        }
        return oplen;
}

static int nand_write_next(NandState *s)
{
        uint64_t offset, addr, iolen, oplen;
        int cmd;
        cmd = s->regs.cmd;

again:

        /*nand addr maybe not page aligned */
        if ((cmd & (CMD_SPARE | CMD_MAIN)) == CMD_SPARE) {
                /*addr include 3 row addr and 2 col addr
                 * one sectior size == pagesize + oobsize,
                 */
                addr = PAGE_ADDR(s) << s->chip->addr_shift;
                offset = COLUM_ADDR(s);
                iolen = s->chip->oob_size - offset;
        } else if ((cmd & (CMD_SPARE | CMD_MAIN)) == (CMD_SPARE | CMD_MAIN)) {
                addr = PAGE_ADDR(s) << s->chip->addr_shift;
                offset = COLUM_ADDR(s);
                iolen = (1 << s->chip->page_shift) + s->chip->oob_size - offset;
        } else {
                addr = PAGE_ADDR(s) << s->chip->addr_shift;
                offset = COLUM_ADDR(s);
                iolen = (1 << s->chip->page_shift) - offset;
        }

        oplen = min(s->regs.op_num, iolen);

        if (oplen && s->chip->iolen >= oplen) {
                s->chip->addr = addr;
                s->chip->offset = offset;
                s->chip->iolen = oplen;

                if (ACCESS_ME(s)) {
                        s->chip->blk_write(s->chip);
                }

                s->chip->ioaddr = s->chip->io;
                s->chip->iolen = 0;
                if (s->chip->iolen >= iolen) {
                        s->regs.addr_h += 0x1;
                }
                s->regs.op_num -= oplen;
                goto again;
        }

        return oplen - s->chip->iolen;
}

static int dma_readnand(NandState *s)
{
        uint32_t copied;

        while (s->dma_desc.active && s->regs.op_num && !(s->dma_desc.cmd &
        DMA_READ_DDR) && (s->regs.cmd & CMD_VALID) && (s->regs.cmd & CMD_READ))
        {
                if (!s->chip->iolen) {
                        nand_load_next(s);
                }

                if (!s->dma_desc.left && !dma_next(s)) {
                        break;
                }

                copied = min(s->dma_desc.left, s->chip->iolen);
                copied = min(copied, s->regs.op_num);

                if (!copied) {
                        break;
                }

                dma_memory_write(s->as,
                                 s->dma_desc.saddr | ((uint64_t) s->dma_desc.
                                                      saddr_hi << 32),
                                 s->chip->ioaddr, copied);

                s->chip->ioaddr += copied;
                s->chip->iolen -= copied;
                s->dma_desc.saddr += copied;
                s->dma_desc.left -= copied;
                s->regs.op_num -= copied;

                if (!s->dma_desc.left) {
                        dma_next(s);
                }

                if (!s->regs.op_num) {
                        s->regs.cmd |= CMD_DONE;
                        s->regs.cmd &= ~CMD_VALID;
                        break;
                }

        }

        return s->dma_desc.active;
}

static int dma_writenand(NandState *s)
{
        uint32_t wantsize, copied;
        wantsize = 0;

        while (s->dma_desc.active && s->regs.op_num && (s->dma_desc.cmd &
        DMA_READ_DDR) && (s->regs.cmd & CMD_VALID) && (s->regs.cmd & (CMD_READ |
        CMD_WRITE))) {
                if (!s->dma_desc.left && !dma_next(s)) {
                        break;
                }

                if (!wantsize) {
                        wantsize = nand_write_next(s);
                }

                copied = min(s->dma_desc.left, wantsize);
                copied = min(copied, s->regs.op_num);

                if (!copied) {
                        break;
                }

                dma_memory_read(s->as,
                                s->dma_desc.saddr | ((uint64_t) s->dma_desc.
                                                     saddr_hi << 32),
                                s->chip->ioaddr, copied);

                s->chip->ioaddr += copied;
                s->chip->iolen += copied;
                s->dma_desc.saddr += copied;
                s->dma_desc.left -= copied;

                if (!s->dma_desc.left) {
                        dma_next(s);
                }

                wantsize = nand_write_next(s);

                if (!s->regs.op_num) {
                        s->regs.cmd |= CMD_DONE;
                        s->regs.cmd &= ~CMD_VALID;
                        break;
                }
        }

        return s->dma_desc.active;
}

static NandState *ls_nand;

void ls_nand_set_dmaaddr(uint64_t val)
{
        NandState *s = ls_nand;
        uint64_t dmaaddr;
        dmaaddr = val & ~0xf;

        if (val & 0x8) {
                if (s->dma_desc.active) {
                        s->dma_desc.nextaddr = dmaaddr;
                } else {
                        dma_memory_read(s->as, dmaaddr,
                                        (uint8_t *)&s->dma_desc, 4 * 10);
                        s->dma_desc.left = s->dma_desc.length * 4;
                        s->dma_desc.step_times--;
                        s->dma_desc.active = 1;
                        s->dma_desc.nextaddr = 0;
                        if (s->dma_desc.cmd & DMA_READ_DDR) {
                                dma_writenand(s);
                        } else {
                                dma_readnand(s);
                        }
                }
        }

        if (val & 0x4) {
                dma_memory_write(s->as, dmaaddr, (uint8_t *)&s->dma_desc,
                                 4 * 10);
        }

}

static void ls_nand_do_cmd(NandState *s, uint32_t cmd)
{
        unsigned int i;
        uint64_t addr;
        s->regs.cmd = cmd & 0x3ff;

        if ((cmd & CMD_VALID) == 0) {
                s->regs.cmd |= CMD_DONE | (0xf << 16);
                return;
        }

        if (cmd & CMD_ERASE) {
                if (cmd & CMD_BLKERASE) {
                        /*fix 1 now,should be s->chip->erase_shift
                         *
                         * erase_shfit + program_shift => linuxkernel erase_shit
                         */

                        for (i = 0, addr = PAGE_ADDR(s) << s->chip->addr_shift;
                                        i < s->regs.op_num;
                                        i++, addr +=
                                                1 << (s->chip->addr_shift +
                                                      s->chip->erase_shift)) {
                                s->chip->addr = addr;
                                if (ACCESS_ME(s)) {
                                        s->chip->blk_erase(s->chip);
                                }
                        }

                } else {
                        s->chip->addr = PAGE_ADDR(s) << s->chip->addr_shift;
                        if (ACCESS_ME(s)) {
                                s->chip->blk_erase(s->chip);
                        }
                }

                s->regs.cmd |= CMD_DONE;
                s->regs.cmd &= ~CMD_VALID;
                s->regs.status_id_h =
                        (s->regs.status_id_h & ~0xff0000) | 0xe00000;
        } else if (cmd & CMD_READID) {
                int i, j;
                unsigned char id;
                s->chip->cmd = NAND_CMD_READID;
                if (ACCESS_ME(s)) {
                        nand_command(s->chip);
                        /*pmon check io[4] not 0 */
                        if (!s->chip->io[4]) {
                                s->chip->io[4] = 0x54;
                        }
                } else {
                        memset(s->chip->io, 0xff, 6);
                }
                s->regs.id_l = 0;
                s->regs.status_id_h = 0;
                for (i = (((s->regs.paramter >> 12) & 0xf) ? : 5), j = 0; i;
                                i--, j++) {
                        id = s->chip->io[j];
                        switch (i) {
                        case 6:
                                s->regs.status_id_h |= id << 8;
                                break;
                        case 5:
                                s->regs.status_id_h |= id;
                                break;
                        case 4:
                                s->regs.id_l |= id << 24;
                                break;
                        case 3:
                                s->regs.id_l |= id << 16;
                                break;
                        case 2:
                                s->regs.id_l |= id << 8;
                                break;
                        case 1:
                                s->regs.id_l |= id;
                                break;
                        }

                }
                s->regs.status_id_h =
                        (s->regs.status_id_h & ~0xff0000) | 0xe00000;
                s->regs.cmd |= CMD_DONE;
                s->regs.cmd &= ~CMD_VALID;
        } else if (cmd & CMD_READSTATUS) {
                s->chip->cmd = NAND_CMD_READSTATUS;
                nand_command(s->chip);
                s->regs.status_id_h =
                        (s->regs.status_id_h & ~0xff0000) | 0xe00000;
                memcpy(s->chip->io, (char *)(&s->regs.status_id_h) + 2, 1);
                s->regs.cmd |= CMD_DONE;
                s->regs.cmd &= ~CMD_VALID;
        } else if (cmd & CMD_READ) {
                s->chip->ioaddr = s->chip->io;
                s->chip->iolen = 0;
                dma_readnand(s);
        } else if (cmd & CMD_WRITE) {
                s->chip->ioaddr = s->chip->io;
                s->chip->iolen = 0;
                s->regs.status_id_h =
                        (s->regs.status_id_h & ~0xff0000) | 0xe00000;
                dma_writenand(s);
        } else
                s->regs.cmd |= CMD_DONE;
        s->regs.cmd |= 0xf << 16;

}

static void nand_nand_writel(void *ptr, hwaddr addr, uint64_t val,
                             unsigned size)
{
        NandState *s = ptr;
        addr &= 0xff;
        switch (addr) {
        case NAND_CMD:
                ls_nand_do_cmd(s, val);
                break;
        case NAND_ID_L:
                break;
        case NAND_STATUS_ID_H:
                break;
        case NAND_DMA_ADDRESS:
                break;
        default:
                *(int *)((void *)&s->regs + addr) = val;
                break;
        }

        DPRINTF("nand_nand_writel:  (addr 0x%08X), val 0x%08lX\n",
                (unsigned)addr, val);
}

static uint64_t nand_nand_readl(void *ptr, hwaddr addr, unsigned size)
{
        NandState *s = ptr;
        uint32_t val = 0;
        addr &= 0xff;
        switch (addr) {
        default:
                val = *(int *)((void *)&s->regs + addr);
                break;
        }

        DPRINTF("nand_nand_readl:  (addr 0x%08X), val 0x%08X\n", (unsigned)addr,
                val);
        return val;
}

static const MemoryRegionOps ls_nand_ops = {
        .read = nand_nand_readl,
        .write = nand_nand_writel,
        .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LSNAND "ls_nand"

#define SYS_BUS_LSNAND(obj)                                           \
        OBJECT_CHECK(nand_sysbus_state, (obj), TYPE_SYS_BUS_LSNAND)

static uint64_t spi_rom_read(void *opaque, hwaddr addr, unsigned size)
{
        nand_sysbus_state *s = opaque;

        switch (size) {
        case 4:
                return *(uint32_t *)(s->buf + addr);
        case 2:
                return *(uint16_t *)(s->buf + addr);
        case 1:
                return *(uint8_t *)(s->buf + addr);
        default:
                return *(uint64_t *)(s->buf + addr);
        }

}

static void spi_rom_write(void *opaque, hwaddr addr, uint64_t data,
                          unsigned size)
{
}

static const MemoryRegionOps spi_rom_ops = {
        .read = spi_rom_read,
        .write = spi_rom_write,
        .endianness = DEVICE_NATIVE_ENDIAN,
};

static int load_bios(uint8_t **buf)
{
        char *filename;
        int bios_size;
        if (bios_name == NULL) {
                return 0;
        }

        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
        if (filename) {
                bios_size = get_image_size(filename);
                *buf = g_malloc(bios_size);
                load_image(filename, *buf);
        } else {
                bios_size = 0;
                *buf = NULL;
        }
        return bios_size;
}

static int ls_nand_init(SysBusDevice *dev)
{
        DriveInfo *nand;
        nand_sysbus_state *d = SYS_BUS_LSNAND(dev);

        nand = drive_get(IF_MTD, 0, 0);

        d->nand.chip =
                (NANDFlashState *) nand_init(nand ? blk_by_legacy_dinfo(nand) :
                                             NULL, d->nand.manf_id ? : 0xec,
                                             d->nand.chip_id ? : 0xa1);
        nand_setpins(DEVICE(d->nand.chip), 0, 0, 0, 0, 0);

        if (!d->nand.as) {
                d->nand.as = &address_space_memory;
        }
        memory_region_init_io(&d->nand.iomem, NULL, &ls_nand_ops,
                              (void *)&d->nand, "ls nand", 0x24);
        memory_region_init_io(&d->nand.iomem1, NULL, &ls_nand_ops,
                              (void *)&d->nand, "ls nand", 0x4);
        if (d->size) {
                NANDFlashState *s = d->nand.chip;
                uint8_t *buf;
                int i, once;
                int pagesize = 1 << s->page_shift;
                int erasesize = 1 << (s->erase_shift + s->page_shift);
                int size = load_bios(&buf);
                Error *err;
                if (size) {
                        for (i = 0; i < size; i += pagesize) {
                                once = min(size - i, pagesize);
                                s->addr = i;
                                s->offset = 0;
                                s->iolen = once;
                                memcpy(s->io, buf + i, once);
                                if ((i & (erasesize - 1)) == 0) {
                                        s->blk_erase(s);
                                }
                                s->blk_write(s);
                        }
                }

                memory_region_init_rom_device(&d->mem, NULL, &spi_rom_ops, d,
                                              "spi flash rom", d->size, &err);
                d->buf = memory_region_get_ram_ptr(&d->mem);

                nand_reset((DeviceState *) s);

                for (i = 0; i < 1024; i = i + pagesize) {
                        s->addr = i;
                        s->blk_load(s, i, 0);
                        memcpy(d->buf + i, s->io, min(1024 - i, pagesize));
                }

                memory_region_add_subregion(get_system_memory(), d->addr,
                                            &d->mem);
                nand_reset((DeviceState *) s);
        }
        sysbus_init_mmio(dev, &d->nand.iomem);
        sysbus_init_mmio(dev, &d->nand.iomem1);
        sysbus_init_irq(dev, &d->nand.irq);
        ls_nand = &d->nand;

        return 0;
}

static Property ls_nand_properties[] = {
        DEFINE_PROP_PTR("as", nand_sysbus_state, nand.as_ptr),
        DEFINE_PROP_UINT8("manufacturer_id", nand_sysbus_state, nand.manf_id,
                          0),
        DEFINE_PROP_UINT8("chip_id", nand_sysbus_state, nand.chip_id, 0),
        DEFINE_PROP_UINT8("cs", nand_sysbus_state, nand.cs, 0),
        DEFINE_PROP_UINT32("size", nand_sysbus_state, size, 0),
        DEFINE_PROP_UINT64("addr", nand_sysbus_state, addr, 0),
        DEFINE_PROP_END_OF_LIST(),
};

static void ls_nand_class_init(ObjectClass *klass, void *data)
{
        DeviceClass *dc = DEVICE_CLASS(klass);
        SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

        k->init = ls_nand_init;
        dc->desc = "ls nand";
        dc->props = ls_nand_properties;
}

static const TypeInfo ls_nand_info = {
        .name = "ls_nand",
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(nand_sysbus_state),
        .class_init = ls_nand_class_init,
};

static void ls_nand_register_types(void)
{
        /* silent compiler */
        (void)nand_register_types;
        type_register_static(&ls_nand_info);
}

type_init(ls_nand_register_types)
