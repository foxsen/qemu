/*
 * Loongson LS MultiMediaCard/SD/SDIO Controller emulation.
 *
 * Copyright (c) 2006 Openedhand Ltd.
 * Written by Andrzej Zaborowski <balrog@zabor.org>
 *
 * This code is licensed under the GPLv2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "hw/sd/sd.h"
#include "hw/qdev.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "sysemu/dma.h"
#include "cpu.h"
#include "qemu/log.h"
#include "ls_mmci.h"

#define REG_FMT                "0x" TARGET_FMT_plx
#define TYPE_LS_MMCI "ls-mmci"
#define LS_MMCI(obj) OBJECT_CHECK(LS_MMCIState, (obj), TYPE_LS_MMCI)

#define TYPE_LS_MMCI_BUS "ls-mmci-bus"
#define LS_MMCI_BUS(obj) OBJECT_CHECK(SDBus, (obj), TYPE_LS_MMCI_BUS)

#define SDICON     0x00
#define SDIPRE     0x04
#define SDICMDARG  0x08
#define SDICMDCON  0x0c
#define SDICMDSTA  0x10
#define SDIRSP0    0x14
#define SDIRSP1    0x18
#define SDIRSP2    0x1C
#define SDIRSP3    0x20
#define SDIDTIMER  0x24
#define SDIBSIZE   0x28
#define SDIDATCON  0x2C
/* #define SDIDCON               (0x2C) */
#define SDIDATCNT  0x30
#define SDIDSTA               (0x34)
#define SDIFSTA    0x38
#define SDIINTMSK  0x3C
#define SDIWRDAT   0x40
#define SDISTAADD0 0x44
#define SDISTAADD1 0x48
#define SDISTAADD2 0x4c
#define SDISTAADD3 0x50
#define SDISTAADD4 0x54
#define SDISTAADD5 0x58
#define SDISTAADD6 0x5c
#define SDISTAADD7 0x60
#define SDIINTEN   0x64

#define SDICON_SDRESET        (1 << 8)
#define SDICON_MMCCLOCK       (1 << 5)
#define SDICON_BYTEORDER      (1 << 4)
#define SDICON_SDIOIRQ        (1 << 3)
#define SDICON_RWAITEN        (1 << 2)
#define SDICON_FIFORESET      (1 << 1)
#define SDICON_CLOCKTYPE      (1 << 0)

#define SDICMDCON_ABORT       (1 << 12)
#define SDICMDCON_WITHDATA    (1 << 11)
#define SDICMDCON_LONGRSP     (1 << 10)
#define SDICMDCON_WAITRSP     (1 << 9)
#define SDICMDCON_CMDSTART    (1 << 8)
#define SDICMDCON_SENDERHOST  (1 << 6)
#define SDICMDCON_INDEX       (0x3f)

#define SDICMDSTAT_CRCFAIL    (1 << 12)
#define SDICMDSTAT_CMDSENT    (1 << 11)
#define SDICMDSTAT_CMDTIMEOUT (1 << 10)
#define SDICMDSTAT_RSPFIN     (1 << 9)
#define SDICMDSTAT_XFERING    (1 << 8)
#define SDICMDSTAT_INDEX      (0xff)

#define S3C2440_SDIDCON_DS_BYTE       (0 << 22)
#define S3C2440_SDIDCON_DS_HALFWORD   (1 << 22)
#define S3C2440_SDIDCON_DS_WORD       (2 << 22)
#define SDIDCON_IRQPERIOD     (1 << 21)
#define SDIDCON_TXAFTERRESP   (1 << 20)
#define SDIDCON_RXAFTERCMD    (1 << 19)
#define SDIDCON_BUSYAFTERCMD  (1 << 18)
#define SDIDCON_BLOCKMODE     (1 << 17)
#define SDIDCON_WIDEBUS       (1 << 16)
#define MISC_CTRL_SDIO_DMAEN         (1 << 15)
#define SDIDCON_STOP          (1 << 14)
#define S3C2440_SDIDCON_DATSTART      (1 << 14)
#define SDIDCON_DATMODE       (3 << 12)
#define SDIDCON_BLKNUM        (0x7ff)

/* constants for SDIDCON_DATMODE */
#define SDIDCON_XFER_READY    (0 << 12)
#define SDIDCON_XFER_CHKSTART (1 << 12)
#define SDIDCON_XFER_RXSTART  (2 << 12)
#define SDIDCON_XFER_TXSTART  (3 << 12)

#define SDIDCON_BLKNUM_MASK   (0xFFF)
#define SDIDCNT_BLKNUM_SHIFT  (12)

#define SDIDSTA_RDYWAITREQ    (1 << 10)
#define SDIDSTA_SDIOIRQDETECT (1 << 9)
#define SDIDSTA_FIFOFAIL      (1 << 8)    /* reserved on 2440 */
#define SDIDSTA_CRCFAIL       (1 << 7)
#define SDIDSTA_RXCRCFAIL     (1 << 6)
#define SDIDSTA_DATATIMEOUT   (1 << 5)
#define SDIDSTA_XFERFINISH    (1 << 4)
#define SDIDSTA_BUSYFINISH    (1 << 3)
#define SDIDSTA_SBITERR       (1 << 2)    /* reserved on 2410a/2440 */
#define SDIDSTA_TXDATAON      (1 << 1)
#define SDIDSTA_RXDATAON      (1 << 0)

#define S3C2440_SDIFSTA_FIFORESET      (1 << 16)
#define S3C2440_SDIFSTA_FIFOFAIL       (3 << 14)  /* 3 is correct (2 bits) */
#define SDIFSTA_TFDET          (1 << 13)
#define SDIFSTA_RFDET          (1 << 12)
#define SDIFSTA_TFHALF         (1 << 11)
#define SDIFSTA_TFEMPTY        (1 << 10)
#define SDIFSTA_RFLAST         (1 << 9)
#define SDIFSTA_RFFULL         (1 << 8)
#define SDIFSTA_RFHALF         (1 << 7)
#define SDIFSTA_COUNTMASK      (0x7f)

/* #define SDIIMSK_RESPONSECRC    (1<<17) */
#define SDIIMSK_RESPONSEND     (1 << 14)
#define SDIIMSK_READWAIT       (1 << 13)
#define SDIIMSK_SDIOIRQ        (1 << 12)
#define SDIIMSK_FIFOFAIL       (1 << 11)


#define SDIIMSK_R1BFIN         (1 << 9)
#define SDIIMSK_RESPONSECRC    (1 << 8)
#define SDIIMSK_CMDTIMEOUT     (1 << 7)
#define SDIIMSK_CMDSENT        (1 << 6)
#define SDIIMSK_PROGERR        (1 << 4)
#define SDIIMSK_TXCRCFAIL      (1 << 3)
#define SDIIMSK_RXCRCFAIL      (1 << 2)
#define SDIIMSK_DATATIMEOUT    (1 << 1)
#define SDIIMSK_DATAFINISH     (1 << 0)

#define SDIIMSK_BUSYFINISH     (1 << 6)
#define SDIIMSK_SBITERR        (1 << 5)   /* reserved 2440/2410a */
#define SDIIMSK_TXFIFOHALF     (1 << 4)
#define SDIIMSK_TXFIFOEMPTY    (1 << 3)
#define SDIIMSK_RXFIFOLAST     (1 << 2)
#define SDIIMSK_RXFIFOFULL     (1 << 1)
#define SDIIMSK_RXFIFOHALF     (1 << 0)
typedef struct ls_sdio_regs {
        int sdicon;
        int sdipre;
        int sdicmdarg;
        int sdicmdcon;
        int sdicmdsta;
        int sdirsp0;
        int sdirsp1;
        int sdirsp2;
        int sdirsp3;
        int sdidtimer;
        int sdibsize;
        int sdidatcon;
        int sdidatcnt;
        int sdidsta;
        int sdifsta;
        int sdiintmsk;
        int sdiwrdat;
        int sdistaadd0;
        int sdistaadd1;
        int sdistaadd2;
        int sdistaadd3;
        int sdistaadd4;
        int sdistaadd5;
        int sdistaadd6;
        int sdistaadd7;
        int sdiinten;
} ls_sdio_regs_t;

struct dma_desc {
        uint32_t ordered;
        uint32_t saddr;
        uint32_t daddr;
        uint32_t length;
        uint32_t step_length;
        uint32_t step_times;
        uint32_t cmd;
        /*used by logic only*/
        uint32_t left;
        uint32_t active;
        uint32_t nextaddr;
};

enum {
        DMA_INT_MASK =  1,
        DMA_INT = 2,
        DMA_SINGLE_TRANS_OVER = 4,
        DMA_TRANS_OVER = 8,
        DMA_ORDER_EN = 1,
        DMA_READ_DDR = 0x1 << 12,
};

struct LS_MMCIState {
        SysBusDevice parent_obj;

        MemoryRegion iomem;
        qemu_irq irq;
        qemu_irq inserted;
        qemu_irq readonly;

        union {
                BlockBackend *blk;
                void *blk_ptr;
        };
        SDBus sdbus;

        int32_t blksize;
        int32_t blknum;
        uint32_t intmask;
        uint32_t intreq;
        int dma_int_status;
        int iolen;
        int32_t cmd;
        uint32_t arg;
        uint8_t *ioaddr;

        struct dma_desc dma_desc;
        uint8_t fifo_buffer[0x1000];

        int32_t cmdreq;

        union {
                AddressSpace *as;
                void *as_ptr;
        };

        union {
                ls_sdio_regs_t sdioregs;
                uint32_t regdata[0x60 / 4];
        };
};


static const VMStateDescription vmstate_ls_mmci = {
        .name = "ls-mmci",
        .version_id = 2,
        .minimum_version_id = 2,
        .fields = (VMStateField[])
        {
                VMSTATE_END_OF_LIST()
        }
};

LS_MMCIState *ls1gp_sdio[2];
static int dma_readsdio(LS_MMCIState *s);
static int dma_writesdio(LS_MMCIState *s);
static void ls_mmci_end_transfer(LS_MMCIState *s);

static void ls_mmci_data_transfer(LS_MMCIState *s);

#define DPRINT_L1 qemu_log
#define ERRPRINT printf
#define min(x, y) ((x) < (y) ? (x) : (y))
static void ls_mmci_update_irq(LS_MMCIState *s)
{
        /* edge interrupt */
        if (s->sdioregs.sdiintmsk) {
                qemu_irq_raise(s->irq);
        } else {
                qemu_irq_lower(s->irq);
        }

}

static void ls_mmci_send_command(LS_MMCIState *s)
{
        SDRequest request;
        uint8_t response[16];
        int rlen;

        s->sdioregs.sdicmdsta = 0;
        s->sdioregs.sdiintmsk = 0;
        if (s->iolen) {
                ERRPRINT("bug iolen is %d\n", s->iolen);
        }
        s->iolen = 0;
        s->ioaddr = s->fifo_buffer;
        request.cmd = s->sdioregs.sdicmdcon & SDICMDCON_INDEX;
        request.arg = s->sdioregs.sdicmdarg;
        request.crc = 0;        /* FIXME */
        DPRINT_L1("sending CMD%u ARG[0x%08x] pc=0x%lx\n", request.cmd,
        request.arg, (long)mypc);
        rlen = sdbus_do_command(&s->sdbus, &request, response);
        DPRINT_L1("sdicmdcon 0x%x len 0x%x\n", s->sdioregs.sdicmdcon, rlen);

        if (s->sdioregs.sdicmdcon  & SDICMDCON_WAITRSP) {
                if (rlen == 4) {
                        s->sdioregs.sdirsp0 = (response[0] << 24) | (response[1]
                        << 16) | (response[2] << 8) | response[3];
                        s->sdioregs.sdirsp1 = s->sdioregs.sdirsp2 =
                        s->sdioregs.sdirsp3 = 0;
                        DPRINT_L1("Response: RSPREG[31..0]=0x%08x\n",
                        s->sdioregs.sdirsp0);
                        s->sdioregs.sdicmdsta |= SDICMDSTAT_RSPFIN;
                        s->sdioregs.sdiintmsk |= SDIIMSK_CMDSENT | SDIIMSK_R1BFIN;
                } else if (rlen == 16) {
                        s->sdioregs.sdirsp3 = (response[11] << 24) |
                        (response[12] << 16) | (response[13] << 8) |
                        response[14];
                        s->sdioregs.sdirsp2 = (response[7] << 24) | (response[8]
                        << 16) | (response[9] << 8)  |  response[10];
                        s->sdioregs.sdirsp1 = (response[3] << 24) | (response[4]
                        << 16) | (response[5] << 8) | response[6];
                        s->sdioregs.sdirsp0 = (response[0] << 16) | (response[1]
                        << 8) |
                                              response[2];
                        DPRINT_L1("Response received:\n RSPREG[127..96]=0x%08x,"
                                  "RSPREG[95..64]=0x%08x,\n RSPREG[63..32]="
                                  "0x%08x, RSPREG[31..0]=0x%08x\n",
                                  s->sdioregs.sdirsp0, s->sdioregs.sdirsp1,
                                  s->sdioregs.sdirsp2, s->sdioregs.sdirsp3);
                        s->sdioregs.sdicmdsta |= SDICMDSTAT_RSPFIN;
                        s->sdioregs.sdiintmsk |= SDIIMSK_CMDSENT;
                } else {
                        ERRPRINT("Timeout waiting for command response\n");
                        s->sdioregs.sdicmdsta |= SDICMDSTAT_CMDTIMEOUT;
                        s->sdioregs.sdiintmsk |= SDIIMSK_CMDTIMEOUT;
                }

        } else {
                s->sdioregs.sdiintmsk |= SDIIMSK_CMDSENT;
        }

        if (s->sdioregs.sdibsize && (s->sdioregs.sdidatcon & SDIDCON_BLKNUM)) {
                s->blknum = s->sdioregs.sdidatcon & SDIDCON_BLKNUM;
                s->blksize = s->sdioregs.sdibsize;
                ls_mmci_data_transfer(s);
        }

        s->sdioregs.sdicmdsta |= SDICMDSTAT_CMDSENT;
        s->sdioregs.sdicmdcon &= ~SDICMDCON_CMDSTART;
        ls_mmci_update_irq(s);
}

static void _ls_sdio_set_dmaaddr(LS_MMCIState *s, uint32_t val)
{
        uint32_t dmaaddr;
        dmaaddr = val & ~0x1f;

        if (val & 0x8) {
                if (s->dma_desc.active) {
                        s->dma_desc.nextaddr = dmaaddr;
                } else {
                        dma_memory_read(s->as, dmaaddr, (uint8_t *)&s->dma_desc,
                        4 * 7);
                        s->dma_desc.left = s->dma_desc.length * 4;
                        s->dma_desc.step_times--;
                        s->dma_desc.active = 1;
                        s->dma_desc.nextaddr = 0;
                        DPRINT_L1("dma count 0x%x\n", s->dma_desc.left);
                        ls_mmci_data_transfer(s);
                }
        }

        if (val & 0x4) {
                dma_memory_write(s->as, dmaaddr, (uint8_t *)&s->dma_desc, 4 *
                7);
        }
}

void ls_sdio_set_dmaaddr(uint32_t val)
{
        _ls_sdio_set_dmaaddr(ls1gp_sdio[0], val);
}

void ls_sdio_set_dmaaddr(int i, uint64_t val)
{
        _ls_sdio_set_dmaaddr(ls1gp_sdio[i], val);
}

static int dma_next(LS_MMCIState *s)
{

        if (!s->dma_desc.step_times) {
                if (s->dma_desc.cmd & DMA_INT_MASK) {
                        s->dma_desc.cmd |= DMA_INT;
                        s->dma_int_status = 1;
                        ls_mmci_update_irq(s);
                }


                if (s->dma_desc.ordered & DMA_ORDER_EN) {
                        dma_memory_read(s->as, s->dma_desc.ordered &
                        ~DMA_ORDER_EN, (uint8_t *)&s->dma_desc, 4 * 7);
                        s->dma_desc.left = s->dma_desc.length * 4;
                        s->dma_desc.step_times--;
                } else if (s->dma_desc.nextaddr) {
                        dma_memory_read(s->as, s->dma_desc.nextaddr, (uint8_t
                        *)&s->dma_desc, 4 * 7);
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

static int dma_readsdio(LS_MMCIState *s)
{
        uint32_t copied;

        while (s->dma_desc.active && (s->blknum || s->iolen)
                        && !(s->dma_desc.cmd & DMA_READ_DDR)
                        && (s->sdioregs.sdicmdcon & SDICMDCON_CMDSTART)) {
                if (!s->iolen) {
                        int i;
                        for (i = 0; i < s->blksize; i++) {
                                s->fifo_buffer[i] = sdbus_read_data(&s->sdbus);
                        }
                        s->ioaddr = s->fifo_buffer;
                        s->blknum--;
                        s->iolen = s->blksize;
                }

                if (!s->dma_desc.left && !dma_next(s)) {
                        break;
                }

                copied = min(s->dma_desc.left, s->iolen);

                if (!copied) {
                        break;
                }

                dma_memory_write(s->as, s->dma_desc.saddr, s->ioaddr, copied);

                s->ioaddr += copied;
                s->iolen -= copied;
                s->dma_desc.saddr += copied;
                s->dma_desc.left -= copied;

                if (!s->dma_desc.left) {
                        dma_next(s);
                }

                if (!s->blknum && !s->iolen) {
                        ls_mmci_end_transfer(s);
                        break;
                }

        }

        return s->dma_desc.active;
}


static int dma_writesdio(LS_MMCIState *s)
{
        uint32_t wantsize, copied;
        wantsize = 0;

        while (s->dma_desc.active && (s->blknum || s->iolen)
                        && (s->dma_desc.cmd & DMA_READ_DDR)
                        && (s->sdioregs.sdicmdcon & SDICMDCON_CMDSTART)) {
                if (!s->dma_desc.left && !dma_next(s)) {
                        break;
                }

                wantsize = s->blksize - s->iolen;
                copied = min(s->dma_desc.left, wantsize);

                if (!copied) {
                        break;
                }

                dma_memory_read(s->as, s->dma_desc.saddr, s->ioaddr, copied);

                s->ioaddr += copied;
                s->iolen += copied;
                s->dma_desc.saddr += copied;
                s->dma_desc.left -= copied;

                if (!s->dma_desc.left) {
                        dma_next(s);
                }

                if (s->iolen == s->blksize) {
                        int i;
                        for (i = 0; i < s->blksize; i++) {
                                sdbus_write_data(&s->sdbus, s->fifo_buffer[i]);
                        }
                        s->iolen = 0;
                        s->ioaddr = s->fifo_buffer;
                        s->blknum--;
                }

                if (!s->blknum && !s->iolen) {
                        ls_mmci_end_transfer(s);
                        break;
                }
        }

        return s->dma_desc.active;
}


static void ls_mmci_end_transfer(LS_MMCIState *s)
{
        /* Automatically send CMD12 to stop transfer if AutoCMD12 enabled */
        if ((s->sdioregs.sdidatcon & SDIDCON_BLKNUM)
                        && (s->sdioregs.sdicmdcon & SDICMDCON_ABORT)) {
                SDRequest request;
                uint8_t response[16];

                request.cmd = 0x0C;
                request.arg = 0;
                DPRINT_L1("Automatically issue CMD%d %08x\n", request.cmd,
                request.arg);
                sdbus_do_command(&s->sdbus, &request, response);
                /* Auto CMD12 response goes to the upper Response register */
                s->sdioregs.sdirsp3 = (response[0] << 24) | (response[1] << 16)
                | (response[2] << 8) | response[3];
        }


        s->sdioregs.sdiintmsk |= SDIIMSK_DATAFINISH;
        ls_mmci_update_irq(s);
}

static void ls_mmci_data_transfer(LS_MMCIState *s)
{

        if (s->sdioregs.sdidatcon  & MISC_CTRL_SDIO_DMAEN) {
                if (s->dma_desc.active && (s->blknum || s->iolen) && s->blksize
                && (s->sdioregs.sdicmdcon & SDICMDCON_CMDSTART)) {
                        if (s->dma_desc.cmd & DMA_READ_DDR) {
                                dma_writesdio(s);
                        } else {
                                dma_readsdio(s);
                        }
                }
        }

}


static uint64_t ls_mmci_read(void *opaque, hwaddr offset, unsigned size)
{
        LS_MMCIState *s = (LS_MMCIState *) opaque;

        switch (offset) {
        case SDIINTMSK:
                return s->sdioregs.sdiintmsk;
        default:
                return s->regdata[offset / 4];
        }

        return 0;
}

static void ls_mmci_write(void *opaque,
                              hwaddr offset, uint64_t value, unsigned size)
{
        LS_MMCIState *s = (LS_MMCIState *) opaque;

        switch (offset) {
        case SDIINTMSK:
                s->sdioregs.sdiintmsk &= ~value;
                ls_mmci_update_irq(s);
                break;
        case SDICMDCON:
                s->regdata[offset / 4] = value;
                if (value & SDICMDCON_CMDSTART) {
                        ls_mmci_send_command(s);
                }
                break;
        case 0x400:
        case 0x800:
                _ls_sdio_set_dmaaddr(s, value);
                break;
        default:
                s->regdata[offset / 4] = value;
        }
}

static const MemoryRegionOps ls_mmci_ops = {
        .read = ls_mmci_read,
        .write = ls_mmci_write,
        .endianness = DEVICE_NATIVE_ENDIAN,
};

void *ls_mmci_init(MemoryRegion *sysmem,
                       hwaddr base,
                       BlockBackend *blk, qemu_irq irq)
{
        DeviceState *dev, *carddev;
        SysBusDevice *sbd;
        LS_MMCIState *s;
        Error *err = NULL;

        dev = qdev_create(NULL, TYPE_LS_MMCI);
        s = LS_MMCI(dev);
        if (!s->as) {
                s->as = &address_space_memory;
        }
        s->ioaddr = s->fifo_buffer;
        sbd = SYS_BUS_DEVICE(dev);
        sysbus_connect_irq(sbd, 0, irq);
        if (sysmem)
                memory_region_add_subregion(sysmem, base, sbd->mmio[0].memory);

        /* Create and plug in the sd card */
        carddev = qdev_create(qdev_get_child_bus(dev, "sd-bus"), TYPE_SD_CARD);
        qdev_prop_set_drive(carddev, "drive", blk, &err);
        if (err) {
                error_report("failed to init SD card: %s",
                error_get_pretty(err));
                return NULL;
        }
        object_property_set_bool(OBJECT(carddev), true, "realized", &err);
        if (err) {
                error_report("failed to init SD card: %s",
                error_get_pretty(err));
                return NULL;
        }

        return s;
}

static void ls_mmci_set_inserted(DeviceState *dev, bool inserted)
{
        LS_MMCIState *s = LS_MMCI(dev);

        qemu_set_irq(s->inserted, inserted);
}

static void ls_mmci_set_readonly(DeviceState *dev, bool readonly)
{
        LS_MMCIState *s = LS_MMCI(dev);

        qemu_set_irq(s->readonly, readonly);
}

void ls_mmci_handlers(LS_MMCIState *s, qemu_irq readonly,
                          qemu_irq coverswitch)
{
        DeviceState *dev = DEVICE(s);

        s->readonly = readonly;
        s->inserted = coverswitch;

        ls_mmci_set_inserted(dev, sdbus_get_inserted(&s->sdbus));
        ls_mmci_set_readonly(dev, sdbus_get_readonly(&s->sdbus));
}

typedef struct initcmd {
        uint8_t cmd;
        uint32_t arg;
} initcmd_t;

static void ls_mmci_reset(DeviceState *d)
{
        if (getenv("SDBOOT")) {
                LS_MMCIState *s = LS_MMCI(d);
                SDRequest request;
                uint8_t response[16];
                int i;
                uint32_t rca = 0;
                initcmd_t initcmd[] = {
                        {0, 0},
                        {8, 0x1aa},
                        {55, 0},
                        {41, 0x40ff8000},
                        {2, 0},
                        {3, 0},
                        {7, 0},
                        {55, 0},
                        {6, 2}
                };

                for (i = 0; i < sizeof(initcmd) / sizeof(initcmd[0]); i++) {
                        request.cmd = initcmd[i].cmd;
                        request.crc = 0;        /* FIXME */

                        switch (initcmd[i].cmd) {
                        case 7:
                        case 55:
                                request.arg = rca;
                                break;
                        default:
                                request.arg = initcmd[i].arg;
                                break;
                        }


                        sdbus_do_command(&s->sdbus, &request, response);

                        switch (initcmd[i].cmd) {
                        case 3:
                                rca = (response[0] << 24) | (response[1] << 16);
                                break;
                        }
                }
        }

}

static void set_sdio_inst(LS_MMCIState * s)
{
	static int i;
	if (i < sizeof(ls1gp_sdio) / sizeof(ls1gp_sdio[0]))
		ls1gp_sdio[i++] = s;
}

static void ls_mmci_instance_init(Object *obj)
{
        LS_MMCIState *s = LS_MMCI(obj);
        SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    /* DeviceState *dev = DEVICE(obj); */

        set_sdio_inst(s);

        memory_region_init_io(&s->iomem, obj, &ls_mmci_ops, s,
                              "ls-mmci", 0x001000);
        sysbus_init_mmio(sbd, &s->iomem);
        sysbus_init_irq(sbd, &s->irq);

        qbus_create_inplace(&s->sdbus, sizeof(s->sdbus),
                            TYPE_LS_MMCI_BUS, DEVICE(obj), "sd-bus");
}

static void ls_mmci_class_init(ObjectClass *klass, void *data)
{
        DeviceClass *dc = DEVICE_CLASS(klass);

        dc->vmsd = &vmstate_ls_mmci;
        dc->reset = ls_mmci_reset;
}

static void ls_mmci_bus_class_init(ObjectClass *klass, void *data)
{
        SDBusClass *sbc = SD_BUS_CLASS(klass);

        sbc->set_inserted = ls_mmci_set_inserted;
        sbc->set_readonly = ls_mmci_set_readonly;
}

static const TypeInfo ls_mmci_info = {
        .name = TYPE_LS_MMCI,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(LS_MMCIState),
        .instance_init = ls_mmci_instance_init,
        .class_init = ls_mmci_class_init,
};

static const TypeInfo ls_mmci_bus_info = {
        .name = TYPE_LS_MMCI_BUS,
        .parent = TYPE_SD_BUS,
        .instance_size = sizeof(SDBus),
        .class_init = ls_mmci_bus_class_init,
};

static void ls_mmci_register_types(void)
{
        type_register_static(&ls_mmci_info);
        type_register_static(&ls_mmci_bus_info);
}

type_init(ls_mmci_register_types)

typedef struct mmc_pci_state {
        PCIDevice dev;
        struct LS_MMCIState sdio;
} mmc_pci_state;

/* PCI interface */
#define LSMMC_VENDOR_ID  0x0014
#define LSMMC_DEVICE_ID  0x7a88

static void
pci_mmci_uninit(PCIDevice *dev)
{
}

static void pci_mmci_init(PCIDevice *pci_dev, Error **errp)
{
        mmc_pci_state *d = DO_UPCAST(mmc_pci_state, dev, pci_dev);
        uint8_t *pci_conf;
        struct LS_MMCIState *s = &d->sdio;
        DeviceState *carddev;
        Error *err = NULL;

        pci_conf = d->dev.config;

        pci_config_set_vendor_id(pci_conf, LSMMC_VENDOR_ID);
        pci_config_set_device_id(pci_conf, LSMMC_DEVICE_ID);
        pci_conf[0x04] = 0x07; /* command = I/O space, Bus Master */
        pci_config_set_class(pci_conf, PCI_CLASS_STORAGE_OTHER);
        pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL | 0x80; /*
        header_type */
        pci_conf[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */
        pci_conf[0x34] = 0;

        memory_region_init_io(&s->iomem, OBJECT(pci_dev), &ls_mmci_ops, s,
                              "ls-mmci", 0x001000);
        pci_register_bar(&d->dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->iomem);
        s->irq = pci_allocate_irq(pci_dev);
        s->as = pci_get_address_space(pci_dev);
        s->ioaddr = s->fifo_buffer;

        qbus_create_inplace(&s->sdbus, sizeof(s->sdbus),
                            TYPE_LS_MMCI_BUS, DEVICE(pci_dev), "sd-bus");

        /* Create and plug in the sd card */
        carddev = qdev_create(&s->sdbus.qbus, TYPE_SD_CARD);
        if (s->blk) {
                qdev_prop_set_drive(carddev, "drive", s->blk, &err);
                set_sdio_inst(s);
        }
        if (err) {
                error_report("failed to init SD card: %s",
                error_get_pretty(err));
        }
        object_property_set_bool(OBJECT(carddev), true, "realized", &err);
        if (err) {
                error_report("failed to init SD card: %s",
                error_get_pretty(err));
        }
}

static Property mmc_pci_properties[] = {
        DEFINE_PROP_PTR("blk", mmc_pci_state, sdio.blk_ptr),
        DEFINE_PROP_END_OF_LIST(),
};

static void mmc_pci_class_init(ObjectClass *klass, void *data)
{
        DeviceClass *dc = DEVICE_CLASS(klass);
        PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

        k->realize = pci_mmci_init;
        k->exit = pci_mmci_uninit;
        k->vendor_id = LSMMC_VENDOR_ID;
        k->device_id = LSMMC_DEVICE_ID;
        k->revision = 0x03;
        k->class_id = PCI_CLASS_STORAGE_OTHER;
        dc->desc = "loongson mmc storage";
        dc->props = mmc_pci_properties;
}

static const TypeInfo mmc_pci_info = {
        .name          = "pci-lsmmc",
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(mmc_pci_state),
        .class_init    = mmc_pci_class_init,
        .interfaces = (InterfaceInfo[])
        {
                { INTERFACE_PCIE_DEVICE },
                { }
        },
};

static void mmc_pci_register_types(void)
{
        type_register_static(&mmc_pci_info);
}

type_init(mmc_pci_register_types)
