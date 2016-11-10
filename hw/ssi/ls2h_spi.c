/*
 * QEMU model of the LS2H SPI Controller
 *
 * Copyright (C) 2016 Fuxin Zhang, Lemote LLC.
 * 
 * This code is licensed under GPL2+
 */

#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "qemu/log.h"
#include "qemu/fifo8.h"

#include "hw/ssi.h"

#ifdef LS2H_SPI_ERR_DEBUG
#define DB_PRINT(...) do { \
    fprintf(stderr,  ": %s: ", __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    } while (0);
#else
    #define DB_PRINT(...)
#endif

#define R_SPCR         (0)
#define CR_IE          (1<<7)
#define CR_ENABLE      (1<<6)

#define R_SPSR         (1)
#define SR_TX_SPIF    (1 << 7)
#define SR_TX_WCOL    (1 << 6)
#define SR_TX_FULL    (1 << 3)
#define SR_TX_EMPTY   (1 << 2)
#define SR_RX_FULL    (1 << 1)
#define SR_RX_EMPTY   (1 << 0)

#define R_SPFIFO       (2)

#define R_SPER         (3)

#define R_SFCPARAM     (4)
#define PARAM_ME       (1 << 0)

#define R_SPCS         (5)
#define CS_EN_0        (1 << 0)
#define CS_0           (1 << 4)

#define R_SPTIMING     (6)

#define R_MAX           8

#define FIFO_CAPACITY 256

#define TYPE_LS2H_SPI "ls2h-spi"
#define LS2H_SPI(obj) OBJECT_CHECK(Ls2hSPI, (obj), TYPE_LS2H_SPI)

typedef struct Ls2hSPI {
    SysBusDevice parent_obj;

    MemoryRegion mmio[2];

    /* irq to system */
    qemu_irq irq;
    int irqline;
    int sent;

    /* chip select to slave */
    uint8_t num_cs;
    qemu_irq *cs_lines;

    SSIBus *spi;

    Fifo8 rx_fifo;
    Fifo8 tx_fifo;

    uint8_t regs[R_MAX];
} Ls2hSPI;

static void txfifo_reset(Ls2hSPI *s)
{
    fifo8_reset(&s->tx_fifo);

    s->regs[R_SPSR] &= ~SR_TX_FULL;
    s->regs[R_SPSR] |= SR_TX_EMPTY;
}

static void rxfifo_reset(Ls2hSPI *s)
{
    fifo8_reset(&s->rx_fifo);

    s->regs[R_SPSR] |= SR_RX_EMPTY;
    s->regs[R_SPSR] &= ~SR_RX_FULL;
}

static void ls2h_spi_update_irq(Ls2hSPI *s)
{
    uint32_t pending;

    if (!fifo8_is_empty(&s->rx_fifo) || s->sent) {
        s->regs[R_SPSR] |= SR_TX_SPIF;
        s->sent = 0;
    }

    pending = s->regs[R_SPSR] & SR_TX_SPIF;

    pending = pending && (s->regs[R_SPCR] & CR_IE);
    pending = !!pending;

    /* This call lies right in the data paths so don't call the
       irq chain unless things really changed.  */
    if (pending != s->irqline) {
        s->irqline = pending;
        DB_PRINT("irq_change of state %d SR:%x CR:%X\n",
                    pending, s->regs[R_SPSR], s->regs[R_SPCR]);
        qemu_set_irq(s->irq, pending);
    }
}

static void ls2h_spi_update_cs(Ls2hSPI *s)
{
    qemu_set_irq(s->cs_lines[0], s->regs[R_SPCS] & CS_0);
}

static void ls2h_spi_do_reset(Ls2hSPI *s)
{
    memset(s->regs, 0, sizeof s->regs);

    rxfifo_reset(s);
    txfifo_reset(s);

    s->regs[R_SFCPARAM] = PARAM_ME;

    ls2h_spi_update_irq(s);
    ls2h_spi_update_cs(s);
}

static void ls2h_spi_reset(DeviceState *d)
{
    ls2h_spi_do_reset(LS2H_SPI(d));
}

static void spi_flush_txfifo(Ls2hSPI *s)
{
    uint32_t tx;
    uint32_t rx;

    while (!fifo8_is_empty(&s->tx_fifo)) {
        tx = (uint32_t)fifo8_pop(&s->tx_fifo);
        DB_PRINT("data tx:%02x\n", tx);
        rx = ssi_transfer(s->spi, tx);
        DB_PRINT("data rx:%02x\n", rx);
        if (fifo8_is_full(&s->rx_fifo)) {
            s->regs[R_SPSR] |= SR_TX_WCOL;
        } else {
            fifo8_push(&s->rx_fifo, (uint8_t)rx);
            if (fifo8_is_full(&s->rx_fifo)) {
                s->regs[R_SPSR] |= SR_RX_FULL;
            }
        }

        s->regs[R_SPSR] &= ~SR_RX_EMPTY;
        s->regs[R_SPSR] &= ~SR_TX_FULL;
        s->regs[R_SPSR] |= SR_TX_EMPTY;

        s->sent = 1;
    }
}

static uint64_t
spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Ls2hSPI *s = opaque;
    uint32_t r = 0;

    switch (addr) {
    case R_SPFIFO:
        if (fifo8_is_empty(&s->rx_fifo)) {
            DB_PRINT("Read from empty FIFO!\n");
            return 0xdeadbeef;
        }

        s->regs[R_SPSR] &= ~SR_RX_FULL;
        r = fifo8_pop(&s->rx_fifo);
        if (fifo8_is_empty(&s->rx_fifo)) {
            s->regs[R_SPSR] |= SR_RX_EMPTY;
        }
        break;

    case R_SPSR:
        r = s->regs[addr];
        break;

    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            r = s->regs[addr];
        }
        break;

    }
    DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr * 4, r);
    ls2h_spi_update_irq(s);
    return r;
}

static void
spi_write(void *opaque, hwaddr addr,
            uint64_t val64, unsigned int size)
{
    Ls2hSPI *s = opaque;
    uint32_t value = val64;

    DB_PRINT("addr=" TARGET_FMT_plx " = %x\n", addr, value);
    switch (addr) {
    case R_SPFIFO:
        s->regs[R_SPSR] &= ~SR_TX_EMPTY;
        fifo8_push(&s->tx_fifo, (uint8_t)value);
        if (fifo8_is_full(&s->tx_fifo)) {
            s->regs[R_SPSR] |= SR_TX_FULL;
        }
        spi_flush_txfifo(s);
        break;

    case R_SPCR:
        if (value & CR_ENABLE) {
            rxfifo_reset(s);
            txfifo_reset(s);
        }
        s->regs[addr] = value;
        break;

    case R_SPSR:
        if (value & SR_TX_SPIF) {
            value &= ~(SR_TX_SPIF);
        }
        s->regs[addr] = value;
        break;

    case R_SPCS:
        s->regs[addr] = value;
        /* memory & CS enabled */
        if ( ((s->regs[R_SFCPARAM] & PARAM_ME) == 0) && 
                (val64 & CS_EN_0) ) {
            ls2h_spi_update_cs(s);
        }
        break;

    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            s->regs[addr] = value;
        }
        break;
    }

    ls2h_spi_update_irq(s);
}

static const MemoryRegionOps spi_ops = {
    .read = spi_read,
    .write = spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1
    }
};

static uint64_t
spi_rom_read(void *opaque, hwaddr addr, unsigned int size)
{
    Ls2hSPI *s = opaque;
    uint64_t r = -1;
    uint32_t rx;
    int i;

    fprintf(stderr, "rom read at %lx size=%d ", addr, size);

    if ((s->regs[R_SFCPARAM] & PARAM_ME) == 0) {
        fprintf(stderr, "spi rom memory not enabled!\n");
        return r;
    };

    s->regs[R_SPCS] = 0x11;
    ls2h_spi_update_cs(s);
    s->regs[R_SPCS] = 0x01;
    ls2h_spi_update_cs(s);

    /* READ */
    rx = ssi_transfer(s->spi, 0x3);
    /* ADDR */
    rx = ssi_transfer(s->spi, (addr >> 16) & 0xff);
    rx = ssi_transfer(s->spi, (addr >> 8) & 0xff);
    rx = ssi_transfer(s->spi, (addr >> 0) & 0xff);
    r = 0;
    for (i = 0; i < size; i++) {
        rx = ssi_transfer(s->spi, 0);
        r |= ((rx & 0xff) << (8*i));
    }

    /* force slave reset to idle state */
    s->regs[R_SPCS] = 0x01;
    ls2h_spi_update_cs(s);

    s->regs[R_SPCS] = 0x11;
    ls2h_spi_update_cs(s);

    fprintf(stderr, "val=%lx\n", r);
    return r;
}

static const MemoryRegionOps spi_rom_ops = {
    .read = spi_rom_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8
    }
};

static int ls2h_spi_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    Ls2hSPI *s = LS2H_SPI(dev);

    DB_PRINT("\n");

    s->spi = ssi_create_bus(dev, "spi");

    sysbus_init_irq(sbd, &s->irq);
    s->cs_lines = g_new0(qemu_irq, 1);
    ssi_auto_connect_slaves(dev, s->cs_lines, s->spi);
    sysbus_init_irq(sbd, &s->cs_lines[0]);

    memory_region_init_io(&s->mmio[0], OBJECT(s), &spi_ops, s,
                          "ls2h-spi", R_MAX);
    sysbus_init_mmio(sbd, &s->mmio[0]);

    memory_region_init_rom_device(&s->mmio[1], OBJECT(s), &spi_rom_ops, s,
                          "ls2h-spi rom0", 0x200000, &error_fatal);
    sysbus_init_mmio(sbd, &s->mmio[1]);

    s->irqline = -1;
    s->sent = 0;

    fifo8_create(&s->tx_fifo, FIFO_CAPACITY);
    fifo8_create(&s->rx_fifo, FIFO_CAPACITY);

    return 0;
}

static const VMStateDescription vmstate_ls2h_spi = {
    .name = "ls2h_spi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_FIFO8(tx_fifo, Ls2hSPI),
        VMSTATE_FIFO8(rx_fifo, Ls2hSPI),
        VMSTATE_UINT8_ARRAY(regs, Ls2hSPI, R_MAX),
        VMSTATE_END_OF_LIST()
    }
};

static Property ls2h_spi_properties[] = {
    DEFINE_PROP_UINT8("num-ss-bits", Ls2hSPI, num_cs, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void ls2h_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls2h_spi_init;
    dc->reset = ls2h_spi_reset;
    dc->props = ls2h_spi_properties;
    dc->vmsd = &vmstate_ls2h_spi;
}

static const TypeInfo ls2h_spi_info = {
    .name           = TYPE_LS2H_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Ls2hSPI),
    .class_init     = ls2h_spi_class_init,
};

static void ls2h_spi_register_types(void)
{
    type_register_static(&ls2h_spi_info);
}

type_init(ls2h_spi_register_types)
