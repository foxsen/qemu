/*
 *  LS2H I2C Bus Serial Interface Emulation
 *
 *  Copyright (C) 2016 Fuxin Zhang <zhangfx@lemote.com>, Lemote LLC.
 *
 *  This program is licensed under GPL2+
 *
 */

#include "hw/i2c/i2c.h"
#include <hw/sysbus.h>

#ifndef DEBUG_LS2H_I2C
#define DEBUG_LS2H_I2C 1
#endif

/* I2C register offset and bit defines */
#define  PRERlo     0
#define  PRERhi     1

#define  CTR        2
#define  CTR_EN     (1 << 7)
#define  CTR_IEN    (1 << 6)

#define  TXR        3
#define  RXR        3

#define  CR         4
#define  CR_START   (1 << 7)
#define  CR_STOP    (1 << 6)
#define  CR_READ    (1 << 5)
#define  CR_WRITE   (1 << 4)
#define  CR_ACK     (1 << 3)
#define  CR_IACK    (1 << 0)

#define  SR         4
#define  SR_RxACK   (1 << 7)
#define  SR_BUSY    (1 << 6)
#define  SR_AL      (1 << 5)
#define  SR_TIP     (1 << 1)
#define  SR_IF      (1 << 0)

#define  ADDR_RESET 0xff

#define TYPE_LS2H_I2C "ls2h-i2c"
#define LS2H_I2C(obj) OBJECT_CHECK(LS2HI2CState, (obj), TYPE_LS2H_I2C)

#define LS2H_I2C_MEM_SIZE           0x1000

typedef struct LS2HI2CState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;

    uint8_t prerlo;
    uint8_t prerhi;
    uint8_t ctr;
    uint8_t cr;
    uint8_t sr;
    uint8_t dr_read;
    uint8_t dr_write;
} LS2HI2CState;

#define DPRINTF(fmt, args...) \
    do { \
        if (DEBUG_LS2H_I2C) { \
            fprintf(stderr, "[%s]%s: " fmt , TYPE_LS2H_I2C, \
                                             __func__, ##args); \
        } \
    } while (0)

static const char *ls2h_i2c_get_regname(unsigned offset, int rd)
{
    switch (offset) {
    case PRERlo:
        return "PRERlo";
    case PRERhi:
        return "PRERhi";
    case CTR:
        return "CTR";
    case TXR:
        return (rd ? "RXR" : "TXR");
    case CR:
        return (rd ? "SR" : "CR");
    default:
        return "[?]";
    }
}

static inline bool ls2h_i2c_is_enabled(LS2HI2CState *s)
{
    return s->ctr & CTR_EN;
}

static inline bool ls2h_i2c_interrupt_is_enabled(LS2HI2CState *s)
{
    return s->ctr & CTR_IEN;
}

static void ls2h_i2c_reset(DeviceState *dev)
{
    LS2HI2CState *s = LS2H_I2C(dev);

    s->prerlo     = 0;
    s->prerhi     = 0;
    s->cr       = 0;
    s->sr       = 0;
    s->dr_read  = 0;
    s->dr_write = 0;
}

static inline void ls2h_i2c_raise_interrupt(LS2HI2CState *s)
{
    /*
     * raise an interrupt if the device is enabled and it is configured
     * to generate some interrupts.
     */
    if (ls2h_i2c_is_enabled(s) && ls2h_i2c_interrupt_is_enabled(s)) {
        s->sr |= SR_IF;
        qemu_irq_raise(s->irq);
    }
}

static uint64_t ls2h_i2c_read(void *opaque, hwaddr offset,
                             unsigned size)
{
    uint8_t value;
    LS2HI2CState *s = LS2H_I2C(opaque);

    switch (offset) {
    case PRERlo:
        value = s->prerlo;
        break;
    case PRERhi:
        value = s->prerhi;
        break;
    case CTR:
        value = s->ctr;
        break;
    case SR:
        value = s->sr;
        break;
    case RXR:
        value = s->dr_read;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "[%s]%s: Bad address at offset 0x%"
                      HWADDR_PRIx "\n", TYPE_LS2H_I2C, __func__, offset);
        value = 0;
        break;
    }

    DPRINTF("read %s [0x%" HWADDR_PRIx "] -> 0x%02x\n",
            ls2h_i2c_get_regname(offset, 1), offset, value);

    return (uint64_t)value;
}

static void ls2h_i2c_write(void *opaque, hwaddr offset,
                          uint64_t value, unsigned size)
{
    LS2HI2CState *s = LS2H_I2C(opaque);

    DPRINTF("write %s [0x%" HWADDR_PRIx "] <- 0x%02x\n",
            ls2h_i2c_get_regname(offset, 0), offset, (int)value);

    value &= 0xff;

    switch (offset) {
    case PRERlo:
        s->prerlo = value;
        break;
    case PRERhi:
        s->prerhi = value;
        break;
    case CTR:
        if (ls2h_i2c_is_enabled(s) && ((value & CTR_EN) == 0)) {
            ls2h_i2c_reset(DEVICE(s));
        } else {
            s->ctr = value;
        }
        break;
    case CR:
        s->cr = value;
        if (s->cr == (CR_START | CR_WRITE)) {
            if (i2c_start_transfer(s->bus, extract32(s->dr_write, 1, 7) << 1,
                        extract32(s->dr_write, 0, 1))) {
                /* if non zero is returned, the adress is not valid */
                s->sr |= SR_RxACK;
            } else {
                s->sr &= ~SR_RxACK;
                ls2h_i2c_raise_interrupt(s);
            }
        } else if (s->cr & CR_WRITE) {
            /* This is a normal data write */
            if (i2c_send(s->bus, s->dr_write)) {
                /* if the target return non zero then end the transfer */
                s->sr |= SR_RxACK;
                i2c_end_transfer(s->bus);
            } else {
                s->sr &= ~SR_RxACK;
                ls2h_i2c_raise_interrupt(s);
            }
        } else if (s->cr & CR_READ) { 
            /* get the next byte */
            s->dr_read = i2c_recv(s->bus);
            s->sr &= ~SR_RxACK;
            ls2h_i2c_raise_interrupt(s);
        } else if (s->cr & CR_STOP) {
            s->sr &= ~SR_BUSY;
            i2c_end_transfer(s->bus);
        } else 
            fprintf(stderr, "error i2c cmd 0x%x\n", s->cr);

        if ((s->sr & SR_IF) && (value & CR_IACK)) {
            s->sr &= ~SR_IF;
            qemu_irq_lower(s->irq);
        }

        break;
    case TXR:
        /* if the device is not enabled, nothing to do */
        if (!ls2h_i2c_is_enabled(s)) {
            break;
        }

        s->dr_write = value;

        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "[%s]%s: Bad address at offset 0x%"
                      HWADDR_PRIx "\n", TYPE_LS2H_I2C, __func__, offset);
        break;
    }
}

static const MemoryRegionOps ls2h_i2c_ops = {
    .read = ls2h_i2c_read,
    .write = ls2h_i2c_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription ls2h_i2c_vmstate = {
    .name = TYPE_LS2H_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(prerlo, LS2HI2CState),
        VMSTATE_UINT8(prerhi, LS2HI2CState),
        VMSTATE_UINT8(cr, LS2HI2CState),
        VMSTATE_UINT8(sr, LS2HI2CState),
        VMSTATE_UINT8(dr_read, LS2HI2CState),
        VMSTATE_UINT8(dr_write, LS2HI2CState),
        VMSTATE_END_OF_LIST()
    }
};

static void ls2h_i2c_realize(DeviceState *dev, Error **errp)
{
    LS2HI2CState *s = LS2H_I2C(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ls2h_i2c_ops, s, 
            TYPE_LS2H_I2C, LS2H_I2C_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->bus = i2c_init_bus(DEVICE(dev), "i2c");
}

static void ls2h_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &ls2h_i2c_vmstate;
    dc->reset = ls2h_i2c_reset;
    dc->realize = ls2h_i2c_realize;
}

static const TypeInfo ls2h_i2c_type_info = {
    .name = TYPE_LS2H_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LS2HI2CState),
    .class_init = ls2h_i2c_class_init,
};

static void ls2h_i2c_register_types(void)
{
    type_register_static(&ls2h_i2c_type_info);
}

type_init(ls2h_i2c_register_types)
