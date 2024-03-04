/*
 * QEMU loongson i2c emulation
 *
 * Copyright (c) 2013 qiaochong@loongson.cn
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
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"

/* I2C Interface */
typedef struct {
        SysBusDevice busdev;
        MemoryRegion iomem;
        I2CBus *bus;
        uint8_t prer_lo;
        uint8_t prer_hi;
        uint16_t control;
        uint16_t status;
        uint8_t ibmr;
        uint8_t data;
        uint8_t bltop;
        uint8_t scaddr;
        qemu_irq irq;
        int shift;
} ls_i2c_state;

#define IBMR    0x80    /* I2C Bus Monitor register */
#define IDBR    0x88    /* I2C Data Buffer register */
#define ICR     0x90    /* I2C Control register */
#define ISR     0x98    /* I2C Status register */
#define ISAR    0xa0    /* I2C Slave Address register */

#define REG_I2C_PRER_LO 0x0
#define REG_I2C_PRER_HI 0x1
#define REG_I2C_CTR     0x2
#define REG_I2C_TXR     0x3
#define REG_I2C_RXR     0x3
#define REG_I2C_CR      0x4
#define REG_I2C_SR      0x4
#define REG_I2C_BLTOP   0x5
#define REG_I2C_RESERVE 0x6
#define REG_I2C_SCADDR  0x7

static uint64_t ls_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
        ls_i2c_state *s = (ls_i2c_state *) opaque;

        switch ((addr >> s->shift) & 0xf) {
        case REG_I2C_PRER_LO:
                return s->prer_lo;

        case REG_I2C_PRER_HI:
                return s->prer_hi;

        case REG_I2C_CTR:
                return s->control;

        case REG_I2C_SR:
                return s->status | (i2c_bus_busy(s->bus) << 2);

        case REG_I2C_RXR:
                return s->data;

        case REG_I2C_BLTOP:
                return s->bltop;

        case REG_I2C_RESERVE:
                return 0;

        case REG_I2C_SCADDR:
                return s->scaddr;

        default:
                hw_error("ls_i2c_read bad reg 0x%x\n", (unsigned)addr);
                break;
        }
        return 0;
}



#define I2C_C_START 0x80
#define I2C_C_STOP 0x40
#define I2C_C_READ 0x20
#define I2C_C_WRITE 0x10
#define I2C_C_WACK 0x8
#define I2C_C_IACK 0x1

#define I2C_S_RUN 0x2
#define I2C_S_BUSY 0x40
#define I2C_S_RNOACK 0x80
#define I2C_S_IF 1

static void ls_i2c_check_irq(ls_i2c_state *s)
{
        if ((s->status & I2C_S_IF) && (s->control & (1 << 6))) {
                qemu_irq_raise(s->irq);
        } else {
                qemu_irq_lower(s->irq);
        }
}

static void ls_i2c_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned size)
{
        ls_i2c_state *s = (ls_i2c_state *) opaque;
        int ack = 0;

        switch ((addr >> s->shift) & 0xf) {
        case REG_I2C_CR:
                if (value & I2C_C_IACK) {
                        s->status &= ~I2C_S_IF;
                }

                /* TODO: slave mode */
                if (value  & (I2C_C_WRITE | I2C_C_READ | I2C_C_STOP)) {
                        if ((value & (I2C_C_START | I2C_C_WRITE)) ==
                        (I2C_C_START | I2C_C_WRITE)) { /* START condition */
                                ack = !i2c_start_transfer(s->bus, (s->data & ~1)
                                >> 1, s->data & 1);
                        } else {
                                if (value & I2C_C_READ) { /* RWM */
                                        s->data = i2c_recv(s->bus);
                                        if (value & I2C_C_WACK) /* ACKNAK */
                                                i2c_nack(s->bus);
                                        ack = 1;
                                } else if (value & I2C_C_WRITE) {
                                        ack = !i2c_send(s->bus, s->data);
                                }
                        }

                        if (value & I2C_C_STOP) { /* STOP condition */
                                i2c_end_transfer(s->bus);
                                ack = 1;
                        }

                        s->status = I2C_S_IF;
                        if (!ack) {
                                s->status |= I2C_S_RNOACK;
                        }
                }

                ls_i2c_check_irq(s);
                break;

        case REG_I2C_PRER_LO:
                s->prer_lo = value;
                break;

        case REG_I2C_PRER_HI:
                s->prer_hi = value;
                break;

        case REG_I2C_CTR:
                s->control = value;
                ls_i2c_check_irq(s);
                break;

        case REG_I2C_TXR:
                s->data = value;
                break;

        case REG_I2C_BLTOP:
                s->bltop = value;
                break;

        case REG_I2C_RESERVE:
                break;

        case REG_I2C_SCADDR:
                s->scaddr = value;
                break;

        default:
                hw_error("ls_i2c_write bad reg 0x%x\n", (unsigned)addr);
        }
}

static const MemoryRegionOps ls_i2c_ops = {
        .read = ls_i2c_read,
        .write = ls_i2c_write,
        .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LSI2C "ls_i2c"

#define SYS_BUS_LSI2C(obj)                                            \
        OBJECT_CHECK(ls_i2c_state, (obj), TYPE_SYS_BUS_LSI2C)

static int ls_i2c_init(SysBusDevice *dev)
{
        ls_i2c_state *d = SYS_BUS_LSI2C(dev);

        memory_region_init_io(&d->iomem, NULL, &ls_i2c_ops, (void *)d,
                              "ls i2c", 0x8 << d->shift);

        sysbus_init_irq(dev, &d->irq);
        sysbus_init_mmio(dev, &d->iomem);
        d->bus = i2c_init_bus(DEVICE(dev), "i2c");

        return 0;
}

static Property ls_i2c_properties[] = {
        DEFINE_PROP_INT32("shift", ls_i2c_state, shift, 0),
        DEFINE_PROP_END_OF_LIST(),
};

static void ls_i2c_class_init(ObjectClass *klass, void *data)
{
        DeviceClass *dc = DEVICE_CLASS(klass);
        SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

        k->init = ls_i2c_init;
        dc->props = ls_i2c_properties;
        dc->desc = "ls i2c";
}

static const TypeInfo ls_i2c_info = {
        .name          = "ls_i2c",
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ls_i2c_state),
        .class_init    = ls_i2c_class_init,
};


static void ls_i2c_register_types(void)
{
        type_register_static(&ls_i2c_info);
}

type_init(ls_i2c_register_types)

