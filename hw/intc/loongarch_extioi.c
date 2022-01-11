/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Loongson 3A5000 ext interrupt controller emulation
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/loongarch/loongarch.h"
#include "hw/qdev-properties.h"
#include "exec/address-spaces.h"
#include "hw/intc/loongarch_extioi.h"
#include "migration/vmstate.h"
#include "trace.h"

static void extioi_update_irq(void *opaque, int irq_num, int level)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(opaque);
    uint8_t  ipnum, cpu;
    unsigned long found1, found2;

    ipnum = s->sw_ipmap[irq_num];
    cpu   = s->sw_coremap[irq_num];
    if (level == 1) {
        if (test_bit(irq_num, (void *)s->enable) == false) {
            return;
        }
        bitmap_set((void *)s->coreisr[cpu], irq_num, 1);
        found1 = find_next_bit((void *)&(s->sw_ipisr[cpu][ipnum]),
                               EXTIOI_IRQS, 0);
        bitmap_set((void *)&(s->sw_ipisr[cpu][ipnum]), irq_num, 1);

        if (found1 >= EXTIOI_IRQS) {
            qemu_set_irq(s->parent_irq[cpu][ipnum], level);
        }
    } else {
        bitmap_clear((void *)s->coreisr[cpu], irq_num, 1);
        found1 = find_next_bit((void *)&(s->sw_ipisr[cpu][ipnum]),
                               EXTIOI_IRQS, 0);
        bitmap_clear((void *)&(s->sw_ipisr[cpu][ipnum]), irq_num, 1);
        found2 = find_next_bit((void *)&(s->sw_ipisr[cpu][ipnum]),
                               EXTIOI_IRQS, 0);

        if ((found1 < EXTIOI_IRQS) && (found2 >= EXTIOI_IRQS)) {
            qemu_set_irq(s->parent_irq[cpu][ipnum], level);
        }
    }
}

static void extioi_setirq(void *opaque, int irq, int level)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(opaque);
    trace_extioi_setirq(irq, level);
    extioi_update_irq(s, irq, level);
}

static uint64_t extioi_readw(void *opaque, hwaddr addr, unsigned size)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(opaque);
    unsigned long offset = addr & 0xffff;
    uint32_t ret, index;
    int cpu;

    if ((offset >= EXTIOI_NODETYPE_START) && (offset < EXTIOI_NODETYPE_END)) {
        index = (offset - EXTIOI_NODETYPE_START) >> 2;
        ret = s->nodetype[index];
    } else if ((offset >= EXTIOI_BOUNCE_START) &&
               (offset < EXTIOI_BOUNCE_END)) {
        index = (offset - EXTIOI_BOUNCE_START) >> 2;
        ret = s->bounce[index];
    } else if ((offset >= EXTIOI_COREISR_START) &&
               (offset < EXTIOI_COREISR_END)) {
        index = ((offset - EXTIOI_COREISR_START) & 0x1f) >> 2;
        cpu = ((offset - EXTIOI_COREISR_START) >> 8) & 0x3;
        ret = s->coreisr[cpu][index];
    }

    trace_loongarch_extioi_readw((uint32_t)addr, ret);
    return ret;
}

static void extioi_writew(void *opaque, hwaddr addr,
                           uint64_t val, unsigned size)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(opaque);
    int cpu, index;
    uint32_t offset, old_data, i, j, bits;

    offset = addr & 0xffff;
    trace_loongarch_extioi_writew(size, (uint32_t)addr, val);

    if ((offset >= EXTIOI_NODETYPE_START) && (offset < EXTIOI_NODETYPE_END)) {
        index = (offset - EXTIOI_NODETYPE_START) >> 2;
        s->nodetype[index] = val;
    } else if ((offset >= EXTIOI_BOUNCE_START) &&
               (offset < EXTIOI_BOUNCE_END)) {
        index = (offset - EXTIOI_BOUNCE_START) >> 2;
        s->bounce[index] = val;
    } else if ((offset >= EXTIOI_COREISR_START) &&
               (offset < EXTIOI_COREISR_END)) {
        index = ((offset - EXTIOI_COREISR_START) & 0x1f) >> 2;
        cpu = ((offset - EXTIOI_COREISR_START) >> 8) & 0x3;

        /* Ext_core_ioisr */
        old_data = s->coreisr[cpu][index];
        s->coreisr[cpu][index] = old_data & ~val;

        if (old_data != s->coreisr[cpu][index]) {
            bits = size * 8;
            while ((i = find_first_bit((void *)&val, bits)) != bits) {
                j = test_bit(i, (unsigned long *)&old_data);
                if (j) {
                    extioi_update_irq(s, i + index * 32, 0);
                }
                clear_bit(i, (void *)&val);
            }
        }
    }
}

static uint64_t extioi_enable_read(void *opaque, hwaddr addr, unsigned size)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(opaque);
    uint8_t ret;

    if (addr < EXTIOI_ENABLE_END) {
        ret = s->enable[addr];
    }

    trace_loongarch_extioi_enable_read((uint8_t)addr, ret);
    return ret;
}

static void extioi_enable_write(void *opaque, hwaddr addr,
                                uint64_t value, unsigned size)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(opaque);
    uint8_t old_data, val = value & 0xff;
    int i, level;

    trace_loongarch_extioi_enable_write(size, (uint8_t)addr, val);
    if (addr < EXTIOI_ENABLE_END) {
        old_data = s->enable[addr];

        if (old_data != val) {
            s->enable[addr] = val;
            old_data = old_data ^ val;

            while ((i = find_first_bit((void *)&old_data, 8)) != 8) {
                level = test_bit(i, (unsigned long *)&val);
                extioi_update_irq(s, i + addr * 8, level);
                clear_bit(i, (void *)&old_data);
            }
        }
    }
}

static uint64_t extioi_ipmap_read(void *opaque, hwaddr addr, unsigned size)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(opaque);
    uint8_t ret;

    if (addr < EXTIOI_IPMAP_END) {
        ret = s->ipmap[addr];
    }

    trace_loongarch_extioi_ipmap_read((uint8_t)addr, ret);
    return ret;
}

static void extioi_ipmap_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(opaque);
    uint8_t val = value & 0xff;
    int i, ipnum, irqnum;

    trace_loongarch_extioi_ipmap_write(size, (uint8_t)addr, val);
    if (addr < EXTIOI_IPMAP_END) {
        s->ipmap[addr] = val;

        /* Routing in groups of 32 interrupt */
        ipnum = find_first_bit((void *)&val, 4);
        for (i = 0; i < 32; i++) {
            irqnum = addr * 32 + i;
            if (ipnum != 4) {
                s->sw_ipmap[irqnum] = ipnum;
            } else {
                s->sw_ipmap[irqnum] = 0;
            }
        }
    }
}

static uint64_t extioi_coremap_read(void *opaque, hwaddr addr, unsigned size)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(opaque);
    uint8_t ret;

    if (addr < EXTIOI_COREMAP_END) {
        ret = s->coremap[addr];
    }

    trace_loongarch_extioi_coremap_read((uint8_t)addr, ret);
    return ret;
}

static void extioi_coremap_write(void *opaque, hwaddr addr,
                                 uint64_t value, unsigned size)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(opaque);
    uint8_t val = value & 0xff;
    int cpu;

    trace_loongarch_extioi_coremap_write(size, (uint8_t)addr, val);
    if (addr < EXTIOI_COREMAP_END) {
        s->coremap[addr] = val;

        /* Only support 1 node now only handle the core map*/
        if (val) {
            cpu = find_first_bit((void *)&val, 4);
            if (cpu != 4) {
                s->sw_coremap[addr] = cpu;
            }
        }
    }
}

static const MemoryRegionOps extioi_reg32_ops = {
    .read = extioi_readw,
    .write = extioi_writew,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps extioi_enable_ops = {
    .read = extioi_enable_read,
    .write = extioi_enable_write,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps extioi_ipmap_ops = {
    .read = extioi_ipmap_read,
    .write = extioi_ipmap_write,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps extioi_coremap_ops = {
    .read = extioi_coremap_read,
    .write = extioi_coremap_write,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void loongarch_extioi_realize(DeviceState *dev, Error **errp)
{
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(dev);
    MachineState *ms = MACHINE(qdev_get_machine());
    int cpu;

    for (cpu = 0; cpu < ms->smp.cpus; cpu++) {
        memory_region_init_io(&s->mmio_reg32[cpu], OBJECT(s),
                              &extioi_reg32_ops, s, TYPE_LOONGARCH_EXTIOI,
                              0x900);
        /*
         * kernel use anysend to handle enable reg.
         * need support different size handle.
         */
        memory_region_init_io(&s->mmio_enable[cpu], OBJECT(s),
                              &extioi_enable_ops, s, TYPE_LOONGARCH_EXTIOI,
                              0x18);
        memory_region_init_io(&s->mmio_ipmap[cpu], OBJECT(s),
                              &extioi_ipmap_ops, s, TYPE_LOONGARCH_EXTIOI,
                              0x8);
        memory_region_init_io(&s->mmio_coremap[cpu], OBJECT(s),
                              &extioi_coremap_ops, s, TYPE_LOONGARCH_EXTIOI,
                              0x100);
    }
}

static const VMStateDescription vmstate_ext_sw_ipisr = {
    .name = "ext_sw_ipisr",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(irq, ext_sw_ipisr, EXTIOI_IRQS),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_loongarch_extioi = {
    .name = TYPE_LOONGARCH_EXTIOI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(bounce, LoongArchExtIOI, EXTIOI_IRQS_GROUP_COUNT),
        VMSTATE_UINT32_2DARRAY(coreisr, LoongArchExtIOI, MAX_CORES,
                               EXTIOI_IRQS_GROUP_COUNT),
        VMSTATE_UINT32_ARRAY(nodetype, LoongArchExtIOI,
                             EXTIOI_IRQS_NODETYPE_COUNT / 2),
        VMSTATE_UINT8_ARRAY(enable, LoongArchExtIOI, EXTIOI_IRQS / 8),
        VMSTATE_UINT8_ARRAY(ipmap, LoongArchExtIOI, 8),
        VMSTATE_UINT8_ARRAY(coremap, LoongArchExtIOI, EXTIOI_IRQS),
        VMSTATE_UINT8_ARRAY(sw_ipmap, LoongArchExtIOI, EXTIOI_IRQS),
        VMSTATE_UINT8_ARRAY(sw_coremap, LoongArchExtIOI, EXTIOI_IRQS),
        VMSTATE_STRUCT_2DARRAY(sw_ipisr, LoongArchExtIOI, MAX_CORES,
                               LS3A_INTC_IP, 1, vmstate_ext_sw_ipisr,
                               ext_sw_ipisr),
        VMSTATE_END_OF_LIST()
    }
};

static void loongarch_extioi_instance_init(Object *obj)
{
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);
    LoongArchExtIOI *s = LOONGARCH_EXTIOI(obj);
    MachineState *ms = MACHINE(qdev_get_machine());
    int i, cpu, pin;

    for (i = 0; i < EXTIOI_IRQS; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq[i]);
    }

    qdev_init_gpio_in(DEVICE(obj), extioi_setirq, EXTIOI_IRQS);

    for (cpu = 0; cpu < ms->smp.cpus; cpu++) {
        sysbus_init_mmio(dev, &s->mmio_reg32[cpu]);
        sysbus_init_mmio(dev, &s->mmio_enable[cpu]);
        sysbus_init_mmio(dev, &s->mmio_ipmap[cpu]);
        sysbus_init_mmio(dev, &s->mmio_coremap[cpu]);
        for (pin = 0; pin < LS3A_INTC_IP; pin++) {
            qdev_init_gpio_out(DEVICE(obj), &s->parent_irq[cpu][pin], 1);
        }
    }
}

static void loongarch_extioi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_loongarch_extioi;
    dc->realize = loongarch_extioi_realize;
}

static const TypeInfo loongarch_extioi_info = {
    .name          = TYPE_LOONGARCH_EXTIOI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_init = loongarch_extioi_instance_init,
    .instance_size = sizeof(struct LoongArchExtIOI),
    .class_init    = loongarch_extioi_class_init,
};

static void loongarch_extioi_register_types(void)
{
    type_register_static(&loongarch_extioi_info);
}

type_init(loongarch_extioi_register_types)
