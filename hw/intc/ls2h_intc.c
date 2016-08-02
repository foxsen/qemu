/*
 * LS2H interrupt controller emulation.
 *
 * Copyright (C) 2016, Fuxin Zhang, Lemote LLC.
 *
 * This program is licensed under GPL2+
 */
#include "hw/hw.h"
#include "hw/mips/ls2h.h"
#include "hw/sysbus.h"

/* Interrupt Handlers */
typedef struct ls2h_intr_handler_bank_s {
    uint32_t intisr;
    uint32_t intien;
    uint32_t intset;
    uint32_t intclr;
    uint32_t intpol;
    uint32_t intedge;
    const char *names[32];

    uint32_t lastisr;
    qemu_irq irq[32];
    qemu_irq parent_irq;
} ls2h_intr_handler_bank;

ls2h_intr_handler_bank irq_map[5] = {
    {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xff7fffff, 0x00000000,
     {"acpi", "hpet", "uart0", "uart1", "uart2", "uart3", "spi", "i2c0",
      "i2c1", "ac97", "dma0", "dma1", "dma2", "lpc", "rtc-0","rtc-1",
      "rtc-2", "toy-0", "toy-1", "toy-2", "rtc-t", "toy-t", "nand", "sys_intn",
      "resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv"}, 
    },
    {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xfeffffff, 0x00000000,
     {"ehci", "ohci", "otg", "gmac0", "gmac1", "sata", "gpu", "dc",
      "pwm0", "pwm1", "pwm2", "pwm3", "ht0", "ht1", "ht2","ht3",
      "ht4", "ht5", "ht6", "ht7", "pcie0", "pcie1", "pcie2", "pcie3",
      "sataphy", "hda", "resv", "resv", "resv", "resv", "resv", "resv"}, 
    },
    {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0x00000000,
     {"gpio0", "gpio1", "gpio2", "gpio3", "gpio4", "gpio5", "gpio6", "gpio7",
      "gpio8", "gpio9", "gpio10", "gpio11", "gpio12", "gpio13", "gpio14","gpio15",
      "resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv",
      "resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv"}, 
    },
    {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0xffffffff,
     {"resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv",
      "resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv", 
      "resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv",
      "resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv"}, 
    },
    {0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0xffffffff, 0xffffffff,
     {"resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv",
      "resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv", 
      "resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv",
      "resv", "resv", "resv", "resv", "resv", "resv", "resv", "resv"}, 
    }
};

#define TYPE_LS2H_INTC "ls2h-intc"
#define LS2H_INTC(obj) OBJECT_CHECK(ls2h_intr_handler, (obj), TYPE_LS2H_INTC)

typedef struct ls2h_intr_handler_s {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    int nbanks;
    uint32_t size;
    uint8_t revision;

    /* state */
    ls2h_intr_handler_bank bank[5];
} ls2h_intr_handler;

static inline void ls2h_inth_update(ls2h_intr_handler *s)
{
    int i;

    for (i = 0; i < s->nbanks; ++i) {
        if (s->bank[i].intisr & s->bank[i].intien) {
            qemu_set_irq(s->bank[i].parent_irq, 1);
        } else {
            qemu_set_irq(s->bank[i].parent_irq, 0);
        }
    }
}

static void ls2h_set_intr(void *opaque, int irq, int req)
{
    ls2h_intr_handler *s = (ls2h_intr_handler *) opaque;

    ls2h_intr_handler_bank *bank = &s->bank[irq >> 5];

    int n = irq & 31;
    int mask = 1 << n;

    req = (bank->intpol & mask) ? req : !req;

    //fprintf(stderr, "irq %d set to %d\n", irq, req);

    if (bank->intedge & mask) {
        /* edge triggered */
        if (req) {
            if ((bank->lastisr & mask) == 0) {
                bank->intisr |= mask;
            }
            bank->lastisr |= mask;
        } else {
            bank->lastisr &= ~mask;
        }
    } else {
        /* level triggered */
        if (req) {
            bank->intisr |= mask;
            bank->lastisr |= mask;
        } else {
            bank->intisr &= ~mask;
            bank->lastisr &= ~mask;
        }
    }

    ls2h_inth_update(s);
}


static uint64_t ls2h_inth_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    ls2h_intr_handler *s = (ls2h_intr_handler *) opaque;
    uint32_t bank_no;
    uint32_t reg_offset;
    ls2h_intr_handler_bank *bank;

    /* MSI port write only*/
    if (addr < 0x40) return -1;

    addr = addr - 0x40;
    bank_no = addr / 24;
    reg_offset = addr % 24;
    bank = &s->bank[bank_no];

    return *(int *)((char *)bank + reg_offset);
}

static void ls2h_inth_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    ls2h_intr_handler *s = (ls2h_intr_handler *) opaque;
    uint32_t bank_no;
    uint32_t reg_offset;
    ls2h_intr_handler_bank *bank;

    /* MSI port */
    if (addr == 0) {
        bank_no = 3;
        /* write to MSI port 5 bit int num will be translated to 32 bit vector
           write to intset 
         */
        reg_offset = 8;
        value = 1 << (value & 0x1f);
    } else {
        if (addr < 0x40) return;
        addr = addr - 0x40;
        bank_no = addr / 24;
        reg_offset = addr % 24;
    }

    bank = &s->bank[bank_no];

    switch (reg_offset) {
        case 0:
            /* status RO */
            //bank->intisr = value;
            break;
        case 4:
            /* normally via intset/intclr */
            bank->intien = value;
            break;
        case 8:
            bank->intset = value;
            bank->intien |= bank->intset;
            break;
        case 12:
            bank->intclr = value;
            bank->intien &= (~bank->intclr);
            /* clr has side effect of clear isr */
            bank->intisr &= (~bank->intclr);
            bank->lastisr &= (~bank->intclr);
            break;
        case 16:
            bank->intpol = value;
            break;
        case 20:
            bank->intedge = value;
            break;
    }

    ls2h_inth_update(s);
}

static const MemoryRegionOps ls2h_inth_mem_ops = {
    .read = ls2h_inth_read,
    .write = ls2h_inth_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void ls2h_inth_reset(DeviceState *dev)
{
    ls2h_intr_handler *s = LS2H_INTC(dev);
    int i,j;

    for (i = 0; i < s->nbanks; i++){
        s->bank[i].intisr = irq_map[i].intisr;
        s->bank[i].intien  = irq_map[i].intien;
        s->bank[i].intset = irq_map[i].intset;
        s->bank[i].intclr = irq_map[i].intclr;
        s->bank[i].intedge = irq_map[i].intedge;
        s->bank[i].intpol = irq_map[i].intpol;
        for (j = 0; j < 32; j ++) {
            s->bank[i].names[j] = irq_map[i].names[j];
            s->bank[i].irq[j] = qdev_get_gpio_in(dev, i * 32 + j);
        }
        s->bank[i].lastisr = 0;
    }
}

static int ls2h_intc_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    ls2h_intr_handler *s = LS2H_INTC(dev);
    int i;

    s->nbanks = 5;

    for (i = 0; i < s->nbanks; i++) {
        sysbus_init_irq(sbd, &s->bank[i].parent_irq);
    }
    qdev_init_gpio_in(dev, ls2h_set_intr, s->nbanks * 32);
    memory_region_init_io(&s->mmio, OBJECT(s), &ls2h_inth_mem_ops, s,
                          "ls2h-intc", s->size);
    sysbus_init_mmio(sbd, &s->mmio);

    return 0;
}

static Property ls2h_intc_properties[] = {
    DEFINE_PROP_UINT32("size", ls2h_intr_handler, size, LS2H_INTC_SIZE),
    DEFINE_PROP_END_OF_LIST(),
};

static void ls2h_intc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls2h_intc_init;
    dc->reset = ls2h_inth_reset;
    dc->props = ls2h_intc_properties;
}

static const TypeInfo ls2h_intc_info = {
    .name          = "ls2h-intc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .class_init    = ls2h_intc_class_init,
    .instance_size = sizeof(ls2h_intr_handler),
};

static void ls2h_intc_register_types(void)
{
    type_register_static(&ls2h_intc_info);
}

type_init(ls2h_intc_register_types)
