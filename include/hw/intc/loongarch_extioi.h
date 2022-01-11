/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * LoongArch 3A5000 ext interrupt controller definitions
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 */

#include "hw/sysbus.h"
#include "hw/loongarch/loongarch.h"

#ifndef LOONGARCH_EXTIOI_H
#define LOONGARCH_EXTIOI_H

#define LS3A_INTC_IP                 8
#define MAX_CORES                    LOONGARCH_MAX_VCPUS
#define EXTIOI_IRQS                  (256)
/* 32 irqs belong to a group */
#define EXTIOI_IRQS_GROUP_COUNT      (256 / 32)
/* map to ipnum per 32 irqs */
#define EXTIOI_IRQS_NODETYPE_COUNT   16

#define APIC_BASE                    0x1400
#define ENABLE_OFFSET                0x1600
#define IPMAP_OFFSET                 0x14c0
#define COREMAP_OFFSET               0x1c00

#define EXTIOI_NODETYPE_START        (0x14a0 - APIC_BASE)
#define EXTIOI_NODETYPE_END          (0x14c0 - APIC_BASE)
#define EXTIOI_BOUNCE_START          (0x1680 - APIC_BASE)
#define EXTIOI_BOUNCE_END            (0x16a0 - APIC_BASE)
#define EXTIOI_COREISR_START         (0x1800 - APIC_BASE)
#define EXTIOI_COREISR_END           (0x1B20 - APIC_BASE)

#define EXTIOI_ENABLE_END            (0x1618 - ENABLE_OFFSET)
#define EXTIOI_IPMAP_END             (0x14c8 - IPMAP_OFFSET)
#define EXTIOI_COREMAP_END           (0x1d00 - COREMAP_OFFSET)

#define TYPE_LOONGARCH_EXTIOI "loongarch.extioi"
DECLARE_INSTANCE_CHECKER(struct LoongArchExtIOI, LOONGARCH_EXTIOI,
                         TYPE_LOONGARCH_EXTIOI)

typedef struct ext_sw_ipisr {
    uint8_t irq[EXTIOI_IRQS];
} ext_sw_ipisr;

typedef struct LoongArchExtIOI {
    SysBusDevice parent_obj;
    /* hardware state */
    uint32_t nodetype[EXTIOI_IRQS_NODETYPE_COUNT / 2];
    uint32_t bounce[EXTIOI_IRQS_GROUP_COUNT];
    uint32_t coreisr[MAX_CORES][EXTIOI_IRQS_GROUP_COUNT];

    uint8_t enable[EXTIOI_IRQS / 8];
    uint8_t ipmap[8];
    uint8_t coremap[EXTIOI_IRQS];
    /*software state */
    uint8_t sw_ipmap[EXTIOI_IRQS];
    uint8_t sw_coremap[EXTIOI_IRQS];
    ext_sw_ipisr sw_ipisr[MAX_CORES][LS3A_INTC_IP];

    qemu_irq parent_irq[MAX_CORES][LS3A_INTC_IP];
    qemu_irq irq[EXTIOI_IRQS];
    MemoryRegion mmio_reg32[MAX_CORES];
    MemoryRegion mmio_enable[MAX_CORES];
    MemoryRegion mmio_ipmap[MAX_CORES];
    MemoryRegion mmio_coremap[MAX_CORES];
} LoongArchExtIOI;

#endif /* LOONGARCH_EXTIOI_H */
