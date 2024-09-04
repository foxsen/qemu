/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * QEMU LoongArch CPU
 *
 * Copyright (c) 2021 Loongson Technology Corporation Limited
 */

#ifndef HW_LS7A_H
#define HW_LS7A_H

#include "hw/pci-host/pam.h"
#include "qemu/units.h"
#include "qemu/range.h"
#include "qom/object.h"

#define LS2K_PCI_MEM_BASE        0x40000000UL
#define LS2K_PCI_MEM_SIZE        0x40000000UL
#define LS2K_PCI_IO_OFFSET       0x4000
#define LS2K_PCI_CFG_BASE        0xFE00000000UL
#define LS2K_PCI_CFG_SIZE        0x10000000
#define LS2K_PCI_IO_BASE         0x18004000UL
#define LS2K_PCI_IO_SIZE         0xC000

#define LS2K_PCH_REG_BASE        0x18000000UL
#define LS2K_PCH_MSI_ADDR_LOW    0x1FE014B0UL

/*
 * GSI_BASE is hard-coded with 64 in linux kernel, else kernel fails to boot
 * 0  - 15  GSI for ISA devices even if there is no ISA devices
 * 16 - 63  GSI for CPU devices such as timers/perf monitor etc
 * 64 -     GSI for external devices
 */
#define LS2K_PCH_PIC_IRQ_NUM     64
#define LS2K_GSI_BASE            64
#define LS2K_DEVICE_IRQS         16
#define LS2K_UART_IRQ            (LS2K_GSI_BASE + 0)
#define LS2K_UART_BASE           0x1fe001e0
#define LS2K_UART_SIZE           0X100
#define LS2K_RTC_IRQ             (LS2K_GSI_BASE + 3)
#define LS2K_MISC_REG_BASE       (LS2K_PCH_REG_BASE + 0x00080000)
#define LS2K_RTC_REG_BASE        (LS2K_MISC_REG_BASE + 0x00050100)
#define LS2K_RTC_LEN             0x100
#define LS2K_SCI_IRQ             (LS2K_GSI_BASE + 4)

#define LS2K_PLATFORM_BUS_BASEADDRESS   0x16000000
#define LS2K_PLATFORM_BUS_SIZE          0x2000000
#define LS2K_PLATFORM_BUS_NUM_IRQS      2
#define LS2K_PLATFORM_BUS_IRQ           (LS2K_GSI_BASE + 5)
#endif
