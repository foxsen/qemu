/*
 * Copyright (C) 2016 Lemote, Inc.
 * Author: Zhang Fuxin<2503799872@qq.com>
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LOONGSON_H
#define __LOONGSON_H

#define PA_BITS 48

#define LOONGSON_REG(seg, off) (0x9000000000000000ULL + (seg) + (off))
#define LOONGSON_PCI_REG(x) LOONGSON_REG(LOONGSON_PCICONFIGBASE + (x))

#define KSEG0_BASE             0xa0000000
#define UNCACHED_BASE64        0x9000000000000000ULL

#define LM1507_IO_REG_BASE     0xb8000000
#define LM1507_SMBUS_REG_BASE     0xb800eee0

#define LM1507_MISC_IO_REG_BASE    0xbfe00000

#define LM1507_SPI_REG_BASE    0xbfe00220

#define LM1507_UART0_REG_BASE  0xbfe001e0

#define LM1507_MC0_REG_BASE    0xaff00000
#define LM1507_MC1_REG_BASE    0xaff00000

#define LM1507_MSI_PORT_REG_BASE 0xbfd01000

/* RTC regs */
#define LM1507_RTC_REG_BASE				    (LM1507_IO_REG_BASE + 0x00ef8000)
#define	LM1507_TOY_TRIM_REG				    (LM1507_RTC_REG_BASE + 0x0020)
#define	LM1507_TOY_WRITE0_REG				(LM1507_RTC_REG_BASE + 0x0024)
#define	LM1507_TOY_WRITE1_REG				(LM1507_RTC_REG_BASE + 0x0028)
#define	LM1507_TOY_READ0_REG				(LM1507_RTC_REG_BASE + 0x002c)
#define	LM1507_TOY_READ1_REG				(LM1507_RTC_REG_BASE + 0x0030)
#define	LM1507_TOY_MATCH0_REG				(LM1507_RTC_REG_BASE + 0x0034)
#define	LM1507_TOY_MATCH1_REG				(LM1507_RTC_REG_BASE + 0x0038)
#define	LM1507_TOY_MATCH2_REG				(LM1507_RTC_REG_BASE + 0x003c)
#define	LM1507_RTC_CTRL_REG				    (LM1507_RTC_REG_BASE + 0x0040)
#define	LM1507_RTC_TRIM_REG				    (LM1507_RTC_REG_BASE + 0x0060)
#define	LM1507_RTC_WRITE0_REG				(LM1507_RTC_REG_BASE + 0x0064)
#define	LM1507_RTC_READ0_REG				(LM1507_RTC_REG_BASE + 0x0068)
#define	LM1507_RTC_MATCH0_REG				(LM1507_RTC_REG_BASE + 0x006c)
#define	LM1507_RTC_MATCH1_REG				(LM1507_RTC_REG_BASE + 0x0070)
#define	LM1507_RTC_MATCH2_REG				(LM1507_RTC_REG_BASE + 0x0074)

/* LPC regs */
#define LS2H_LPC_IO_BASE				(LS2H_IO_REG_BASE + 0x00f00000)
#define LS2H_LPC_REG_BASE				(LS2H_IO_REG_BASE + 0x00f10000)
#define LS2H_LPC_CFG0_REG				(LS2H_LPC_REG_BASE + 0x0)
#define LS2H_LPC_CFG1_REG				(LS2H_LPC_REG_BASE + 0x4)
#define LS2H_LPC_CFG2_REG				(LS2H_LPC_REG_BASE + 0x8)
#define LS2H_LPC_CFG3_REG				(LS2H_LPC_REG_BASE + 0xc)

#define LM1507_CHIP_CFG_REG_BASE   LOONGSON_REG(0x3ff00000, 0)

#define LM1507_HT_CTLCONF_REG_BASE   LOONGSON_REG(0xefdfb000000, 0)
#define LM1507_HT_IO_REG_BASE        LOONGSON_REG(0xefdfc000000, 0)
#define LM1507_HT_IO_REG_BASE32      LM1507_IO_REG_BASE
#define LM1507_HT_BUSCONF_REG_BASE   LOONGSON_REG(0xefdfe000000, 0)
#define LM1507_HT_BUSCONF_REG_BASE32 0xba000000

#define LM1507_HT_MEM_REG_BASE   LOONGSON_REG(0xe0000000000, 0)
#define LM1507_ISA_MEM_REG_BASE  0xb0000000

#define LOONGSON_FLASH_BASE	0x1c000000
#define LOONGSON_FLASH_SIZE	0x02000000	/* 32M */
#define LOONGSON_FLASH_TOP	(LOONGSON_FLASH_BASE+LOONGSON_FLASH_SIZE-1)

#define LOONGSON_LIO0_BASE	0x1e000000
#define LOONGSON_LIO0_SIZE	0x01C00000	/* 28M */
#define LOONGSON_LIO0_TOP	(LOONGSON_LIO0_BASE+LOONGSON_LIO0_SIZE-1)

#define LOONGSON_BOOT_BASE	0x1fc00000
#define LOONGSON_BOOT_SIZE	0x00100000	/* 1M */
#define LOONGSON_BOOT_TOP	(LOONGSON_BOOT_BASE+LOONGSON_BOOT_SIZE-1)
#define LOONGSON_REG_BASE	0x1fe00000
#define LOONGSON_REG_SIZE	0x00100000	/* 256Bytes + 256Bytes + ??? */
#define LOONGSON_REG_TOP	(LOONGSON_REG_BASE+LOONGSON_REG_SIZE-1)
#define LOONGSON3_REG_BASE	0x3ff00000
#define LOONGSON3_REG_SIZE 	0x00100000	/* 256Bytes + 256Bytes + ??? */
#define LOONGSON3_REG_TOP	(LOONGSON3_REG_BASE+LOONGSON3_REG_SIZE-1)

#define LOONGSON_LIO1_BASE	0x1ff00000
#define LOONGSON_LIO1_SIZE	0x00100000	/* 1M */
#define LOONGSON_LIO1_TOP	(LOONGSON_LIO1_BASE+LOONGSON_LIO1_SIZE-1)

#define LOONGSON_PCILO0_BASE	0x10000000
#define LOONGSON_PCILO1_BASE	0x14000000
#define LOONGSON_PCILO2_BASE	0x18000000
#define LOONGSON_PCILO_BASE	LOONGSON_PCILO0_BASE
#define LOONGSON_PCILO_SIZE	0x0c000000	/* 64M * 3 */
#define LOONGSON_PCILO_TOP	(LOONGSON_PCILO0_BASE+LOONGSON_PCILO_SIZE-1)

#define LOONGSON_PCICFG_BASE	0x1fe80000
#define LOONGSON_PCICFG_SIZE	0x00000800	/* 2K */
#define LOONGSON_PCICFG_TOP	(LOONGSON_PCICFG_BASE+LOONGSON_PCICFG_SIZE-1)

#if defined(CONFIG_HT_PCI)
#define LOONGSON_PCIIO_BASE	loongson_pciio_base
#else
#define LOONGSON_PCIIO_BASE	0x1fd00000
#endif

#define LOONGSON_PCIIO_SIZE	0x00100000	/* 1M */
#define LOONGSON_PCIIO_TOP	(LOONGSON_PCIIO_BASE+LOONGSON_PCIIO_SIZE-1)

/* Loongson Register Bases */

#define LOONGSON_PCICONFIGBASE	0x00
#define LOONGSON_REGBASE	0x100

/* PCI Configuration Registers */

#define LOONGSON_PCI_REG(x)	LOONGSON_REG(LOONGSON_PCICONFIGBASE + (x))
#define LOONGSON_PCIDID		LOONGSON_PCI_REG(0x00)
#define LOONGSON_PCICMD		LOONGSON_PCI_REG(0x04)
#define LOONGSON_PCICLASS	LOONGSON_PCI_REG(0x08)
#define LOONGSON_PCILTIMER	LOONGSON_PCI_REG(0x0c)
#define LOONGSON_PCIBASE0	LOONGSON_PCI_REG(0x10)
#define LOONGSON_PCIBASE1	LOONGSON_PCI_REG(0x14)
#define LOONGSON_PCIBASE2	LOONGSON_PCI_REG(0x18)
#define LOONGSON_PCIBASE3	LOONGSON_PCI_REG(0x1c)
#define LOONGSON_PCIBASE4	LOONGSON_PCI_REG(0x20)
#define LOONGSON_PCIEXPRBASE	LOONGSON_PCI_REG(0x30)
#define LOONGSON_PCIINT		LOONGSON_PCI_REG(0x3c)

#define LOONGSON_PCI_ISR4C	LOONGSON_PCI_REG(0x4c)

#define LOONGSON_PCICMD_PERR_CLR	0x80000000
#define LOONGSON_PCICMD_SERR_CLR	0x40000000
#define LOONGSON_PCICMD_MABORT_CLR	0x20000000
#define LOONGSON_PCICMD_MTABORT_CLR	0x10000000
#define LOONGSON_PCICMD_TABORT_CLR	0x08000000
#define LOONGSON_PCICMD_MPERR_CLR	0x01000000
#define LOONGSON_PCICMD_PERRRESPEN	0x00000040
#define LOONGSON_PCICMD_ASTEPEN		0x00000080
#define LOONGSON_PCICMD_SERREN		0x00000100
#define LOONGSON_PCILTIMER_BUSLATENCY	0x0000ff00
#define LOONGSON_PCILTIMER_BUSLATENCY_SHIFT	8

/* Loongson h/w Configuration */

#define LOONGSON_GENCFG_OFFSET		0x4
#define LOONGSON_GENCFG LOONGSON_REG(LOONGSON_REGBASE + LOONGSON_GENCFG_OFFSET)

#define LOONGSON_GENCFG_DEBUGMODE	0x00000001
#define LOONGSON_GENCFG_SNOOPEN		0x00000002
#define LOONGSON_GENCFG_CPUSELFRESET	0x00000004

#define LOONGSON_GENCFG_FORCE_IRQA	0x00000008
#define LOONGSON_GENCFG_IRQA_ISOUT	0x00000010
#define LOONGSON_GENCFG_IRQA_FROM_INT1	0x00000020
#define LOONGSON_GENCFG_BYTESWAP	0x00000040

#define LOONGSON_GENCFG_UNCACHED	0x00000080
#define LOONGSON_GENCFG_PREFETCHEN	0x00000100
#define LOONGSON_GENCFG_WBEHINDEN	0x00000200
#define LOONGSON_GENCFG_CACHEALG	0x00000c00
#define LOONGSON_GENCFG_CACHEALG_SHIFT	10
#define LOONGSON_GENCFG_PCIQUEUE	0x00001000
#define LOONGSON_GENCFG_CACHESTOP	0x00002000
#define LOONGSON_GENCFG_MSTRBYTESWAP	0x00004000
#define LOONGSON_GENCFG_BUSERREN	0x00008000
#define LOONGSON_GENCFG_NORETRYTIMEOUT	0x00010000
#define LOONGSON_GENCFG_SHORTCOPYTIMEOUT	0x00020000

/* PCI address map control */

#define LOONGSON_PCIMAP			LOONGSON_REG(LOONGSON_REGBASE + 0x10)
#define LOONGSON_PCIMEMBASECFG		LOONGSON_REG(LOONGSON_REGBASE + 0x14)
#define LOONGSON_PCIMAP_CFG		LOONGSON_REG(LOONGSON_REGBASE + 0x18)

/* GPIO Regs - r/w */

#define LOONGSON_GPIODATA		LOONGSON_REG(LOONGSON_REGBASE + 0x1c)
#define LOONGSON_GPIOIE			LOONGSON_REG(LOONGSON_REGBASE + 0x20)

/* ICU Configuration Regs - r/w */

#define LOONGSON_INTEDGE		LOONGSON_REG(LOONGSON_REGBASE + 0x24)
#define LOONGSON_INTSTEER		LOONGSON_REG(LOONGSON_REGBASE + 0x28)
#define LOONGSON_INTPOL			LOONGSON_REG(LOONGSON_REGBASE + 0x2c)

/* ICU Enable Regs - IntEn & IntISR are r/o. */

#define LOONGSON_INTENSET		LOONGSON_REG(LOONGSON_REGBASE + 0x30)
#define LOONGSON_INTENCLR		LOONGSON_REG(LOONGSON_REGBASE + 0x34)
#define LOONGSON_INTEN			LOONGSON_REG(LOONGSON_REGBASE + 0x38)
#define LOONGSON_INTISR			LOONGSON_REG(LOONGSON_REGBASE + 0x3c)

/* ICU */
#define LOONGSON_ICU_MBOXES		0x0000000f
#define LOONGSON_ICU_MBOXES_SHIFT	0
#define LOONGSON_ICU_DMARDY		0x00000010
#define LOONGSON_ICU_DMAEMPTY		0x00000020
#define LOONGSON_ICU_COPYRDY		0x00000040
#define LOONGSON_ICU_COPYEMPTY		0x00000080
#define LOONGSON_ICU_COPYERR		0x00000100
#define LOONGSON_ICU_PCIIRQ		0x00000200
#define LOONGSON_ICU_MASTERERR		0x00000400
#define LOONGSON_ICU_SYSTEMERR		0x00000800
#define LOONGSON_ICU_DRAMPERR		0x00001000
#define LOONGSON_ICU_RETRYERR		0x00002000
#define LOONGSON_ICU_GPIOS		0x01ff0000
#define LOONGSON_ICU_GPIOS_SHIFT		16
#define LOONGSON_ICU_GPINS		0x7e000000
#define LOONGSON_ICU_GPINS_SHIFT		25
#define LOONGSON_ICU_MBOX(N)		(1<<(LOONGSON_ICU_MBOXES_SHIFT+(N)))
#define LOONGSON_ICU_GPIO(N)		(1<<(LOONGSON_ICU_GPIOS_SHIFT+(N)))
#define LOONGSON_ICU_GPIN(N)		(1<<(LOONGSON_ICU_GPINS_SHIFT+(N)))

/* PCI prefetch window base & mask */

#define LOONGSON_MEM_WIN_BASE_L		LOONGSON_REG(LOONGSON_REGBASE + 0x40)
#define LOONGSON_MEM_WIN_BASE_H		LOONGSON_REG(LOONGSON_REGBASE + 0x44)
#define LOONGSON_MEM_WIN_MASK_L		LOONGSON_REG(LOONGSON_REGBASE + 0x48)
#define LOONGSON_MEM_WIN_MASK_H		LOONGSON_REG(LOONGSON_REGBASE + 0x4c)

/* PCI_Hit*_Sel_* */

#define LOONGSON_PCI_HIT0_SEL_L		LOONGSON_REG(LOONGSON_REGBASE + 0x50)
#define LOONGSON_PCI_HIT0_SEL_H		LOONGSON_REG(LOONGSON_REGBASE + 0x54)
#define LOONGSON_PCI_HIT1_SEL_L		LOONGSON_REG(LOONGSON_REGBASE + 0x58)
#define LOONGSON_PCI_HIT1_SEL_H		LOONGSON_REG(LOONGSON_REGBASE + 0x5c)
#define LOONGSON_PCI_HIT2_SEL_L		LOONGSON_REG(LOONGSON_REGBASE + 0x60)
#define LOONGSON_PCI_HIT2_SEL_H		LOONGSON_REG(LOONGSON_REGBASE + 0x64)

/* PXArb Config & Status */

#define LOONGSON_PXARB_CFG		LOONGSON_REG(LOONGSON_REGBASE + 0x68)
#define LOONGSON_PXARB_STATUS		LOONGSON_REG(LOONGSON_REGBASE + 0x6c)

#define MAX_PACKAGES 4

/* Chip Config registor of each physical cpu package, for Loongson-2F and successor */
extern unsigned long loongson_chipcfg[MAX_PACKAGES];
#define LOONGSON_CHIPCFG(id) (*(volatile u32 *)(loongson_chipcfg[id]))

/* Chip Temperature registor of each physical cpu package, for Loongson-3A and successor */
extern unsigned long loongson_chiptemp[MAX_PACKAGES];
#define LOONGSON_CHIPTEMP(id) (*(volatile u32 *)(loongson_chiptemp[id]))

/* Freq Control register of each physical cpu package, for Loongson-3B and successor */
extern unsigned long loongson_freqctrl[MAX_PACKAGES];
#define LOONGSON_FREQCTRL(id) (*(volatile u32 *)(loongson_freqctrl[id]))

/* pcimap */

#define LOONGSON_PCIMAP_PCIMAP_LO0	0x0000003f
#define LOONGSON_PCIMAP_PCIMAP_LO0_SHIFT	0
#define LOONGSON_PCIMAP_PCIMAP_LO1	0x00000fc0
#define LOONGSON_PCIMAP_PCIMAP_LO1_SHIFT	6
#define LOONGSON_PCIMAP_PCIMAP_LO2	0x0003f000
#define LOONGSON_PCIMAP_PCIMAP_LO2_SHIFT	12
#define LOONGSON_PCIMAP_PCIMAP_2	0x00040000
#define LOONGSON_PCIMAP_WIN(WIN, ADDR)	\
	((((ADDR)>>26) & LOONGSON_PCIMAP_PCIMAP_LO0) << ((WIN)*6))

/*
 * address windows configuration module
 *
 * loongson2e do not have this module
 */

/* address window config module base address */
#define LOONGSON_ADDRWINCFG_BASE		0x900000003ff00000ul
#define LOONGSON_ADDRWINCFG_SIZE		0x180

#define LOONGSON_ADDRWINCFG(offset) \
	(LOONGSON_ADDRWINCFG_BASE + (offset))

#define CPU_WIN0_BASE	LOONGSON_ADDRWINCFG(0x00)
#define CPU_WIN1_BASE	LOONGSON_ADDRWINCFG(0x08)
#define CPU_WIN2_BASE	LOONGSON_ADDRWINCFG(0x10)
#define CPU_WIN3_BASE	LOONGSON_ADDRWINCFG(0x18)

#define CPU_WIN0_MASK	LOONGSON_ADDRWINCFG(0x20)
#define CPU_WIN1_MASK	LOONGSON_ADDRWINCFG(0x28)
#define CPU_WIN2_MASK	LOONGSON_ADDRWINCFG(0x30)
#define CPU_WIN3_MASK	LOONGSON_ADDRWINCFG(0x38)

#define CPU_WIN0_MMAP	LOONGSON_ADDRWINCFG(0x40)
#define CPU_WIN1_MMAP	LOONGSON_ADDRWINCFG(0x48)
#define CPU_WIN2_MMAP	LOONGSON_ADDRWINCFG(0x50)
#define CPU_WIN3_MMAP	LOONGSON_ADDRWINCFG(0x58)

#define PCIDMA_WIN0_BASE	LOONGSON_ADDRWINCFG(0x60)
#define PCIDMA_WIN1_BASE	LOONGSON_ADDRWINCFG(0x68)
#define PCIDMA_WIN2_BASE	LOONGSON_ADDRWINCFG(0x70)
#define PCIDMA_WIN3_BASE	LOONGSON_ADDRWINCFG(0x78)

#define PCIDMA_WIN0_MASK	LOONGSON_ADDRWINCFG(0x80)
#define PCIDMA_WIN1_MASK	LOONGSON_ADDRWINCFG(0x88)
#define PCIDMA_WIN2_MASK	LOONGSON_ADDRWINCFG(0x90)
#define PCIDMA_WIN3_MASK	LOONGSON_ADDRWINCFG(0x98)

#define PCIDMA_WIN0_MMAP	LOONGSON_ADDRWINCFG(0xa0)
#define PCIDMA_WIN1_MMAP	LOONGSON_ADDRWINCFG(0xa8)
#define PCIDMA_WIN2_MMAP	LOONGSON_ADDRWINCFG(0xb0)
#define PCIDMA_WIN3_MMAP	LOONGSON_ADDRWINCFG(0xb8)

#define ADDRWIN_WIN0	0
#define ADDRWIN_WIN1	1
#define ADDRWIN_WIN2	2
#define ADDRWIN_WIN3	3

#define ADDRWIN_MAP_DST_DDR	0
#define ADDRWIN_MAP_DST_PCI	1
#define ADDRWIN_MAP_DST_LIO	1

/*
 * s: CPU, PCIDMA
 * d: DDR, PCI, LIO
 * win: 0, 1, 2, 3
 * src: map source
 * dst: map destination
 * size: ~mask + 1
 */
#define LOONGSON_ADDRWIN_CFG(s, d, w, src, dst, size) do {\
	s##_WIN##w##_BASE = (src); \
	s##_WIN##w##_MMAP = (dst) | ADDRWIN_MAP_DST_##d; \
	s##_WIN##w##_MASK = ~(size-1); \
} while (0)

#define LOONGSON_ADDRWIN_CPUTOPCI(win, src, dst, size) \
	LOONGSON_ADDRWIN_CFG(CPU, PCI, win, src, dst, size)
#define LOONGSON_ADDRWIN_CPUTODDR(win, src, dst, size) \
	LOONGSON_ADDRWIN_CFG(CPU, DDR, win, src, dst, size)
#define LOONGSON_ADDRWIN_PCITODDR(win, src, dst, size) \
	LOONGSON_ADDRWIN_CFG(PCIDMA, DDR, win, src, dst, size)

#define MAX_CPUS 4

#define SLOCK0_ADDR            0x200
#define SLOCK0_MASK            0x240
#define SLOCK1_ADDR            0x208
#define SLOCK1_MASK            0x248

#define CORE0_STATUS_OFF       0x000
#define CORE0_EN_OFF           0x004
#define CORE0_SET_OFF          0x008
#define CORE0_CLEAR_OFF        0x00c
#define CORE0_BUF_20           0x020
#define CORE0_BUF_28           0x028
#define CORE0_BUF_30           0x030
#define CORE0_BUF_38           0x038

#define CORE1_STATUS_OFF       0x100
#define CORE1_EN_OFF           0x104
#define CORE1_SET_OFF          0x108
#define CORE1_CLEAR_OFF        0x10c
#define CORE1_BUF_20           0x120
#define CORE1_BUF_28           0x128
#define CORE1_BUF_30           0x130
#define CORE1_BUF_38           0x138

#define CORE2_STATUS_OFF       0x200
#define CORE2_EN_OFF           0x204
#define CORE2_SET_OFF          0x208
#define CORE2_CLEAR_OFF        0x20c
#define CORE2_BUF_20           0x220
#define CORE2_BUF_28           0x228
#define CORE2_BUF_30           0x230
#define CORE2_BUF_38           0x238

#define CORE3_STATUS_OFF       0x300
#define CORE3_EN_OFF           0x304
#define CORE3_SET_OFF          0x308
#define CORE3_CLEAR_OFF        0x30c
#define CORE3_BUF_20           0x320
#define CORE3_BUF_28           0x328
#define CORE3_BUF_30           0x330
#define CORE3_BUF_38           0x338

/* Inter-processor interrupts */
typedef struct gipi_single {
    uint32_t status;
    uint32_t en;
    uint32_t set;
    uint32_t clear;
    uint32_t buf[8];
    qemu_irq irq;
} gipi_single;

typedef struct gintcState  gintcState;
struct gintcState {
	gipi_single core[4];

    /* the following must be aligned with physical regs */
    uint32_t route[32/4];
    uint32_t status;
    uint32_t en;
    uint32_t set;
    uint32_t clear;
    uint32_t pol;
    uint32_t edge;

    uint32_t pad[2];

    uint32_t core0_intisr;
    uint32_t pad0;
    uint32_t core1_intisr;
    uint32_t pad1;
    uint32_t core2_intisr;
    uint32_t pad2;
    uint32_t core3_intisr;
    uint32_t pad3;
};

#define MAX_PILS 16

#define INT_ROUTER_REGS_BASE   0x3ff01400

#define INT_ROUTER_REGS_SYS_INT0	0x00
#define INT_ROUTER_REGS_SYS_INT1	0x01
#define INT_ROUTER_REGS_SYS_INT2	0x02
#define INT_ROUTER_REGS_SYS_INT3	0x03
#define INT_ROUTER_REGS_PCI_INT0	0x04
#define INT_ROUTER_REGS_PCI_INT1	0x05
#define INT_ROUTER_REGS_PCI_INT2	0x06
#define INT_ROUTER_REGS_PCI_INT3	0x07
#define INT_ROUTER_REGS_MATRIX_INT0	0x08
#define INT_ROUTER_REGS_MATRIX_INT1	0x09
#define INT_ROUTER_REGS_LPC_INT		0x0a
#define INT_ROUTER_REGS_MC0		0x0B
#define INT_ROUTER_REGS_MC1		0x0C
#define INT_ROUTER_REGS_BARRIER		0x0d
#define INT_ROUTER_REGS_RESERVE		0x0e
#define INT_ROUTER_REGS_PCI_PERR	0x0f

#define INT_ROUTER_REGS_HT0_INT0	0x10
#define INT_ROUTER_REGS_HT0_INT1	0x11
#define INT_ROUTER_REGS_HT0_INT2	0x12
#define INT_ROUTER_REGS_HT0_INT3	0x13
#define INT_ROUTER_REGS_HT0_INT4	0x14
#define INT_ROUTER_REGS_HT0_INT5	0x15
#define INT_ROUTER_REGS_HT0_INT6	0x16
#define INT_ROUTER_REGS_HT0_INT7	0x17
#define INT_ROUTER_REGS_HT1_INT0	0x18
#define INT_ROUTER_REGS_HT1_INT1	0x19
#define INT_ROUTER_REGS_HT1_INT2	0x1a
#define INT_ROUTER_REGS_HT1_INT3	0x1b
#define INT_ROUTER_REGS_HT1_INT4	0x1c
#define INT_ROUTER_REGS_HT1_INT5	0x1d
#define INT_ROUTER_REGS_HT1_INT6	0x1e
#define INT_ROUTER_REGS_HT1_INT7	0x1f
#define IO_CONTROL_REGS_INTISR  	0x20
#define IO_CONTROL_REGS_INTEN		0x24	
#define IO_CONTROL_REGS_INTENSET	0x28	
#define IO_CONTROL_REGS_INTENCLR	0x2c	
#define IO_CONTROL_REGS_INTEDGE		0x38	
#define IO_CONTROL_REGS_CORE0_INTISR	0x40	
#define IO_CONTROL_REGS_CORE1_INTISR	0x48	
#define IO_CONTROL_REGS_CORE2_INTISR	0x50	
#define IO_CONTROL_REGS_CORE3_INTISR	0x58	

#define HT_LINK_CONFIG_REG  0x44
#define HT_IRQ_VECTOR_REG0	0x80	
#define HT_IRQ_VECTOR_REG1	0x84	
#define HT_IRQ_VECTOR_REG2	0x88	
#define HT_IRQ_VECTOR_REG3	0x8C	
#define HT_IRQ_VECTOR_REG4	0x90	
#define HT_IRQ_VECTOR_REG5	0x94	
#define HT_IRQ_VECTOR_REG6	0x98	
#define HT_IRQ_VECTOR_REG7	0x9C	

#define HT_IRQ_ENABLE_REG0	0xA0	
#define HT_IRQ_ENABLE_REG1	0xA4	
#define HT_IRQ_ENABLE_REG2	0xA8	
#define HT_IRQ_ENABLE_REG3	0xAC	
#define HT_IRQ_ENABLE_REG4	0xB0	
#define HT_IRQ_ENABLE_REG5	0xB4	
#define HT_IRQ_ENABLE_REG6	0xB8	
#define HT_IRQ_ENABLE_REG7	0xBC	

#define HT_UNCACHE_ENABLE_REG0	0xF0
#define HT_UNCACHE_BASE_REG0	0xF4
#define HT_UNCACHE_ENABLE_REG1	0xF8
#define HT_UNCACHE_BASE_REG1	0xFC

typedef struct LM1507State {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/
    
    CPUMIPSState *mycpu[4];

    gintcState intc;

	unsigned int ht_ctlconf_reg[0x120/4];

    int memory_initialized;
    MemoryRegion mc0_mem;
    MemoryRegion mc0_io;
    MemoryRegion mc1_mem;
    MemoryRegion mc1_io;
    ram_addr_t ram_size;

    uint64_t scache0_addr, scache0_mask;
    MemoryRegion scache0_ram;

    uint64_t scache1_addr, scache1_mask;
    MemoryRegion scache1_ram;

    MemoryRegion ram;
    MemoryRegion ram_hi;
    MemoryRegion ram_hi2;
    MemoryRegion flash;
    MemoryRegion bios;
    MemoryRegion creg_io;
    MemoryRegion misc_io;
    MemoryRegion gpu_io;
    MemoryRegion nand_io;

    MemoryRegion sata_io;
    MemoryRegion acpi_io;

    MemoryRegion ht1_lo;
    AddressSpace ht1_lo_as;

    MemoryRegion ht1_lo_io;
    MemoryRegion ht1_lo_io_alias;
    MemoryRegion ht1_lo_io_alias32;
    MemoryRegion ht1_lo_mem;
    MemoryRegion ht1_lo_mem_alias;
    MemoryRegion ht1_lo_ctlconf;
    MemoryRegion ht1_lo_ctlconf_alias;
    MemoryRegion ht1_lo_busconf;
    MemoryRegion ht1_lo_busconf_alias;
    MemoryRegion ht1_lo_busconf_alias32;

    MemoryRegion pci_mem;
    AddressSpace pci_mem_as;
    MemoryRegion pci_mem_alias;
    MemoryRegion *pci_io_p;
    MemoryRegion pci_io_alias;

    MemoryRegion isa_mem;
    MemoryRegion smbus_io;

    DeviceState *netdev0;
    DeviceState *netdev1;
    DeviceState *spidev;
    DeviceState *i2c0_dev, *i2c1_dev;
    DeviceState *lpc_dev;

    ISABus      *isa_bus;
    PCIBus      *pci_bus;
    qemu_irq    *pci_irq;

} LM1507State;

#endif /* __LOONGSON_H */
