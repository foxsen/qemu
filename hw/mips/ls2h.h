#ifndef _LS2H_H_
#define _LS2H_H_

#define LS2H_IO_REG_BASE				0xbf000000
#define KSEG0_BASE					0xa0000000
#define PA_BITS                     48

/* Memory controller regs */
#define LS2H_MC_REG_BASE            0xaff00000

/* CHIP CONFIG regs */
#define LS2H_CHIP_CFG_REG_BASE				(LS2H_IO_REG_BASE + 0x00d00000)
#define LS2H_MSI_PORT_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0)
#define LS2H_INTC_SIZE                  0xc0

#define LS2H_INT_REG_BASE				(LS2H_CHIP_CFG_REG_BASE + 0x0040)

#define LS2H_INT_ISR0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0040)
#define LS2H_INT_IEN0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0044)
#define LS2H_INT_SET0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0048)
#define LS2H_INT_CLR0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x004c)
#define LS2H_INT_POL0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0050)
#define LS2H_INT_EDGE0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0054)

#define LS2H_GPIO_CFG_REG				(LS2H_CHIP_CFG_REG_BASE + 0x00c0)
#define LS2H_GPIO_OUT_EN_REG				(LS2H_CHIP_CFG_REG_BASE + 0x00c4)
#define LS2H_GPIO_IN_REG				(LS2H_CHIP_CFG_REG_BASE + 0x00c8)
#define LS2H_GPIO_OUT_REG				(LS2H_CHIP_CFG_REG_BASE + 0x00cc)

#define LS2H_DMA_ORDER_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0100)
#define LS2H_CHIP_CFG0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0200)
#define LS2H_CHIP_CFG1_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0204)
#define LS2H_CHIP_CFG2_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0208)
#define LS2H_CHIP_CFG3_REG				(LS2H_CHIP_CFG_REG_BASE + 0x020c)
#define LS2H_CHIP_SAMP0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0210)
#define LS2H_CHIP_SAMP1_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0214)
#define LS2H_CHIP_SAMP2_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0218)
#define LS2H_CHIP_SAMP3_REG				(LS2H_CHIP_CFG_REG_BASE + 0x021c)
#define LS2H_CHIP_SAMP4_REG				(LS2H_CHIP_CFG_REG_BASE + 0x00d8)
#define LS2H_CHIP_SAMP5_REG				(LS2H_CHIP_CFG_REG_BASE + 0x00e8)

#define LS2H_CLK_CTRL0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0220)
#define LS2H_CLK_CTRL1_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0224)
#define LS2H_CLK_CTRL2_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0228)
#define LS2H_CLK_CTRL3_REG				(LS2H_CHIP_CFG_REG_BASE + 0x022c)

#define LS2H_PIXCLK0_CTRL0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0230)
#define LS2H_PIXCLK0_CTRL1_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0234)
#define LS2H_PIXCLK1_CTRL0_REG				(LS2H_CHIP_CFG_REG_BASE + 0x0238)
#define LS2H_PIXCLK1_CTRL1_REG				(LS2H_CHIP_CFG_REG_BASE + 0x023c)

#define LS2H_CLOCK_CTRL0_REG                            (LS2H_CHIP_CFG_REG_BASE + 0x0220)
#define LS2H_CLOCK_CTRL1_REG                            (LS2H_CHIP_CFG_REG_BASE + 0x0224)
#define LS2H_CLOCK_CTRL2_REG                            (LS2H_CHIP_CFG_REG_BASE + 0x0228)
#define LS2H_CLOCK_CTRL3_REG                            (LS2H_CHIP_CFG_REG_BASE + 0x022c)

#define LS2H_WIN_CFG_BASE				(LS2H_CHIP_CFG_REG_BASE + 0x80000)
#define LS2H_M4_WIN0_BASE_REG				(LS2H_WIN_CFG_BASE + 0x0400)
#define LS2H_M4_WIN0_MASK_REG				(LS2H_WIN_CFG_BASE + 0x0440)
#define LS2H_M4_WIN0_MMAP_REG				(LS2H_WIN_CFG_BASE + 0x0480)
#define LS2H_QOS_CFG6_REG				(LS2H_WIN_CFG_BASE + 0x0630)

/* USB regs */
#define LS2H_EHCI_REG_BASE				(LS2H_IO_REG_BASE + 0x00e00000)
#define LS2H_OHCI_REG_BASE				(LS2H_IO_REG_BASE + 0x00e08000)

/* GMAC regs */
#define LS2H_GMAC0_REG_BASE				(LS2H_IO_REG_BASE + 0x00e10000)
#define LS2H_GMAC1_REG_BASE				(LS2H_IO_REG_BASE + 0x00e18000)

/* HDA regs */
#define LS2H_HDA_REG_BASE				(LS2H_IO_REG_BASE + 0x00e20000)

/* SATAregs */
#define LS2H_SATA_REG_BASE				(LS2H_IO_REG_BASE + 0x00e30000)

/* GPU regs */
#define LS2H_GPU_REG_BASE				(LS2H_IO_REG_BASE + 0x00e40000)

/* DC regs */
#define LS2H_DC_REG_BASE				(LS2H_IO_REG_BASE + 0x00e50000)

#define LS2H_FB_CFG_DVO_REG				(LS2H_DC_REG_BASE + 0x1240)
#define LS2H_FB_CFG_VGA_REG				(LS2H_DC_REG_BASE + 0x1250)
#define LS2H_FB_ADDR0_DVO_REG				(LS2H_DC_REG_BASE + 0x1260)
#define LS2H_FB_ADDR0_VGA_REG				(LS2H_DC_REG_BASE + 0x1270)
#define LS2H_FB_ADDR1_DVO_REG				(LS2H_DC_REG_BASE + 0x1580)
#define LS2H_FB_ADDR1_VGA_REG				(LS2H_DC_REG_BASE + 0x1590)
#define LS2H_FB_STRI_DVO_REG				(LS2H_DC_REG_BASE + 0x1280)
#define LS2H_FB_STRI_VGA_REG				(LS2H_DC_REG_BASE + 0x1290)

#define LS2H_FB_CUR_CFG_REG				(LS2H_DC_REG_BASE + 0x1520)
#define LS2H_FB_CUR_ADDR_REG				(LS2H_DC_REG_BASE + 0x1530)
#define LS2H_FB_CUR_LOC_ADDR_REG			(LS2H_DC_REG_BASE + 0x1540)
#define LS2H_FB_CUR_BACK_REG				(LS2H_DC_REG_BASE + 0x1550)
#define LS2H_FB_CUR_FORE_REG				(LS2H_DC_REG_BASE + 0x1560)

#define LS2H_FB_DAC_CTRL_REG				(LS2H_DC_REG_BASE + 0x1600)
/* OTG regs */
#define LS2H_OTG_REG_BASE				(LS2H_IO_REG_BASE + 0x00e60000)

/* SPI regs */
#define LS2H_SPI_REG_BASE				(LS2H_IO_REG_BASE + 0x00e70000)

/* UART regs */
#define LS2H_UART0_REG_BASE				(LS2H_IO_REG_BASE + 0x00e80000)
#define LS2H_UART1_REG_BASE				(LS2H_IO_REG_BASE + 0x00e81000)
#define LS2H_UART2_REG_BASE				(LS2H_IO_REG_BASE + 0x00e82000)
#define LS2H_UART3_REG_BASE				(LS2H_IO_REG_BASE + 0x00e83000)

/* I2C regs */
#define LS2H_I2C0_REG_BASE				(LS2H_IO_REG_BASE + 0x00e90000)
#define LS2H_I2C0_PRER_LO_REG				(LS2H_I2C0_REG_BASE + 0x0)
#define LS2H_I2C0_PRER_HI_REG				(LS2H_I2C0_REG_BASE + 0x1)
#define LS2H_I2C0_CTR_REG   				(LS2H_I2C0_REG_BASE + 0x2)
#define LS2H_I2C0_TXR_REG   				(LS2H_I2C0_REG_BASE + 0x3)
#define LS2H_I2C0_RXR_REG    				(LS2H_I2C0_REG_BASE + 0x3)
#define LS2H_I2C0_CR_REG     				(LS2H_I2C0_REG_BASE + 0x4)
#define LS2H_I2C0_SR_REG     				(LS2H_I2C0_REG_BASE + 0x4)

#define LS2H_I2C1_REG_BASE				(LS2H_IO_REG_BASE + 0x00e91000)
#define LS2H_I2C1_PRER_LO_REG				(LS2H_I2C1_REG_BASE + 0x0)
#define LS2H_I2C1_PRER_HI_REG				(LS2H_I2C1_REG_BASE + 0x1)
#define LS2H_I2C1_CTR_REG    				(LS2H_I2C1_REG_BASE + 0x2)
#define LS2H_I2C1_TXR_REG    				(LS2H_I2C1_REG_BASE + 0x3)
#define LS2H_I2C1_RXR_REG    				(LS2H_I2C1_REG_BASE + 0x3)
#define LS2H_I2C1_CR_REG     				(LS2H_I2C1_REG_BASE + 0x4)
#define LS2H_I2C1_SR_REG     				(LS2H_I2C1_REG_BASE + 0x4)

#define CR_START					0x80
#define CR_STOP						0x40
#define CR_READ						0x20
#define CR_WRITE					0x10
#define CR_ACK						0x8
#define CR_IACK						0x1

#define SR_NOACK					0x80
#define SR_BUSY						0x40
#define SR_AL						0x20
#define SR_TIP						0x2
#define	SR_IF						0x1

/* PWM regs */
#define LS2H_PWM_REG_BASE				(LS2H_IO_REG_BASE + 0x00ea0000)

/* HPET regs */
#define LS2H_HPET_REG_BASE				(LS2H_IO_REG_BASE + 0x00ec0000)

/* AC97 regs */
#define LS2H_AC97_REG_BASE				(LS2H_IO_REG_BASE + 0x00ed0000)

/* NAND regs */
#define LS2H_NAND_REG_BASE				(LS2H_IO_REG_BASE + 0x00ee0000)
#define LS2H_NAND_CMD_REG				(LS2H_NAND_REG_BASE + 0x0000)
#define LS2H_NAND_ADDR_C_REG				(LS2H_NAND_REG_BASE + 0x0004)
#define LS2H_NAND_ADDR_R_REG				(LS2H_NAND_REG_BASE + 0x0008)
#define LS2H_NAND_TIMING_REG				(LS2H_NAND_REG_BASE + 0x000c)
#define LS2H_NAND_IDL_REG				(LS2H_NAND_REG_BASE + 0x0010)
#define LS2H_NAND_STA_IDH_REG				(LS2H_NAND_REG_BASE + 0x0014)
#define LS2H_NAND_PARAM_REG				(LS2H_NAND_REG_BASE + 0x0018)
#define LS2H_NAND_OP_NUM_REG				(LS2H_NAND_REG_BASE + 0x001c)
#define LS2H_NAND_CSRDY_MAP_REG				(LS2H_NAND_REG_BASE + 0x0020)
#define LS2H_NAND_DMA_ACC_REG				(LS2H_NAND_REG_BASE + 0x0040)

/* ACPI regs */
#define LS2H_ACPI_REG_BASE				(LS2H_IO_REG_BASE + 0x00ef0000)
#define LS2H_PM_SOC_REG					(LS2H_ACPI_REG_BASE + 0x0000)
#define LS2H_PM_RESUME_REG				(LS2H_ACPI_REG_BASE + 0x0004)
#define LS2H_PM_RTC_REG					(LS2H_ACPI_REG_BASE + 0x0008)
#define LS2H_PM1_STS_REG				(LS2H_ACPI_REG_BASE + 0x000c)
#define LS2H_PM1_EN_REG					(LS2H_ACPI_REG_BASE + 0x0010)
#define LS2H_PM1_CNT_REG				(LS2H_ACPI_REG_BASE + 0x0014)
#define LS2H_PM1_TMR_REG				(LS2H_ACPI_REG_BASE + 0x0018)
#define LS2H_P_CNT_REG					(LS2H_ACPI_REG_BASE + 0x001c)
#define LS2H_P_LVL2_REG					(LS2H_ACPI_REG_BASE + 0x0020)
#define LS2H_P_LVL3_REG					(LS2H_ACPI_REG_BASE + 0x0024)
#define LS2H_GPE0_STS_REG				(LS2H_ACPI_REG_BASE + 0x0028)
#define LS2H_GPE0_EN_REG				(LS2H_ACPI_REG_BASE + 0x002c)
#define LS2H_RST_CNT_REG				(LS2H_ACPI_REG_BASE + 0x0030)
#define LS2H_WD_SET_REG					(LS2H_ACPI_REG_BASE + 0x0034)
#define LS2H_WD_TIMER_REG				(LS2H_ACPI_REG_BASE + 0x0038)
#define LS2H_DVFS_CNT_REG				(LS2H_ACPI_REG_BASE + 0x003c)
#define LS2H_DVFS_STS_REG				(LS2H_ACPI_REG_BASE + 0x0040)
#define LS2H_MS_CNT_REG					(LS2H_ACPI_REG_BASE + 0x0044)
#define LS2H_MS_THT_REG					(LS2H_ACPI_REG_BASE + 0x0048)
#define	LS2H_THSENS_CNT_REG				(LS2H_ACPI_REG_BASE + 0x004c)
#define LS2H_GEN_RTC1_REG				(LS2H_ACPI_REG_BASE + 0x0050)
#define LS2H_GEN_RTC2_REG				(LS2H_ACPI_REG_BASE + 0x0054)

/* RTC regs */
#define LS2H_RTC_REG_BASE				(LS2H_IO_REG_BASE + 0x00ef8000)
#define	LS2H_TOY_TRIM_REG				(LS2H_RTC_REG_BASE + 0x0020)
#define	LS2H_TOY_WRITE0_REG				(LS2H_RTC_REG_BASE + 0x0024)
#define	LS2H_TOY_WRITE1_REG				(LS2H_RTC_REG_BASE + 0x0028)
#define	LS2H_TOY_READ0_REG				(LS2H_RTC_REG_BASE + 0x002c)
#define	LS2H_TOY_READ1_REG				(LS2H_RTC_REG_BASE + 0x0030)
#define	LS2H_TOY_MATCH0_REG				(LS2H_RTC_REG_BASE + 0x0034)
#define	LS2H_TOY_MATCH1_REG				(LS2H_RTC_REG_BASE + 0x0038)
#define	LS2H_TOY_MATCH2_REG				(LS2H_RTC_REG_BASE + 0x003c)
#define	LS2H_RTC_CTRL_REG				(LS2H_RTC_REG_BASE + 0x0040)
#define	LS2H_RTC_TRIM_REG				(LS2H_RTC_REG_BASE + 0x0060)
#define	LS2H_RTC_WRITE0_REG				(LS2H_RTC_REG_BASE + 0x0064)
#define	LS2H_RTC_READ0_REG				(LS2H_RTC_REG_BASE + 0x0068)
#define	LS2H_RTC_MATCH0_REG				(LS2H_RTC_REG_BASE + 0x006c)
#define	LS2H_RTC_MATCH1_REG				(LS2H_RTC_REG_BASE + 0x0070)
#define	LS2H_RTC_MATCH2_REG				(LS2H_RTC_REG_BASE + 0x0074)

/* LPC regs */
#define LS2H_LPC_IO_BASE				(LS2H_IO_REG_BASE + 0x00f00000)
#define LS2H_LPC_REG_BASE				(LS2H_IO_REG_BASE + 0x00f10000)
#define LS2H_LPC_CFG0_REG				(LS2H_LPC_REG_BASE + 0x0)
#define LS2H_LPC_CFG1_REG				(LS2H_LPC_REG_BASE + 0x4)
#define LS2H_LPC_CFG2_REG				(LS2H_LPC_REG_BASE + 0x8)
#define LS2H_LPC_CFG3_REG				(LS2H_LPC_REG_BASE + 0xc)

#define LS2H_PCIE_MAX_PORTNUM       3
#define LS2H_PCIE_PORT0             0
#define LS2H_PCIE_PORT1             1
#define LS2H_PCIE_PORT2             2
#define LS2H_PCIE_PORT3             3
#define LS2H_PCIE_GET_PORTNUM(sysdata) \
        ((((struct pci_controller *)(sysdata))->mem_resource->start \
                        & ~LS2H_PCIE_MEM0_DOWN_MASK) >> 25)

#define LS2H_CHIP_CFG_REG_CLK_CTRL3     0x22c
#define LS2H_CLK_CTRL3_BIT_PEREF_EN(portnum) (1 << (24 + portnum))

#define LS2H_PCIE_MEM0_BASE_PORT(portnum)       (0x10000000 + (portnum << 25))
#define LS2H_PCIE_IO_BASE_PORT(portnum)         (0x18100000 + (portnum << 22))

#define LS2H_PCIE_REG_BASE_PORT(portnum)        (0x18118000 + (portnum << 22))
#define LS2H_PCIE_PORT_REG_CTR0			0x0
#define LS2H_PCIE_REG_CTR0_BIT_LTSSM_EN			(1 << 3)
#define LS2H_PCIE_REG_CTR0_BIT_REQ_L1			(1 << 12)
#define LS2H_PCIE_REG_CTR0_BIT_RDY_L23			(1 << 13)
#define LS2H_PCIE_PORT_REG_STAT1		0xC
#define LS2H_PCIE_REG_STAT1_MASK_LTSSM		0x0000003f
#define LS2H_PCIE_REG_STAT1_BIT_LINKUP			(1 << 6)
#define LS2H_PCIE_PORT_REG_CFGADDR		0x24
#define LS2H_PCIE_PORT_REG_CTR_STAT		0x28
#define LS2H_PCIE_REG_CTR_STAT_BIT_ISX4			(1 << 26)
#define LS2H_PCIE_REG_CTR_STAT_BIT_ISRC			(1 << 27)

#define LS2H_PCIE_PORT_HEAD_BASE_PORT(portnum)  (0x18114000 + (portnum << 22))
#define LS2H_PCIE_DEV_HEAD_BASE_PORT(portnum)   (0x18116000 + (portnum << 22))

#define LIE_IN_WINDOW(addr,base,mask)   ((addr & mask) == base)
#define MAP_2_WINDOW(addr,mmap,mask)    ((addr & (~(mask))) | (mmap & mask))
#define LS2H_PCIE_MEM0_DOWN_BASE		0x10000000
#define LS2H_PCIE_MEM0_DOWN_MASK		0xf8000000
#define LS2H_PCIE_MEM0_UP_BASE			0x10000000
#define LS2H_PCIE_MEM0_UP_MASK			0xfe000000
#define LS2H_PCIE_IO_DOWN_BASE			0x18100000
#define LS2H_PCIE_IO_DOWN_MASK			0xff3f0000
#define LS2H_PCIE_IO_UP_BASE			0x0
#define LS2H_PCIE_IO_UP_MASK			0xffff0000

typedef struct LS2hState {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/
    
    MIPSCPU *cpu;
    //qemu_irq irq[AW_A10_PIC_INT_NR];
    //LS2hPICState intc;
    //LS2hEmacState emac;
    //SysbusAHCIState sata;
    //PMSMBus smb;

    int memory_initialized;
    MemoryRegion mc_mem;
    MemoryRegion mc_io;
    ram_addr_t ram_size;

    uint64_t scache0_addr, scache0_mask;
    MemoryRegion scache0_ram;
    uint64_t scache1_addr, scache1_mask;
    MemoryRegion scache1_ram;

    MemoryRegion ram;
    MemoryRegion ram_hi;
    MemoryRegion ram_hi2;
    MemoryRegion bios;
    MemoryRegion uart0_mem;
    MemoryRegion uart0_io;
    MemoryRegion uart1_mem;
    MemoryRegion uart1_io;
    MemoryRegion uart2_mem;
    MemoryRegion uart2_io;
    MemoryRegion uart3_mem;
    MemoryRegion uart3_io;
    MemoryRegion creg_mem;
    MemoryRegion creg_io;
    MemoryRegion gpu_mem;
    MemoryRegion gpu_io;
    MemoryRegion nand_mem;
    MemoryRegion nand_io;

    MemoryRegion sata_mem;
    MemoryRegion sata_io;
    MemoryRegion acpi_mem;
    MemoryRegion acpi_io;

    MemoryRegion lpc_mem;
    qemu_irq isa_irqs[16];

    DeviceState *netdev0;
    DeviceState *netdev1;
    DeviceState *spidev;
    DeviceState *intc_dev;
    DeviceState *i2c0_dev, *i2c1_dev;
    DeviceState *lpc_dev;

} LS2hState;

#endif /*_LS2H_H*/

