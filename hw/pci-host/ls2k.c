/*
 * Loongson 2k1000 PCI controller support
 *
 * Copyright (c) 2013 QiaoCong(qiaochong@loongson.cn)
 *
 * This code is licensed under the GNU GPL v3.
 *
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/pci/pci_device.h"
#include "hw/irq.h"
#include "hw/pci-host/bonito.h"
#include "hw/pci/pci_host.h"
#include "hw/pci/msi.h"
#include "migration/vmstate.h"
#include "sysemu/runstate.h"
#include "hw/misc/unimp.h"
#include "hw/registerfields.h"
#include "qom/object.h"
#include "trace.h"

static int pci_ls2k_map_irq(PCIDevice *d, int pin)
{
        int dev = (d->devfn >> 3) & 0x1f;
        int fn = d->devfn & 7;

        if (pci_get_bus(d) != ls2k_pci_bus) {
                return pin;
        }

        switch (dev) {
        case 2:
                /*APB 2*/
                break;

        case 3:
                /*GMAC0 3 0*/
                /*GMAC1 3 1*/
                return (fn == 0) ? 12 : 14;

        case 4:
                /*
                  OTG: 4 0
                  EHCI: 4 1
                  OHCI: 4 2
                */
                return (fn == 0) ? 49 : (fn == 1) ? 50 : 51;
                break;

        case 5:
                /*GPU*/
                return 29;
                break;

        case 6:
                /*DC*/
                return 28;
                break;

        case 7:
                /*HDA*/
                return 4;
                break;

        case 8:
                /*SATA*/
                return 19;
                break;

        case 9:
                /*PCIE PORT 0*/
                return 32;;
                break;

        case 10:
                /*PCIE PORT 1*/
                return 33;
                break;

        case 11:
                /*PCIE PORT 2*/
                return 34;
                break;

        case 12:
                return 35;
                /*PCIE PORT 3*/
                break;

        case 13:
                return 36;
                /*PCIE1 PORT 0*/
                break;

        case 14:
                /*PCIE1 PORT 1*/
                return 37;
                break;

        case 15:
                /*DMA*/
                break;

        }

        return  0;
}

static void pci_ls2k_set_irq(void *opaque, int irq_num, int level)
{
        if (irq_num < 32) {
                qemu_set_irq(ls2k_irq[irq_num], level);
        } else {
                qemu_set_irq(ls2k_irq1[irq_num - 32], level);
        }
}

/*self pci header*/
#define LS2K_PCIE_PORT_HEAD_BASE_PORT(portnum)  (0x18114000 + (portnum << 22))
/*devices pci header*/
#define LS3H_PCIE_DEV_HEAD_BASE_PORT(portnum)   (0x18116000 + (portnum << 22))
/*pci map*/
#define LS2K_PCIE_REG_BASE_PORT(portnum)        (0x18118000 + (portnum << 22))
#define LS2K_PCIE_PORT_REG_STAT1                0xC
#define LS2K_PCIE_PORT_REG_CTR0                 0x0
#define LS2K_PCIE_PORT_REG_CFGADDR              0x24
#define LS2K_PCIE_PORT_REG_CTR_STAT             0x28


#define TYPE_LS2K_PCI_HOST_BRIDGE "ls2k-pcihost"
typedef struct BridgeState BridgeState;

#define LS2K_PCI_HOST_BRIDGE(obj)                                     \
        OBJECT_CHECK(BridgeState, (obj), TYPE_LS2K_PCI_HOST_BRIDGE)

typedef struct PCIBridgeState {
        /*< private >*/
        PCIBridge parent_obj;
        /*< public >*/
        BridgeState *pcihost;
        MemoryRegion iomem;
        MemoryRegion conf_mem;
        struct pcilocalreg {
                /*0*/
                unsigned int portctr0;
                unsigned int portctr1;
                unsigned int portstat0;
                unsigned int portstat1;
                /*0x10*/
                unsigned int usrmsgid;
                unsigned int nouse;
                unsigned int portintsts;
                unsigned int portintclr;
                /*0x20*/
                unsigned int portintmsk;
                unsigned int portcfg;
                unsigned int portctrsts;
                unsigned int physts;
                /*0x30*/
                unsigned int nouse1[2];
                unsigned int usrmsg0;
                unsigned int usrmsg1;
                /*0x40*/
                unsigned int usrmsgsend0;
                unsigned int usrmsgsend1;
                unsigned int noused2[5];
                /*0x5c*/
                unsigned int msi;
                unsigned int noused3[2];
                /*0x68*/
                unsigned int addrmsk;
                unsigned int addrmsk1;
                /*0x70*/
                unsigned int addrtrans;
                unsigned int addrtrans1;
                unsigned int dataload0;
                unsigned int dataload1;
        } mypcilocalreg;
} PCIBridgeState;

struct BridgeState {
        PCIExpressHost parent_obj;
        PCIBus *bus;
        qemu_irq *pic;
        PCIBridgeState *pci_dev;
        MemoryRegion iomem_mem;
        MemoryRegion iomem_submem;
        MemoryRegion iomem_subbigmem;
        MemoryRegion iomem_io;
        AddressSpace as_mem;
        AddressSpace as_io;
        MemoryRegion data_mem;
        MemoryRegion data_mem1;
        int (*pci_map_irq)(PCIDevice *d, int irq_num);
};



static void ls2k_pciconf_writel(void *opaque, hwaddr addr,
                                  uint64_t val, unsigned size)
{
        PCIBridgeState *s = opaque;
        PCIDevice *d = PCI_DEVICE(s);

        d->config_write(d, addr, val, 4);
}

static uint64_t ls2k_pciconf_readl(void *opaque, hwaddr addr,
                                     unsigned size)
{

        PCIBridgeState *s = opaque;
        PCIDevice *d = PCI_DEVICE(s);

        return d->config_read(d, addr, 4);
}

/* north bridge PCI configure space. APBBASE+0x0000 - APBBASE+0x00ff */

static const MemoryRegionOps ls2k_pciconf_ops = {
        .read = ls2k_pciconf_readl,
        .write = ls2k_pciconf_writel,
        .endianness = DEVICE_NATIVE_ENDIAN,
        .valid = {
                .min_access_size = 4,
                .max_access_size = 4,
        },
};

static void ls2k_initfn(PCIDevice *dev, Error **errp)
{
        PCIBridgeState *s = DO_UPCAST(PCIBridgeState, parent_obj.parent_obj,
        dev);
        SysBusDevice *sysbus = SYS_BUS_DEVICE(s->pcihost);

        pci_bridge_initfn(dev, TYPE_PCI_BUS);

        /* set the north bridge pci configure  mapping */
        memory_region_init_io(&s->conf_mem, NULL, &ls2k_pciconf_ops, s,
                              "north-bridge-pci-config", 0x100);
        sysbus_init_mmio(sysbus, &s->conf_mem);


        pci_config_set_prog_interface(dev->config,
        PCI_CLASS_BRIDGE_PCI_INF_SUB);
        /* set the default value of north bridge pci config */

        pci_set_word(dev->config + PCI_COMMAND,
                     PCI_COMMAND_MEMORY | PCI_COMMAND_IO);
        pci_set_word(dev->config + PCI_STATUS,
                     PCI_STATUS_FAST_BACK | PCI_STATUS_66MHZ |
                     PCI_STATUS_DEVSEL_MEDIUM);
        pci_set_word(dev->config + PCI_SUBSYSTEM_VENDOR_ID, 0x0000);
        pci_set_word(dev->config + PCI_SUBSYSTEM_ID, 0x0000);

        pci_set_byte(dev->config + PCI_INTERRUPT_LINE, 0x00);
        pci_set_byte(dev->config + PCI_INTERRUPT_PIN, 0x01);
        pci_set_byte(dev->config + PCI_MIN_GNT, 0x3c);
        pci_set_byte(dev->config + PCI_MAX_LAT, 0x00);
        pci_set_word(dev->config + PCI_CLASS_DEVICE, 0x0604);

}


static void ls2k_class_init(ObjectClass *klass, void *data)
{
        DeviceClass *dc = DEVICE_CLASS(klass);
        PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

        k->realize = ls2k_initfn;
        k->exit = pci_bridge_exitfn;
        k->vendor_id = 0xdf53;
        k->device_id = 0x00d5;
        k->revision = 0x01;
        k->class_id = PCI_CLASS_BRIDGE_HOST;
        k->is_bridge = 1;
        k->config_write = pci_bridge_write_config;
        dc->reset = pci_bridge_reset;
        dc->desc = "Host bridge";
}

static const TypeInfo ls2k_info = {
        .name          = "LS2K_Bridge",
        .parent        = TYPE_PCI_BRIDGE,
        .instance_size = sizeof(PCIBridgeState),
        .class_init    = ls2k_class_init,
        .interfaces = (InterfaceInfo[])
        {
                { INTERFACE_CONVENTIONAL_PCI_DEVICE },
                { },
        },
};

static AddressSpace *pci_dma_context_fn(PCIBus *bus, void *opaque, int devfn);

#define MAX_SATA_PORTS     6
static PCIBus **pcibus_ls2k_init(int busno, qemu_irq *pic, int
(*board_map_irq)(PCIDevice *d, int irq_num), MemoryRegion *ram, MemoryRegion
*ram1, MemoryRegion *ram2)
{
        DeviceState *dev;
        BridgeState *pcihost;
        PCIBridgeState *s;
        PCIDevice *d;
        SysBusDevice *sysbus;
        PCIBridge *br;
        PCIBus *bus2;
        DriveInfo *hd[MAX_SATA_PORTS];
        static PCIBus *pci_bus[4];
        int i;

        dev = qdev_create(NULL, TYPE_LS2K_PCI_HOST_BRIDGE);
        pcihost = LS2K_PCI_HOST_BRIDGE(dev);
        pcihost->pic = pic;
        pcihost->pci_map_irq = board_map_irq;
        qdev_init_nofail(dev);

        /* set the pcihost pointer before ls2k_initfn is called */
        for (i = 0; i < 4; i++) {
                char buf[16];
                d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(9 + i, 0),
                true, "LS2K_Bridge");
                sprintf(buf, "pcie-%d.0", 9 + i);
                qdev_set_id(DEVICE(d), g_strdup(buf));

                s = DO_UPCAST(PCIBridgeState, parent_obj.parent_obj, d);
                s->pcihost = pcihost;
                pcihost->pci_dev = s;
                br = PCI_BRIDGE(d);
                pci_bridge_map_irq(br, NULL, board_map_irq);
                qdev_init_nofail(DEVICE(d));
                bus2 = pci_bridge_get_sec_bus(br);

                pci_setup_iommu(bus2, pci_dma_context_fn, pcihost);
                pci_bus[i] = bus2;
        }


        d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(3, 0), true,
                                     "pci-synopgmac");
        dev = DEVICE(d);
        if (nd_table[0].used) {
                qdev_set_nic_properties(dev, &nd_table[0]);
        }
        qdev_prop_set_int32(dev, "enh_desc", 1);
        qdev_prop_set_uint32(dev, "version", 0xd137);
        qdev_prop_set_uint32(dev, "hwcap", 0x1b082fbf);
        qdev_init_nofail(DEVICE(d));
        pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
        pci_set_word(d->config + PCI_DEVICE_ID, 0x7a03);

        d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(3, 1), true,
                                     "pci-synopgmac");
        dev = DEVICE(d);
        if (nd_table[1].used) {
                qdev_set_nic_properties(dev, &nd_table[1]);
        }
        qdev_prop_set_int32(dev, "enh_desc", 1);
        qdev_prop_set_uint32(dev, "version", 0xd137);
        qdev_prop_set_uint32(dev, "hwcap", 0x1b082fbf);
        qdev_init_nofail(DEVICE(d));
        pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
        pci_set_word(d->config + PCI_DEVICE_ID, 0x7a03);

        d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(4, 0), true,
        "pciram");
        qdev_prop_set_uint32(DEVICE(d), "bar0", ~(0x00001000 - 1));
        qdev_init_nofail(DEVICE(d));
        pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
        pci_set_word(d->config + PCI_DEVICE_ID, 0x7a04);

#if 1
        d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(4, 1), true,
        "usb-ehci");
        qdev_init_nofail(DEVICE(d));
        pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
        pci_set_word(d->config + PCI_DEVICE_ID, 0x7a14);
#endif

        d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(4, 2), true,
        "pci-ohci");
        qdev_init_nofail(DEVICE(d));
        pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
        pci_set_word(d->config + PCI_DEVICE_ID, 0x7a24);

        /*hda*/

        d = pci_create_multifunction(pcihost->bus,
                                            PCI_DEVFN(7, 0),
                                            true, "intel-hda");

        qdev_prop_set_uint32(DEVICE(d), "debug", 100);
        qdev_init_nofail(DEVICE(d));
        {
                BusState *hdabus;
                DeviceState *codec;

                hdabus = QLIST_FIRST(&DEVICE(d)->child_bus);
                codec = qdev_create(hdabus, "hda-duplex");
                qdev_prop_set_uint32(codec, "debug", 100);
                qdev_init_nofail(codec);
        }
        /*
         * pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
         * pci_set_word(d->config + PCI_DEVICE_ID, 0x7a07);
         */

        /* ahci and SATA device */
        d = pci_create_simple_multifunction(pcihost->bus,
                                            PCI_DEVFN(8, 0),
                                            true, "ls2k-ahci");

        pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
        pci_set_word(d->config + PCI_DEVICE_ID, 0x7a08);

        ide_drive_get(hd, LS2K_AHCI(d)->ahci.ports);
        ls2k_ahci_ide_create_devs(d, hd);

        {
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &loongarch_qemu_ops,
                                      (void *)GPUBASE, "gpu", 0x8000);
                d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(5, 0),
                true, "pciram");
                qdev_prop_set_uint32(DEVICE(d), "bar0", ~(0x00008000 - 1) | 4);
                qdev_prop_set_ptr(DEVICE(d), "iomem0", iomem);

                qdev_init_nofail(DEVICE(d));
                pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
                pci_set_word(d->config + PCI_DEVICE_ID, 0x7a15);
        }

        pci_create_simple_multifunction(pcihost->bus, PCI_DEVFN(6, 0), true,
                                        "pci_ls2h_fb");

        sysbus = SYS_BUS_DEVICE(pcihost);
        /*devices header*/
        /*
         * sysbus_mmio_map(sysbus, 0, 0x1a000000);
         */
        sysbus_mmio_map(sysbus, 1, 0xfe00000000ULL);

        memory_region_add_subregion(get_system_memory(), 0x10000000UL,
                                    &pcihost->iomem_submem);
        memory_region_add_subregion(get_system_memory(), 0x40000000UL,
                                    &pcihost->iomem_subbigmem);
        memory_region_add_subregion(get_system_memory(), 0x18000000UL,
                                    &pcihost->iomem_io);

        ALIAS_REGION_FROM_RAS_TO_RA(ram, 0, 0x80000000, &pcihost->iomem_mem,
                                    0x80000000);
        ALIAS_REGION_FROM_RAS_TO_RA(ram, 0, 0x80000000, &pcihost->iomem_mem, 0x9000000080000000ULL);
        ALIAS_REGION_FROM_RAS_TO_RA(ram1, 0, memory_region_size(ram1), &pcihost->iomem_mem, 0x9000000000000000ULL);
        ALIAS_REGION_FROM_RAS_TO_RA(ram, 0, memory_region_size(ram), &pcihost->iomem_mem, 0x9000000100000000ULL);

        memory_region_add_subregion(&pcihost->iomem_mem, 0x0UL, ram1);
        if (ram2) {
                memory_region_add_subregion(&pcihost->iomem_mem, 0x20000000,
                ram2);
        }
        memory_region_add_subregion(&pcihost->iomem_mem, 0x100000000UL, ram);

        ls2k_pci_bus = pcihost->bus;

        return pci_bus;
}


static AddressSpace *pci_dma_context_fn(PCIBus *bus, void *opaque, int devfn)
{
        BridgeState *pcihost = opaque;
        return &pcihost->as_mem;
}

static void pci_ls2k_config_writel(void *opaque, hwaddr addr,
                                   uint64_t val, unsigned size)
{
        BridgeState *phb = opaque;

        addr &= 0xffffff;

        pci_data_write(phb->bus,  addr, val, size);
}

static uint64_t pci_ls2k_config_readl(void *opaque, hwaddr addr, unsigned size)
{
        BridgeState *phb = opaque;
        uint32_t val;

        addr &= 0xffffff;


        val = pci_data_read(phb->bus, addr, size);
        return val;
}


static const MemoryRegionOps pci_ls2k_config_ops = {
        .read = pci_ls2k_config_readl,
        .write = pci_ls2k_config_writel,
        .endianness = DEVICE_NATIVE_ENDIAN,
};


/*
  two way to translate pci dma address:
  pci_setup_iommu
  memory_region_init_iommu
  pci_setup_iommu will not change addr.
  memory_region_init_iommu can translate region and addr.
*/

static void ls2k_pcihost_initfn(DeviceState *dev, Error **errp)
{
        BridgeState *pcihost;
        PCIHostState *phb;
        SysBusDevice *sysbus;
        pcihost = LS2K_PCI_HOST_BRIDGE(dev);
        sysbus = SYS_BUS_DEVICE(pcihost);

        memory_region_init(&pcihost->iomem_mem, OBJECT(pcihost), "system",
        UINT64_MAX);
        address_space_init(&pcihost->as_mem, &pcihost->iomem_mem,
                           "pcie memory");
#ifdef DEBUG_PCIEDMA
        ls2k_pci_as = &pcihost->as_mem;
#endif

        /* Host memory as seen from the PCI side, via the IOMMU.  */

        memory_region_init_alias(&pcihost->iomem_submem, NULL, "pcisubmem",
                                 &pcihost->iomem_mem, 0x10000000, 0x2000000);
        memory_region_init_alias(&pcihost->iomem_subbigmem, NULL, "pcisubmem",
                                 &pcihost->iomem_mem, 0x40000000, 0x20000000);

        memory_region_init(&pcihost->iomem_io, OBJECT(pcihost), "system",
        0x10000);
        address_space_init(&pcihost->as_io, &pcihost->iomem_io, "pcie io");

        phb = PCI_HOST_BRIDGE(dev);
        pcihost->bus = phb->bus = pci_register_root_bus(DEVICE(dev), "pci",
        pci_ls2k_set_irq, pcihost->pci_map_irq, pcihost->pic,
        &pcihost->iomem_mem, &pcihost->iomem_io, PCI_DEVFN(0, 0), 64,
        TYPE_PCIE_BUS);


        pci_setup_iommu(pcihost->bus, pci_dma_context_fn, pcihost);

        /* set the south bridge pci configure  mapping */
        memory_region_init_io(&pcihost->data_mem, NULL, &pci_ls2k_config_ops,
        pcihost, "south-bridge-pci-config", 0x2000000);
        sysbus_init_mmio(sysbus, &pcihost->data_mem);

        memory_region_init_io(&pcihost->data_mem1, NULL, &pci_ls2k_config_ops,
        pcihost, "south-bridge-pci-config", 0x200000000);
        sysbus_init_mmio(sysbus, &pcihost->data_mem1);
}

static const char *ls2k_host_root_bus_path(PCIHostState *host_bridge,
                PCIBus *rootbus)
{
        return "0000:00";
}

static void ls2k_pcihost_class_init(ObjectClass *klass, void *data)
{
        DeviceClass *dc = DEVICE_CLASS(klass);
        PCIHostBridgeClass *hc = PCI_HOST_BRIDGE_CLASS(klass);

        hc->root_bus_path = ls2k_host_root_bus_path;
        dc->realize = ls2k_pcihost_initfn;
}

static const TypeInfo ls2k_pcihost_info = {
        .name          = TYPE_LS2K_PCI_HOST_BRIDGE,
        .parent        = TYPE_PCIE_HOST_BRIDGE,
        .instance_size = sizeof(BridgeState),
        .class_init    = ls2k_pcihost_class_init,
};

static void ls2k_register_types(void)
{
        type_register_static(&ls2k_pcihost_info);
        type_register_static(&ls2k_info);
}

type_init(ls2k_register_types)
