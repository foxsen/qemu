/*
 * QEMU model of LS2H STMMAC Ethernet. The mac is based on sysnopsis IP.
 * It is similiar with stmmac.
 *
 * derived from the Xilinx AXI-Ethernet by Edgar E. Iglesias.
 *
 * Copyright (c) 2016 Fuxin Zhang,Lemote Inc.
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

#include "hw/sysbus.h"
#include "sysemu/char.h"
#include "qemu/log.h"
#include "net/net.h"
#include "net/checksum.h"

//#define DEBUG_STMMAC

#ifdef DEBUG_STMMAC
#define DPRINTF(message, args...) do { \
                                         fprintf(stderr, (message), ## args); \
                                     } while (0)
#else
#define DPRINTF(message, args...) do { } while (0)
#endif

#define STMMAC_CONTROL           0x00000000   /* MAC Configuration */
#define STMMAC_FRAME_FILTER      0x00000001   /* MAC Frame Filter */
#define STMMAC_HASH_TABLE_HI     0x00000002   /* MAC Hash table high */
#define STMMAC_HASH_TABLE_LO     0x00000003   /* MAC hash table low */
#define STMMAC_GMII_ADDRESS      0x00000004   /* MAC GMII address */
#define STMMAC_GMII_DATA         0x00000005   /* MAC GMII data */
#define STMMAC_FLOW_CTRL         0x00000006   /* MAC Flow Control */
#define STMMAC_VLAN_TAG          0x00000007   /* VLAN Tags */
#define STMMAC_VERSION           0x00000008   /* Version */
#define STMMAC_INT_STATUS        0x0000000e   /* Interrupt Status */
#define STMMAC_INT_MASK          0x0000000f   /* Interrupt Mask */

#define STMMAC_ADDR_HIGH(reg)    (0x00000010+((reg) * 2))
#define STMMAC_ADDR_LOW(reg)     (0x00000011+((reg) * 2))

#define DMA_BUS_MODE            0x00000400   /* Bus Mode */
#define DMA_XMT_POLL_DEMAND     0x00000401   /* Transmit Poll Demand */
#define DMA_RCV_POLL_DEMAND     0x00000402   /* Received Poll Demand */
#define DMA_RCV_BASE_ADDR       0x00000403   /* Receive List Base */
#define DMA_TX_BASE_ADDR        0x00000404   /* Transmit List Base */
#define DMA_STATUS              0x00000405   /* Status Register */
#define DMA_CONTROL             0x00000406   /* Ctrl (Operational Mode) */
#define DMA_INTR_ENA            0x00000407   /* Interrupt Enable */
#define DMA_MISSED_FRAME_CTR    0x00000408   /* Missed Frame Counter */
/* Receive Interrupt Watchdog Timer */
#define DMA_RI_WATCHDOG_TIMER   0x00000409
#define DMA_AXI_BUS             0x0000040a   /* AXI Bus Mode */
#define DMA_AXI_STATUS          0x0000040b   /* AXI Status */
#define DMA_CUR_TX_DESC_ADDR    0x00000412   /* Current Host Tx Descriptor */
#define DMA_CUR_RX_DESC_ADDR    0x00000413   /* Current Host Rx Descriptor */
#define DMA_CUR_TX_BUF_ADDR     0x00000414   /* Current Host Tx Buffer */
#define DMA_CUR_RX_BUF_ADDR     0x00000415   /* Current Host Rx Buffer */

/* DMA Status register defines */
#define DMA_STATUS_GMI          0x08000000   /* MMC interrupt */
#define DMA_STATUS_GLI          0x04000000   /* GMAC Line interface int */
#define DMA_STATUS_EB_MASK      0x00380000   /* Error Bits Mask */
#define DMA_STATUS_EB_TX_ABORT  0x00080000   /* Error Bits - TX Abort */
#define DMA_STATUS_EB_RX_ABORT  0x00100000   /* Error Bits - RX Abort */
#define DMA_STATUS_TS_MASK      0x00700000   /* Transmit Process State */
#define DMA_STATUS_TS_SHIFT     20
#define DMA_STATUS_RS_MASK      0x000e0000   /* Receive Process State */
#define DMA_STATUS_RS_SHIFT     17
#define DMA_STATUS_NIS          0x00010000   /* Normal Interrupt Summary */
#define DMA_STATUS_AIS          0x00008000   /* Abnormal Interrupt Summary */
#define DMA_STATUS_ERI          0x00004000   /* Early Receive Interrupt */
#define DMA_STATUS_FBI          0x00002000   /* Fatal Bus Error Interrupt */
#define DMA_STATUS_ETI          0x00000400   /* Early Transmit Interrupt */
#define DMA_STATUS_RWT          0x00000200   /* Receive Watchdog Timeout */
#define DMA_STATUS_RPS          0x00000100   /* Receive Process Stopped */
#define DMA_STATUS_RU           0x00000080   /* Receive Buffer Unavailable */
#define DMA_STATUS_RI           0x00000040   /* Receive Interrupt */
#define DMA_STATUS_UNF          0x00000020   /* Transmit Underflow */
#define DMA_STATUS_OVF          0x00000010   /* Receive Overflow */
#define DMA_STATUS_TJT          0x00000008   /* Transmit Jabber Timeout */
#define DMA_STATUS_TU           0x00000004   /* Transmit Buffer Unavailable */
#define DMA_STATUS_TPS          0x00000002   /* Transmit Process Stopped */
#define DMA_STATUS_TI           0x00000001   /* Transmit Interrupt */

/* DMA Control register defines */
#define DMA_CONTROL_ST          0x00002000   /* Start/Stop Transmission */
#define DMA_CONTROL_SR          0x00000002   /* Start/Stop Receive */
#define DMA_CONTROL_DFF         0x01000000   /* Disable flush of rx frames */

struct desc {
    uint32_t ctl_stat;
    uint16_t buffer1_size;
    uint16_t buffer2_size;
    uint32_t buffer1_addr;
    uint32_t buffer2_addr;
    uint32_t res[2];
    uint32_t timestamp[2];
};

#define R_MAX 0x420

typedef struct RxTxStats {
    uint64_t rx_bytes;
    uint64_t tx_bytes;

    uint64_t rx;
    uint64_t rx_bcast;
    uint64_t rx_mcast;
} RxTxStats;

/* phy support borrowed from xilinx_axienet with adaptions */

/* Advertisement control register. */
#define ADVERTISE_10HALF        0x0020  /* Try for 10mbps half-duplex  */
#define ADVERTISE_10FULL        0x0040  /* Try for 10mbps full-duplex  */
#define ADVERTISE_100HALF       0x0080  /* Try for 100mbps half-duplex */
#define ADVERTISE_100FULL       0x0100  /* Try for 100mbps full-duplex */

struct PHY {
    uint32_t regs[32];

    int link;

    unsigned int (*read)(struct PHY *phy, unsigned int req);
    void (*write)(struct PHY *phy, unsigned int req,
                  unsigned int data);
};

struct MDIOBus {
    struct PHY *devs[32];
};

/* Indirect registers.  */
struct TEMAC  {
    struct MDIOBus mdio_bus;
    struct PHY phy;

    void *parent;
};

static unsigned int phy_read(struct PHY *phy, unsigned int req)
{
    int regnum;
    unsigned r = 0;

    regnum = req & 0x1f;

    switch (regnum) {
        case 1:
            if (!phy->link) {
                break;
            }
            /* MR1.  */
            /* Speeds and modes.  */
            r |= (1 << 13) | (1 << 14);
            r |= (1 << 11) | (1 << 12);
            r |= (1 << 5); /* Autoneg complete.  */
            r |= (1 << 3); /* Autoneg able.  */
            r |= (1 << 2); /* link.  */
            r |= (1 << 1); /* link.  */
            break;
        case 5:
            /* Link partner ability.
               We are kind; always agree with whatever best mode
               the guest advertises.  */
            r = 1 << 14; /* Success.  */
            /* Copy advertised modes.  */
            r |= phy->regs[4] & (15 << 5);
            /* Autoneg support.  */
            r |= 1;
            break;
        case 17:
            /* Marvell PHY on many xilinx boards.  */
            r = 0x8000; /* 1000Mb  */
            break;
        case 18:
            {
                /* Diagnostics reg.  */
                int duplex = 0;
                int speed_100 = 0;

                if (!phy->link) {
                    break;
                }

                /* Are we advertising 100 half or 100 duplex ? */
                speed_100 = !!(phy->regs[4] & ADVERTISE_100HALF);
                speed_100 |= !!(phy->regs[4] & ADVERTISE_100FULL);

                /* Are we advertising 10 duplex or 100 duplex ? */
                duplex = !!(phy->regs[4] & ADVERTISE_100FULL);
                duplex |= !!(phy->regs[4] & ADVERTISE_10FULL);
                r = (speed_100 << 10) | (duplex << 11);
            }
            break;

        default:
            r = phy->regs[regnum];
            break;
    }
    DPRINTF("\n%s %x = reg[%d]\n", __func__, r, regnum);
    return r;
}

static void
phy_write(struct PHY *phy, unsigned int req, unsigned int data)
{
    int regnum;

    regnum = req & 0x1f;
    DPRINTF("%s reg[%d] = %x\n", __func__, regnum, data);
    switch (regnum) {
        default:
            phy->regs[regnum] = data;
            break;
    }

    /* Unconditionally clear regs[BMCR][BMCR_RESET] */
    phy->regs[0] &= ~0x8000;
}

static void
phy_init(struct PHY *phy)
{
    phy->regs[0] = 0x3100;
    /* PHY Id.  */
    phy->regs[2] = 0x0300;
    phy->regs[3] = 0xe400;
    /* Autonegotiation advertisement reg.  */
    phy->regs[4] = 0x01E1;
    phy->link = 1;

    phy->read = phy_read;
    phy->write = phy_write;
}

static void
mdio_attach(struct MDIOBus *bus, struct PHY *phy, unsigned int addr)
{
    bus->devs[addr & 0x1f] = phy;
}

static uint16_t mdio_read_req(struct MDIOBus *bus, unsigned int addr,
                  unsigned int reg)
{
    struct PHY *phy;
    uint16_t data;

    phy = bus->devs[addr];
    if (phy && phy->read) {
        data = phy->read(phy, reg);
    } else {
        data = 0xffff;
    }
    return data;
}

static void mdio_write_req(struct MDIOBus *bus, unsigned int addr,
               unsigned int reg, uint16_t data)
{
    struct PHY *phy;

    phy = bus->devs[addr];
    if (phy && phy->write) {
        phy->write(phy, reg, data);
    }
}

#define TYPE_STMMAC "stmmac"
#define STMMAC(obj) OBJECT_CHECK(StmmacState, (obj), TYPE_STMMAC)

typedef struct StmmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq sbd_irq;
    qemu_irq pmt_irq;
    qemu_irq mci_irq;
    NICState *nic;
    NICConf conf;

    struct RxTxStats stats;
    uint32_t regs[R_MAX];

    uint32_t c_phyaddr;

    struct TEMAC TEMAC;

    uint32_t descriptor_size;
    uint32_t skip_size;
} StmmacState;

static const VMStateDescription vmstate_rxtx_stats = {
    .name = "stmmac_stats",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64(rx_bytes, RxTxStats),
        VMSTATE_UINT64(tx_bytes, RxTxStats),
        VMSTATE_UINT64(rx, RxTxStats),
        VMSTATE_UINT64(rx_bcast, RxTxStats),
        VMSTATE_UINT64(rx_mcast, RxTxStats),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_stmmac = {
    .name = "stmmac",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(stats, StmmacState, 0, vmstate_rxtx_stats, RxTxStats),
        VMSTATE_UINT32_ARRAY(regs, StmmacState, R_MAX),
        VMSTATE_END_OF_LIST()
    }
};

static void stmmac_read_desc(StmmacState *s, struct desc *d, int rx)
{
    uint32_t addr = rx ? s->regs[DMA_CUR_RX_DESC_ADDR] :
        s->regs[DMA_CUR_TX_DESC_ADDR];
    cpu_physical_memory_read(addr, d, s->descriptor_size);
}

static void stmmac_write_desc(StmmacState *s, struct desc *d, int rx)
{
    int reg = rx ? DMA_CUR_RX_DESC_ADDR : DMA_CUR_TX_DESC_ADDR;
    uint32_t addr = s->regs[reg];

    if (!rx && (d->ctl_stat & 0x00200000)) {
        s->regs[reg] = s->regs[DMA_TX_BASE_ADDR];
    } else if (rx && (d->buffer1_size & 0x8000)) {
        s->regs[reg] = s->regs[DMA_RCV_BASE_ADDR];
    } else {
        s->regs[reg] += s->descriptor_size + s->skip_size;
    }
    cpu_physical_memory_write(addr, d, s->descriptor_size);
}

static void stmmac_enet_send(StmmacState *s)
{
    struct desc bd;
    int frame_size;
    int len;
    uint8_t frame[8192];
    uint8_t *ptr;

    ptr = frame;
    frame_size = 0;
    while (1) {
        stmmac_read_desc(s, &bd, 0);
        if ((bd.ctl_stat & 0x80000000) == 0) {
            /* Run out of descriptors to transmit.  */
            break;
        }
        len = (bd.buffer1_size & 0xfff) + (bd.buffer2_size & 0xfff);

        if ((bd.buffer1_size & 0xfff) > 2048) {
            DPRINTF("qemu:%s:ERROR...ERROR...ERROR... -- "
                        "stmmac buffer 1 len on send > 2048 (0x%x)\n",
                         __func__, bd.buffer1_size & 0xfff);
        }
        if ((bd.buffer2_size & 0xfff) != 0) {
            DPRINTF("qemu:%s:ERROR...ERROR...ERROR... -- "
                        "stmmac buffer 2 len on send != 0 (0x%x)\n",
                        __func__, bd.buffer2_size & 0xfff);
        }
        if (len >= sizeof(frame)) {
            DPRINTF("qemu:%s: buffer overflow %d read into %zu "
                        "buffer\n" , __func__, len, sizeof(frame));
            DPRINTF("qemu:%s: buffer1.size=%d; buffer2.size=%d\n",
                        __func__, bd.buffer1_size, bd.buffer2_size);
        }

        cpu_physical_memory_read(bd.buffer1_addr, ptr, len);
        ptr += len;
        frame_size += len;
        if (bd.ctl_stat & 0x20000000) {
            /* Last buffer in frame.  */
            qemu_send_packet(qemu_get_queue(s->nic), frame, len);
            ptr = frame;
            frame_size = 0;
            s->regs[DMA_STATUS] |= DMA_STATUS_TI | DMA_STATUS_NIS;
        }
        bd.ctl_stat &= ~0x80000000;
        /* Write back the modified descriptor.  */
        stmmac_write_desc(s, &bd, 0);
    }
}

static void enet_update_irq(StmmacState *s)
{
    int stat = s->regs[DMA_STATUS] & s->regs[DMA_INTR_ENA];
    qemu_set_irq(s->sbd_irq, !!stat);
}

static uint64_t enet_read(void *opaque, hwaddr addr, unsigned size)
{
    StmmacState *s = opaque;
    struct TEMAC *t = &s->TEMAC;
    uint64_t r = 0;
    addr >>= 2;

    switch (addr) {
    case STMMAC_VERSION:
        r = 0x1012;
        break;
    case STMMAC_GMII_ADDRESS:
    /* clear the GMII busy bit */
        r = s->regs[STMMAC_GMII_ADDRESS] & ~0x1;
        break;
    case STMMAC_GMII_DATA:
        {
            uint32_t addr = s->regs[STMMAC_GMII_ADDRESS];
            uint32_t phyaddr = (addr >> 11) & 0x1f;
            uint32_t regaddr = (addr >> 6) & 0x1f;
            uint32_t op = addr & 0x2;

            if (op == 0) {
                r = mdio_read_req(&t->mdio_bus, phyaddr, regaddr);
                s->regs[STMMAC_GMII_DATA] = r;
            } else {
                fprintf(stderr, "READ from MII DATA with wrong op in addr\n");
            }
            break;
        }
        break;
    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            r = s->regs[addr];
        }
        break;
    }

    DPRINTF("stmmac read %lx, val=%lx, size=%d\n", addr, r, size);
    return r;
}

static void enet_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned size)
{
    StmmacState *s = opaque;
    struct TEMAC *t = &s->TEMAC;

    addr >>= 2;
    DPRINTF("stmmac write %lx, val=%lx, size=%d\n", addr, value, size);
    switch (addr) {
        case STMMAC_GMII_ADDRESS: 
            s->regs[STMMAC_GMII_ADDRESS] = value;
            break;
        case STMMAC_GMII_DATA:
        {
            uint32_t addr = s->regs[STMMAC_GMII_ADDRESS];
            uint32_t phyaddr = (addr >> 11) & 0x1f;
            uint32_t regaddr = (addr >> 6) & 0x1f;
            uint32_t op = addr & 0x2;

            if (op == 1) {
                s->regs[STMMAC_GMII_DATA] = value;
                mdio_write_req(&t->mdio_bus, phyaddr, regaddr, value);
            } else {
                fprintf(stderr, "Write to MII DATA with wrong op in addr\n");
            }
            break;
        }
        break;
    case DMA_BUS_MODE:
        s->regs[DMA_BUS_MODE] = value & ~0x1;
        if (value & 0x80) {
            s->descriptor_size = 32;
        } else {
            s->descriptor_size = 16;
        }
        s->skip_size = (value & 0x7c) * 4;
        DPRINTF("descriptor size %d, skip size %d\n", \
                s->descriptor_size, s->skip_size);
        break;
    case DMA_XMT_POLL_DEMAND:
        stmmac_enet_send(s);
        break;
    case DMA_STATUS:
        s->regs[DMA_STATUS] = s->regs[DMA_STATUS] & ~value;
        break;
    case DMA_RCV_BASE_ADDR:
        s->regs[DMA_RCV_BASE_ADDR] = s->regs[DMA_CUR_RX_DESC_ADDR] = value;
        break;
    case DMA_TX_BASE_ADDR:
        s->regs[DMA_TX_BASE_ADDR] = s->regs[DMA_CUR_TX_DESC_ADDR] = value;
        break;
    default:
        if (addr < ARRAY_SIZE(s->regs)) {
            s->regs[addr] = value;
        }
        break;
    }
    enet_update_irq(s);
}

static const MemoryRegionOps enet_mem_ops = {
    .read = enet_read,
    .write = enet_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static int eth_can_rx(StmmacState *s)
{
    /* RX enabled?  */
    return s->regs[DMA_CONTROL] & DMA_CONTROL_SR;
}

static ssize_t eth_rx(NetClientState *nc, const uint8_t *buf, size_t size)
{
    StmmacState *s = qemu_get_nic_opaque(nc);
    static const unsigned char sa_bcast[6] = {0xff, 0xff, 0xff,
                                              0xff, 0xff, 0xff};
    int unicast, broadcast, multicast;
    struct desc bd;
    ssize_t ret;

    if (!eth_can_rx(s)) {
        return -1;
    }
    unicast = ~buf[0] & 0x1;
    broadcast = memcmp(buf, sa_bcast, 6) == 0;
    multicast = !unicast && !broadcast;
    if (size < 12) {
        s->regs[DMA_STATUS] |= DMA_STATUS_RI | DMA_STATUS_NIS;
        ret = -1;
        goto out;
    }

    stmmac_read_desc(s, &bd, 1);
    if ((bd.ctl_stat & 0x80000000) == 0) {
        s->regs[DMA_STATUS] |= DMA_STATUS_RU | DMA_STATUS_AIS;
        ret = size;
        goto out;
    }

    cpu_physical_memory_write(bd.buffer1_addr, buf, size);

    /* Add in the 4 bytes for crc (the real hw returns length incl crc) */
    size += 4;
    bd.ctl_stat = (size << 16) | 0x300;
    stmmac_write_desc(s, &bd, 1);

    s->stats.rx_bytes += size;
    s->stats.rx++;
    if (multicast) {
        s->stats.rx_mcast++;
    } else if (broadcast) {
        s->stats.rx_bcast++;
    }

    s->regs[DMA_STATUS] |= DMA_STATUS_RI | DMA_STATUS_NIS;
    ret = size;

out:
    enet_update_irq(s);
    return ret;
}

static NetClientInfo net_stmmac_enet_info = {
    .type = NET_CLIENT_OPTIONS_KIND_NIC,
    .size = sizeof(NICState),
    .receive = eth_rx,
};

static int stmmac_enet_init(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    StmmacState *s = STMMAC(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &enet_mem_ops, s,
                          "stmmac", 0x8000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->sbd_irq);
    sysbus_init_irq(sbd, &s->pmt_irq);
    sysbus_init_irq(sbd, &s->mci_irq);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_stmmac_enet_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);

    s->regs[STMMAC_ADDR_HIGH(0)] = (s->conf.macaddr.a[5] << 8) |
                                   s->conf.macaddr.a[4];
    s->regs[STMMAC_ADDR_LOW(0)] = (s->conf.macaddr.a[3] << 24) |
                                 (s->conf.macaddr.a[2] << 16) |
                                 (s->conf.macaddr.a[1] << 8) |
                                  s->conf.macaddr.a[0];
    s->c_phyaddr = 1;
    phy_init(&s->TEMAC.phy);
    mdio_attach(&s->TEMAC.mdio_bus, &s->TEMAC.phy, s->c_phyaddr);

    s->TEMAC.parent = s;

    s->descriptor_size = 16;
    s->skip_size = 0;

    return 0;
}

static Property stmmac_properties[] = {
    DEFINE_PROP_UINT32("phyaddr", StmmacState, c_phyaddr, 1),
    DEFINE_NIC_PROPERTIES(StmmacState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void stmmac_enet_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sbc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    sbc->init = stmmac_enet_init;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "stmmac network device";
    dc->vmsd = &vmstate_stmmac;
    dc->props = stmmac_properties;
}

static const TypeInfo stmmac_enet_info = {
    .name          = TYPE_STMMAC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(StmmacState),
    .class_init    = stmmac_enet_class_init,
};

static void stmmac_enet_register_types(void)
{
    type_register_static(&stmmac_enet_info);
}

type_init(stmmac_enet_register_types)
