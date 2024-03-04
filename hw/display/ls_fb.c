/*
 * QEMU loongson framebuffer emulation
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
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "sysemu/dma.h"
#include "framebuffer.h"
#define GPIO_PRINTF(...)

#define DC_REG(d, addr) ((d)->reg[(addr - 0x1240) / 4])
struct ls_fb_states;
typedef struct ls_fb_state {
        struct ls_fb_states *fb;
        MemoryRegionSection fbsection;
        QemuConsole *con;
        ram_addr_t vram_offset, vram_offset1;
        ram_addr_t vram_hoffset, vram_hoffset1;
        int head;
        int width;
        int height;
        int stride;
        int invalidate;
        int depth;
        int bypp;
        int enable;
        int config;
        int need_update;
        int index;
        int switch_panel;
        int syncing;
        void *type;
} ls_fb_state ;

typedef struct ls_fb_states {
        SysBusDevice busdev;
        MemoryRegion iomem;
        qemu_irq irq;

        int vram_size;
        struct ls_fb_state fb_con[2];

        union {
                MemoryRegion *root;
                void *root_ptr;
        };

        int irqstat;
        int reg[(0x1670 - 0x1240) / 4];
        qemu_irq gpio_out[4];
        uint32_t ilevel, olevel, oconf;
} ls_fb_states;


#define BITS 8
#include "ls_fb_template.h"
#define BITS 15
#include "ls_fb_template.h"
#define BITS 16
#include "ls_fb_template.h"
#define BITS 24
#include "ls_fb_template.h"
#define BITS 32
#include "ls_fb_template.h"


static void ls_fb_gpio_check(ls_fb_states *d);
static void ls_fb_raise_irq(ls_fb_state *s);
static inline void ls_fb_size(ls_fb_state *s)
{
        int idx = s - s->fb->fb_con;
        QemuConsole *con = s->switch_panel ? s->fb->fb_con[1 - idx].con :
        s->con;
        s->invalidate = 1;
        qemu_console_resize(con, s->width, s->height);
}

static void ls_fb_invalidate_screen(void *opaque)
{
        ls_fb_state *s = opaque;
        int idx = s - s->fb->fb_con;
        QemuConsole *con = s->switch_panel ? s->fb->fb_con[1 - idx].con :
        s->con;
        s->invalidate = 1;
        qemu_console_resize(con, s->width, s->height);
}

static void ls_fb_update_screen(void *opaque)
{
        ls_fb_state *s = opaque;
        int idx = s - s->fb->fb_con;
        QemuConsole *con = s->switch_panel ? s->fb->fb_con[1 - idx].con :
        s->con;
        DisplaySurface *surface = qemu_console_surface(con);
        ram_addr_t vram_offset;
        int first = 0;
        int last = 0;
        drawfn fn;
        int dest_width;
        if (!s->enable) {
                return ;
        }

        dest_width = s->width;

        switch (surface_bits_per_pixel(surface)) {
        case 0:
                return;
        case 8:
                fn = draw_line_8;
                break;
        case 15:
                fn = draw_line_15;
                dest_width *= 2;
                break;
        case 16:
                fn = draw_line_16;
                dest_width *= 2;
                break;
        case 24:
                fn = draw_line_24;
                dest_width *= 3;
                break;
        case 32:
                fn = draw_line_32;
                dest_width *= 4;
                break;
        default:
                hw_error("milkymist_vgafb: bad color depth\n");
                break;
        }

        vram_offset = s->index ? (s->vram_hoffset1 << 32) +
                s->vram_offset1 : (s->vram_hoffset << 32) + s->vram_offset;

        if (s->invalidate) {
                framebuffer_update_memory_section(&s->fbsection,
                                                  s->fb->root,
                                                  vram_offset,
                                                  s->width, s->width * s->bypp);
        }

        framebuffer_update_display(surface, &s->fbsection, s->width, s->height,
                                   ((s->width * s->bypp) & 255) ==
                                   0 ? s->stride : (s->width * s->bypp),
                                   dest_width, 0, 1, fn, s, &first, &last);

        if (first >= 0) {
                dpy_gfx_update(con, 0, first, s->width, last - first + 1);
        }
        s->invalidate = 0;
        ls_fb_raise_irq(s);
}

static void ls_fb_reset(ls_fb_state *s)
{
        int width = 800, height = 600, depth = 16;
        unsigned int fb_offset = 0;

        if (getenv("SIMPLEVGA")) {
                if (sscanf(getenv("SIMPLEVGA"), "%dx%d-%d:0x%x", &width,
                &height, &depth, &fb_offset) < 3) {
                        printf("usage: SIMPLEVGA=800x600-16:0xc3000\n");
                        exit(1);
                }
        }
        s->index = 0;
        s->need_update = 1;
        s->enable = 0;
        s->config = 0;
        s->width = width;
        s->height = height;
        s->depth = depth;
        s->stride = width * height * depth / 8;

        s->vram_offset = fb_offset;
        s->vram_hoffset = 0;
        s->vram_offset1 = fb_offset;
        s->vram_hoffset1 = 0;
        s->bypp = (s->depth + 7) >> 3;
        s->syncing = 0;
        ls_fb_size(s);
}


#define LS_DC_REG_BASE                   0

#define LS_FB_CFG_DVO_REG                (LS_DC_REG_BASE + 0x1240)
#define LS_FB_CFG_VGA_REG                (LS_DC_REG_BASE + 0x1250)
#define LS_FB_ADDR0_DVO_REG              (LS_DC_REG_BASE + 0x1260)
#define LS_FB_ADDR0_VGA_REG              (LS_DC_REG_BASE + 0x1270)
#define LS_FB_STRI_DVO_REG               (LS_DC_REG_BASE + 0x1280)
#define LS_FB_STRI_VGA_REG               (LS_DC_REG_BASE + 0x1290)
#define LS_FB_HDISPLAY_DVO_REG           (LS_DC_REG_BASE + 0x1400)
#define LS_FB_HDISPLAY_VGA_REG           (LS_DC_REG_BASE + 0x1410)
#define LS_FB_VDISPLAY_DVO_REG           (LS_DC_REG_BASE + 0x1480)
#define LS_FB_VDISPLAY_VGA_REG           (LS_DC_REG_BASE + 0x1490)
#define LS_FB_ADDR1_DVO_REG              (LS_DC_REG_BASE + 0x1580)
#define LS_FB_ADDR1_VGA_REG              (LS_DC_REG_BASE + 0x1590)
#define LS_FB_INT_REG                    (LS_DC_REG_BASE + 0x1570)
#define LS_FB_CUR_CFG_REG                (LS_DC_REG_BASE + 0x1520)
#define LS_FB_CUR_ADDR_REG               (LS_DC_REG_BASE + 0x1530)
#define LS_FB_CUR_LOC_ADDR_REG           (LS_DC_REG_BASE + 0x1540)
#define LS_FB_CUR_BACK_REG               (LS_DC_REG_BASE + 0x1550)
#define LS_FB_CUR_FORE_REG               (LS_DC_REG_BASE + 0x1560)
#define LS_FB_HADDR0_DVO_REG             (LS_DC_REG_BASE + 0x15a0)
#define LS_FB_HADDR1_DVO_REG              (LS_DC_REG_BASE + 0x15c0)
#define LS_FB_GPIO_INOUT                (LS_DC_REG_BASE + 0x1650)
#define LS_FB_GPIO_CONF                 (LS_DC_REG_BASE + 0x1660)


#define LSDC_INT_FB1_VSYNC             0
#define LSDC_INT_FB1_HSYNC             1
#define LSDC_INT_FB0_VSYNC             2
#define LSDC_INT_FB0_HSYNC             3
#define LSDC_INT_CURSOR_END            4
#define LSDC_INT_FB1_END               5
#define LSDC_INT_FB0_END               6

static void ls_fb_check_irq(ls_fb_states *d)
{
        uint32_t irqreg = DC_REG(d, LS_FB_INT_REG) ;
        if ((irqreg >> 16) & d->irqstat) {
                qemu_irq_raise(d->irq);
        } else {
                qemu_irq_lower(d->irq);
        }
}

static void ls_fb_raise_irq(ls_fb_state *s)
{
        ls_fb_states *d = s->fb;
        int idx = s - d->fb_con;

        if (idx) {
                d->irqstat |= 1 << LSDC_INT_FB1_VSYNC;
        } else {
                d->irqstat |= 1 << LSDC_INT_FB0_VSYNC;
        }
#ifndef CONFIG_SDL
        /*vnc only support one console*/
        d->irqstat |= (1 << LSDC_INT_FB0_VSYNC)  | (1 << LSDC_INT_FB1_VSYNC);
#endif
        ls_fb_check_irq(d);
}

/*
add gpio i2c
*/

static void ls_fb_writel(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
        ls_fb_states *d = opaque;
        ls_fb_state *s;
        if (addr >= 0x1240 && (addr - 0x1240) < sizeof(d->reg)) {
                DC_REG(d, addr) = val;
        }

        s = (addr & 0x10) ? &d->fb_con[1] : &d->fb_con[0];
        switch (addr) {
        case LS_FB_INT_REG:
                d->irqstat = 0;
                ls_fb_check_irq(d);
                break;
        case LS_FB_CFG_DVO_REG:
        case LS_FB_CFG_VGA_REG:
                s->enable = (val >> 8) & 1;
                s->switch_panel = (val >> 9) & 1;
                if (val & 0x80) {
                        s->index ^= 1;
                }
                DC_REG(d, addr) = (val & ~0x880) | (s->index << 11);
                if (s->enable) {
                        switch (val & 7) {
                        case 4:
                                s->depth = 32;
                                break;
                        case 3:
                                s->depth = 16;
                                break;
                        case 2:
                                s->depth = 15;
                                break;
                        default:
                                s->depth = 16;
                                break;
                        }
                        s->bypp = (s->depth + 7) >> 3;
                        s->need_update = 1;
                        ls_fb_size(s);
                }
                break;
        case LS_FB_HDISPLAY_DVO_REG:
        case LS_FB_HDISPLAY_VGA_REG:
                s->width = val & 0xffff;
                s->need_update = 1;
                ls_fb_size(s);
                break;
        case LS_FB_VDISPLAY_DVO_REG:
        case LS_FB_VDISPLAY_VGA_REG:
                s->height = val & 0xffff;
                s->need_update = 1;
                ls_fb_size(s);
                break;

        case LS_FB_ADDR0_DVO_REG:
        case LS_FB_ADDR0_VGA_REG:
                s->vram_offset = val;
                s->need_update = 1;
                ls_fb_size(s);
                break;

        case LS_FB_ADDR1_DVO_REG:
        case LS_FB_ADDR1_VGA_REG:
                s->vram_offset1 = val;
                s->need_update = 1;
                ls_fb_size(s);
                break;
        case LS_FB_HADDR0_DVO_REG:
                s->vram_hoffset = val;
                s->need_update = 1;
                ls_fb_size(s);
                break;
        case LS_FB_HADDR1_DVO_REG:
                s->vram_hoffset1 = val;
                s->need_update = 1;
                ls_fb_size(s);
                break;
        case LS_FB_STRI_DVO_REG:
        case LS_FB_STRI_VGA_REG:
                s->stride = val;
                ls_fb_size(s);
                break;
        case LS_FB_GPIO_INOUT ... LS_FB_GPIO_INOUT + 7:
                ls_fb_gpio_check(d);
                break;
        case LS_FB_GPIO_CONF ... LS_FB_GPIO_CONF + 7:
                d->ilevel |= ((d->oconf ^ DC_REG(d, LS_FB_GPIO_CONF)) & 0xf) &
                        DC_REG(d, LS_FB_GPIO_CONF);
                d->oconf = DC_REG(d, LS_FB_GPIO_CONF);
                ls_fb_gpio_check(d);
                break;
        }

#ifdef DEBUGPC
        printf("%s 0x%llx 0x%llx pc 0x%llx\n", __func__, (long long)addr,
               (long long) val, (long long)mypc);
        MIPSCPU *cpu = MIPS_CPU(current_cpu);
        CPUMIPSState *env = &cpu->env;
        env->active_tc.PC = mypc;
        do_raise_exception_err(env, EXCP_DEBUG, 0, GETPC());
#endif
}

static uint64_t ls_fb_readl(void *opaque, hwaddr addr, unsigned size)
{
        ls_fb_states *d = opaque;
        if (addr >= 0x1240 && (addr - 0x1240) < sizeof(d->reg)) {
                switch (addr) {
                case LS_FB_INT_REG:
                        return (DC_REG(d, addr) & 0xffff0000) |
                        d->irqstat;
                        break;
                case LS_FB_GPIO_INOUT:
                        ls_fb_gpio_check(d);

                        return d->olevel;
                        break;
                default:
                        return  DC_REG(d, addr);
                }
        }
        return 0;
}

static void ls_fb_gpio_check(ls_fb_states *d)
{
    uint32_t level, diff;
    int bit;
    level = (DC_REG(d, LS_FB_GPIO_CONF) & d->ilevel) | (~DC_REG(d,
    LS_FB_GPIO_CONF) & DC_REG(d, LS_FB_GPIO_INOUT));

    for (diff = (d->olevel ^ level) & 0xf; diff; diff ^= 1 << bit) {
        bit = ctz32(diff);
        GPIO_PRINTF("gpio %d set %d\n", bit, (level >> bit) & 1);
        qemu_set_irq(d->gpio_out[bit], (level >> bit) & 1);
    }

    level = (DC_REG(d, LS_FB_GPIO_CONF) & d->ilevel) | (~DC_REG(d,
    LS_FB_GPIO_CONF) & DC_REG(d, LS_FB_GPIO_INOUT));
    d->olevel = level;
}

static void ls_fb_gpio_set(void *opaque, int line, int level)
{
        ls_fb_states *d = opaque;
        GPIO_PRINTF("gpio %d : level %d\n", line, level);

        if (level) {
                d->ilevel |= 1 << line;
        } else {
                d->ilevel &= ~(1 << line);
        }
}

static const MemoryRegionOps ls_fb_ops = {
        .read = ls_fb_readl,
        .write = ls_fb_writel,
        .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LSFB "ls_fb"

#define SYS_BUS_LSFB(obj)                                             \
        OBJECT_CHECK(ls_fb_states, (obj), TYPE_SYS_BUS_LSFB)

static const GraphicHwOps lsfb_fb_ops = {
        .invalidate  = ls_fb_invalidate_screen,
        .gfx_update  = ls_fb_update_screen,
};

static int ls_fb_sysbus_init(SysBusDevice *dev)
{

        ls_fb_states *d = SYS_BUS_LSFB(dev);

        memory_region_init_io(&d->iomem, NULL, &ls_fb_ops, (void *)d,
                              "ls fb", 0x10000);

        sysbus_init_mmio(dev, &d->iomem);
        d->vram_size = 0;
        d->root = sysbus_address_space(dev);

        d->fb_con[0].con = graphic_console_init(DEVICE(dev), 0, &lsfb_fb_ops,
                                                &d->fb_con[0]);
        d->fb_con[1].con = graphic_console_init(DEVICE(dev), 1, &lsfb_fb_ops,
                                                &d->fb_con[1]);
        d->fb_con[0].fb = d;
        d->fb_con[1].fb = d;
        ls_fb_reset(&d->fb_con[0]);
        ls_fb_reset(&d->fb_con[1]);
        sysbus_init_irq(dev, &d->irq);

        qdev_init_gpio_in(DEVICE(dev), ls_fb_gpio_set, 4);
        qdev_init_gpio_out(DEVICE(dev), d->gpio_out, 4);
        d->ilevel = 0xf;
        d->olevel = 0xf;
        DC_REG(d, LS_FB_GPIO_CONF) = 0xf;
        DC_REG(d, LS_FB_GPIO_INOUT) = 0xf;
        return 0;
}


static Property ls_fb_properties[] = {
        DEFINE_PROP_PTR("root", ls_fb_states, root_ptr),
        DEFINE_PROP_END_OF_LIST(),
};

static void ls_fb_sysbus_class_init(ObjectClass *klass, void *data)
{
        DeviceClass *dc = DEVICE_CLASS(klass);
        SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

        k->init = ls_fb_sysbus_init;
        dc->desc = "ls fb";
        dc->props = ls_fb_properties;
}

static const TypeInfo ls_fb_sysbus_info = {
        .name          = "ls_fb",
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ls_fb_states),
        .class_init    = ls_fb_sysbus_class_init,
};


static void ls_fb_sysbus_register_types(void)
{
        type_register_static(&ls_fb_sysbus_info);
}

type_init(ls_fb_sysbus_register_types)


typedef struct ls_fb_pci_state {
        PCIDevice dev;
        ls_fb_states dc;
} ls_fb_pci_state;

/* PCI interface */
#define LS2KDC_VENDOR_ID  0x0014
#define LS2KDC_DEVICE_ID  0x7a06

static void ls_pci_fb_gpio_set(void *opaque, int line, int level)
{
        ls_fb_pci_state *d = opaque;
        ls_fb_gpio_set(&d->dc, line, level);
}

static void ls_fb_pci_init(PCIDevice *pci_dev, Error **errp)
{
        ls_fb_pci_state *d = DO_UPCAST(ls_fb_pci_state, dev, pci_dev);
        uint8_t *pci_conf;

        pci_conf = d->dev.config;

        pci_config_set_vendor_id(pci_conf, LS2KDC_VENDOR_ID);
        pci_config_set_device_id(pci_conf, LS2KDC_DEVICE_ID);
        pci_conf[0x04] = 0x07; /* command = I/O space, Bus Master */
        pci_config_set_class(pci_conf, PCI_CLASS_DISPLAY_VGA);
        pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL; /* header_type */
        pci_conf[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */
        pci_conf[0x34] = 0xdc;

        d->dc.root = pci_address_space(pci_dev);
        memory_region_init_io(&d->dc.iomem, NULL, &ls_fb_ops, (void *)&d->dc,
                              "ls fb", 0x10000);
        pci_register_bar(&d->dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY,
        &d->dc.iomem);
        d->dc.irq = pci_allocate_irq(pci_dev);

        d->dc.vram_size = 0;

        d->dc.fb_con[0].con = graphic_console_init(DEVICE(pci_dev), 0,
        &lsfb_fb_ops, &d->dc.fb_con[0]);
        d->dc.fb_con[1].con = graphic_console_init(DEVICE(pci_dev), 1,
        &lsfb_fb_ops,
                              &d->dc.fb_con[1]);
        d->dc.fb_con[0].fb = &d->dc;
        d->dc.fb_con[1].fb = &d->dc;
        ls_fb_reset(&d->dc.fb_con[0]);
        ls_fb_reset(&d->dc.fb_con[1]);

        qdev_init_gpio_in(DEVICE(pci_dev), ls_pci_fb_gpio_set, 4);
        qdev_init_gpio_out(DEVICE(pci_dev), d->dc.gpio_out, 4);
        d->dc.ilevel = 0xf;
        d->dc.olevel = 0xf;
        DC_REG(&d->dc, LS_FB_GPIO_CONF) = 0xf;
        DC_REG(&d->dc, LS_FB_GPIO_INOUT) = 0xf;
}


static Property ls_fb_pci_properties[] = {
        DEFINE_PROP_PTR("root", ls_fb_pci_state, dc.root_ptr),
        DEFINE_PROP_END_OF_LIST(),
};

static void ls_fb_pci_class_init(ObjectClass *klass, void *data)
{
        DeviceClass *dc = DEVICE_CLASS(klass);
        PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

        k->realize = ls_fb_pci_init;
        k->vendor_id = LS2KDC_VENDOR_ID;
        k->device_id = LS2KDC_DEVICE_ID;
        k->revision = 0x03;
        k->class_id = PCI_CLASS_DISPLAY_VGA;
        dc->desc = "ls dc pci";
        dc->props = ls_fb_pci_properties;
}

static const TypeInfo ls_fb_pci_info = {
        .name          = "pci_ls_fb",
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(ls_fb_pci_state),
        .class_init    = ls_fb_pci_class_init,
        .interfaces = (InterfaceInfo[])
        {
                { INTERFACE_PCIE_DEVICE },
                { }
        },
};

static void ls_fb_pci_register_types(void)
{
        type_register_static(&ls_fb_pci_info);
}

type_init(ls_fb_pci_register_types)
