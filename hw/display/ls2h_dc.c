/*
 * Arm PrimeCell LS2H Color LCD Controller
 *
 * Copyright (c) 2005-2009 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GNU LGPL
 */

#include "hw/sysbus.h"
#include "ui/console.h"
#include "framebuffer.h"
#include "ui/pixel_ops.h"

enum ls2hdc_bppmode
{
    BPP_12 = 0,
    BPP_16_565,
    BPP_16,
    BPP_32,
};

#define TYPE_LS2H_DC "ls2h-dc"
#define LS2H_DC(obj) OBJECT_CHECK(LS2HDCState, (obj), TYPE_LS2H_DC)

typedef struct LS2HDCState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegionSection fbsection;
    QemuConsole *con;
    
    int version;

    uint32_t fb_config;
    uint32_t fb_base0;
    uint32_t fb_base1;
    uint32_t stride;
    uint32_t origin;
    uint32_t cj_config;
    uint32_t cj_lut_lo;
    uint32_t cj_lut_hi;
    uint32_t panel_config;
    uint32_t width;
    uint32_t hsync;
    uint32_t height;
    uint32_t vsync;
    uint32_t gamma_index;
    uint32_t gamma_value;
    uint32_t cursor_config;
    uint32_t cursor_addr;
    uint32_t cursor_pos;
    uint32_t cursor_fgcolor;
    uint32_t cursor_bgcolor;
    uint32_t interrupt;
    uint32_t dac_ctrl;
    uint32_t dac_sense;
    uint32_t dac_sense_config;
    uint32_t dvo_config;

    int cols;
    int rows;
    enum ls2hdc_bppmode bpp;
    int invalidate;
    uint32_t palette[256];
    qemu_irq irq;
} LS2HDCState;

static int vmstate_ls2hdc_post_load(void *opaque, int version_id);

static const VMStateDescription vmstate_ls2hdc = {
    .name = "ls2h_dc",
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = vmstate_ls2hdc_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(version, LS2HDCState),
        VMSTATE_UINT32(fb_config, LS2HDCState),
        VMSTATE_UINT32(fb_base0, LS2HDCState),
        VMSTATE_UINT32(fb_base1, LS2HDCState),
        VMSTATE_UINT32(interrupt, LS2HDCState),
        VMSTATE_INT32(cols, LS2HDCState),
        VMSTATE_INT32(rows, LS2HDCState),
        VMSTATE_UINT32(bpp, LS2HDCState),
        VMSTATE_INT32(invalidate, LS2HDCState),
        VMSTATE_UINT32_ARRAY(palette, LS2HDCState, 256),
        VMSTATE_END_OF_LIST()
    }
};

#define BITS 8
#include "ls2h_dc_template.h"
#define BITS 15
#include "ls2h_dc_template.h"
#define BITS 16
#include "ls2h_dc_template.h"
#define BITS 24
#include "ls2h_dc_template.h"
#define BITS 32
#include "ls2h_dc_template.h"

static int ls2hdc_enabled(LS2HDCState *s)
{
  return (s->fb_config & (1<<8));
}

static void ls2hdc_update_display(void *opaque)
{
    LS2HDCState *s = LS2H_DC(opaque);
    SysBusDevice *sbd;
    DisplaySurface *surface = qemu_console_surface(s->con);
    drawfn* fntable;
    drawfn fn;
    int dest_width;
    int src_width;
    int bpp_offset;
    int first;
    int last;

    if (!ls2hdc_enabled(s)) {
        return;
    }

    sbd = SYS_BUS_DEVICE(s);

    switch (surface_bits_per_pixel(surface)) {
    case 0:
        return;
    case 8:
        fntable = ls2hdc_draw_fn_8;
        dest_width = 1;
        break;
    case 15:
        fntable = ls2hdc_draw_fn_15;
        dest_width = 2;
        break;
    case 16:
        fntable = ls2hdc_draw_fn_16;
        dest_width = 2;
        break;
    case 24:
        fntable = ls2hdc_draw_fn_24;
        dest_width = 3;
        break;
    case 32:
        fntable = ls2hdc_draw_fn_32;
        dest_width = 4;
        break;
    default:
        fprintf(stderr, "ls2hdc: Bad color depth\n");
        exit(1);
    }
    if (s->dvo_config & (1<<9)) /* BGR */
        bpp_offset = 12;
    else
        bpp_offset = 0;

    fn = fntable[s->bpp + bpp_offset];

    src_width = s->cols;
    switch (s->bpp) {
    case BPP_16:
    case BPP_16_565:
    case BPP_12:
        src_width <<= 1;
        break;
    case BPP_32:
        src_width <<= 2;
        break;
    }
    dest_width *= s->cols;
    first = 0;
    if (s->invalidate) {
        framebuffer_update_memory_section(&s->fbsection,
                                          sysbus_address_space(sbd),
                                          s->fb_base0,
                                          s->rows, src_width);
    }

    framebuffer_update_display(surface, &s->fbsection,
                               s->cols, s->rows,
                               src_width, dest_width, 0,
                               s->invalidate,
                               fn, s->palette,
                               &first, &last);

    if (first >= 0) {
        dpy_gfx_update(s->con, 0, first, s->cols, last - first + 1);
    }
    s->invalidate = 0;
}

static void ls2hdc_invalidate_display(void * opaque)
{
    LS2HDCState *s = LS2H_DC(opaque);
    s->invalidate = 1;
    if (ls2hdc_enabled(s)) {
        qemu_console_resize(s->con, s->cols, s->rows);
    }
}

static void ls2hdc_init_palette(LS2HDCState *s)
{
    DisplaySurface *surface = qemu_console_surface(s->con);
    int i;
    unsigned int r, g, b;

    for (i = 0; i < 256; i++) {
        r = i;
        g = i;
        b = i;

        switch (surface_bits_per_pixel(surface)) {
        case 8:
            s->palette[i] = rgb_to_pixel8(r, g, b);
            break;
        case 15:
            s->palette[i] = rgb_to_pixel15(r, g, b);
            break;
        case 16:
            s->palette[i] = rgb_to_pixel16(r, g, b);
            break;
        case 24:
        case 32:
            s->palette[i] = rgb_to_pixel32(r, g, b);
            break;
        }
    }
}

#if 0
static void ls2hdc_resize(LS2HDCState *s, int width, int height)
{
    if (width != s->cols || height != s->rows) {
        if (ls2hdc_enabled(s)) {
            qemu_console_resize(s->con, width, height);
        }
    }
    s->cols = width;
    s->rows = height;
}

/* Update interrupts.  */
static void ls2hdc_update(LS2HDCState *s)
{
  /* TODO: Implement interrupts.  */
}
#endif

static uint64_t ls2hdc_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    LS2HDCState *s = LS2H_DC(opaque);

    //fprintf(stderr,"dc_read %lx\n", offset);

    switch (offset - 0x10) {
    case 0x1240: /* fb_config*/
        return s->fb_config;
    case 0x1260: /* fb_base0 */
        return s->fb_base0;
    case 0x1580: /* fb_base1 */
        return s->fb_base1;
    case 0x1280: /* stride */
        return s->stride;
    case 0x1300: /* origin */
        return s->origin;
    case 0x1360: /* color jit config */
        return s->cj_config;
    case 0x1380: /* color jit lookup table low */
        return s->cj_lut_lo;
    case 0x13a0: /* color jit lookup table high */
        return s->cj_lut_hi;
    case 0x13c0: /* panel config */
        return s->panel_config;
    case 0x1400: /* width */
        return s->width;
    case 0x1420: /* hsync */
        return s->hsync;
    case 0x1480: /* height */
        return s->height;
    case 0x14a0: /* vsync */
        return s->vsync;
    case 0x14e0: /* gamma index */
        return s->gamma_index;
    case 0x1500: /* gamma value */
        return s->gamma_value;
    case 0x1520: /* cursor config */
        return s->cursor_config;
    case 0x1530: /* cursor addr */
        return s->cursor_addr;
    case 0x1540: /* cursor position */
        return s->cursor_pos;
    case 0x1550: /* cursor bg color*/
        return s->cursor_bgcolor;
    case 0x1560: /* cursor fg color*/
        return s->cursor_fgcolor;
    case 0x1570: /* interrupt*/
        return s->interrupt;
    case 0x1600: /* DAC control*/
        return s->dac_ctrl;
    case 0x1610: /* DAC sense */
        return s->dac_sense;
    case 0x1620: /* DAC sense config */
        return s->dac_sense_config;
    case 0x1630: /* DVO config*/
        return s->dvo_config;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ls2hdc_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void ls2hdc_write(void *opaque, hwaddr offset,
                        uint64_t val, unsigned size)
{
    LS2HDCState *s = LS2H_DC(opaque);

    //fprintf(stderr,"dc_write %lx val %lx\n", offset, val);
    /* For simplicity invalidate the display whenever a control register
       is written to.  */
    s->invalidate = 1;
    switch (offset - 0x10) {
    case 0x1240: /* fb_config*/
        s->fb_config = val;
        s->bpp = (val & 0x7) - 1;
        //fprintf(stderr, "bpp set to %d\n", s->bpp);
        if (val & (1<<8)) 
            qemu_console_resize(s->con, s->cols, s->rows);
        break;
    case 0x1260: /* fb_base0 */
        s->fb_base0 = val;
        //fprintf(stderr, "base0 set to %x\n", s->fb_base0);
        break;
    case 0x1580: /* fb_base1 */
        s->fb_base1 = val;
        //fprintf(stderr, "base1 set to %x\n", s->fb_base1);
        break;
    case 0x1280: /* stride */
        s->stride = val;
        break;
    case 0x1300: /* origin */
        s->origin = val;
        break;
    case 0x1360: /* color jit config */
        s->cj_config = val;
        break;
    case 0x1380: /* color jit lookup table low */
        s->cj_lut_lo = val;
        break;
    case 0x13a0: /* color jit lookup table high */
        s->cj_lut_hi = val;
        break;
    case 0x13c0: /* panel config */
        s->panel_config = val;
        break;
    case 0x1400: /* width */
        s->width = val;
        s->cols = val & 0x7ff;
        //fprintf(stderr, "cols set to %d\n", s->cols);
        break;
    case 0x1420: /* hsync */
        s->hsync = val;
        break;
    case 0x1480: /* height */
        s->height = val;
        s->rows = val & 0x7ff;
        //fprintf(stderr, "rows set to %d\n", s->rows);
        break;
    case 0x14a0: /* vsync */
        s->vsync = val;
        break;
    case 0x14e0: /* gamma index */
        s->gamma_index = val;
        break;
    case 0x1500: /* gamma value */
        s->gamma_value = val;
        break;
    case 0x1520: /* cursor config */
        s->cursor_config = val;
        break;
    case 0x1530: /* cursor addr */
        s->cursor_addr = val;
        break;
    case 0x1540: /* cursor position */
        s->cursor_pos = val;
        break;
    case 0x1550: /* cursor bg color*/
        s->cursor_bgcolor = val;
        break;
    case 0x1560: /* cursor fg color*/
        s->cursor_fgcolor = val;
        break;
    case 0x1570: /* interrupt*/
        s->interrupt = val;
        break;
    case 0x1600: /* DAC control*/
        s->dac_ctrl = val;
        break;
    case 0x1610: /* DAC sense */
        s->dac_sense = val;
        break;
    case 0x1620: /* DAC sense config */
        s->dac_sense_config = val;
        break;
    case 0x1630: /* DVO config*/
        s->dvo_config = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ls2hdc_write: Bad offset %x\n", (int)offset);
    }
}

static const MemoryRegionOps ls2hdc_ops = {
    .read = ls2hdc_read,
    .write = ls2hdc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int vmstate_ls2hdc_post_load(void *opaque, int version_id)
{
    LS2HDCState *s = opaque;
    /* Make sure we redraw, and at the right size */
    ls2hdc_invalidate_display(s);
    return 0;
}

static const GraphicHwOps ls2hdc_gfx_ops = {
    .invalidate  = ls2hdc_invalidate_display,
    .gfx_update  = ls2hdc_update_display,
};

static int ls2hdc_initfn(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    LS2HDCState *s = LS2H_DC(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ls2hdc_ops, s, "ls2h-dc", 0x8000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->con = graphic_console_init(dev, 0, &ls2hdc_gfx_ops, s);
    ls2hdc_init_palette(s);
    return 0;
}

static void ls2hdc_init(Object *obj)
{
    LS2HDCState *s = LS2H_DC(obj);

    s->fb_config = 0;
    s->version = 0;
}

static void ls2hdc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls2hdc_initfn;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    dc->vmsd = &vmstate_ls2hdc;
}

static const TypeInfo ls2hdc_info = {
    .name          = TYPE_LS2H_DC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LS2HDCState),
    .instance_init = ls2hdc_init,
    .class_init    = ls2hdc_class_init,
};

static void ls2hdc_register_types(void)
{
    type_register_static(&ls2hdc_info);
}

type_init(ls2hdc_register_types)
