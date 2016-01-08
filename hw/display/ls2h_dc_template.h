/*
 * LS2H Display Controller
 *
 * Copyright (c) 2016 Lemote, LLC.
 * Written by Fuxin Zhang, modified from pl110_template.h
 *
 * This code is licensed under the GNU LGPL
 *
 * Framebuffer format conversion routines.
 */

#ifndef ORDER

#if BITS == 8
#define COPY_PIXEL(to, from) *(to++) = from
#elif BITS == 15 || BITS == 16
#define COPY_PIXEL(to, from) do { *(uint16_t *)to = from; to += 2; } while (0)
#elif BITS == 24
#define COPY_PIXEL(to, from)    \
    do {                        \
        *(to++) = from;         \
        *(to++) = (from) >> 8;  \
        *(to++) = (from) >> 16; \
    } while (0)
#elif BITS == 32
#define COPY_PIXEL(to, from) do { *(uint32_t *)to = from; to += 4; } while (0)
#else
#error unknown bit depth
#endif

#undef RGB
#define BORDER bgr
#define ORDER 0
#include "ls2h_dc_template.h"
#define ORDER 1
#include "ls2h_dc_template.h"
#define ORDER 2
#include "ls2h_dc_template.h"
#undef BORDER
#define RGB
#define BORDER rgb
#define ORDER 0
#include "ls2h_dc_template.h"
#define ORDER 1
#include "ls2h_dc_template.h"
#define ORDER 2
#include "ls2h_dc_template.h"
#undef BORDER

static drawfn glue(ls2hdc_draw_fn_,BITS)[48] =
{
    glue(ls2hdc_draw_line12_lblp_bgr,BITS),
    glue(ls2hdc_draw_line16_555_lblp_bgr,BITS),
    glue(ls2hdc_draw_line16_lblp_bgr,BITS),
    glue(ls2hdc_draw_line32_lblp_bgr,BITS),

    glue(ls2hdc_draw_line12_bbbp_bgr,BITS),
    glue(ls2hdc_draw_line16_555_bbbp_bgr,BITS),
    glue(ls2hdc_draw_line16_bbbp_bgr,BITS),
    glue(ls2hdc_draw_line32_bbbp_bgr,BITS),

    glue(ls2hdc_draw_line12_lbbp_bgr,BITS),
    glue(ls2hdc_draw_line16_555_lbbp_bgr,BITS),
    glue(ls2hdc_draw_line16_lbbp_bgr,BITS),
    glue(ls2hdc_draw_line32_lbbp_bgr,BITS),

    glue(ls2hdc_draw_line12_lblp_rgb,BITS),
    glue(ls2hdc_draw_line16_lblp_rgb,BITS),
    glue(ls2hdc_draw_line16_555_lblp_rgb,BITS),
    glue(ls2hdc_draw_line32_lblp_rgb,BITS),

    glue(ls2hdc_draw_line12_bbbp_rgb,BITS),
    glue(ls2hdc_draw_line16_555_bbbp_rgb,BITS),
    glue(ls2hdc_draw_line16_bbbp_rgb,BITS),
    glue(ls2hdc_draw_line32_bbbp_rgb,BITS),

    glue(ls2hdc_draw_line12_lbbp_rgb,BITS),
    glue(ls2hdc_draw_line16_555_lbbp_rgb,BITS),
    glue(ls2hdc_draw_line16_lbbp_rgb,BITS),
    glue(ls2hdc_draw_line32_lbbp_rgb,BITS),
};

#undef BITS
#undef COPY_PIXEL

#else

#if ORDER == 0
#define NAME glue(glue(lblp_, BORDER), BITS)
#ifdef HOST_WORDS_BIGENDIAN
#define SWAP_WORDS 1
#endif
#elif ORDER == 1
#define NAME glue(glue(bbbp_, BORDER), BITS)
#ifndef HOST_WORDS_BIGENDIAN
#define SWAP_WORDS 1
#endif
#else
#define SWAP_PIXELS 1
#define NAME glue(glue(lbbp_, BORDER), BITS)
#ifdef HOST_WORDS_BIGENDIAN
#define SWAP_WORDS 1
#endif
#endif

#define FN_2(x, y) FN(x, y) FN(x+1, y)
#define FN_4(x, y) FN_2(x, y) FN_2(x+2, y)
#define FN_8(y) FN_4(0, y) FN_4(4, y)

static void glue(ls2hdc_draw_line12_,NAME)(void *opaque, uint8_t *d, const uint8_t *src, int width, int deststep)
{
    /* RGB 444 with 4 bits of zeroes at the top of each halfword */
    uint32_t data;
    unsigned int r, g, b;
    while (width > 0) {
        data = *(uint32_t *)src;
#ifdef SWAP_WORDS
        data = bswap32(data);
#endif
#ifdef RGB
#define LSB r
#define MSB b
#else
#define LSB b
#define MSB r
#endif
        LSB = (data & 0xf) << 4;
        data >>= 4;
        g = (data & 0xf) << 4;
        data >>= 4;
        MSB = (data & 0xf) << 4;
        data >>= 8;
        COPY_PIXEL(d, glue(rgb_to_pixel,BITS)(r, g, b));
        LSB = (data & 0xf) << 4;
        data >>= 4;
        g = (data & 0xf) << 4;
        data >>= 4;
        MSB = (data & 0xf) << 4;
        data >>= 8;
        COPY_PIXEL(d, glue(rgb_to_pixel,BITS)(r, g, b));
#undef MSB
#undef LSB
        width -= 2;
        src += 4;
    }
}

static void glue(ls2hdc_draw_line16_555_,NAME)(void *opaque, uint8_t *d, const uint8_t *src, int width, int deststep)
{
    /* RGB 555 plus an intensity bit (which we ignore) */
    uint32_t data;
    unsigned int r, g, b;
    while (width > 0) {
        data = *(uint32_t *)src;
#ifdef SWAP_WORDS
        data = bswap32(data);
#endif
#ifdef RGB
#define LSB r
#define MSB b
#else
#define LSB b
#define MSB r
#endif
        LSB = (data & 0x1f) << 3;
        data >>= 5;
        g = (data & 0x1f) << 3;
        data >>= 5;
        MSB = (data & 0x1f) << 3;
        data >>= 5;
        COPY_PIXEL(d, glue(rgb_to_pixel,BITS)(r, g, b));
        LSB = (data & 0x1f) << 3;
        data >>= 5;
        g = (data & 0x1f) << 3;
        data >>= 5;
        MSB = (data & 0x1f) << 3;
        data >>= 6;
        COPY_PIXEL(d, glue(rgb_to_pixel,BITS)(r, g, b));
#undef MSB
#undef LSB
        width -= 2;
        src += 4;
    }
}

static void glue(ls2hdc_draw_line16_,NAME)(void *opaque, uint8_t *d, const uint8_t *src, int width, int deststep)
{
    uint32_t data;
    unsigned int r, g, b;
    while (width > 0) {
        data = *(uint32_t *)src;
#ifdef SWAP_WORDS
        data = bswap32(data);
#endif
#ifdef RGB
#define LSB r
#define MSB b
#else
#define LSB b
#define MSB r
#endif
#if 0
        LSB = data & 0x1f;
        data >>= 5;
        g = data & 0x3f;
        data >>= 6;
        MSB = data & 0x1f;
        data >>= 5;
#else
        LSB = (data & 0x1f) << 3;
        data >>= 5;
        g = (data & 0x3f) << 2;
        data >>= 6;
        MSB = (data & 0x1f) << 3;
        data >>= 5;
#endif
        COPY_PIXEL(d, glue(rgb_to_pixel,BITS)(r, g, b));
        LSB = (data & 0x1f) << 3;
        data >>= 5;
        g = (data & 0x3f) << 2;
        data >>= 6;
        MSB = (data & 0x1f) << 3;
        data >>= 5;
        COPY_PIXEL(d, glue(rgb_to_pixel,BITS)(r, g, b));
#undef MSB
#undef LSB
        width -= 2;
        src += 4;
    }
}

static void glue(ls2hdc_draw_line32_,NAME)(void *opaque, uint8_t *d, const uint8_t *src, int width, int deststep)
{
    uint32_t data;
    unsigned int r, g, b;
    while (width > 0) {
        data = *(uint32_t *)src;
#ifdef RGB
#define LSB r
#define MSB b
#else
#define LSB b
#define MSB r
#endif
#ifndef SWAP_WORDS
        LSB = data & 0xff;
        g = (data >> 8) & 0xff;
        MSB = (data >> 16) & 0xff;
#else
        LSB = (data >> 24) & 0xff;
        g = (data >> 16) & 0xff;
        MSB = (data >> 8) & 0xff;
#endif
        COPY_PIXEL(d, glue(rgb_to_pixel,BITS)(r, g, b));
#undef MSB
#undef LSB
        width--;
        src += 4;
    }
}



#undef SWAP_PIXELS
#undef NAME
#undef SWAP_WORDS
#undef ORDER

#endif
