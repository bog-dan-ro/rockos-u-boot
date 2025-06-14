// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DesignWare High-Definition Multimedia Interface (HDMI) driver
 *
 * Copyright (C) 2013-2015 Mentor Graphics Inc.
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2010, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *****************************************************************************
 * ESWIN hdmi driver
 *
 * Copyright 2024, Beijing ESWIN Computing Technology Co., Ltd.. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Authors: DengLei <denglei@eswincomputing.com>
 */

#include <linux/kernel.h>
#include <compiler.h>
#include <div64.h>
#include <drm_modes.h>
#include <edid.h>
#include <errno.h>
#include <fdtdec.h>
#include <hexdump.h>
#include <malloc.h>
#include <linux/compat.h>
#include <linux/ctype.h>
#include <linux/fb.h>
#include <linux/hdmi.h>
#include <linux/string.h>
#include "eswin_edid.h"
#include "eswin_modes.h"

#define EDID_EST_TIMINGS 16
#define EDID_STD_TIMINGS 8
#define EDID_DETAILED_TIMINGS 4
#define BIT_WORD(nr)             ((nr) / BITS_PER_LONG)
#define BITMAP_FIRST_WORD_MASK(start) (~0UL << ((start) & (BITS_PER_LONG - 1)))
#define BITMAP_LAST_WORD_MASK(nbits) (~0UL >> (-(nbits) & (BITS_PER_LONG - 1)))
#define EDID_PRODUCT_ID(e) ((e)->prod_code[0] | ((e)->prod_code[1] << 8))
#define version_greater(edid, maj, min) \
    (((edid)->version > (maj)) || \
     ((edid)->version == (maj) && (edid)->revision > (min)))

/*
 * EDID blocks out in the wild have a variety of bugs, try to collect
 * them here (note that userspace may work around broken monitors first,
 * but fixes should make their way here so that the kernel "just works"
 * on as many displays as possible).
 */

/* First detailed mode wrong, use largest 60Hz mode */
#define EDID_QUIRK_PREFER_LARGE_60      BIT(0)
/* Reported 135MHz pixel clock is too high, needs adjustment */
#define EDID_QUIRK_135_CLOCK_TOO_HIGH       BIT(1)
/* Prefer the largest mode at 75 Hz */
#define EDID_QUIRK_PREFER_LARGE_75      BIT(2)
/* Detail timing is in cm not mm */
#define EDID_QUIRK_DETAILED_IN_CM       BIT(3)
/* Detailed timing descriptors have bogus size values, so just take the
 * maximum size and use that.
 */
#define EDID_QUIRK_DETAILED_USE_MAXIMUM_SIZE    BIT(4)
/* Monitor forgot to set the first detailed is preferred bit. */
#define EDID_QUIRK_FIRST_DETAILED_PREFERRED BIT(5)
/* use +hsync +vsync for detailed mode */
#define EDID_QUIRK_DETAILED_SYNC_PP     BIT(6)
/* Force reduced-blanking timings for detailed modes */
#define EDID_QUIRK_FORCE_REDUCED_BLANKING   BIT(7)
/* Force 8bpc */
#define EDID_QUIRK_FORCE_8BPC           BIT(8)
/* Force 12bpc */
#define EDID_QUIRK_FORCE_12BPC          BIT(9)
/* Force 6bpc */
#define EDID_QUIRK_FORCE_6BPC           BIT(10)
/* Force 10bpc */
#define EDID_QUIRK_FORCE_10BPC          BIT(11)

#define EXT_VIDEO_CAPABILITY_BLOCK 0x00
#define EXT_VIDEO_DATA_BLOCK_420        0x0E
#define EXT_VIDEO_CAP_BLOCK_Y420CMDB 0x0F
#define EDID_BASIC_AUDIO        BIT(6)
#define EDID_CEA_YCRCB444       BIT(5)
#define EDID_CEA_YCRCB422       BIT(4)
#define EDID_CEA_VCDB_QS        BIT(6)

struct detailed_mode_closure {
    struct edid *edid;
    struct hdmi_edid_data *data;
    bool preferred;
    u32 quirks;
    int modes;
};

#define LEVEL_DMT   0
#define LEVEL_GTF   1
#define LEVEL_GTF2  2
#define LEVEL_CVT   3

static struct edid_quirk {
    char vendor[4];
    int product_id;
    u32 quirks;
} edid_quirk_list[] = {
    /* Acer AL1706 */
    { "ACR", 44358, EDID_QUIRK_PREFER_LARGE_60 },
    /* Acer F51 */
    { "API", 0x7602, EDID_QUIRK_PREFER_LARGE_60 },
    /* Unknown Acer */
    { "ACR", 2423, EDID_QUIRK_FIRST_DETAILED_PREFERRED },

    /* AEO model 0 reports 8 bpc, but is a 6 bpc panel */
    { "AEO", 0, EDID_QUIRK_FORCE_6BPC },

    /* Belinea 10 15 55 */
    { "MAX", 1516, EDID_QUIRK_PREFER_LARGE_60 },
    { "MAX", 0x77e, EDID_QUIRK_PREFER_LARGE_60 },

    /* Envision Peripherals, Inc. EN-7100e */
    { "EPI", 59264, EDID_QUIRK_135_CLOCK_TOO_HIGH },
    /* Envision EN2028 */
    { "EPI", 8232, EDID_QUIRK_PREFER_LARGE_60 },

    /* Funai Electronics PM36B */
    { "FCM", 13600, EDID_QUIRK_PREFER_LARGE_75 |
      EDID_QUIRK_DETAILED_IN_CM },

    /* LGD panel of HP zBook 17 G2, eDP 10 bpc, but reports unknown bpc */
    { "LGD", 764, EDID_QUIRK_FORCE_10BPC },

    /* LG Philips LCD LP154W01-A5 */
    { "LPL", 0, EDID_QUIRK_DETAILED_USE_MAXIMUM_SIZE },
    { "LPL", 0x2a00, EDID_QUIRK_DETAILED_USE_MAXIMUM_SIZE },

    /* Philips 107p5 CRT */
    { "PHL", 57364, EDID_QUIRK_FIRST_DETAILED_PREFERRED },

    /* Proview AY765C */
    { "PTS", 765, EDID_QUIRK_FIRST_DETAILED_PREFERRED },

    /* Samsung SyncMaster 205BW.  Note: irony */
    { "SAM", 541, EDID_QUIRK_DETAILED_SYNC_PP },
    /* Samsung SyncMaster 22[5-6]BW */
    { "SAM", 596, EDID_QUIRK_PREFER_LARGE_60 },
    { "SAM", 638, EDID_QUIRK_PREFER_LARGE_60 },

    /* Sony PVM-2541A does up to 12 bpc, but only reports max 8 bpc */
    { "SNY", 0x2541, EDID_QUIRK_FORCE_12BPC },

    /* ViewSonic VA2026w */
    { "VSC", 5020, EDID_QUIRK_FORCE_REDUCED_BLANKING },

    /* Medion MD 30217 PG */
    { "MED", 0x7b8, EDID_QUIRK_PREFER_LARGE_75 },

    /* Panel in Samsung NP700G7A-S01PL notebook reports 6bpc */
    { "SEC", 0xd033, EDID_QUIRK_FORCE_8BPC },

    /* Rotel RSX-1058 forwards sink's EDID but only does HDMI 1.1*/
    { "ETR", 13896, EDID_QUIRK_FORCE_8BPC },
};

/*
 * Probably taken from CEA-861 spec.
 * This table is converted from xorg's hw/xfree86/modes/xf86EdidModes.c.
 *
 * Index using the VIC.
 */
static const struct drm_display_mode edid_cea_modes[] = {
    /* 0 - dummy, VICs start at 1 */
    { },
    /* 1 - 640x480@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 25175, 640, 656,
           752, 800, 480, 490, 492, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 2 - 720x480@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 27000, 720, 736,
           798, 858, 480, 489, 495, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 3 - 720x480@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 27000, 720, 736,
           798, 858, 480, 489, 495, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 4 - 1280x720@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1280, 1390,
           1430, 1650, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 5 - 1920x1080i@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1920, 2008,
           2052, 2200, 1080, 1084, 1094, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC |
            DRM_MODE_FLAG_INTERLACE),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 6 - 720(1440)x480i@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 13500, 720, 739,
           801, 858, 480, 488, 494, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 7 - 720(1440)x480i@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 13500, 720, 739,
           801, 858, 480, 488, 494, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 8 - 720(1440)x240@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 13500, 720, 739,
           801, 858, 240, 244, 247, 262, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 9 - 720(1440)x240@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 13500, 720, 739,
           801, 858, 240, 244, 247, 262, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 10 - 2880x480i@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 2880, 2956,
           3204, 3432, 480, 488, 494, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 11 - 2880x480i@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 2880, 2956,
           3204, 3432, 480, 488, 494, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 12 - 2880x240@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 2880, 2956,
           3204, 3432, 240, 244, 247, 262, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 13 - 2880x240@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 2880, 2956,
           3204, 3432, 240, 244, 247, 262, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 14 - 1440x480@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 1440, 1472,
           1596, 1716, 480, 489, 495, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 15 - 1440x480@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 1440, 1472,
           1596, 1716, 480, 489, 495, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 16 - 1920x1080@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1920, 2008,
           2052, 2200, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 17 - 720x576@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 27000, 720, 732,
           796, 864, 576, 581, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 18 - 720x576@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 27000, 720, 732,
           796, 864, 576, 581, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 19 - 1280x720@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1280, 1720,
           1760, 1980, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 20 - 1920x1080i@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1920, 2448,
           2492, 2640, 1080, 1084, 1094, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC |
            DRM_MODE_FLAG_INTERLACE),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 21 - 720(1440)x576i@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 13500, 720, 732,
           795, 864, 576, 580, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 22 - 720(1440)x576i@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 13500, 720, 732,
           795, 864, 576, 580, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 23 - 720(1440)x288@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 13500, 720, 732,
           795, 864, 288, 290, 293, 312, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 24 - 720(1440)x288@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 13500, 720, 732,
           795, 864, 288, 290, 293, 312, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 25 - 2880x576i@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 2880, 2928,
           3180, 3456, 576, 580, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 26 - 2880x576i@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 2880, 2928,
           3180, 3456, 576, 580, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 27 - 2880x288@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 2880, 2928,
           3180, 3456, 288, 290, 293, 312, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 28 - 2880x288@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 2880, 2928,
           3180, 3456, 288, 290, 293, 312, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 29 - 1440x576@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 1440, 1464,
           1592, 1728, 576, 581, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 30 - 1440x576@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 1440, 1464,
           1592, 1728, 576, 581, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 31 - 1920x1080@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1920, 2448,
           2492, 2640, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 32 - 1920x1080@24Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1920, 2558,
           2602, 2750, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 24, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 33 - 1920x1080@25Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1920, 2448,
           2492, 2640, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 25, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 34 - 1920x1080@30Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1920, 2008,
           2052, 2200, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 30, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 35 - 2880x480@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 2880, 2944,
           3192, 3432, 480, 489, 495, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 36 - 2880x480@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 2880, 2944,
           3192, 3432, 480, 489, 495, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 37 - 2880x576@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 2880, 2928,
           3184, 3456, 576, 581, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 38 - 2880x576@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 2880, 2928,
           3184, 3456, 576, 581, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 39 - 1920x1080i@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 72000, 1920, 1952,
           2120, 2304, 1080, 1126, 1136, 1250, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 40 - 1920x1080i@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1920, 2448,
           2492, 2640, 1080, 1084, 1094, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC |
            DRM_MODE_FLAG_INTERLACE),
      .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 41 - 1280x720@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1280, 1720,
           1760, 1980, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 42 - 720x576@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 720, 732,
           796, 864, 576, 581, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 43 - 720x576@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 720, 732,
           796, 864, 576, 581, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 44 - 720(1440)x576i@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 27000, 720, 732,
           795, 864, 576, 580, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 45 - 720(1440)x576i@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 27000, 720, 732,
           795, 864, 576, 580, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 46 - 1920x1080i@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1920, 2008,
           2052, 2200, 1080, 1084, 1094, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC |
            DRM_MODE_FLAG_INTERLACE),
      .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 47 - 1280x720@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1280, 1390,
           1430, 1650, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 48 - 720x480@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 720, 736,
           798, 858, 480, 489, 495, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 49 - 720x480@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 720, 736,
           798, 858, 480, 489, 495, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 50 - 720(1440)x480i@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 27000, 720, 739,
           801, 858, 480, 488, 494, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 51 - 720(1440)x480i@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 27000, 720, 739,
           801, 858, 480, 488, 494, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 52 - 720x576@200Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 720, 732,
           796, 864, 576, 581, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 200, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 53 - 720x576@200Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 720, 732,
           796, 864, 576, 581, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 200, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 54 - 720(1440)x576i@200Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 720, 732,
           795, 864, 576, 580, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 200, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 55 - 720(1440)x576i@200Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 720, 732,
           795, 864, 576, 580, 586, 625, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 200, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 56 - 720x480@240Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 720, 736,
           798, 858, 480, 489, 495, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 240, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 57 - 720x480@240Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 720, 736,
           798, 858, 480, 489, 495, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
      .vrefresh = 240, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 58 - 720(1440)x480i@240 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 720, 739,
           801, 858, 480, 488, 494, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 240, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_4_3, },
    /* 59 - 720(1440)x480i@240 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 54000, 720, 739,
           801, 858, 480, 488, 494, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC |
            DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK),
      .vrefresh = 240, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 60 - 1280x720@24Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 59400, 1280, 3040,
           3080, 3300, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 24, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 61 - 1280x720@25Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1280, 3700,
           3740, 3960, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 25, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 62 - 1280x720@30Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1280, 3040,
           3080, 3300, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 30, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 63 - 1920x1080@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 1920, 2008,
           2052, 2200, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
     .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 64 - 1920x1080@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 1920, 2448,
           2492, 2640, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
     .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 65 - 1280x720@24Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 59400, 1280, 3040,
           3080, 3300, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 24, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 66 - 1280x720@25Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1280, 3700,
           3740, 3960, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 25, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 67 - 1280x720@30Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1280, 3040,
           3080, 3300, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 30, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 68 - 1280x720@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1280, 1720,
           1760, 1980, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 69 - 1280x720@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1280, 1390,
           1430, 1650, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 70 - 1280x720@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1280, 1720,
           1760, 1980, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 71 - 1280x720@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1280, 1390,
           1430, 1650, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 72 - 1920x1080@24Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1920, 2558,
           2602, 2750, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 24, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 73 - 1920x1080@25Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1920, 2448,
           2492, 2640, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 25, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 74 - 1920x1080@30Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1920, 2008,
           2052, 2200, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 30, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 75 - 1920x1080@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1920, 2448,
           2492, 2640, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 76 - 1920x1080@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1920, 2008,
           2052, 2200, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 77 - 1920x1080@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 1920, 2448,
           2492, 2640, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
     .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 78 - 1920x1080@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 1920, 2008,
           2052, 2200, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
     .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 79 - 1680x720@24Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 59400, 1680, 3040,
        3080, 3300, 720, 725, 730, 750, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 24, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 80 - 1680x720@25Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 59400, 1680, 2908,
        2948, 3168, 720, 725, 730, 750, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 25, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 81 - 1680x720@30Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 59400, 1680, 2380,
        2420, 2640, 720, 725, 730, 750, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 30, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 82 - 1680x720@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 82500, 1680, 1940,
        1980, 2200, 720, 725, 730, 750, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 83 - 1680x720@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 99000, 1680, 1940,
        1980, 2200, 720, 725, 730, 750, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 84 - 1680x720@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 165000, 1680, 1740,
        1780, 2000, 720, 725, 730, 825, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 85 - 1680x720@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 198000, 1680, 1740,
        1780, 2000, 720, 725, 730, 825, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 86 - 2560x1080@24Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 99000, 2560, 3558,
        3602, 3750, 1080, 1084, 1089, 1100, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 24, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 87 - 2560x1080@25Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 90000, 2560, 3008,
        3052, 3200, 1080, 1084, 1089, 1125, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 25, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 88 - 2560x1080@30Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 118800, 2560, 3328,
        3372, 3520, 1080, 1084, 1089, 1125, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 30, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 89 - 2560x1080@50Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 185625, 2560, 3108,
        3152, 3300, 1080, 1084, 1089, 1125, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 90 - 2560x1080@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 198000, 2560, 2808,
        2852, 3000, 1080, 1084, 1089, 1100, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 91 - 2560x1080@100Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 371250, 2560, 2778,
        2822, 2970, 1080, 1084, 1089, 1250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 100, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 92 - 2560x1080@120Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 495000, 2560, 3108,
        3152, 3300, 1080, 1084, 1089, 1250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 120, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 93 - 3840x2160p@24Hz 16:9 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 3840, 5116,
        5204, 5500, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 24, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 94 - 3840x2160p@25Hz 16:9 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 3840, 4896,
        4984, 5280, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 25, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 95 - 3840x2160p@30Hz 16:9 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 3840, 4016,
        4104, 4400, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 30, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 96 - 3840x2160p@50Hz 16:9 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 594000, 3840, 4896,
        4984, 5280, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 97 - 3840x2160p@60Hz 16:9 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 594000, 3840, 4016,
        4104, 4400, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
    /* 98 - 4096x2160p@24Hz 256:135 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 4096, 5116,
        5204, 5500, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 24, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_256_135, },
    /* 99 - 4096x2160p@25Hz 256:135 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 4096, 5064,
        5152, 5280, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 25, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_256_135, },
    /* 100 - 4096x2160p@30Hz 256:135 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 4096, 4184,
        4272, 4400, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 30, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_256_135, },
    /* 101 - 4096x2160p@50Hz 256:135 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 594000, 4096, 5064,
        5152, 5280, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_256_135, },
    /* 102 - 4096x2160p@60Hz 256:135 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 594000, 4096, 4184,
        4272, 4400, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_256_135, },
    /* 103 - 3840x2160p@24Hz 64:27 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 3840, 5116,
        5204, 5500, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 24, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 104 - 3840x2160p@25Hz 64:27 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 3840, 4016,
        4104, 4400, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 25, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 105 - 3840x2160p@30Hz 64:27 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 3840, 4016,
        4104, 4400, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 30, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 106 - 3840x2160p@50Hz 64:27 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 594000, 3840, 4896,
        4984, 5280, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 50, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
    /* 107 - 3840x2160p@60Hz 64:27 */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 594000, 3840, 4016,
        4104, 4400, 2160, 2168, 2178, 2250, 0,
        DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
    .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_64_27, },
};

/*
 * HDMI 1.4 4k modes. Index using the VIC.
 */
static const struct drm_display_mode edid_4k_modes[] = {
    /* 0 - dummy, VICs start at 1 */
    { },
    /* 1 - 3840x2160@30Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000,
           3840, 4016, 4104, 4400,
           2160, 2168, 2178, 2250, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 30, },
    /* 2 - 3840x2160@25Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000,
           3840, 4896, 4984, 5280,
           2160, 2168, 2178, 2250, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 25, },
    /* 3 - 3840x2160@24Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000,
           3840, 5116, 5204, 5500,
           2160, 2168, 2178, 2250, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 24, },
    /* 4 - 4096x2160@24Hz (SMPTE) */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000,
           4096, 5116, 5204, 5500,
           2160, 2168, 2178, 2250, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
      .vrefresh = 24, },
};

/*
 * Autogenerated from the DMT spec.
 * This table is copied from xfree86/modes/xf86EdidModes.c.
 */
static const struct drm_display_mode drm_dmt_modes[] = {
    /* 0x01 - 640x350@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 31500, 640, 672,
           736, 832, 350, 382, 385, 445, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x02 - 640x400@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 31500, 640, 672,
           736, 832, 400, 401, 404, 445, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x03 - 720x400@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 35500, 720, 756,
           828, 936, 400, 401, 404, 446, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x04 - 640x480@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 25175, 640, 656,
           752, 800, 480, 490, 492, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x05 - 640x480@72Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 31500, 640, 664,
           704, 832, 480, 489, 492, 520, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x06 - 640x480@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 31500, 640, 656,
           720, 840, 480, 481, 484, 500, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x07 - 640x480@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 36000, 640, 696,
           752, 832, 480, 481, 484, 509, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x08 - 800x600@56Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 36000, 800, 824,
           896, 1024, 600, 601, 603, 625, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x09 - 800x600@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 40000, 800, 840,
           968, 1056, 600, 601, 605, 628, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x0a - 800x600@72Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 50000, 800, 856,
           976, 1040, 600, 637, 643, 666, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x0b - 800x600@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 49500, 800, 816,
           896, 1056, 600, 601, 604, 625, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x0c - 800x600@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 56250, 800, 832,
           896, 1048, 600, 601, 604, 631, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x0d - 800x600@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 73250, 800, 848,
           880, 960, 600, 603, 607, 636, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x0e - 848x480@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 33750, 848, 864,
           976, 1088, 480, 486, 494, 517, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x0f - 1024x768@43Hz, interlace */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 44900, 1024, 1032,
           1208, 1264, 768, 768, 772, 817, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC |
           DRM_MODE_FLAG_INTERLACE) },
    /* 0x10 - 1024x768@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 65000, 1024, 1048,
           1184, 1344, 768, 771, 777, 806, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x11 - 1024x768@70Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 75000, 1024, 1048,
           1184, 1328, 768, 771, 777, 806, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x12 - 1024x768@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 78750, 1024, 1040,
           1136, 1312, 768, 769, 772, 800, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x13 - 1024x768@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 94500, 1024, 1072,
           1168, 1376, 768, 769, 772, 808, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x14 - 1024x768@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 115500, 1024, 1072,
           1104, 1184, 768, 771, 775, 813, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x15 - 1152x864@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 1152, 1216,
           1344, 1600, 864, 865, 868, 900, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x55 - 1280x720@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 74250, 1280, 1390,
           1430, 1650, 720, 725, 730, 750, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x16 - 1280x768@60Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 68250, 1280, 1328,
           1360, 1440, 768, 771, 778, 790, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x17 - 1280x768@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 79500, 1280, 1344,
           1472, 1664, 768, 771, 778, 798, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x18 - 1280x768@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 102250, 1280, 1360,
           1488, 1696, 768, 771, 778, 805, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x19 - 1280x768@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 117500, 1280, 1360,
           1496, 1712, 768, 771, 778, 809, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x1a - 1280x768@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 140250, 1280, 1328,
           1360, 1440, 768, 771, 778, 813, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x1b - 1280x800@60Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 71000, 1280, 1328,
           1360, 1440, 800, 803, 809, 823, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x1c - 1280x800@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 83500, 1280, 1352,
           1480, 1680, 800, 803, 809, 831, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x1d - 1280x800@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 106500, 1280, 1360,
           1488, 1696, 800, 803, 809, 838, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x1e - 1280x800@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 122500, 1280, 1360,
           1496, 1712, 800, 803, 809, 843, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x1f - 1280x800@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 146250, 1280, 1328,
           1360, 1440, 800, 803, 809, 847, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x20 - 1280x960@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 1280, 1376,
           1488, 1800, 960, 961, 964, 1000, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x21 - 1280x960@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1280, 1344,
           1504, 1728, 960, 961, 964, 1011, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x22 - 1280x960@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 175500, 1280, 1328,
           1360, 1440, 960, 963, 967, 1017, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x23 - 1280x1024@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 1280, 1328,
           1440, 1688, 1024, 1025, 1028, 1066, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x24 - 1280x1024@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 135000, 1280, 1296,
           1440, 1688, 1024, 1025, 1028, 1066, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x25 - 1280x1024@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 157500, 1280, 1344,
           1504, 1728, 1024, 1025, 1028, 1072, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x26 - 1280x1024@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 187250, 1280, 1328,
           1360, 1440, 1024, 1027, 1034, 1084, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x27 - 1360x768@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 85500, 1360, 1424,
           1536, 1792, 768, 771, 777, 795, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x28 - 1360x768@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148250, 1360, 1408,
           1440, 1520, 768, 771, 776, 813, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x51 - 1366x768@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 85500, 1366, 1436,
           1579, 1792, 768, 771, 774, 798, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x56 - 1366x768@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 72000, 1366, 1380,
           1436, 1500, 768, 769, 772, 800, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x29 - 1400x1050@60Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 101000, 1400, 1448,
           1480, 1560, 1050, 1053, 1057, 1080, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x2a - 1400x1050@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 121750, 1400, 1488,
           1632, 1864, 1050, 1053, 1057, 1089, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x2b - 1400x1050@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 156000, 1400, 1504,
           1648, 1896, 1050, 1053, 1057, 1099, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x2c - 1400x1050@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 179500, 1400, 1504,
           1656, 1912, 1050, 1053, 1057, 1105, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x2d - 1400x1050@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 208000, 1400, 1448,
           1480, 1560, 1050, 1053, 1057, 1112, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x2e - 1440x900@60Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 88750, 1440, 1488,
           1520, 1600, 900, 903, 909, 926, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x2f - 1440x900@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 106500, 1440, 1520,
           1672, 1904, 900, 903, 909, 934, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x30 - 1440x900@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 136750, 1440, 1536,
           1688, 1936, 900, 903, 909, 942, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x31 - 1440x900@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 157000, 1440, 1544,
           1696, 1952, 900, 903, 909, 948, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x32 - 1440x900@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 182750, 1440, 1488,
           1520, 1600, 900, 903, 909, 953, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x53 - 1600x900@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 1600, 1624,
           1704, 1800, 900, 901, 904, 1000, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x33 - 1600x1200@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 162000, 1600, 1664,
           1856, 2160, 1200, 1201, 1204, 1250, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x34 - 1600x1200@65Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 175500, 1600, 1664,
           1856, 2160, 1200, 1201, 1204, 1250, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x35 - 1600x1200@70Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 189000, 1600, 1664,
           1856, 2160, 1200, 1201, 1204, 1250, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x36 - 1600x1200@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 202500, 1600, 1664,
           1856, 2160, 1200, 1201, 1204, 1250, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x37 - 1600x1200@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 229500, 1600, 1664,
           1856, 2160, 1200, 1201, 1204, 1250, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x38 - 1600x1200@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 268250, 1600, 1648,
           1680, 1760, 1200, 1203, 1207, 1271, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x39 - 1680x1050@60Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 119000, 1680, 1728,
           1760, 1840, 1050, 1053, 1059, 1080, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x3a - 1680x1050@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 146250, 1680, 1784,
           1960, 2240, 1050, 1053, 1059, 1089, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x3b - 1680x1050@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 187000, 1680, 1800,
           1976, 2272, 1050, 1053, 1059, 1099, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x3c - 1680x1050@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 214750, 1680, 1808,
           1984, 2288, 1050, 1053, 1059, 1105, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x3d - 1680x1050@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 245500, 1680, 1728,
           1760, 1840, 1050, 1053, 1059, 1112, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x3e - 1792x1344@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 204750, 1792, 1920,
           2120, 2448, 1344, 1345, 1348, 1394, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x3f - 1792x1344@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 261000, 1792, 1888,
           2104, 2456, 1344, 1345, 1348, 1417, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x40 - 1792x1344@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 333250, 1792, 1840,
           1872, 1952, 1344, 1347, 1351, 1423, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x41 - 1856x1392@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 218250, 1856, 1952,
           2176, 2528, 1392, 1393, 1396, 1439, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x42 - 1856x1392@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 288000, 1856, 1984,
           2208, 2560, 1392, 1393, 1396, 1500, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x43 - 1856x1392@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 356500, 1856, 1904,
           1936, 2016, 1392, 1395, 1399, 1474, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x52 - 1920x1080@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 148500, 1920, 2008,
           2052, 2200, 1080, 1084, 1089, 1125, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x44 - 1920x1200@60Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 154000, 1920, 1968,
           2000, 2080, 1200, 1203, 1209, 1235, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x45 - 1920x1200@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 193250, 1920, 2056,
           2256, 2592, 1200, 1203, 1209, 1245, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x46 - 1920x1200@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 245250, 1920, 2056,
           2264, 2608, 1200, 1203, 1209, 1255, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x47 - 1920x1200@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 281250, 1920, 2064,
           2272, 2624, 1200, 1203, 1209, 1262, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x48 - 1920x1200@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 317000, 1920, 1968,
           2000, 2080, 1200, 1203, 1209, 1271, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x49 - 1920x1440@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 234000, 1920, 2048,
           2256, 2600, 1440, 1441, 1444, 1500, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x4a - 1920x1440@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 297000, 1920, 2064,
           2288, 2640, 1440, 1441, 1444, 1500, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x4b - 1920x1440@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 380500, 1920, 1968,
           2000, 2080, 1440, 1443, 1447, 1525, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x54 - 2048x1152@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 162000, 2048, 2074,
           2154, 2250, 1152, 1153, 1156, 1200, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x4c - 2560x1600@60Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 268500, 2560, 2608,
           2640, 2720, 1600, 1603, 1609, 1646, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x4d - 2560x1600@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 348500, 2560, 2752,
           3032, 3504, 1600, 1603, 1609, 1658, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x4e - 2560x1600@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 443250, 2560, 2768,
           3048, 3536, 1600, 1603, 1609, 1672, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x4f - 2560x1600@85Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 505250, 2560, 2768,
           3048, 3536, 1600, 1603, 1609, 1682, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 0x50 - 2560x1600@120Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 552750, 2560, 2608,
           2640, 2720, 1600, 1603, 1609, 1694, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x57 - 4096x2160@60Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 556744, 4096, 4104,
           4136, 4176, 2160, 2208, 2216, 2222, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 0x58 - 4096x2160@59.94Hz RB */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 556188, 4096, 4104,
           4136, 4176, 2160, 2208, 2216, 2222, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
};

/*
 * These more or less come from the DMT spec.  The 720x400 modes are
 * inferred from historical 80x25 practice.  The 640x480@67 and 832x624@75
 * modes are old-school Mac modes.  The EDID spec says the 1152x864@75 mode
 * should be 1152x870, again for the Mac, but instead we use the x864 DMT
 * mode.
 *
 * The DMT modes have been fact-checked; the rest are mild guesses.
 */
static const struct drm_display_mode edid_est_modes[] = {
    /* 800x600@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 40000, 800, 840,
           968, 1056, 600, 601, 605, 628, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 800x600@56Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 36000, 800, 824,
           896, 1024, 600, 601, 603,  625, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 640x480@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 31500, 640, 656,
           720, 840, 480, 481, 484, 500, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 640x480@72Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 31500, 640, 664,
           704,  832, 480, 489, 492, 520, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 640x480@67Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 30240, 640, 704,
           768,  864, 480, 483, 486, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 640x480@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 25175, 640, 656,
           752, 800, 480, 490, 492, 525, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 720x400@88Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 35500, 720, 738,
           846, 900, 400, 421, 423,  449, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 720x400@70Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 28320, 720, 738,
           846,  900, 400, 412, 414, 449, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 1280x1024@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 135000, 1280, 1296,
           1440, 1688, 1024, 1025, 1028, 1066, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 1024x768@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 78750, 1024, 1040,
           1136, 1312,  768, 769, 772, 800, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 1024x768@70Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 75000, 1024, 1048,
           1184, 1328, 768, 771, 777, 806, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 1024x768@60Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 65000, 1024, 1048,
           1184, 1344, 768, 771, 777, 806, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 1024x768@43Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 44900, 1024, 1032,
           1208, 1264, 768, 768, 776, 817, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC |
           DRM_MODE_FLAG_INTERLACE) },
    /* 832x624@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 57284, 832, 864,
           928, 1152, 624, 625, 628, 667, 0,
           DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) },
    /* 800x600@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 49500, 800, 816,
           896, 1056, 600, 601, 604,  625, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 800x600@72Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 50000, 800, 856,
           976, 1040, 600, 637, 643, 666, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
    /* 1152x864@75Hz */
    { DRM_MODE(DRM_MODE_TYPE_DRIVER, 108000, 1152, 1216,
           1344, 1600, 864, 865, 868, 900, 0,
           DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) },
};
struct minimode {
    short w;
    short h;
    short r;
    short rb;
};

static const struct minimode est3_modes[] = {
    /* byte 6 */
    { 640, 350, 85, 0 },
    { 640, 400, 85, 0 },
    { 720, 400, 85, 0 },
    { 640, 480, 85, 0 },
    { 848, 480, 60, 0 },
    { 800, 600, 85, 0 },
    { 1024, 768, 85, 0 },
    { 1152, 864, 75, 0 },
    /* byte 7 */
    { 1280, 768, 60, 1 },
    { 1280, 768, 60, 0 },
    { 1280, 768, 75, 0 },
    { 1280, 768, 85, 0 },
    { 1280, 960, 60, 0 },
    { 1280, 960, 85, 0 },
    { 1280, 1024, 60, 0 },
    { 1280, 1024, 85, 0 },
    /* byte 8 */
    { 1360, 768, 60, 0 },
    { 1440, 900, 60, 1 },
    { 1440, 900, 60, 0 },
    { 1440, 900, 75, 0 },
    { 1440, 900, 85, 0 },
    { 1400, 1050, 60, 1 },
    { 1400, 1050, 60, 0 },
    { 1400, 1050, 75, 0 },
    /* byte 9 */
    { 1400, 1050, 85, 0 },
    { 1680, 1050, 60, 1 },
    { 1680, 1050, 60, 0 },
    { 1680, 1050, 75, 0 },
    { 1680, 1050, 85, 0 },
    { 1600, 1200, 60, 0 },
    { 1600, 1200, 65, 0 },
    { 1600, 1200, 70, 0 },
    /* byte 10 */
    { 1600, 1200, 75, 0 },
    { 1600, 1200, 85, 0 },
    { 1792, 1344, 60, 0 },
    { 1792, 1344, 75, 0 },
    { 1856, 1392, 60, 0 },
    { 1856, 1392, 75, 0 },
    { 1920, 1200, 60, 1 },
    { 1920, 1200, 60, 0 },
    /* byte 11 */
    { 1920, 1200, 75, 0 },
    { 1920, 1200, 85, 0 },
    { 1920, 1440, 60, 0 },
    { 1920, 1440, 75, 0 },
};

static const struct minimode extra_modes[] = {
    { 1024, 576,  60, 0 },
    { 1366, 768,  60, 0 },
    { 1600, 900,  60, 0 },
    { 1680, 945,  60, 0 },
    { 1920, 1080, 60, 0 },
    { 2048, 1152, 60, 0 },
    { 2048, 1536, 60, 0 },
};

/**
 * decode_mode() - Decoding an 18-byte detailed timing record
 *
 * @buf:    Pointer to EDID detailed timing record
 * @timing: Place to put timing
 */
static void decode_mode(u8 *buf, struct drm_display_mode *mode)
{
    uint x_mm, y_mm;
    unsigned int ha, hbl, hso, hspw, hborder;
    unsigned int va, vbl, vso, vspw, vborder;
    struct edid_detailed_timing *t = (struct edid_detailed_timing *)buf;

    x_mm = (buf[12] + ((buf[14] & 0xf0) << 4));
    y_mm = (buf[13] + ((buf[14] & 0x0f) << 8));
    ha = (buf[2] + ((buf[4] & 0xf0) << 4));
    hbl = (buf[3] + ((buf[4] & 0x0f) << 8));
    hso = (buf[8] + ((buf[11] & 0xc0) << 2));
    hspw = (buf[9] + ((buf[11] & 0x30) << 4));
    hborder = buf[15];
    va = (buf[5] + ((buf[7] & 0xf0) << 4));
    vbl = (buf[6] + ((buf[7] & 0x0f) << 8));
    vso = ((buf[10] >> 4) + ((buf[11] & 0x0c) << 2));
    vspw = ((buf[10] & 0x0f) + ((buf[11] & 0x03) << 4));
    vborder = buf[16];

    /* Edid contains pixel clock in terms of 10KHz */
    mode->clock = (buf[0] + (buf[1] << 8)) * 10;
    mode->hdisplay = ha;
    mode->hsync_start = ha + hso;
    mode->hsync_end = ha + hso + hspw;
    mode->htotal = ha + hbl;
    mode->vdisplay = va;
    mode->vsync_start = va + vso;
    mode->vsync_end = va + vso + vspw;
    mode->vtotal = va + vbl;

    mode->flags = EDID_DETAILED_TIMING_FLAG_HSYNC_POLARITY(*t) ?
        DRM_MODE_FLAG_PHSYNC : DRM_MODE_FLAG_NHSYNC;
    mode->flags |= EDID_DETAILED_TIMING_FLAG_VSYNC_POLARITY(*t) ?
        DRM_MODE_FLAG_PVSYNC : DRM_MODE_FLAG_NVSYNC;

    if (EDID_DETAILED_TIMING_FLAG_INTERLEAVED(*t))
        mode->flags |= DRM_MODE_FLAG_INTERLACE;

    debug("Detailed mode clock %u kHz, %d mm x %d mm, flags[%x]\n"
          "     %04d %04d %04d %04d hborder %d\n"
          "     %04d %04d %04d %04d vborder %d\n",
          mode->clock,
          x_mm, y_mm, mode->flags,
          mode->hdisplay, mode->hsync_start, mode->hsync_end,
          mode->htotal, hborder,
          mode->vdisplay, mode->vsync_start, mode->vsync_end,
          mode->vtotal, vborder);
}

/**
 * edid_vendor - match a string against EDID's obfuscated vendor field
 * @edid: EDID to match
 * @vendor: vendor string
 *
 * Returns true if @vendor is in @edid, false otherwise
 */
static bool edid_vendor(struct edid *edid, char *vendor)
{
    char edid_vendor[3];

    edid_vendor[0] = ((edid->mfg_id[0] & 0x7c) >> 2) + '@';
    edid_vendor[1] = (((edid->mfg_id[0] & 0x3) << 3) |
              ((edid->mfg_id[1] & 0xe0) >> 5)) + '@';
    edid_vendor[2] = (edid->mfg_id[1] & 0x1f) + '@';

    return !strncmp(edid_vendor, vendor, 3);
}

static int drm_get_vrefresh(const struct drm_display_mode *mode)
{
    int refresh = 0;
    unsigned int calc_val;

    if (mode->vrefresh > 0) {
        refresh = mode->vrefresh;
    } else if (mode->htotal > 0 && mode->vtotal > 0) {
        int vtotal;

        vtotal = mode->vtotal;
        /* work out vrefresh the value will be x1000 */
        calc_val = (mode->clock * 1000);
        calc_val /= mode->htotal;
        refresh = (calc_val + vtotal / 2) / vtotal;

        if (mode->flags & DRM_MODE_FLAG_INTERLACE)
            refresh *= 2;
        if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
            refresh /= 2;
        if (mode->vscan > 1)
            refresh /= mode->vscan;
    }
    return refresh;
}

int edid_get_drm_mode(u8 *buf, int buf_size, struct drm_display_mode *mode,
              int *panel_bits_per_colourp)
{
    struct edid1_info *edid = (struct edid1_info *)buf;
    bool timing_done;
    int i;

    if (buf_size < sizeof(*edid) || edid_check_info(edid)) {
        debug("%s: Invalid buffer\n", __func__);
        return -EINVAL;
    }

    if (!EDID1_INFO_FEATURE_PREFERRED_TIMING_MODE(*edid)) {
        debug("%s: No preferred timing\n", __func__);
        return -ENOENT;
    }

    /* Look for detailed timing */
    timing_done = false;
    for (i = 0; i < 4; i++) {
        struct edid_monitor_descriptor *desc;

        desc = &edid->monitor_details.descriptor[i];
        if (desc->zero_flag_1 != 0) {
            decode_mode((u8 *)desc, mode);
            timing_done = true;
            break;
        }
    }
    if (!timing_done)
        return -EINVAL;

    if (!EDID1_INFO_VIDEO_INPUT_DIGITAL(*edid)) {
        debug("%s: Not a digital display\n", __func__);
        return -ENOSYS;
    }
    if (edid->version != 1 || edid->revision < 4) {
        debug("%s: EDID version %d.%d does not have required info\n",
              __func__, edid->version, edid->revision);
        *panel_bits_per_colourp = -1;
    } else  {
        *panel_bits_per_colourp =
            ((edid->video_input_definition & 0x70) >> 3) + 4;
    }

    return 0;
}


static int
cea_db_payload_len(const u8 *db)
{
    return db[0] & 0x1f;
}

static int
cea_db_extended_tag(const u8 *db)
{
    return db[1];
}

static int
cea_db_tag(const u8 *db)
{
    return db[0] >> 5;
}

#define for_each_cea_db(cea, i, start, end) \
    for ((i) = (start); (i) < (end) && (i) + \
    cea_db_payload_len(&(cea)[(i)]) < \
    (end); (i) += cea_db_payload_len(&(cea)[(i)]) + 1)

static int
cea_revision(const u8 *cea)
{
    return cea[1];
}

static int
cea_db_offsets(const u8 *cea, int *start, int *end)
{
    /* Data block offset in CEA extension block */
    *start = 4;
    *end = cea[2];
    if (*end == 0)
        *end = 127;
    if (*end < 4 || *end > 127)
        return -ERANGE;

    /*
     * XXX: cea[2] is equal to the real value minus one in some sink edid.
     */
    if (*end != 4) {
        int i;

        i = *start;
        while (i < (*end) &&
               i + cea_db_payload_len(&(cea)[i]) < (*end))
            i += cea_db_payload_len(&(cea)[i]) + 1;

        if (cea_db_payload_len(&(cea)[i]) &&
            i + cea_db_payload_len(&(cea)[i]) == (*end))
            (*end)++;
    }

    return 0;
}

static bool cea_db_is_hdmi_vsdb(const u8 *db)
{
    int hdmi_id;

    if (cea_db_tag(db) != EDID_CEA861_DB_VENDOR)
        return false;

    if (cea_db_payload_len(db) < 5)
        return false;

    hdmi_id = db[1] | (db[2] << 8) | (db[3] << 16);

    return hdmi_id == HDMI_IEEE_OUI;
}

static bool cea_db_is_hdmi_forum_vsdb(const u8 *db)
{
    unsigned int oui;

    if (cea_db_tag(db) != EDID_CEA861_DB_VENDOR)
        return false;

    if (cea_db_payload_len(db) < 7)
        return false;

    oui = db[3] << 16 | db[2] << 8 | db[1];

    return oui == HDMI_FORUM_IEEE_OUI;
}

static bool cea_db_is_y420cmdb(const u8 *db)
{
    // if (cea_db_tag(db) != EDID_CEA861_DB_USE_EXTENDED)
    //     return false;

    if (!cea_db_payload_len(db))
        return false;

    if (cea_db_extended_tag(db) != EXT_VIDEO_CAP_BLOCK_Y420CMDB)
        return false;

    return true;
}

static bool cea_db_is_y420vdb(const u8 *db)
{
    // if (cea_db_tag(db) != EDID_CEA861_DB_USE_EXTENDED)
    //     return false;

    if (!cea_db_payload_len(db))
        return false;

    if (cea_db_extended_tag(db) != EXT_VIDEO_DATA_BLOCK_420)
        return false;

    return true;
}

static bool drm_valid_hdmi_vic(u8 vic)
{
    return vic > 0 && vic < ARRAY_SIZE(edid_4k_modes);
}

static void drm_add_hdmi_modes(struct hdmi_edid_data *data,
                   const struct drm_display_mode *mode)
{
    struct drm_display_mode *mode_buf = data->mode_buf;

    if (data->modes >= MODE_LEN)
        return;
    mode_buf[(data->modes)++] = *mode;
}

static bool drm_valid_cea_vic(u8 vic)
{
    return vic > 0 && vic < ARRAY_SIZE(edid_cea_modes);
}

static u8 svd_to_vic(u8 svd)
{
    /* 0-6 bit vic, 7th bit native mode indicator */
    if ((svd >= 1 &&  svd <= 64) || (svd >= 129 && svd <= 192))
        return svd & 127;

    return svd;
}

static struct drm_display_mode *
drm_display_mode_from_vic_index(const u8 *video_db, u8 video_len,
                u8 video_index)
{
    struct drm_display_mode *newmode;
    u8 vic;

    if (!video_db || video_index >= video_len)
        return NULL;

    /* CEA modes are numbered 1..127 */
    vic = svd_to_vic(video_db[video_index]);
    if (!drm_valid_cea_vic(vic))
        return NULL;

    newmode = drm_mode_create();
    if (!newmode)
        return NULL;

    *newmode = edid_cea_modes[vic];
    newmode->vrefresh = 0;

    return newmode;
}

static void bitmap_set(unsigned long *map, unsigned int start, int len)
{
    unsigned long *p = map + BIT_WORD(start);
    const unsigned int size = start + len;
    int bits_to_set = BITS_PER_LONG - (start % BITS_PER_LONG);
    unsigned long mask_to_set = BITMAP_FIRST_WORD_MASK(start);

    while (len - bits_to_set >= 0) {
        *p |= mask_to_set;
        len -= bits_to_set;
        bits_to_set = BITS_PER_LONG;
        mask_to_set = ~0UL;
        p++;
    }
    if (len) {
        mask_to_set &= BITMAP_LAST_WORD_MASK(size);
        *p |= mask_to_set;
    }
}

static void
drm_add_cmdb_modes(u8 svd, struct drm_hdmi_info *hdmi)
{
    u8 vic = svd_to_vic(svd);

    if (!drm_valid_cea_vic(vic))
        return;

    bitmap_set(hdmi->y420_cmdb_modes, vic, 1);
}

int do_cea_modes(struct hdmi_edid_data *data, const u8 *db, u8 len)
{
    int i, modes = 0;
    struct drm_hdmi_info *hdmi = &data->display_info.hdmi;

    for (i = 0; i < len; i++) {
        struct drm_display_mode *mode;

        mode = drm_display_mode_from_vic_index(db, len, i);
        if (mode) {
            /*
             * YCBCR420 capability block contains a bitmap which
             * gives the index of CEA modes from CEA VDB, which
             * can support YCBCR 420 sampling output also (apart
             * from RGB/YCBCR444 etc).
             * For example, if the bit 0 in bitmap is set,
             * first mode in VDB can support YCBCR420 output too.
             * Add YCBCR420 modes only if sink is HDMI 2.0 capable.
             */
            if (i < 64 && hdmi->y420_cmdb_map & (1ULL << i))
                drm_add_cmdb_modes(db[i], hdmi);
            drm_add_hdmi_modes(data, mode);
            drm_mode_destroy(mode);
            modes++;
        }
    }

    return modes;
}

/*
 * do_y420vdb_modes - Parse YCBCR 420 only modes
 * @data: the structure that save parsed hdmi edid data
 * @svds: start of the data block of CEA YCBCR 420 VDB
 * @svds_len: length of the CEA YCBCR 420 VDB
 * @hdmi: runtime information about the connected HDMI sink
 *
 * Parse the CEA-861-F YCBCR 420 Video Data Block (Y420VDB)
 * which contains modes which can be supported in YCBCR 420
 * output format only.
 */
static int
do_y420vdb_modes(struct hdmi_edid_data *data, const u8 *svds, u8 svds_len)
{
    int modes = 0, i;
    struct drm_hdmi_info *hdmi = &data->display_info.hdmi;

    for (i = 0; i < svds_len; i++) {
        u8 vic = svd_to_vic(svds[i]);

        if (!drm_valid_cea_vic(vic))
            continue;

        bitmap_set(hdmi->y420_vdb_modes, vic, 1);
        drm_add_hdmi_modes(data, &edid_cea_modes[vic]);
        modes++;
    }

    return modes;
}

struct stereo_mandatory_mode {
    int width, height, vrefresh;
    unsigned int flags;
};

static const struct stereo_mandatory_mode stereo_mandatory_modes[] = {
    { 1920, 1080, 24, DRM_MODE_FLAG_3D_TOP_AND_BOTTOM },
    { 1920, 1080, 24, DRM_MODE_FLAG_3D_FRAME_PACKING },
    { 1920, 1080, 50,
      DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_3D_SIDE_BY_SIDE_HALF },
    { 1920, 1080, 60,
      DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_3D_SIDE_BY_SIDE_HALF },
    { 1280, 720,  50, DRM_MODE_FLAG_3D_TOP_AND_BOTTOM },
    { 1280, 720,  50, DRM_MODE_FLAG_3D_FRAME_PACKING },
    { 1280, 720,  60, DRM_MODE_FLAG_3D_TOP_AND_BOTTOM },
    { 1280, 720,  60, DRM_MODE_FLAG_3D_FRAME_PACKING }
};

static bool
stereo_match_mandatory(const struct drm_display_mode *mode,
               const struct stereo_mandatory_mode *stereo_mode)
{
    unsigned int interlaced = mode->flags & DRM_MODE_FLAG_INTERLACE;

    return mode->hdisplay == stereo_mode->width &&
           mode->vdisplay == stereo_mode->height &&
           interlaced == (stereo_mode->flags & DRM_MODE_FLAG_INTERLACE) &&
           drm_get_vrefresh(mode) == stereo_mode->vrefresh;
}

static int add_hdmi_mandatory_stereo_modes(struct hdmi_edid_data *data)
{
    const struct drm_display_mode *mode;
    int num = data->modes, modes = 0, i, k;

    for (k = 0; k < num; k++) {
        mode = &data->mode_buf[k];
        for (i = 0; i < ARRAY_SIZE(stereo_mandatory_modes); i++) {
            const struct stereo_mandatory_mode *mandatory;
            struct drm_display_mode *new_mode;

            if (!stereo_match_mandatory(mode,
                            &stereo_mandatory_modes[i]))
                continue;

            mandatory = &stereo_mandatory_modes[i];
            new_mode = drm_mode_create();
            if (!new_mode)
                continue;

            *new_mode = *mode;
            new_mode->flags |= mandatory->flags;
            drm_add_hdmi_modes(data, new_mode);
            drm_mode_destroy(new_mode);
            modes++;
        }
    }

    return modes;
}

static int add_3d_struct_modes(struct hdmi_edid_data *data, u16 structure,
                   const u8 *video_db, u8 video_len, u8 video_index)
{
    struct drm_display_mode *newmode;
    int modes = 0;

    if (structure & (1 << 0)) {
        newmode = drm_display_mode_from_vic_index(video_db,
                              video_len,
                              video_index);
        if (newmode) {
            newmode->flags |= DRM_MODE_FLAG_3D_FRAME_PACKING;
            drm_add_hdmi_modes(data, newmode);
            modes++;
            drm_mode_destroy(newmode);
        }
    }
    if (structure & (1 << 6)) {
        newmode = drm_display_mode_from_vic_index(video_db,
                              video_len,
                              video_index);
        if (newmode) {
            newmode->flags |= DRM_MODE_FLAG_3D_TOP_AND_BOTTOM;
            drm_add_hdmi_modes(data, newmode);
            modes++;
            drm_mode_destroy(newmode);
        }
    }
    if (structure & (1 << 8)) {
        newmode = drm_display_mode_from_vic_index(video_db,
                              video_len,
                              video_index);
        if (newmode) {
            newmode->flags |= DRM_MODE_FLAG_3D_SIDE_BY_SIDE_HALF;
            drm_add_hdmi_modes(data, newmode);
            modes++;
            drm_mode_destroy(newmode);
        }
    }

    return modes;
}

static int add_hdmi_mode(struct hdmi_edid_data *data, u8 vic)
{
    if (!drm_valid_hdmi_vic(vic)) {
        debug("Unknown HDMI VIC: %d\n", vic);
        return 0;
    }

    drm_add_hdmi_modes(data, &edid_4k_modes[vic]);

    return 1;
}

/*
 * do_hdmi_vsdb_modes - Parse the HDMI Vendor Specific data block
 * @db: start of the CEA vendor specific block
 * @len: length of the CEA block payload, ie. one can access up to db[len]
 *
 * Parses the HDMI VSDB looking for modes to add to @data. This function
 * also adds the stereo 3d modes when applicable.
 */
static int
do_hdmi_vsdb_modes(const u8 *db, u8 len, const u8 *video_db, u8 video_len,
           struct hdmi_edid_data *data)
{
    int modes = 0, offset = 0, i, multi_present = 0, multi_len;
    u8 vic_len, hdmi_3d_len = 0;
    u16 mask;
    u16 structure_all;

    if (len < 8)
        goto out;

    /* no HDMI_Video_Present */
    if (!(db[8] & (1 << 5)))
        goto out;

    /* Latency_Fields_Present */
    if (db[8] & (1 << 7))
        offset += 2;

    /* I_Latency_Fields_Present */
    if (db[8] & (1 << 6))
        offset += 2;

    /* the declared length is not long enough for the 2 first bytes
     * of additional video format capabilities
     */
    if (len < (8 + offset + 2))
        goto out;

    /* 3D_Present */
    offset++;
    if (db[8 + offset] & (1 << 7)) {
        modes += add_hdmi_mandatory_stereo_modes(data);

        /* 3D_Multi_present */
        multi_present = (db[8 + offset] & 0x60) >> 5;
    }

    offset++;
    vic_len = db[8 + offset] >> 5;
    hdmi_3d_len = db[8 + offset] & 0x1f;

    for (i = 0; i < vic_len && len >= (9 + offset + i); i++) {
        u8 vic;

        vic = db[9 + offset + i];
        modes += add_hdmi_mode(data, vic);
    }

    offset += 1 + vic_len;

    if (multi_present == 1)
        multi_len = 2;
    else if (multi_present == 2)
        multi_len = 4;
    else
        multi_len = 0;

    if (len < (8 + offset + hdmi_3d_len - 1))
        goto out;

    if (hdmi_3d_len < multi_len)
        goto out;

    if (multi_present == 1 || multi_present == 2) {
        /* 3D_Structure_ALL */
        structure_all = (db[8 + offset] << 8) | db[9 + offset];

        /* check if 3D_MASK is present */
        if (multi_present == 2)
            mask = (db[10 + offset] << 8) | db[11 + offset];
        else
            mask = 0xffff;

        for (i = 0; i < 16; i++) {
            if (mask & (1 << i))
                modes += add_3d_struct_modes(data,
                        structure_all,
                        video_db,
                        video_len, i);
        }
    }

    offset += multi_len;

    for (i = 0; i < (hdmi_3d_len - multi_len); i++) {
        int vic_index;
        struct drm_display_mode *newmode = NULL;
        unsigned int newflag = 0;
        bool detail_present;

        detail_present = ((db[8 + offset + i] & 0x0f) > 7);

        if (detail_present && (i + 1 == hdmi_3d_len - multi_len))
            break;

        /* 2D_VIC_order_X */
        vic_index = db[8 + offset + i] >> 4;

        /* 3D_Structure_X */
        switch (db[8 + offset + i] & 0x0f) {
        case 0:
            newflag = DRM_MODE_FLAG_3D_FRAME_PACKING;
            break;
        case 6:
            newflag = DRM_MODE_FLAG_3D_TOP_AND_BOTTOM;
            break;
        case 8:
            /* 3D_Detail_X */
            if ((db[9 + offset + i] >> 4) == 1)
                newflag = DRM_MODE_FLAG_3D_SIDE_BY_SIDE_HALF;
            break;
        }

        if (newflag != 0) {
            newmode = drm_display_mode_from_vic_index(
                                  video_db,
                                  video_len,
                                  vic_index);

            if (newmode) {
                newmode->flags |= newflag;
                drm_add_hdmi_modes(data, newmode);
                modes++;
                drm_mode_destroy(newmode);
            }
        }

        if (detail_present)
            i++;
    }

out:
    return modes;
}

/**
 * edid_get_quirks - return quirk flags for a given EDID
 * @edid: EDID to process
 *
 * This tells subsequent routines what fixes they need to apply.
 */
static u32 edid_get_quirks(struct edid *edid)
{
    struct edid_quirk *quirk;
    int i;

    for (i = 0; i < ARRAY_SIZE(edid_quirk_list); i++) {
        quirk = &edid_quirk_list[i];

        if (edid_vendor(edid, quirk->vendor) &&
            (EDID_PRODUCT_ID(edid) == quirk->product_id))
            return quirk->quirks;
    }

    return 0;
}

static void drm_parse_y420cmdb_bitmap(struct hdmi_edid_data *data,
                      const u8 *db)
{
    struct drm_display_info *info = &data->display_info;
    struct drm_hdmi_info *hdmi = &info->hdmi;
    u8 map_len = cea_db_payload_len(db) - 1;
    u8 count;
    u64 map = 0;

    if (map_len == 0) {
        /* All CEA modes support ycbcr420 sampling also.*/
        hdmi->y420_cmdb_map = U64_MAX;
        info->color_formats |= DRM_COLOR_FORMAT_YCRCB420;
        return;
    }

    /*
     * This map indicates which of the existing CEA block modes
     * from VDB can support YCBCR420 output too. So if bit=0 is
     * set, first mode from VDB can support YCBCR420 output too.
     * We will parse and keep this map, before parsing VDB itself
     * to avoid going through the same block again and again.
     *
     * Spec is not clear about max possible size of this block.
     * Clamping max bitmap block size at 8 bytes. Every byte can
     * address 8 CEA modes, in this way this map can address
     * 8*8 = first 64 SVDs.
     */
    if (map_len > 8)
        map_len = 8;

    for (count = 0; count < map_len; count++)
        map |= (u64)db[2 + count] << (8 * count);

    if (map)
        info->color_formats |= DRM_COLOR_FORMAT_YCRCB420;

    hdmi->y420_cmdb_map = map;
}

static void drm_parse_ycbcr420_deep_color_info(struct hdmi_edid_data *data,
                           const u8 *db)
{
    u8 dc_mask;
    struct drm_hdmi_info *hdmi = &data->display_info.hdmi;

    dc_mask = db[7] & DRM_EDID_YCBCR420_DC_MASK;
    hdmi->y420_dc_modes |= dc_mask;
}

static void drm_parse_hdmi_forum_vsdb(struct hdmi_edid_data *data,
                      const u8 *hf_vsdb)
{
    struct drm_display_info *display = &data->display_info;
    struct drm_hdmi_info *hdmi = &display->hdmi;

    if (hf_vsdb[6] & 0x80) {
        hdmi->scdc.supported = true;
        if (hf_vsdb[6] & 0x40)
            hdmi->scdc.read_request = true;
    }

    /*
     * All HDMI 2.0 monitors must support scrambling at rates > 340 MHz.
     * And as per the spec, three factors confirm this:
     * * Availability of a HF-VSDB block in EDID (check)
     * * Non zero Max_TMDS_Char_Rate filed in HF-VSDB (let's check)
     * * SCDC support available (let's check)
     * Lets check it out.
     */

    if (hf_vsdb[5]) {
        /* max clock is 5000 KHz times block value */
        u32 max_tmds_clock = hf_vsdb[5] * 5000;
        struct drm_scdc *scdc = &hdmi->scdc;

        if (max_tmds_clock > 340000) {
            display->max_tmds_clock = max_tmds_clock;
            debug("HF-VSDB: max TMDS clock %d kHz\n",
                  display->max_tmds_clock);
        }

        if (scdc->supported) {
            scdc->scrambling.supported = true;

            /* Few sinks support scrambling for cloks < 340M */
            if ((hf_vsdb[6] & 0x8))
                scdc->scrambling.low_rates = true;
        }
    }

    drm_parse_ycbcr420_deep_color_info(data, hf_vsdb);
}

/**
 * drm_default_rgb_quant_range - default RGB quantization range
 * @mode: display mode
 *
 * Determine the default RGB quantization range for the mode,
 * as specified in CEA-861.
 *
 * Return: The default RGB quantization range for the mode
 */
enum hdmi_quantization_range
drm_default_rgb_quant_range(struct drm_display_mode *mode)
{
    /* All CEA modes other than VIC 1 use limited quantization range. */
    return drm_match_cea_mode(mode) > 1 ?
        HDMI_QUANTIZATION_RANGE_LIMITED :
        HDMI_QUANTIZATION_RANGE_FULL;
}

static void drm_parse_hdmi_deep_color_info(struct hdmi_edid_data *data,
                       const u8 *hdmi)
{
    struct drm_display_info *info = &data->display_info;
    unsigned int dc_bpc = 0;

    /* HDMI supports at least 8 bpc */
    info->bpc = 8;

    if (cea_db_payload_len(hdmi) < 6)
        return;

    if (hdmi[6] & DRM_EDID_HDMI_DC_30) {
        dc_bpc = 10;
        info->edid_hdmi_dc_modes |= DRM_EDID_HDMI_DC_30;
        debug("HDMI sink does deep color 30.\n");
    }

    if (hdmi[6] & DRM_EDID_HDMI_DC_36) {
        dc_bpc = 12;
        info->edid_hdmi_dc_modes |= DRM_EDID_HDMI_DC_36;
        debug("HDMI sink does deep color 36.\n");
    }

    if (hdmi[6] & DRM_EDID_HDMI_DC_48) {
        dc_bpc = 16;
        info->edid_hdmi_dc_modes |= DRM_EDID_HDMI_DC_48;
        debug("HDMI sink does deep color 48.\n");
    }

    if (dc_bpc == 0) {
        debug("No deep color support on this HDMI sink.\n");
        return;
    }

    debug("Assigning HDMI sink color depth as %d bpc.\n", dc_bpc);
    info->bpc = dc_bpc;

    /* YCRCB444 is optional according to spec. */
    if (hdmi[6] & DRM_EDID_HDMI_DC_Y444) {
        info->edid_hdmi_dc_modes |= DRM_EDID_HDMI_DC_Y444;
        debug("HDMI sink does YCRCB444 in deep color.\n");
    }

    /*
     * Spec says that if any deep color mode is supported at all,
     * then deep color 36 bit must be supported.
     */
    if (!(hdmi[6] & DRM_EDID_HDMI_DC_36))
        debug("HDMI sink should do DC_36, but does not!\n");
}

/*
 * Search EDID for CEA extension block.
 */
static u8 *drm_find_edid_extension(struct edid *edid, int ext_id)
{
    u8 *edid_ext = NULL;
    int i;

    /* No EDID or EDID extensions */
    if (!edid || !edid->extensions)
        return NULL;

    /* Find CEA extension */
    for (i = 0; i < edid->extensions; i++) {
        edid_ext = (u8 *)edid + EDID_SIZE * (i + 1);
        if (edid_ext[0] == ext_id)
            break;
    }

    if (i == edid->extensions)
        return NULL;

    return edid_ext;
}

static u8 *drm_find_cea_extension(struct edid *edid)
{
    return drm_find_edid_extension(edid, 0x02);
}

#define AUDIO_BLOCK 0x01
#define VIDEO_BLOCK     0x02
#define VENDOR_BLOCK    0x03
#define SPEAKER_BLOCK   0x04
#define EDID_BASIC_AUDIO BIT(6)

/**
 * drm_detect_hdmi_monitor - detect whether monitor is HDMI
 * @edid: monitor EDID information
 *
 * Parse the CEA extension according to CEA-861-B.
 *
 * Return: True if the monitor is HDMI, false if not or unknown.
 */
bool drm_detect_hdmi_monitor(struct edid *edid)
{
    u8 *edid_ext;
    int i;
    int start_offset, end_offset;

    edid_ext = drm_find_cea_extension(edid);
    if (!edid_ext)
        return false;

    if (cea_db_offsets(edid_ext, &start_offset, &end_offset))
        return false;

    /*
     * Because HDMI identifier is in Vendor Specific Block,
     * search it from all data blocks of CEA extension.
     */
    for_each_cea_db(edid_ext, i, start_offset, end_offset) {
        if (cea_db_is_hdmi_vsdb(&edid_ext[i]))
            return true;
    }

    return false;
}

/**
 * drm_detect_monitor_audio - check monitor audio capability
 * @edid: EDID block to scan
 *
 * Monitor should have CEA extension block.
 * If monitor has 'basic audio', but no CEA audio blocks, it's 'basic
 * audio' only. If there is any audio extension block and supported
 * audio format, assume at least 'basic audio' support, even if 'basic
 * audio' is not defined in EDID.
 *
 * Return: True if the monitor supports audio, false otherwise.
 */
bool drm_detect_monitor_audio(struct edid *edid)
{
    u8 *edid_ext;
    int i, j;
    bool has_audio = false;
    int start_offset, end_offset;

    edid_ext = drm_find_cea_extension(edid);
    if (!edid_ext)
        goto end;

    has_audio = ((edid_ext[3] & EDID_BASIC_AUDIO) != 0);

    if (has_audio) {
        debug("Monitor has basic audio support\n");
        goto end;
    }

    if (cea_db_offsets(edid_ext, &start_offset, &end_offset))
        goto end;

    for_each_cea_db(edid_ext, i, start_offset, end_offset) {
        if (cea_db_tag(&edid_ext[i]) == AUDIO_BLOCK) {
            has_audio = true;
            for (j = 1; j < cea_db_payload_len(&edid_ext[i]) + 1;
                 j += 3)
                debug("CEA audio format %d\n",
                      (edid_ext[i + j] >> 3) & 0xf);
            goto end;
        }
    }
end:
    return has_audio;
}

static void
drm_parse_hdmi_vsdb_video(struct hdmi_edid_data *data, const u8 *db)
{
    struct drm_display_info *info = &data->display_info;
    u8 len = cea_db_payload_len(db);

    if (len >= 6)
        info->dvi_dual = db[6] & 1;
    if (len >= 7)
        info->max_tmds_clock = db[7] * 5000;

    drm_parse_hdmi_deep_color_info(data, db);
}

static void drm_parse_cea_ext(struct hdmi_edid_data *data,
                  struct edid *edid)
{
    struct drm_display_info *info = &data->display_info;
    const u8 *edid_ext;
    int i, start, end;

    edid_ext = drm_find_cea_extension(edid);
    if (!edid_ext)
        return;

    info->cea_rev = edid_ext[1];

    /* The existence of a CEA block should imply RGB support */
    info->color_formats = DRM_COLOR_FORMAT_RGB444;
    if (edid_ext[3] & EDID_CEA_YCRCB444)
        info->color_formats |= DRM_COLOR_FORMAT_YCRCB444;
    if (edid_ext[3] & EDID_CEA_YCRCB422)
        info->color_formats |= DRM_COLOR_FORMAT_YCRCB422;

    if (cea_db_offsets(edid_ext, &start, &end))
        return;

    for_each_cea_db(edid_ext, i, start, end) {
        const u8 *db = &edid_ext[i];

        if (cea_db_is_hdmi_vsdb(db))
            drm_parse_hdmi_vsdb_video(data, db);
        if (cea_db_is_hdmi_forum_vsdb(db))
            drm_parse_hdmi_forum_vsdb(data, db);
        if (cea_db_is_y420cmdb(db))
            drm_parse_y420cmdb_bitmap(data, db);
    }
}

static void drm_add_display_info(struct hdmi_edid_data *data, struct edid *edid)
{
    struct drm_display_info *info = &data->display_info;

    info->width_mm = edid->width_cm * 10;
    info->height_mm = edid->height_cm * 10;

    /* driver figures it out in this case */
    info->bpc = 0;
    info->color_formats = 0;
    info->cea_rev = 0;
    info->max_tmds_clock = 0;
    info->dvi_dual = false;
    info->edid_hdmi_dc_modes = 0;

    memset(&info->hdmi, 0, sizeof(info->hdmi));

    if (edid->revision < 3)
        return;

    if (!(edid->input & DRM_EDID_INPUT_DIGITAL))
        return;

    drm_parse_cea_ext(data, edid);

    /*
     * Digital sink with "DFP 1.x compliant TMDS" according to EDID 1.3?
     *
     * For such displays, the DFP spec 1.0, section 3.10 "EDID support"
     * tells us to assume 8 bpc color depth if the EDID doesn't have
     * extensions which tell otherwise.
     */
    if ((info->bpc == 0) && (edid->revision < 4) &&
        (edid->input & DRM_EDID_DIGITAL_TYPE_DVI)) {
        info->bpc = 8;
        debug("Assigning DFP sink color depth as %d bpc.\n", info->bpc);
    }

    /* Only defined for 1.4 with digital displays */
    if (edid->revision < 4)
        return;

    switch (edid->input & DRM_EDID_DIGITAL_DEPTH_MASK) {
    case DRM_EDID_DIGITAL_DEPTH_6:
        info->bpc = 6;
        break;
    case DRM_EDID_DIGITAL_DEPTH_8:
        info->bpc = 8;
        break;
    case DRM_EDID_DIGITAL_DEPTH_10:
        info->bpc = 10;
        break;
    case DRM_EDID_DIGITAL_DEPTH_12:
        info->bpc = 12;
        break;
    case DRM_EDID_DIGITAL_DEPTH_14:
        info->bpc = 14;
        break;
    case DRM_EDID_DIGITAL_DEPTH_16:
        info->bpc = 16;
        break;
    case DRM_EDID_DIGITAL_DEPTH_UNDEF:
    default:
        info->bpc = 0;
        break;
    }

    debug("Assigning EDID-1.4 digital sink color depth as %d bpc.\n",
          info->bpc);

    info->color_formats |= DRM_COLOR_FORMAT_RGB444;
    if (edid->features & DRM_EDID_FEATURE_RGB_YCRCB444)
        info->color_formats |= DRM_COLOR_FORMAT_YCRCB444;
    if (edid->features & DRM_EDID_FEATURE_RGB_YCRCB422)
        info->color_formats |= DRM_COLOR_FORMAT_YCRCB422;
}

static
int add_cea_modes(struct hdmi_edid_data *data, struct edid *edid)
{
    const u8 *cea = drm_find_cea_extension(edid);
    const u8 *db, *hdmi = NULL, *video = NULL;
    u8 dbl, hdmi_len, video_len = 0;
    int modes = 0;

    if (cea && cea_revision(cea) >= 3) {
        int i, start, end;

        if (cea_db_offsets(cea, &start, &end))
            return 0;

        for_each_cea_db(cea, i, start, end) {
            db = &cea[i];
            dbl = cea_db_payload_len(db);

            if (cea_db_tag(db) == EDID_CEA861_DB_VIDEO) {
                video = db + 1;
                video_len = dbl;
                modes += do_cea_modes(data, video, dbl);
            } else if (cea_db_is_hdmi_vsdb(db)) {
                hdmi = db;
                hdmi_len = dbl;
            } else if (cea_db_is_y420vdb(db)) {
                const u8 *vdb420 = &db[2];

                /* Add 4:2:0(only) modes present in EDID */
                modes += do_y420vdb_modes(data, vdb420,
                              dbl - 1);
            }
        }
    }

    /*
     * We parse the HDMI VSDB after having added the cea modes as we will
     * be patching their flags when the sink supports stereo 3D.
     */
    if (hdmi)
        modes += do_hdmi_vsdb_modes(hdmi, hdmi_len, video,
                        video_len, data);

    return modes;
}

typedef void detailed_cb(struct detailed_timing *timing, void *closure);

static void
cea_for_each_detailed_block(u8 *ext, detailed_cb *cb, void *closure)
{
    int i, n = 0;
    u8 d = ext[0x02];
    u8 *det_base = ext + d;

    n = (127 - d) / 18;
    for (i = 0; i < n; i++)
        cb((struct detailed_timing *)(det_base + 18 * i), closure);
}

static void
vtb_for_each_detailed_block(u8 *ext, detailed_cb *cb, void *closure)
{
    unsigned int i, n = min((int)ext[0x02], 6);
    u8 *det_base = ext + 5;

    if (ext[0x01] != 1)
        return; /* unknown version */

    for (i = 0; i < n; i++)
        cb((struct detailed_timing *)(det_base + 18 * i), closure);
}

static void
drm_for_each_detailed_block(u8 *raw_edid, detailed_cb *cb, void *closure)
{
    int i;
    struct edid *edid = (struct edid *)raw_edid;

    if (!edid)
        return;

    for (i = 0; i < EDID_DETAILED_TIMINGS; i++)
        cb(&edid->detailed_timings[i], closure);

    for (i = 1; i <= raw_edid[0x7e]; i++) {
        u8 *ext = raw_edid + (i * EDID_SIZE);

        switch (*ext) {
        case CEA_EXT:
            cea_for_each_detailed_block(ext, cb, closure);
            break;
        case VTB_EXT:
            vtb_for_each_detailed_block(ext, cb, closure);
            break;
        default:
            break;
        }
    }
}

/*
 * EDID is delightfully ambiguous about how interlaced modes are to be
 * encoded.  Our internal representation is of frame height, but some
 * HDTV detailed timings are encoded as field height.
 *
 * The format list here is from CEA, in frame size.  Technically we
 * should be checking refresh rate too.  Whatever.
 */
static void
drm_mode_do_interlace_quirk(struct drm_display_mode *mode,
                struct detailed_pixel_timing *pt)
{
    int i;

    static const struct {
        int w, h;
    } cea_interlaced[] = {
        { 1920, 1080 },
        {  720,  480 },
        { 1440,  480 },
        { 2880,  480 },
        {  720,  576 },
        { 1440,  576 },
        { 2880,  576 },
    };

    if (!(pt->misc & DRM_EDID_PT_INTERLACED))
        return;

    for (i = 0; i < ARRAY_SIZE(cea_interlaced); i++) {
        if ((mode->hdisplay == cea_interlaced[i].w) &&
            (mode->vdisplay == cea_interlaced[i].h / 2)) {
            mode->vdisplay *= 2;
            mode->vsync_start *= 2;
            mode->vsync_end *= 2;
            mode->vtotal *= 2;
            mode->vtotal |= 1;
        }
    }

    mode->flags |= DRM_MODE_FLAG_INTERLACE;
}

/**
 * drm_mode_detailed - create a new mode from an EDID detailed timing section
 * @edid: EDID block
 * @timing: EDID detailed timing info
 * @quirks: quirks to apply
 *
 * An EDID detailed timing block contains enough info for us to create and
 * return a new struct drm_display_mode.
 */
static
struct drm_display_mode *drm_mode_detailed(struct edid *edid,
                       struct detailed_timing *timing,
                       u32 quirks)
{
    struct drm_display_mode *mode;
    struct detailed_pixel_timing *pt = &timing->data.pixel_data;
    unsigned hactive = (pt->hactive_hblank_hi & 0xf0) << 4 | pt->hactive_lo;
    unsigned vactive = (pt->vactive_vblank_hi & 0xf0) << 4 | pt->vactive_lo;
    unsigned hblank = (pt->hactive_hblank_hi & 0xf) << 8 | pt->hblank_lo;
    unsigned vblank = (pt->vactive_vblank_hi & 0xf) << 8 | pt->vblank_lo;
    unsigned hsync_offset =
        (pt->hsync_vsync_offset_pulse_width_hi & 0xc0) << 2 |
        pt->hsync_offset_lo;
    unsigned hsync_pulse_width =
        (pt->hsync_vsync_offset_pulse_width_hi & 0x30) << 4 |
        pt->hsync_pulse_width_lo;
    unsigned vsync_offset = (pt->hsync_vsync_offset_pulse_width_hi & 0xc) <<
        2 | pt->vsync_offset_pulse_width_lo >> 4;
    unsigned vsync_pulse_width =
        (pt->hsync_vsync_offset_pulse_width_hi & 0x3) << 4 |
        (pt->vsync_offset_pulse_width_lo & 0xf);

    /* ignore tiny modes */
    if (hactive < 64 || vactive < 64)
        return NULL;

    if (pt->misc & DRM_EDID_PT_STEREO) {
        debug("stereo mode not supported\n");
        return NULL;
    }
    if (!(pt->misc & DRM_EDID_PT_SEPARATE_SYNC))
        debug("composite sync not supported\n");

    /* it is incorrect if hsync/vsync width is zero */
    if (!hsync_pulse_width || !vsync_pulse_width) {
        debug("Incorrect Detailed timing. ");
        debug("Wrong Hsync/Vsync pulse width\n");
        return NULL;
    }

    if (quirks & EDID_QUIRK_FORCE_REDUCED_BLANKING) {
        mode = drm_cvt_mode(hactive, vactive, 60, true, false, false);
        if (!mode)
            return NULL;

        goto set_refresh;
    }

    mode = drm_mode_create();
    if (!mode)
        return NULL;

    if (quirks & EDID_QUIRK_135_CLOCK_TOO_HIGH)
        timing->pixel_clock = cpu_to_le16(1088);

    mode->clock = le16_to_cpu(timing->pixel_clock) * 10;

    mode->hdisplay = hactive;
    mode->hsync_start = mode->hdisplay + hsync_offset;
    mode->hsync_end = mode->hsync_start + hsync_pulse_width;
    mode->htotal = mode->hdisplay + hblank;

    mode->vdisplay = vactive;
    mode->vsync_start = mode->vdisplay + vsync_offset;
    mode->vsync_end = mode->vsync_start + vsync_pulse_width;
    mode->vtotal = mode->vdisplay + vblank;

    /* Some EDIDs have bogus h/vtotal values */
    if (mode->hsync_end > mode->htotal)
        mode->htotal = mode->hsync_end + 1;
    if (mode->vsync_end > mode->vtotal)
        mode->vtotal = mode->vsync_end + 1;

    drm_mode_do_interlace_quirk(mode, pt);

    if (quirks & EDID_QUIRK_DETAILED_SYNC_PP)
        pt->misc |= DRM_EDID_PT_HSYNC_POSITIVE |
            DRM_EDID_PT_VSYNC_POSITIVE;

    mode->flags |= (pt->misc & DRM_EDID_PT_HSYNC_POSITIVE) ?
        DRM_MODE_FLAG_PHSYNC : DRM_MODE_FLAG_NHSYNC;
    mode->flags |= (pt->misc & DRM_EDID_PT_VSYNC_POSITIVE) ?
        DRM_MODE_FLAG_PVSYNC : DRM_MODE_FLAG_NVSYNC;

set_refresh:

    mode->type = DRM_MODE_TYPE_DRIVER;
    mode->vrefresh = drm_get_vrefresh(mode);

    return mode;
}

/*
 * Calculate the alternate clock for the CEA mode
 * (60Hz vs. 59.94Hz etc.)
 */
static unsigned int
cea_mode_alternate_clock(const struct drm_display_mode *cea_mode)
{
    unsigned int clock = cea_mode->clock;

    if (cea_mode->vrefresh % 6 != 0)
        return clock;

    /*
     * edid_cea_modes contains the 59.94Hz
     * variant for 240 and 480 line modes,
     * and the 60Hz variant otherwise.
     */
    if (cea_mode->vdisplay == 240 || cea_mode->vdisplay == 480)
        clock = DIV_ROUND_CLOSEST(clock * 1001, 1000);
    else
        clock = DIV_ROUND_CLOSEST(clock * 1000, 1001);

    return clock;
}

static
u8 drm_match_cea_mode_clock_tolerance(const struct drm_display_mode *to_match,
                      unsigned int clock_tolerance)
{
    u8 vic;

    if (!to_match->clock)
        return 0;

    for (vic = 1; vic < ARRAY_SIZE(edid_cea_modes); vic++) {
        const struct drm_display_mode *cea_mode = &edid_cea_modes[vic];
        unsigned int clock1, clock2;

        /* Check both 60Hz and 59.94Hz */
        clock1 = cea_mode->clock;
        clock2 = cea_mode_alternate_clock(cea_mode);

        if (abs(to_match->clock - clock1) > clock_tolerance &&
            abs(to_match->clock - clock2) > clock_tolerance)
            continue;

        if (drm_mode_equal_no_clocks(to_match, cea_mode))
            return vic;
    }

    return 0;
}

static unsigned int
hdmi_mode_alternate_clock(const struct drm_display_mode *hdmi_mode)
{
    if (hdmi_mode->vdisplay == 4096 && hdmi_mode->hdisplay == 2160)
        return hdmi_mode->clock;

    return cea_mode_alternate_clock(hdmi_mode);
}

static
u8 drm_match_hdmi_mode_clock_tolerance(const struct drm_display_mode *to_match,
                       unsigned int clock_tolerance)
{
    u8 vic;

    if (!to_match->clock)
        return 0;

    for (vic = 1; vic < ARRAY_SIZE(edid_4k_modes); vic++) {
        const struct drm_display_mode *hdmi_mode = &edid_4k_modes[vic];
        unsigned int clock1, clock2;

        /* Make sure to also match alternate clocks */
        clock1 = hdmi_mode->clock;
        clock2 = hdmi_mode_alternate_clock(hdmi_mode);

        if (abs(to_match->clock - clock1) > clock_tolerance &&
            abs(to_match->clock - clock2) > clock_tolerance)
            continue;

        if (drm_mode_equal_no_clocks(to_match, hdmi_mode))
            return vic;
    }

    return 0;
}

static void fixup_detailed_cea_mode_clock(struct drm_display_mode *mode)
{
    const struct drm_display_mode *cea_mode;
    int clock1, clock2, clock;
    u8 vic;
    const char *type;

    /*
     * allow 5kHz clock difference either way to account for
     * the 10kHz clock resolution limit of detailed timings.
     */
    vic = drm_match_cea_mode_clock_tolerance(mode, 5);
    if (drm_valid_cea_vic(vic)) {
        type = "CEA";
        cea_mode = &edid_cea_modes[vic];
        clock1 = cea_mode->clock;
        clock2 = cea_mode_alternate_clock(cea_mode);
    } else {
        vic = drm_match_hdmi_mode_clock_tolerance(mode, 5);
        if (drm_valid_hdmi_vic(vic)) {
            type = "HDMI";
            cea_mode = &edid_4k_modes[vic];
            clock1 = cea_mode->clock;
            clock2 = hdmi_mode_alternate_clock(cea_mode);
        } else {
            return;
        }
    }

    /* pick whichever is closest */
    if (abs(mode->clock - clock1) < abs(mode->clock - clock2))
        clock = clock1;
    else
        clock = clock2;

    if (mode->clock == clock)
        return;

    debug("detailed mode matches %s VIC %d, adjusting clock %d -> %d\n",
          type, vic, mode->clock, clock);
    mode->clock = clock;
}

static void
do_detailed_mode(struct detailed_timing *timing, void *c)
{
    struct detailed_mode_closure *closure = c;
    struct drm_display_mode *newmode;

    if (timing->pixel_clock) {
        newmode = drm_mode_detailed(
                        closure->edid, timing,
                        closure->quirks);
        if (!newmode)
            return;

        if (closure->preferred)
            newmode->type |= DRM_MODE_TYPE_PREFERRED;

        /*
         * Detailed modes are limited to 10kHz pixel clock resolution,
         * so fix up anything that looks like CEA/HDMI mode,
         * but the clock is just slightly off.
         */
        fixup_detailed_cea_mode_clock(newmode);
        drm_add_hdmi_modes(closure->data, newmode);
        drm_mode_destroy(newmode);
        closure->modes++;
        closure->preferred = 0;
    }
}

/*
 * add_detailed_modes - Add modes from detailed timings
 * @data: attached data
 * @edid: EDID block to scan
 * @quirks: quirks to apply
 */
static int
add_detailed_modes(struct hdmi_edid_data *data, struct edid *edid,
           u32 quirks)
{
    struct detailed_mode_closure closure = {
        .data = data,
        .edid = edid,
        .preferred = 1,
        .quirks = quirks,
    };

    if (closure.preferred && !version_greater(edid, 1, 3))
        closure.preferred =
            (edid->features & DRM_EDID_FEATURE_PREFERRED_TIMING);

    drm_for_each_detailed_block((u8 *)edid, do_detailed_mode, &closure);

    return closure.modes;
}

static int drm_cvt_modes(struct hdmi_edid_data *data,
             struct detailed_timing *timing)
{
    int i, j, modes = 0;
    struct drm_display_mode *newmode;
    struct cvt_timing *cvt;
    const int rates[] = { 60, 85, 75, 60, 50 };
    const u8 empty[3] = { 0, 0, 0 };

    for (i = 0; i < 4; i++) {
        int uninitialized_var(width), height;

        cvt = &timing->data.other_data.data.cvt[i];

        if (!memcmp(cvt->code, empty, 3))
            continue;

        height = (cvt->code[0] + ((cvt->code[1] & 0xf0) << 4) + 1) * 2;
        switch (cvt->code[1] & 0x0c) {
        case 0x00:
            width = height * 4 / 3;
            break;
        case 0x04:
            width = height * 16 / 9;
            break;
        case 0x08:
            width = height * 16 / 10;
            break;
        case 0x0c:
            width = height * 15 / 9;
            break;
        }

        for (j = 1; j < 5; j++) {
            if (cvt->code[2] & (1 << j)) {
                newmode = drm_cvt_mode(width, height,
                               rates[j], j == 0,
                               false, false);
                if (newmode) {
                    drm_add_hdmi_modes(data, newmode);
                    modes++;
                    drm_mode_destroy(newmode);
                }
            }
        }
    }

    return modes;
}

static void
do_cvt_mode(struct detailed_timing *timing, void *c)
{
    struct detailed_mode_closure *closure = c;
    struct detailed_non_pixel *data = &timing->data.other_data;

    if (data->type == EDID_DETAIL_CVT_3BYTE)
        closure->modes += drm_cvt_modes(closure->data, timing);
}

static int
add_cvt_modes(struct hdmi_edid_data *data, struct edid *edid)
{
    struct detailed_mode_closure closure = {
        .data = data,
        .edid = edid,
    };

    if (version_greater(edid, 1, 2))
        drm_for_each_detailed_block((u8 *)edid, do_cvt_mode, &closure);

    /* XXX should also look for CVT codes in VTB blocks */

    return closure.modes;
}

static void
find_gtf2(struct detailed_timing *t, void *data)
{
    u8 *r = (u8 *)t;

    if (r[3] == EDID_DETAIL_MONITOR_RANGE && r[10] == 0x02)
        *(u8 **)data = r;
}

/* Secondary GTF curve kicks in above some break frequency */
static int
drm_gtf2_hbreak(struct edid *edid)
{
    u8 *r = NULL;

    drm_for_each_detailed_block((u8 *)edid, find_gtf2, &r);
    return r ? (r[12] * 2) : 0;
}

static int
drm_gtf2_2c(struct edid *edid)
{
    u8 *r = NULL;

    drm_for_each_detailed_block((u8 *)edid, find_gtf2, &r);
    return r ? r[13] : 0;
}

static int
drm_gtf2_m(struct edid *edid)
{
    u8 *r = NULL;

    drm_for_each_detailed_block((u8 *)edid, find_gtf2, &r);
    return r ? (r[15] << 8) + r[14] : 0;
}

static int
drm_gtf2_k(struct edid *edid)
{
    u8 *r = NULL;

    drm_for_each_detailed_block((u8 *)edid, find_gtf2, &r);
    return r ? r[16] : 0;
}

static int
drm_gtf2_2j(struct edid *edid)
{
    u8 *r = NULL;

    drm_for_each_detailed_block((u8 *)edid, find_gtf2, &r);
    return r ? r[17] : 0;
}

/**
 * standard_timing_level - get std. timing level(CVT/GTF/DMT)
 * @edid: EDID block to scan
 */
static int standard_timing_level(struct edid *edid)
{
    if (edid->revision >= 2) {
        if (edid->revision >= 4 &&
            (edid->features & DRM_EDID_FEATURE_DEFAULT_GTF))
            return LEVEL_CVT;
        if (drm_gtf2_hbreak(edid))
            return LEVEL_GTF2;
        return LEVEL_GTF;
    }
    return LEVEL_DMT;
}

/*
 * 0 is reserved.  The spec says 0x01 fill for unused timings.  Some old
 * monitors fill with ascii space (0x20) instead.
 */
static int
bad_std_timing(u8 a, u8 b)
{
    return (a == 0x00 && b == 0x00) ||
           (a == 0x01 && b == 0x01) ||
           (a == 0x20 && b == 0x20);
}

static void
is_rb(struct detailed_timing *t, void *data)
{
    u8 *r = (u8 *)t;

    if (r[3] == EDID_DETAIL_MONITOR_RANGE)
        if (r[15] & 0x10)
            *(bool *)data = true;
}

/* EDID 1.4 defines this explicitly.  For EDID 1.3, we guess, badly. */
static bool
drm_monitor_supports_rb(struct edid *edid)
{
    if (edid->revision >= 4) {
        bool ret = false;

        drm_for_each_detailed_block((u8 *)edid, is_rb, &ret);
        return ret;
    }

    return ((edid->input & DRM_EDID_INPUT_DIGITAL) != 0);
}

static bool
mode_is_rb(const struct drm_display_mode *mode)
{
    return (mode->htotal - mode->hdisplay == 160) &&
           (mode->hsync_end - mode->hdisplay == 80) &&
           (mode->hsync_end - mode->hsync_start == 32) &&
           (mode->vsync_start - mode->vdisplay == 3);
}

/*
 * drm_mode_find_dmt - Create a copy of a mode if present in DMT
 * @hsize: Mode width
 * @vsize: Mode height
 * @fresh: Mode refresh rate
 * @rb: Mode reduced-blanking-ness
 *
 * Walk the DMT mode list looking for a match for the given parameters.
 *
 * Return: A newly allocated copy of the mode, or NULL if not found.
 */
static struct drm_display_mode *drm_mode_find_dmt(
                       int hsize, int vsize, int fresh,
                       bool rb)
{
    int i;
    struct drm_display_mode *newmode;

    for (i = 0; i < ARRAY_SIZE(drm_dmt_modes); i++) {
        const struct drm_display_mode *ptr = &drm_dmt_modes[i];

        if (hsize != ptr->hdisplay)
            continue;
        if (vsize != ptr->vdisplay)
            continue;
        if (fresh != drm_get_vrefresh(ptr))
            continue;
        if (rb != mode_is_rb(ptr))
            continue;

        newmode = drm_mode_create();
        *newmode = *ptr;
        return newmode;
    }

    return NULL;
}

/** drm_mode_hsync - get the hsync of a mode
 * @mode: mode
 *
 * Returns:
 * @modes's hsync rate in kHz, rounded to the nearest integer. Calculates the
 * value first if it is not yet set.
 */
static int drm_mode_hsync(const struct drm_display_mode *mode)
{
    unsigned int calc_val;

    if (mode->htotal < 0)
        return 0;

    calc_val = (mode->clock * 1000) / mode->htotal; /* hsync in Hz */
    calc_val += 500;                /* round to 1000Hz */
    calc_val /= 1000;               /* truncate to kHz */

    return calc_val;
}

/**
 * drm_mode_std - convert standard mode info (width, height, refresh) into mode
 * @data: the structure that save parsed hdmi edid data
 * @edid: EDID block to scan
 * @t: standard timing params
 *
 * Take the standard timing params (in this case width, aspect, and refresh)
 * and convert them into a real mode using CVT/GTF/DMT.
 */
static struct drm_display_mode *
drm_mode_std(struct hdmi_edid_data *data, struct edid *edid,
         struct std_timing *t)
{
    struct drm_display_mode *mode = NULL;
    int i, hsize, vsize;
    int vrefresh_rate;
    int num = data->modes;
    unsigned aspect_ratio = (t->vfreq_aspect & EDID_TIMING_ASPECT_MASK)
        >> EDID_TIMING_ASPECT_SHIFT;
    unsigned vfreq = (t->vfreq_aspect & EDID_TIMING_VFREQ_MASK)
        >> EDID_TIMING_VFREQ_SHIFT;
    int timing_level = standard_timing_level(edid);

    if (bad_std_timing(t->hsize, t->vfreq_aspect))
        return NULL;

    /* According to the EDID spec, the hdisplay = hsize * 8 + 248 */
    hsize = t->hsize * 8 + 248;
    /* vrefresh_rate = vfreq + 60 */
    vrefresh_rate = vfreq + 60;
    /* the vdisplay is calculated based on the aspect ratio */
    if (aspect_ratio == 0) {
        if (edid->revision < 3)
            vsize = hsize;
        else
            vsize = (hsize * 10) / 16;
    } else if (aspect_ratio == 1) {
        vsize = (hsize * 3) / 4;
    } else if (aspect_ratio == 2) {
        vsize = (hsize * 4) / 5;
    } else {
        vsize = (hsize * 9) / 16;
    }

    /* HDTV hack, part 1 */
    if (vrefresh_rate == 60 &&
        ((hsize == 1360 && vsize == 765) ||
         (hsize == 1368 && vsize == 769))) {
        hsize = 1366;
        vsize = 768;
    }

    /*
     * If we already has a mode for this size and refresh
     * rate (because it came from detailed or CVT info), use that
     * instead.  This way we don't have to guess at interlace or
     * reduced blanking.
     */
    for (i = 0; i < num; i++)
        if (data->mode_buf[i].hdisplay == hsize &&
            data->mode_buf[i].vdisplay == vsize &&
            drm_get_vrefresh(&data->mode_buf[i]) == vrefresh_rate)
            return NULL;

    /* HDTV hack, part 2 */
    if (hsize == 1366 && vsize == 768 && vrefresh_rate == 60) {
        mode = drm_cvt_mode(1366, 768, vrefresh_rate, 0, 0,
                    false);
        mode->hdisplay = 1366;
        mode->hsync_start = mode->hsync_start - 1;
        mode->hsync_end = mode->hsync_end - 1;
        return mode;
    }

    /* check whether it can be found in default mode table */
    if (drm_monitor_supports_rb(edid)) {
        mode = drm_mode_find_dmt(hsize, vsize, vrefresh_rate,
                     true);
        if (mode)
            return mode;
    }

    mode = drm_mode_find_dmt(hsize, vsize, vrefresh_rate, false);
    if (mode)
        return mode;

    /* okay, generate it */
    switch (timing_level) {
    case LEVEL_DMT:
        break;
    case LEVEL_GTF:
        mode = drm_gtf_mode(hsize, vsize, vrefresh_rate, 0, 0);
        break;
    case LEVEL_GTF2:
        /*
         * This is potentially wrong if there's ever a monitor with
         * more than one ranges section, each claiming a different
         * secondary GTF curve.  Please don't do that.
         */
        mode = drm_gtf_mode(hsize, vsize, vrefresh_rate, 0, 0);
        if (!mode)
            return NULL;
        if (drm_mode_hsync(mode) > drm_gtf2_hbreak(edid)) {
            drm_mode_destroy(mode);
            mode = drm_gtf_mode_complex(hsize, vsize,
                            vrefresh_rate, 0, 0,
                            drm_gtf2_m(edid),
                            drm_gtf2_2c(edid),
                            drm_gtf2_k(edid),
                            drm_gtf2_2j(edid));
        }
        break;
    case LEVEL_CVT:
        mode = drm_cvt_mode(hsize, vsize, vrefresh_rate, 0, 0,
                    false);
        break;
    }

    return mode;
}

static void
do_standard_modes(struct detailed_timing *timing, void *c)
{
    struct detailed_mode_closure *closure = c;
    struct detailed_non_pixel *data = &timing->data.other_data;
    struct edid *edid = closure->edid;

    if (data->type == EDID_DETAIL_STD_MODES) {
        int i;

        for (i = 0; i < 6; i++) {
            struct std_timing *std;
            struct drm_display_mode *newmode;

            std = &data->data.timings[i];
            newmode = drm_mode_std(closure->data, edid, std);
            if (newmode) {
                drm_add_hdmi_modes(closure->data, newmode);
                closure->modes++;
                drm_mode_destroy(newmode);
            }
        }
    }
}

/**
 * add_standard_modes - get std. modes from EDID and add them
 * @data: data to add mode(s) to
 * @edid: EDID block to scan
 *
 * Standard modes can be calculated using the appropriate standard (DMT,
 * GTF or CVT. Grab them from @edid and add them to the list.
 */
static int
add_standard_modes(struct hdmi_edid_data *data, struct edid *edid)
{
    int i, modes = 0;
    struct detailed_mode_closure closure = {
        .data = data,
        .edid = edid,
    };

    for (i = 0; i < EDID_STD_TIMINGS; i++) {
        struct drm_display_mode *newmode;

        newmode = drm_mode_std(data, edid,
                       &edid->standard_timings[i]);
        if (newmode) {
            drm_add_hdmi_modes(data, newmode);
            modes++;
            drm_mode_destroy(newmode);
        }
    }

    if (version_greater(edid, 1, 0))
        drm_for_each_detailed_block((u8 *)edid, do_standard_modes,
                        &closure);

    /* XXX should also look for standard codes in VTB blocks */

    return modes + closure.modes;
}

static int
drm_est3_modes(struct hdmi_edid_data *data, struct detailed_timing *timing)
{
    int i, j, m, modes = 0;
    struct drm_display_mode *mode;
    u8 *est = ((u8 *)timing) + 6;

    for (i = 0; i < 6; i++) {
        for (j = 7; j >= 0; j--) {
            m = (i * 8) + (7 - j);
            if (m >= ARRAY_SIZE(est3_modes))
                break;
            if (est[i] & (1 << j)) {
                mode = drm_mode_find_dmt(
                             est3_modes[m].w,
                             est3_modes[m].h,
                             est3_modes[m].r,
                             est3_modes[m].rb);
                if (mode) {
                    drm_add_hdmi_modes(data, mode);
                    modes++;
                    drm_mode_destroy(mode);
                }
            }
        }
    }

    return modes;
}

static void
do_established_modes(struct detailed_timing *timing, void *c)
{
    struct detailed_mode_closure *closure = c;
    struct detailed_non_pixel *data = &timing->data.other_data;

    if (data->type == EDID_DETAIL_EST_TIMINGS)
        closure->modes += drm_est3_modes(closure->data, timing);
}

/**
 * add_established_modes - get est. modes from EDID and add them
 * @data: data to add mode(s) to
 * @edid: EDID block to scan
 *
 * Each EDID block contains a bitmap of the supported "established modes" list
 * (defined above).  Tease them out and add them to the modes list.
 */
static int
add_established_modes(struct hdmi_edid_data *data, struct edid *edid)
{
    unsigned long est_bits = edid->established_timings.t1 |
        (edid->established_timings.t2 << 8) |
        ((edid->established_timings.mfg_rsvd & 0x80) << 9);
    int i, modes = 0;
    struct detailed_mode_closure closure = {
        .data = data,
        .edid = edid,
    };

    for (i = 0; i <= EDID_EST_TIMINGS; i++) {
        if (est_bits & (1 << i)) {
            struct drm_display_mode *newmode = drm_mode_create();
            *newmode = edid_est_modes[i];
            if (newmode) {
                drm_add_hdmi_modes(data, newmode);
                modes++;
                drm_mode_destroy(newmode);
            }
        }
    }

    if (version_greater(edid, 1, 0))
        drm_for_each_detailed_block((u8 *)edid,
                        do_established_modes, &closure);

    return modes + closure.modes;
}

static u8 drm_match_hdmi_mode(const struct drm_display_mode *to_match)
{
    u8 vic;

    if (!to_match->clock)
        return 0;

    for (vic = 1; vic < ARRAY_SIZE(edid_4k_modes); vic++) {
        const struct drm_display_mode *hdmi_mode = &edid_4k_modes[vic];
        unsigned int clock1, clock2;

        /* Make sure to also match alternate clocks */
        clock1 = hdmi_mode->clock;
        clock2 = hdmi_mode_alternate_clock(hdmi_mode);

        if ((KHZ2PICOS(to_match->clock) == KHZ2PICOS(clock1) ||
             KHZ2PICOS(to_match->clock) == KHZ2PICOS(clock2)) &&
            drm_mode_equal_no_clocks_no_stereo(to_match, hdmi_mode))
            return vic;
    }
    return 0;
}

static int
add_alternate_cea_modes(struct hdmi_edid_data *data, struct edid *edid)
{
    struct drm_display_mode *mode;
    int i, num, modes = 0;

    /* Don't add CEA modes if the CEA extension block is missing */
    if (!drm_find_cea_extension(edid))
        return 0;

    /*
     * Go through all probed modes and create a new mode
     * with the alternate clock for certain CEA modes.
     */
    num = data->modes;

    for (i = 0; i < num; i++) {
        const struct drm_display_mode *cea_mode = NULL;
        struct drm_display_mode *newmode;
        u8 vic;
        unsigned int clock1, clock2;

        mode = &data->mode_buf[i];
        vic = drm_match_cea_mode(mode);

        if (drm_valid_cea_vic(vic)) {
            cea_mode = &edid_cea_modes[vic];
            clock2 = cea_mode_alternate_clock(cea_mode);
        } else {
            vic = drm_match_hdmi_mode(mode);
            if (drm_valid_hdmi_vic(vic)) {
                cea_mode = &edid_4k_modes[vic];
                clock2 = hdmi_mode_alternate_clock(cea_mode);
            }
        }

        if (!cea_mode)
            continue;

        clock1 = cea_mode->clock;

        if (clock1 == clock2)
            continue;

        if (mode->clock != clock1 && mode->clock != clock2)
            continue;

        newmode = drm_mode_create();
        *newmode = *cea_mode;
        if (!newmode)
            continue;

        /* Carry over the stereo flags */
        newmode->flags |= mode->flags & DRM_MODE_FLAG_3D_MASK;

        /*
         * The current mode could be either variant. Make
         * sure to pick the "other" clock for the new mode.
         */
        if (mode->clock != clock1)
            newmode->clock = clock1;
        else
            newmode->clock = clock2;

        drm_add_hdmi_modes(data, newmode);
        modes++;
        drm_mode_destroy(newmode);
    }

    return modes;
}

static u8 *drm_find_displayid_extension(struct edid *edid)
{
    return drm_find_edid_extension(edid, DISPLAYID_EXT);
}

static int validate_displayid(u8 *displayid, int length, int idx)
{
    int i;
    u8 csum = 0;
    struct displayid_hdr *base;

    base = (struct displayid_hdr *)&displayid[idx];

    debug("base revision 0x%x, length %d, %d %d\n",
          base->rev, base->bytes, base->prod_id, base->ext_count);

    if (base->bytes + 5 > length - idx)
        return -EINVAL;
    for (i = idx; i <= base->bytes + 5; i++)
        csum += displayid[i];
    if (csum) {
        debug("DisplayID checksum invalid, remainder is %d\n", csum);
        return -EINVAL;
    }
    return 0;
}

static struct
drm_display_mode *drm_displayid_detailed(struct displayid_detailed_timings_1
                          *timings)
{
    struct drm_display_mode *mode;
    unsigned pixel_clock = (timings->pixel_clock[0] |
                (timings->pixel_clock[1] << 8) |
                (timings->pixel_clock[2] << 16));
    unsigned hactive = (timings->hactive[0] | timings->hactive[1] << 8) + 1;
    unsigned hblank = (timings->hblank[0] | timings->hblank[1] << 8) + 1;
    unsigned hsync = (timings->hsync[0] |
        (timings->hsync[1] & 0x7f) << 8) + 1;
    unsigned hsync_width = (timings->hsw[0] | timings->hsw[1] << 8) + 1;
    unsigned vactive = (timings->vactive[0] |
        timings->vactive[1] << 8) + 1;
    unsigned vblank = (timings->vblank[0] | timings->vblank[1] << 8) + 1;
    unsigned vsync = (timings->vsync[0] |
        (timings->vsync[1] & 0x7f) << 8) + 1;
    unsigned vsync_width = (timings->vsw[0] | timings->vsw[1] << 8) + 1;
    bool hsync_positive = (timings->hsync[1] >> 7) & 0x1;
    bool vsync_positive = (timings->vsync[1] >> 7) & 0x1;

    mode = drm_mode_create();
    if (!mode)
        return NULL;

    mode->clock = pixel_clock * 10;
    mode->hdisplay = hactive;
    mode->hsync_start = mode->hdisplay + hsync;
    mode->hsync_end = mode->hsync_start + hsync_width;
    mode->htotal = mode->hdisplay + hblank;

    mode->vdisplay = vactive;
    mode->vsync_start = mode->vdisplay + vsync;
    mode->vsync_end = mode->vsync_start + vsync_width;
    mode->vtotal = mode->vdisplay + vblank;

    mode->flags = 0;
    mode->flags |=
        hsync_positive ? DRM_MODE_FLAG_PHSYNC : DRM_MODE_FLAG_NHSYNC;
    mode->flags |=
        vsync_positive ? DRM_MODE_FLAG_PVSYNC : DRM_MODE_FLAG_NVSYNC;
    mode->type = DRM_MODE_TYPE_DRIVER;

    if (timings->flags & 0x80)
        mode->type |= DRM_MODE_TYPE_PREFERRED;
    mode->vrefresh = drm_get_vrefresh(mode);

    return mode;
}

static int add_displayid_detailed_1_modes(struct hdmi_edid_data *data,
                      struct displayid_block *block)
{
    struct displayid_detailed_timing_block *det;
    int i;
    int num_timings;
    struct drm_display_mode *newmode;
    int num_modes = 0;

    det = (struct displayid_detailed_timing_block *)block;
    /* blocks must be multiple of 20 bytes length */
    if (block->num_bytes % 20)
        return 0;

    num_timings = block->num_bytes / 20;
    for (i = 0; i < num_timings; i++) {
        struct displayid_detailed_timings_1 *timings =
            &det->timings[i];

        newmode = drm_displayid_detailed(timings);
        if (!newmode)
            continue;

        drm_add_hdmi_modes(data, newmode);
        num_modes++;
        drm_mode_destroy(newmode);
    }
    return num_modes;
}

static int add_displayid_detailed_modes(struct hdmi_edid_data *data,
                    struct edid *edid)
{
    u8 *displayid;
    int ret;
    int idx = 1;
    int length = EDID_SIZE;
    struct displayid_block *block;
    int num_modes = 0;

    displayid = drm_find_displayid_extension(edid);
    if (!displayid)
        return 0;

    ret = validate_displayid(displayid, length, idx);
    if (ret)
        return 0;

    idx += sizeof(struct displayid_hdr);
    while (block = (struct displayid_block *)&displayid[idx],
           idx + sizeof(struct displayid_block) <= length &&
           idx + sizeof(struct displayid_block) + block->num_bytes <=
           length && block->num_bytes > 0) {
        idx += block->num_bytes + sizeof(struct displayid_block);
        switch (block->tag) {
        case DATA_BLOCK_TYPE_1_DETAILED_TIMING:
            num_modes +=
                add_displayid_detailed_1_modes(data, block);
            break;
        }
    }
    return num_modes;
}

static bool
mode_in_hsync_range(const struct drm_display_mode *mode,
            struct edid *edid, u8 *t)
{
    int hsync, hmin, hmax;

    hmin = t[7];
    if (edid->revision >= 4)
        hmin += ((t[4] & 0x04) ? 255 : 0);
    hmax = t[8];
    if (edid->revision >= 4)
        hmax += ((t[4] & 0x08) ? 255 : 0);
    hsync = drm_mode_hsync(mode);

    return (hsync <= hmax && hsync >= hmin);
}

static bool
mode_in_vsync_range(const struct drm_display_mode *mode,
            struct edid *edid, u8 *t)
{
    int vsync, vmin, vmax;

    vmin = t[5];
    if (edid->revision >= 4)
        vmin += ((t[4] & 0x01) ? 255 : 0);
    vmax = t[6];
    if (edid->revision >= 4)
        vmax += ((t[4] & 0x02) ? 255 : 0);
    vsync = drm_get_vrefresh(mode);

    return (vsync <= vmax && vsync >= vmin);
}

static u32
range_pixel_clock(struct edid *edid, u8 *t)
{
    /* unspecified */
    if (t[9] == 0 || t[9] == 255)
        return 0;

    /* 1.4 with CVT support gives us real precision, yay */
    if (edid->revision >= 4 && t[10] == 0x04)
        return (t[9] * 10000) - ((t[12] >> 2) * 250);

    /* 1.3 is pathetic, so fuzz up a bit */
    return t[9] * 10000 + 5001;
}

static bool
mode_in_range(const struct drm_display_mode *mode, struct edid *edid,
          struct detailed_timing *timing)
{
    u32 max_clock;
    u8 *t = (u8 *)timing;

    if (!mode_in_hsync_range(mode, edid, t))
        return false;

    if (!mode_in_vsync_range(mode, edid, t))
        return false;

    max_clock = range_pixel_clock(edid, t);
    if (max_clock)
        if (mode->clock > max_clock)
            return false;

    /* 1.4 max horizontal check */
    if (edid->revision >= 4 && t[10] == 0x04)
        if (t[13] && mode->hdisplay > 8 *
            (t[13] + (256 * (t[12] & 0x3))))
            return false;

    if (mode_is_rb(mode) && !drm_monitor_supports_rb(edid))
        return false;

    return true;
}

static bool valid_inferred_mode(struct hdmi_edid_data *data,
                const struct drm_display_mode *mode)
{
    const struct drm_display_mode *m;
    bool ok = false;
    int i;

    for (i = 0; i < data->modes; i++) {
        m = &data->mode_buf[i];
        if (mode->hdisplay == m->hdisplay &&
            mode->vdisplay == m->vdisplay &&
            drm_get_vrefresh(mode) == drm_get_vrefresh(m))
            return false; /* duplicated */
        if (mode->hdisplay <= m->hdisplay &&
            mode->vdisplay <= m->vdisplay)
            ok = true;
    }
    return ok;
}

static int
drm_dmt_modes_for_range(struct hdmi_edid_data *data, struct edid *edid,
            struct detailed_timing *timing)
{
    int i, modes = 0;

    for (i = 0; i < ARRAY_SIZE(drm_dmt_modes); i++) {
        if (mode_in_range(drm_dmt_modes + i, edid, timing) &&
            valid_inferred_mode(data, drm_dmt_modes + i)) {
            drm_add_hdmi_modes(data, &drm_dmt_modes[i]);
            modes++;
        }
    }

    return modes;
}

/* fix up 1366x768 mode from 1368x768;
 * GFT/CVT can't express 1366 width which isn't dividable by 8
 */
static void fixup_mode_1366x768(struct drm_display_mode *mode)
{
    if (mode->hdisplay == 1368 && mode->vdisplay == 768) {
        mode->hdisplay = 1366;
        mode->hsync_start--;
        mode->hsync_end--;
    }
}

static int
drm_gtf_modes_for_range(struct hdmi_edid_data *data, struct edid *edid,
            struct detailed_timing *timing)
{
    int i, modes = 0;
    struct drm_display_mode *newmode;

    for (i = 0; i < ARRAY_SIZE(extra_modes); i++) {
        const struct minimode *m = &extra_modes[i];

        newmode = drm_gtf_mode(m->w, m->h, m->r, 0, 0);
        if (!newmode)
            return modes;

        fixup_mode_1366x768(newmode);
        if (!mode_in_range(newmode, edid, timing) ||
            !valid_inferred_mode(data, newmode)) {
            drm_mode_destroy(newmode);
            continue;
        }

        drm_add_hdmi_modes(data, newmode);
        modes++;
        drm_mode_destroy(newmode);
    }

    return modes;
}

static int
drm_cvt_modes_for_range(struct hdmi_edid_data *data, struct edid *edid,
            struct detailed_timing *timing)
{
    int i, modes = 0;
    struct drm_display_mode *newmode;
    bool rb = drm_monitor_supports_rb(edid);

    for (i = 0; i < ARRAY_SIZE(extra_modes); i++) {
        const struct minimode *m = &extra_modes[i];

        newmode = drm_cvt_mode(m->w, m->h, m->r, rb, 0, 0);
        if (!newmode)
            return modes;

        fixup_mode_1366x768(newmode);
        if (!mode_in_range(newmode, edid, timing) ||
            !valid_inferred_mode(data, newmode)) {
            drm_mode_destroy(newmode);
            continue;
        }

        drm_add_hdmi_modes(data, newmode);
        modes++;
        drm_mode_destroy(newmode);
    }

    return modes;
}

static void
do_inferred_modes(struct detailed_timing *timing, void *c)
{
    struct detailed_mode_closure *closure = c;
    struct detailed_non_pixel *data = &timing->data.other_data;
    struct detailed_data_monitor_range *range = &data->data.range;

    if (data->type != EDID_DETAIL_MONITOR_RANGE)
        return;

    closure->modes += drm_dmt_modes_for_range(closure->data,
                          closure->edid,
                          timing);

    if (!version_greater(closure->edid, 1, 1))
        return; /* GTF not defined yet */

    switch (range->flags) {
    case 0x02: /* secondary gtf, XXX could do more */
    case 0x00: /* default gtf */
        closure->modes += drm_gtf_modes_for_range(closure->data,
                              closure->edid,
                              timing);
        break;
    case 0x04: /* cvt, only in 1.4+ */
        if (!version_greater(closure->edid, 1, 3))
            break;

        closure->modes += drm_cvt_modes_for_range(closure->data,
                              closure->edid,
                              timing);
        break;
    case 0x01: /* just the ranges, no formula */
    default:
        break;
    }
}

static int
add_inferred_modes(struct hdmi_edid_data *data, struct edid *edid)
{
    struct detailed_mode_closure closure = {
        .data = data,
        .edid = edid,
    };

    if (version_greater(edid, 1, 0))
        drm_for_each_detailed_block((u8 *)edid, do_inferred_modes,
                        &closure);

    return closure.modes;
}

#define MODE_SIZE(m) ((m)->hdisplay * (m)->vdisplay)
#define MODE_REFRESH_DIFF(c, t) (abs((c) - (t)))

/**
 * edid_fixup_preferred - set preferred modes based on quirk list
 * @data: the structure that save parsed hdmi edid data
 * @quirks: quirks list
 *
 * Walk the mode list, clearing the preferred status
 * on existing modes and setting it anew for the right mode ala @quirks.
 */
static void edid_fixup_preferred(struct hdmi_edid_data *data,
                 u32 quirks)
{
    struct drm_display_mode *cur_mode, *preferred_mode;
    int i, target_refresh = 0;
    int num = data->modes;
    int cur_vrefresh, preferred_vrefresh;

    if (!num)
        return;

    preferred_mode = data->preferred_mode;

    if (quirks & EDID_QUIRK_PREFER_LARGE_60)
        target_refresh = 60;
    if (quirks & EDID_QUIRK_PREFER_LARGE_75)
        target_refresh = 75;

    for (i = 0; i < num; i++) {
        cur_mode = &data->mode_buf[i];
        cur_mode->type &= ~DRM_MODE_TYPE_PREFERRED;

        if (cur_mode == preferred_mode)
            continue;

        /* Largest mode is preferred */
        if (MODE_SIZE(cur_mode) > MODE_SIZE(preferred_mode))
            preferred_mode = cur_mode;

        cur_vrefresh = cur_mode->vrefresh ?
        cur_mode->vrefresh : drm_get_vrefresh(cur_mode);
        preferred_vrefresh = preferred_mode->vrefresh ?
        preferred_mode->vrefresh : drm_get_vrefresh(preferred_mode);
        /* At a given size, try to get closest to target refresh */
        if ((MODE_SIZE(cur_mode) == MODE_SIZE(preferred_mode)) &&
            MODE_REFRESH_DIFF(cur_vrefresh, target_refresh) <
            MODE_REFRESH_DIFF(preferred_vrefresh, target_refresh)) {
            preferred_mode = cur_mode;
        }
    }
    preferred_mode->type |= DRM_MODE_TYPE_PREFERRED;
    data->preferred_mode = preferred_mode;
}

static const u8 edid_header[] = {
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00
};

/**
 * drm_edid_header_is_valid - sanity check the header of the base EDID block
 * @raw_edid: pointer to raw base EDID block
 *
 * Sanity check the header of the base EDID block.
 *
 * Return: 8 if the header is perfect, down to 0 if it's totally wrong.
 */
static int drm_edid_header_is_valid(const u8 *raw_edid)
{
    int i, score = 0;

    for (i = 0; i < sizeof(edid_header); i++)
        if (raw_edid[i] == edid_header[i])
            score++;

    return score;
}

static int drm_edid_block_checksum(const u8 *raw_edid)
{
    int i;
    u8 csum = 0;

    for (i = 0; i < EDID_SIZE; i++)
        csum += raw_edid[i];

    return csum;
}

static bool drm_edid_is_zero(const u8 *in_edid, int length)
{
    if (memchr_inv(in_edid, 0, length))
        return false;

    return true;
}

/**
 * drm_edid_block_valid - Sanity check the EDID block (base or extension)
 * @raw_edid: pointer to raw EDID block
 * @block: type of block to validate (0 for base, extension otherwise)
 * @print_bad_edid: if true, dump bad EDID blocks to the console
 * @edid_corrupt: if true, the header or checksum is invalid
 *
 * Validate a base or extension EDID block and optionally dump bad blocks to
 * the console.
 *
 * Return: True if the block is valid, false otherwise.
 */
static
bool drm_edid_block_valid(u8 *raw_edid, int block, bool print_bad_edid,
              bool *edid_corrupt)
{
    u8 csum;
    int edid_fixup = 6;
    struct edid *edid = (struct edid *)raw_edid;

    if ((!raw_edid))
        return false;

    if (block == 0) {
        int score = drm_edid_header_is_valid(raw_edid);

        if (score == 8) {
            if (edid_corrupt)
                *edid_corrupt = false;
        } else if (score >= edid_fixup) {
            /* Displayport Link CTS Core 1.2 rev1.1 test 4.2.2.6
             * The corrupt flag needs to be set here otherwise, the
             * fix-up code here will correct the problem, the
             * checksum is correct and the test fails
             */
            if (edid_corrupt)
                *edid_corrupt = true;
            debug("Fixing header, your hardware may be failing\n");
            memcpy(raw_edid, edid_header, sizeof(edid_header));
        } else {
            if (edid_corrupt)
                *edid_corrupt = true;
            goto bad;
        }
    }

    csum = drm_edid_block_checksum(raw_edid);
    if (csum) {
        if (print_bad_edid) {
            debug("EDID checksum is invalid, remainder is %d\n",
                  csum);
        }

        if (edid_corrupt)
            *edid_corrupt = true;

        /* allow CEA to slide through, switches mangle this */
        if (raw_edid[0] != 0x02)
            goto bad;
    }

    /* per-block-type checks */
    switch (raw_edid[0]) {
    case 0: /* base */
        if (edid->version != 1) {
            debug("EDID has major version %d, instead of 1\n",
                  edid->version);
            goto bad;
        }

        if (edid->revision > 4)
            debug("minor > 4, assuming backward compatibility\n");
        break;

    default:
        break;
    }

    return true;

bad:
    if (print_bad_edid) {
        if (drm_edid_is_zero(raw_edid, EDID_SIZE)) {
            debug("EDID block is all zeroes\n");
        } else {
            debug("Raw EDID:\n");
            print_hex_dump("", DUMP_PREFIX_NONE, 16, 1,
                       raw_edid, EDID_SIZE, false);
        }
    }
    return false;
}

/**
 * drm_edid_is_valid - sanity check EDID data
 * @edid: EDID data
 *
 * Sanity-check an entire EDID record (including extensions)
 *
 * Return: True if the EDID data is valid, false otherwise.
 */
static bool drm_edid_is_valid(struct edid *edid)
{
    int i;
    u8 *raw = (u8 *)edid;

    if (!edid)
        return false;

    for (i = 0; i <= edid->extensions; i++)
        if (!drm_edid_block_valid(raw + i * EDID_SIZE, i, true, NULL))
            return false;

    return true;
}

/**
 * drm_add_edid_modes - add modes from EDID data, if available
 * @data: data we're probing
 * @edid: EDID data
 *
 * Add the specified modes to the data's mode list.
 *
 * Return: The number of modes added or 0 if we couldn't find any.
 */
int drm_add_edid_modes(struct hdmi_edid_data *data, u8 *raw_edid)
{
    int num_modes = 0;
    u32 quirks;
    struct edid *edid = (struct edid *)raw_edid;

    if (!edid) {
        debug("no edid\n");
        return 0;
    }

    if (!drm_edid_is_valid(edid)) {
        debug("EDID invalid\n");
        return 0;
    }

    if (!data->mode_buf) {
        debug("mode buff is null\n");
        return 0;
    }

    quirks = edid_get_quirks(edid);
    /*
     * CEA-861-F adds ycbcr capability map block, for HDMI 2.0 sinks.
     * To avoid multiple parsing of same block, lets parse that map
     * from sink info, before parsing CEA modes.
     */
    drm_add_display_info(data, edid);

    /*
     * EDID spec says modes should be preferred in this order:
     * - preferred detailed mode
     * - other detailed modes from base block
     * - detailed modes from extension blocks
     * - CVT 3-byte code modes
     * - standard timing codes
     * - established timing codes
     * - modes inferred from GTF or CVT range information
     *
     * We get this pretty much right.
     *
     * XXX order for additional mode types in extension blocks?
     */
    num_modes += add_detailed_modes(data, edid, quirks);
    num_modes += add_cvt_modes(data, edid);
    num_modes += add_standard_modes(data, edid);
    num_modes += add_established_modes(data, edid);
    num_modes += add_cea_modes(data, edid);
    num_modes += add_alternate_cea_modes(data, edid);
    num_modes += add_displayid_detailed_modes(data, edid);

    if (edid->features & DRM_EDID_FEATURE_DEFAULT_GTF)
        num_modes += add_inferred_modes(data, edid);

    if (num_modes > 0)
        data->preferred_mode = &data->mode_buf[0];

    if (quirks & (EDID_QUIRK_PREFER_LARGE_60 | EDID_QUIRK_PREFER_LARGE_75))
        edid_fixup_preferred(data, quirks);

    if (quirks & EDID_QUIRK_FORCE_6BPC)
        data->display_info.bpc = 6;

    if (quirks & EDID_QUIRK_FORCE_8BPC)
        data->display_info.bpc = 8;

    if (quirks & EDID_QUIRK_FORCE_10BPC)
        data->display_info.bpc = 10;

    if (quirks & EDID_QUIRK_FORCE_12BPC)
        data->display_info.bpc = 12;

    return num_modes;
}

u8 drm_match_cea_mode(struct drm_display_mode *to_match)
{
    u8 vic;

    if (!to_match->clock) {
        printf("can't find to match\n");
        return 0;
    }

    for (vic = 1; vic < ARRAY_SIZE(edid_cea_modes); vic++) {
        const struct drm_display_mode *cea_mode = &edid_cea_modes[vic];
        unsigned int clock1, clock2;

        /* Check both 60Hz and 59.94Hz */
        clock1 = cea_mode->clock;
        clock2 = cea_mode_alternate_clock(cea_mode);
        if ((KHZ2PICOS(to_match->clock) == KHZ2PICOS(clock1) ||
             KHZ2PICOS(to_match->clock) == KHZ2PICOS(clock2)) &&
            drm_mode_equal_no_clocks_no_stereo(to_match, cea_mode))
            return vic;
    }

    return 0;
}

static enum hdmi_picture_aspect drm_get_cea_aspect_ratio(const u8 video_code)
{
    return edid_cea_modes[video_code].picture_aspect_ratio;
}

int
drm_hdmi_avi_infoframe_from_display_mode(struct hdmi_avi_infoframe *frame,
                     struct drm_display_mode *mode,
                     bool is_hdmi2_sink)
{
    int err;

    if (!frame || !mode)
        return -EINVAL;

    err = hdmi_avi_infoframe_init(frame);
    if (err < 0)
        return err;

    if (mode->flags & DRM_MODE_FLAG_DBLCLK)
        frame->pixel_repeat = 1;

    frame->video_code = drm_match_cea_mode(mode);

    /*
     * HDMI 1.4 VIC range: 1 <= VIC <= 64 (CEA-861-D) but
     * HDMI 2.0 VIC range: 1 <= VIC <= 107 (CEA-861-F). So we
     * have to make sure we dont break HDMI 1.4 sinks.
     */
    if (!is_hdmi2_sink && frame->video_code > 64)
        frame->video_code = 0;

    /*
     * HDMI spec says if a mode is found in HDMI 1.4b 4K modes
     * we should send its VIC in vendor infoframes, else send the
     * VIC in AVI infoframes. Lets check if this mode is present in
     * HDMI 1.4b 4K modes
     */
    if (frame->video_code) {
        u8 vendor_if_vic = drm_match_hdmi_mode(mode);
        bool is_s3d = mode->flags & DRM_MODE_FLAG_3D_MASK;

        if (drm_valid_hdmi_vic(vendor_if_vic) && !is_s3d)
            frame->video_code = 0;
    }

    frame->picture_aspect = HDMI_PICTURE_ASPECT_NONE;

    /*
     * Populate picture aspect ratio from either
     * user input (if specified) or from the CEA mode list.
     */
    if (mode->picture_aspect_ratio == HDMI_PICTURE_ASPECT_4_3 ||
        mode->picture_aspect_ratio == HDMI_PICTURE_ASPECT_16_9)
        frame->picture_aspect = mode->picture_aspect_ratio;
    else if (frame->video_code > 0)
        frame->picture_aspect = drm_get_cea_aspect_ratio(
                        frame->video_code);

    frame->active_aspect = HDMI_ACTIVE_ASPECT_PICTURE;
    frame->scan_mode = HDMI_SCAN_MODE_UNDERSCAN;

    return 0;
}

/**
 * hdmi_vendor_infoframe_init() - initialize an HDMI vendor infoframe
 * @frame: HDMI vendor infoframe
 *
 * Returns 0 on success or a negative error code on failure.
 */
int hdmi_vendor_infoframe_init(struct hdmi_vendor_infoframe *frame)
{
    memset(frame, 0, sizeof(*frame));

    frame->type = HDMI_INFOFRAME_TYPE_VENDOR;
    frame->version = 1;

    frame->oui = HDMI_IEEE_OUI;

    /*
     * 0 is a valid value for s3d_struct, so we use a special "not set"
     * value
     */
    frame->s3d_struct = HDMI_3D_STRUCTURE_INVALID;

    return 0;
}

/**
 * drm_hdmi_avi_infoframe_quant_range() - fill the HDMI AVI infoframe
 *                                        quantization range information
 * @frame: HDMI AVI infoframe
 * @rgb_quant_range: RGB quantization range (Q)
 * @rgb_quant_range_selectable: Sink support selectable RGB quantization range (QS)
 */
void
drm_hdmi_avi_infoframe_quant_range(struct hdmi_avi_infoframe *frame,
                   struct drm_display_mode *mode,
                   enum hdmi_quantization_range rgb_quant_range,
                   bool rgb_quant_range_selectable)
{
    /*
     * CEA-861:
     * "A Source shall not send a non-zero Q value that does not correspond
     *  to the default RGB Quantization Range for the transmitted Picture
     *  unless the Sink indicates support for the Q bit in a Video
     *  Capabilities Data Block."
     *
     * HDMI 2.0 recommends sending non-zero Q when it does match the
     * default RGB quantization range for the mode, even when QS=0.
     */
    if (rgb_quant_range_selectable ||
        rgb_quant_range == drm_default_rgb_quant_range(mode))
        frame->quantization_range = rgb_quant_range;
    else
        frame->quantization_range = HDMI_QUANTIZATION_RANGE_DEFAULT;

    /*
     * CEA-861-F:
     * "When transmitting any RGB colorimetry, the Source should set the
     *  YQ-field to match the RGB Quantization Range being transmitted
     *  (e.g., when Limited Range RGB, set YQ=0 or when Full Range RGB,
     *  set YQ=1) and the Sink shall ignore the YQ-field."
     */
    if (rgb_quant_range == HDMI_QUANTIZATION_RANGE_LIMITED)
        frame->ycc_quantization_range =
            HDMI_YCC_QUANTIZATION_RANGE_LIMITED;
    else
        frame->ycc_quantization_range =
            HDMI_YCC_QUANTIZATION_RANGE_FULL;
}

static enum hdmi_3d_structure
s3d_structure_from_display_mode(const struct drm_display_mode *mode)
{
    u32 layout = mode->flags & DRM_MODE_FLAG_3D_MASK;

    switch (layout) {
    case DRM_MODE_FLAG_3D_FRAME_PACKING:
        return HDMI_3D_STRUCTURE_FRAME_PACKING;
    case DRM_MODE_FLAG_3D_FIELD_ALTERNATIVE:
        return HDMI_3D_STRUCTURE_FIELD_ALTERNATIVE;
    case DRM_MODE_FLAG_3D_LINE_ALTERNATIVE:
        return HDMI_3D_STRUCTURE_LINE_ALTERNATIVE;
    case DRM_MODE_FLAG_3D_SIDE_BY_SIDE_FULL:
        return HDMI_3D_STRUCTURE_SIDE_BY_SIDE_FULL;
    case DRM_MODE_FLAG_3D_L_DEPTH:
        return HDMI_3D_STRUCTURE_L_DEPTH;
    case DRM_MODE_FLAG_3D_L_DEPTH_GFX_GFX_DEPTH:
        return HDMI_3D_STRUCTURE_L_DEPTH_GFX_GFX_DEPTH;
    case DRM_MODE_FLAG_3D_TOP_AND_BOTTOM:
        return HDMI_3D_STRUCTURE_TOP_AND_BOTTOM;
    case DRM_MODE_FLAG_3D_SIDE_BY_SIDE_HALF:
        return HDMI_3D_STRUCTURE_SIDE_BY_SIDE_HALF;
    default:
        return HDMI_3D_STRUCTURE_INVALID;
    }
}

int
drm_hdmi_vendor_infoframe_from_display_mode(struct hdmi_vendor_infoframe *frame,
                        struct drm_display_mode *mode)
{
    int err;
    u32 s3d_flags;
    u8 vic;

    if (!frame || !mode)
        return -EINVAL;

    vic = drm_match_hdmi_mode(mode);

    s3d_flags = mode->flags & DRM_MODE_FLAG_3D_MASK;

    if (!vic && !s3d_flags)
        return -EINVAL;

    if (vic && s3d_flags)
        return -EINVAL;

    err = hdmi_vendor_infoframe_init(frame);
    if (err < 0)
        return err;

    if (vic)
        frame->vic = vic;
    else
        frame->s3d_struct = s3d_structure_from_display_mode(mode);

    return 0;
}

static u8 hdmi_infoframe_checksum(u8 *ptr, size_t size)
{
    u8 csum = 0;
    size_t i;

    /* compute checksum */
    for (i = 0; i < size; i++)
        csum += ptr[i];

    return 256 - csum;
}

static void hdmi_infoframe_set_checksum(void *buffer, size_t size)
{
    u8 *ptr = buffer;

    ptr[3] = hdmi_infoframe_checksum(buffer, size);
}

/**
 * hdmi_avi_infoframe_init() - initialize an HDMI AVI infoframe
 * @frame: HDMI AVI infoframe
 *
 * Returns 0 on success or a negative error code on failure.
 */
int hdmi_avi_infoframe_init(struct hdmi_avi_infoframe *frame)
{
    memset(frame, 0, sizeof(*frame));

    frame->type = HDMI_INFOFRAME_TYPE_AVI;
    frame->version = 2;
    frame->length = HDMI_AVI_INFOFRAME_SIZE;

    return 0;
}
EXPORT_SYMBOL(hdmi_avi_infoframe_init);

/**
 * hdmi_avi_infoframe_pack() - write HDMI AVI infoframe to binary buffer
 * @frame: HDMI AVI infoframe
 * @buffer: destination buffer
 * @size: size of buffer
 *
 * Packs the information contained in the @frame structure into a binary
 * representation that can be written into the corresponding controller
 * registers. Also computes the checksum as required by section 5.3.5 of
 * the HDMI 1.4 specification.
 *
 * Returns the number of bytes packed into the binary buffer or a negative
 * error code on failure.
 */
ssize_t hdmi_avi_infoframe_pack(struct hdmi_avi_infoframe *frame, void *buffer,
                size_t size)
{
    u8 *ptr = buffer;
    size_t length;

    length = HDMI_INFOFRAME_HEADER_SIZE + frame->length;

    if (size < length)
        return -ENOSPC;

    memset(buffer, 0, size);

    ptr[0] = frame->type;
    ptr[1] = frame->version;
    ptr[2] = frame->length;
    ptr[3] = 0; /* checksum */

    /* start infoframe payload */
    ptr += HDMI_INFOFRAME_HEADER_SIZE;

    ptr[0] = ((frame->colorspace & 0x3) << 5) | (frame->scan_mode & 0x3);

    /*
     * Data byte 1, bit 4 has to be set if we provide the active format
     * aspect ratio
     */
    if (frame->active_aspect & 0xf)
        ptr[0] |= BIT(4);

    /* Bit 3 and 2 indicate if we transmit horizontal/vertical bar data */
    if (frame->top_bar || frame->bottom_bar)
        ptr[0] |= BIT(3);

    if (frame->left_bar || frame->right_bar)
        ptr[0] |= BIT(2);

    ptr[1] = ((frame->colorimetry & 0x3) << 6) |
         ((frame->picture_aspect & 0x3) << 4) |
         (frame->active_aspect & 0xf);

    ptr[2] = ((frame->extended_colorimetry & 0x7) << 4) |
         ((frame->quantization_range & 0x3) << 2) |
         (frame->nups & 0x3);

    if (frame->itc)
        ptr[2] |= BIT(7);

    ptr[3] = frame->video_code & 0x7f;

    ptr[4] = ((frame->ycc_quantization_range & 0x3) << 6) |
         ((frame->content_type & 0x3) << 4) |
         (frame->pixel_repeat & 0xf);

    ptr[5] = frame->top_bar & 0xff;
    ptr[6] = (frame->top_bar >> 8) & 0xff;
    ptr[7] = frame->bottom_bar & 0xff;
    ptr[8] = (frame->bottom_bar >> 8) & 0xff;
    ptr[9] = frame->left_bar & 0xff;
    ptr[10] = (frame->left_bar >> 8) & 0xff;
    ptr[11] = frame->right_bar & 0xff;
    ptr[12] = (frame->right_bar >> 8) & 0xff;

    hdmi_infoframe_set_checksum(buffer, length);

    return length;
}
EXPORT_SYMBOL(hdmi_avi_infoframe_pack);

/**
 * hdmi_spd_infoframe_init() - initialize an HDMI SPD infoframe
 * @frame: HDMI SPD infoframe
 * @vendor: vendor string
 * @product: product string
 *
 * Returns 0 on success or a negative error code on failure.
 */
int hdmi_spd_infoframe_init(struct hdmi_spd_infoframe *frame,
                const char *vendor, const char *product)
{
    memset(frame, 0, sizeof(*frame));

    frame->type = HDMI_INFOFRAME_TYPE_SPD;
    frame->version = 1;
    frame->length = HDMI_SPD_INFOFRAME_SIZE;

    strncpy(frame->vendor, vendor, sizeof(frame->vendor));
    strncpy(frame->product, product, sizeof(frame->product));

    return 0;
}
EXPORT_SYMBOL(hdmi_spd_infoframe_init);

/**
 * hdmi_spd_infoframe_pack() - write HDMI SPD infoframe to binary buffer
 * @frame: HDMI SPD infoframe
 * @buffer: destination buffer
 * @size: size of buffer
 *
 * Packs the information contained in the @frame structure into a binary
 * representation that can be written into the corresponding controller
 * registers. Also computes the checksum as required by section 5.3.5 of
 * the HDMI 1.4 specification.
 *
 * Returns the number of bytes packed into the binary buffer or a negative
 * error code on failure.
 */
ssize_t hdmi_spd_infoframe_pack(struct hdmi_spd_infoframe *frame, void *buffer,
                size_t size)
{
    u8 *ptr = buffer;
    size_t length;

    length = HDMI_INFOFRAME_HEADER_SIZE + frame->length;

    if (size < length)
        return -ENOSPC;

    memset(buffer, 0, size);

    ptr[0] = frame->type;
    ptr[1] = frame->version;
    ptr[2] = frame->length;
    ptr[3] = 0; /* checksum */

    /* start infoframe payload */
    ptr += HDMI_INFOFRAME_HEADER_SIZE;

    memcpy(ptr, frame->vendor, sizeof(frame->vendor));
    memcpy(ptr + 8, frame->product, sizeof(frame->product));

    ptr[24] = frame->sdi;

    hdmi_infoframe_set_checksum(buffer, length);

    return length;
}
EXPORT_SYMBOL(hdmi_spd_infoframe_pack);

/**
 * hdmi_audio_infoframe_init() - initialize an HDMI audio infoframe
 * @frame: HDMI audio infoframe
 *
 * Returns 0 on success or a negative error code on failure.
 */
int hdmi_audio_infoframe_init(struct hdmi_audio_infoframe *frame)
{
    memset(frame, 0, sizeof(*frame));

    frame->type = HDMI_INFOFRAME_TYPE_AUDIO;
    frame->version = 1;
    frame->length = HDMI_AUDIO_INFOFRAME_SIZE;

    return 0;
}

/**
 * hdmi_audio_infoframe_pack() - write HDMI audio infoframe to binary buffer
 * @frame: HDMI audio infoframe
 * @buffer: destination buffer
 * @size: size of buffer
 *
 * Packs the information contained in the @frame structure into a binary
 * representation that can be written into the corresponding controller
 * registers. Also computes the checksum as required by section 5.3.5 of
 * the HDMI 1.4 specification.
 *
 * Returns the number of bytes packed into the binary buffer or a negative
 * error code on failure.
 */
ssize_t hdmi_audio_infoframe_pack(struct hdmi_audio_infoframe *frame,
                  void *buffer, size_t size)
{
    unsigned char channels;
    char *ptr = buffer;
    size_t length;

    length = HDMI_INFOFRAME_HEADER_SIZE + frame->length;

    if (size < length)
        return -ENOSPC;

    memset(buffer, 0, size);

    if (frame->channels >= 2)
        channels = frame->channels - 1;
    else
        channels = 0;

    ptr[0] = frame->type;
    ptr[1] = frame->version;
    ptr[2] = frame->length;
    ptr[3] = 0; /* checksum */

    /* start infoframe payload */
    ptr += HDMI_INFOFRAME_HEADER_SIZE;

    ptr[0] = ((frame->coding_type & 0xf) << 4) | (channels & 0x7);
    ptr[1] = ((frame->sample_frequency & 0x7) << 2) |
         (frame->sample_size & 0x3);
    ptr[2] = frame->coding_type_ext & 0x1f;
    ptr[3] = frame->channel_allocation;
    ptr[4] = (frame->level_shift_value & 0xf) << 3;

    if (frame->downmix_inhibit)
        ptr[4] |= BIT(7);

    hdmi_infoframe_set_checksum(buffer, length);

    return length;
}

/**
 * hdmi_vendor_infoframe_pack() - write a HDMI vendor infoframe to binary buffer
 * @frame: HDMI infoframe
 * @buffer: destination buffer
 * @size: size of buffer
 *
 * Packs the information contained in the @frame structure into a binary
 * representation that can be written into the corresponding controller
 * registers. Also computes the checksum as required by section 5.3.5 of
 * the HDMI 1.4 specification.
 *
 * Returns the number of bytes packed into the binary buffer or a negative
 * error code on failure.
 */
ssize_t hdmi_vendor_infoframe_pack(struct hdmi_vendor_infoframe *frame,
                   void *buffer, size_t size)
{
    char *ptr = buffer;
    size_t length;

    /* empty info frame */
    if (frame->vic == 0 && frame->s3d_struct == HDMI_3D_STRUCTURE_INVALID)
        return -EINVAL;

    /* only one of those can be supplied */
    if (frame->vic != 0 && frame->s3d_struct != HDMI_3D_STRUCTURE_INVALID)
        return -EINVAL;

    /* for side by side (half) we also need to provide 3D_Ext_Data */
    if (frame->s3d_struct >= HDMI_3D_STRUCTURE_SIDE_BY_SIDE_HALF)
        frame->length = 6;
    else
        frame->length = 5;

    length = HDMI_INFOFRAME_HEADER_SIZE + frame->length;

    if (size < length)
        return -ENOSPC;

    memset(buffer, 0, size);

    ptr[0] = frame->type;
    ptr[1] = frame->version;
    ptr[2] = frame->length;
    ptr[3] = 0; /* checksum */

    /* HDMI OUI */
    ptr[4] = 0x03;
    ptr[5] = 0x0c;
    ptr[6] = 0x00;

    if (frame->vic) {
        ptr[7] = 0x1 << 5;  /* video format */
        ptr[8] = frame->vic;
    } else {
        ptr[7] = 0x2 << 5;  /* video format */
        ptr[8] = (frame->s3d_struct & 0xf) << 4;
        if (frame->s3d_struct >= HDMI_3D_STRUCTURE_SIDE_BY_SIDE_HALF)
            ptr[9] = (frame->s3d_ext_data & 0xf) << 4;
    }

    hdmi_infoframe_set_checksum(buffer, length);

    return length;
}

/**
 * hdmi_drm_infoframe_init() - initialize an HDMI Dynaminc Range and
 * mastering infoframe
 * @frame: HDMI DRM infoframe
 *
 * Returns 0 on success or a negative error code on failure.
 */
int hdmi_drm_infoframe_init(struct hdmi_drm_infoframe *frame)
{
    memset(frame, 0, sizeof(*frame));

    frame->type = HDMI_INFOFRAME_TYPE_DRM;
    frame->version = 1;

    return 0;
}

/**
 * hdmi_drm_infoframe_pack() - write HDMI DRM infoframe to binary buffer
 * @frame: HDMI DRM infoframe
 * @buffer: destination buffer
 * @size: size of buffer
 *
 * Packs the information contained in the @frame structure into a binary
 * representation that can be written into the corresponding controller
 * registers. Also computes the checksum as required by section 5.3.5 of
 * the HDMI 1.4 specification.
 *
 * Returns the number of bytes packed into the binary buffer or a negative
 * error code on failure.
 */
ssize_t hdmi_drm_infoframe_pack(struct hdmi_drm_infoframe *frame, void *buffer,
                size_t size)
{
    u8 *ptr = buffer;
    size_t length;

    length = HDMI_INFOFRAME_HEADER_SIZE + frame->length;

    if (size < length)
        return -ENOSPC;

    memset(buffer, 0, size);

    ptr[0] = frame->type;
    ptr[1] = frame->version;
    ptr[2] = frame->length;
    ptr[3] = 0; /* checksum */

    /* start infoframe payload */
    ptr += HDMI_INFOFRAME_HEADER_SIZE;

    ptr[0] = frame->eotf;
    ptr[1] = frame->metadata_type;

    ptr[2] = frame->display_primaries_x[0] & 0xff;
    ptr[3] = frame->display_primaries_x[0] >> 8;

    ptr[4] = frame->display_primaries_x[1] & 0xff;
    ptr[5] = frame->display_primaries_x[1] >> 8;

    ptr[6] = frame->display_primaries_x[2] & 0xff;
    ptr[7] = frame->display_primaries_x[2] >> 8;

    ptr[9] = frame->display_primaries_y[0] & 0xff;
    ptr[10] = frame->display_primaries_y[0] >> 8;

    ptr[11] = frame->display_primaries_y[1] & 0xff;
    ptr[12] = frame->display_primaries_y[1] >> 8;

    ptr[13] = frame->display_primaries_y[2] & 0xff;
    ptr[14] = frame->display_primaries_y[2] >> 8;

    ptr[15] = frame->white_point_x & 0xff;
    ptr[16] = frame->white_point_x >> 8;

    ptr[17] = frame->white_point_y & 0xff;
    ptr[18] = frame->white_point_y >> 8;

    ptr[19] = frame->max_mastering_display_luminance & 0xff;
    ptr[20] = frame->max_mastering_display_luminance >> 8;

    ptr[21] = frame->min_mastering_display_luminance & 0xff;
    ptr[22] = frame->min_mastering_display_luminance >> 8;

    ptr[23] = frame->max_cll & 0xff;
    ptr[24] = frame->max_cll >> 8;

    ptr[25] = frame->max_fall & 0xff;
    ptr[26] = frame->max_fall >> 8;

    hdmi_infoframe_set_checksum(buffer, length);

    return length;
}

/*
 * hdmi_vendor_any_infoframe_pack() - write a vendor infoframe to binary buffer
 */
static ssize_t
hdmi_vendor_any_infoframe_pack(union hdmi_vendor_any_infoframe *frame,
                   void *buffer, size_t size)
{
    /* we only know about HDMI vendor infoframes */
    if (frame->any.oui != HDMI_IEEE_OUI)
        return -EINVAL;

    return hdmi_vendor_infoframe_pack(&frame->hdmi, buffer, size);
}

/**
 * hdmi_infoframe_pack() - write a HDMI infoframe to binary buffer
 * @frame: HDMI infoframe
 * @buffer: destination buffer
 * @size: size of buffer
 *
 * Packs the information contained in the @frame structure into a binary
 * representation that can be written into the corresponding controller
 * registers. Also computes the checksum as required by section 5.3.5 of
 * the HDMI 1.4 specification.
 *
 * Returns the number of bytes packed into the binary buffer or a negative
 * error code on failure.
 */
ssize_t
hdmi_infoframe_pack(union hdmi_infoframe *frame, void *buffer, size_t size)
{
    ssize_t length;

    switch (frame->any.type) {
    case HDMI_INFOFRAME_TYPE_AVI:
        length = hdmi_avi_infoframe_pack(&frame->avi, buffer, size);
        break;
    case HDMI_INFOFRAME_TYPE_DRM:
        length = hdmi_drm_infoframe_pack(&frame->drm, buffer, size);
        break;
    case HDMI_INFOFRAME_TYPE_SPD:
        length = hdmi_spd_infoframe_pack(&frame->spd, buffer, size);
        break;
    case HDMI_INFOFRAME_TYPE_AUDIO:
        length = hdmi_audio_infoframe_pack(&frame->audio, buffer, size);
        break;
    case HDMI_INFOFRAME_TYPE_VENDOR:
        length = hdmi_vendor_any_infoframe_pack(&frame->vendor,
                            buffer, size);
        break;
    default:
        printf("Bad infoframe type %d\n", frame->any.type);
        length = -EINVAL;
    }

    return length;
}

/**
 * hdmi_avi_infoframe_unpack() - unpack binary buffer to a HDMI AVI infoframe
 * @buffer: source buffer
 * @frame: HDMI AVI infoframe
 *
 * Unpacks the information contained in binary @buffer into a structured
 * @frame of the HDMI Auxiliary Video (AVI) information frame.
 * Also verifies the checksum as required by section 5.3.5 of the HDMI 1.4
 * specification.
 *
 * Returns 0 on success or a negative error code on failure.
 */
static int hdmi_avi_infoframe_unpack(struct hdmi_avi_infoframe *frame,
                     void *buffer)
{
    u8 *ptr = buffer;
    int ret;

    if (ptr[0] != HDMI_INFOFRAME_TYPE_AVI ||
        ptr[1] != 2 ||
        ptr[2] != HDMI_AVI_INFOFRAME_SIZE)
        return -EINVAL;

    if (hdmi_infoframe_checksum(buffer, HDMI_INFOFRAME_SIZE(AVI)) != 0)
        return -EINVAL;

    ret = hdmi_avi_infoframe_init(frame);
    if (ret)
        return ret;

    ptr += HDMI_INFOFRAME_HEADER_SIZE;

    frame->colorspace = (ptr[0] >> 5) & 0x3;
    if (ptr[0] & 0x10)
        frame->active_aspect = ptr[1] & 0xf;
    if (ptr[0] & 0x8) {
        frame->top_bar = (ptr[5] << 8) + ptr[6];
        frame->bottom_bar = (ptr[7] << 8) + ptr[8];
    }
    if (ptr[0] & 0x4) {
        frame->left_bar = (ptr[9] << 8) + ptr[10];
        frame->right_bar = (ptr[11] << 8) + ptr[12];
    }
    frame->scan_mode = ptr[0] & 0x3;

    frame->colorimetry = (ptr[1] >> 6) & 0x3;
    frame->picture_aspect = (ptr[1] >> 4) & 0x3;
    frame->active_aspect = ptr[1] & 0xf;

    frame->itc = ptr[2] & 0x80 ? true : false;
    frame->extended_colorimetry = (ptr[2] >> 4) & 0x7;
    frame->quantization_range = (ptr[2] >> 2) & 0x3;
    frame->nups = ptr[2] & 0x3;

    frame->video_code = ptr[3] & 0x7f;
    frame->ycc_quantization_range = (ptr[4] >> 6) & 0x3;
    frame->content_type = (ptr[4] >> 4) & 0x3;

    frame->pixel_repeat = ptr[4] & 0xf;

    return 0;
}

/**
 * hdmi_spd_infoframe_unpack() - unpack binary buffer to a HDMI SPD infoframe
 * @buffer: source buffer
 * @frame: HDMI SPD infoframe
 *
 * Unpacks the information contained in binary @buffer into a structured
 * @frame of the HDMI Source Product Description (SPD) information frame.
 * Also verifies the checksum as required by section 5.3.5 of the HDMI 1.4
 * specification.
 *
 * Returns 0 on success or a negative error code on failure.
 */
static int hdmi_spd_infoframe_unpack(struct hdmi_spd_infoframe *frame,
                     void *buffer)
{
    char *ptr = buffer;
    int ret;

    if (ptr[0] != HDMI_INFOFRAME_TYPE_SPD ||
        ptr[1] != 1 ||
        ptr[2] != HDMI_SPD_INFOFRAME_SIZE) {
        return -EINVAL;
    }

    if (hdmi_infoframe_checksum(buffer, HDMI_INFOFRAME_SIZE(SPD)) != 0)
        return -EINVAL;

    ptr += HDMI_INFOFRAME_HEADER_SIZE;

    ret = hdmi_spd_infoframe_init(frame, ptr, ptr + 8);
    if (ret)
        return ret;

    frame->sdi = ptr[24];

    return 0;
}

/**
 * hdmi_audio_infoframe_unpack() - unpack binary buffer to a HDMI AUDIO infoframe
 * @buffer: source buffer
 * @frame: HDMI Audio infoframe
 *
 * Unpacks the information contained in binary @buffer into a structured
 * @frame of the HDMI Audio information frame.
 * Also verifies the checksum as required by section 5.3.5 of the HDMI 1.4
 * specification.
 *
 * Returns 0 on success or a negative error code on failure.
 */
static int hdmi_audio_infoframe_unpack(struct hdmi_audio_infoframe *frame,
                       void *buffer)
{
    u8 *ptr = buffer;
    int ret;

    if (ptr[0] != HDMI_INFOFRAME_TYPE_AUDIO ||
        ptr[1] != 1 ||
        ptr[2] != HDMI_AUDIO_INFOFRAME_SIZE) {
        return -EINVAL;
    }

    if (hdmi_infoframe_checksum(buffer, HDMI_INFOFRAME_SIZE(AUDIO)) != 0)
        return -EINVAL;

    ret = hdmi_audio_infoframe_init(frame);
    if (ret)
        return ret;

    ptr += HDMI_INFOFRAME_HEADER_SIZE;

    frame->channels = ptr[0] & 0x7;
    frame->coding_type = (ptr[0] >> 4) & 0xf;
    frame->sample_size = ptr[1] & 0x3;
    frame->sample_frequency = (ptr[1] >> 2) & 0x7;
    frame->coding_type_ext = ptr[2] & 0x1f;
    frame->channel_allocation = ptr[3];
    frame->level_shift_value = (ptr[4] >> 3) & 0xf;
    frame->downmix_inhibit = ptr[4] & 0x80 ? true : false;

    return 0;
}

/**
 * hdmi_vendor_infoframe_unpack() - unpack binary buffer to a HDMI vendor infoframe
 * @buffer: source buffer
 * @frame: HDMI Vendor infoframe
 *
 * Unpacks the information contained in binary @buffer into a structured
 * @frame of the HDMI Vendor information frame.
 * Also verifies the checksum as required by section 5.3.5 of the HDMI 1.4
 * specification.
 *
 * Returns 0 on success or a negative error code on failure.
 */
static int
hdmi_vendor_any_infoframe_unpack(union hdmi_vendor_any_infoframe *frame,
                 void *buffer)
{
    u8 *ptr = buffer;
    size_t length;
    int ret;
    u8 hdmi_video_format;
    struct hdmi_vendor_infoframe *hvf = &frame->hdmi;

    if (ptr[0] != HDMI_INFOFRAME_TYPE_VENDOR ||
        ptr[1] != 1 ||
        (ptr[2] != 4 && ptr[2] != 5 && ptr[2] != 6))
        return -EINVAL;

    length = ptr[2];

    if (hdmi_infoframe_checksum(buffer,
                    HDMI_INFOFRAME_HEADER_SIZE + length) != 0)
        return -EINVAL;

    ptr += HDMI_INFOFRAME_HEADER_SIZE;

    /* HDMI OUI */
    if (ptr[0] != 0x03 ||
        ptr[1] != 0x0c ||
        ptr[2] != 0x00)
        return -EINVAL;

    hdmi_video_format = ptr[3] >> 5;

    if (hdmi_video_format > 0x2)
        return -EINVAL;

    ret = hdmi_vendor_infoframe_init(hvf);
    if (ret)
        return ret;

    hvf->length = length;

    if (hdmi_video_format == 0x2) {
        if (length != 5 && length != 6)
            return -EINVAL;
        hvf->s3d_struct = ptr[4] >> 4;
        if (hvf->s3d_struct >= HDMI_3D_STRUCTURE_SIDE_BY_SIDE_HALF) {
            if (length != 6)
                return -EINVAL;
            hvf->s3d_ext_data = ptr[5] >> 4;
        }
    } else if (hdmi_video_format == 0x1) {
        if (length != 5)
            return -EINVAL;
        hvf->vic = ptr[4];
    } else {
        if (length != 4)
            return -EINVAL;
    }

    return 0;
}

/**
 * hdmi_infoframe_unpack() - unpack binary buffer to a HDMI infoframe
 * @buffer: source buffer
 * @frame: HDMI infoframe
 *
 * Unpacks the information contained in binary buffer @buffer into a structured
 * @frame of a HDMI infoframe.
 * Also verifies the checksum as required by section 5.3.5 of the HDMI 1.4
 * specification.
 *
 * Returns 0 on success or a negative error code on failure.
 */
int hdmi_infoframe_unpack(union hdmi_infoframe *frame, void *buffer)
{
    int ret;
    u8 *ptr = buffer;

    switch (ptr[0]) {
    case HDMI_INFOFRAME_TYPE_AVI:
        ret = hdmi_avi_infoframe_unpack(&frame->avi, buffer);
        break;
    case HDMI_INFOFRAME_TYPE_SPD:
        ret = hdmi_spd_infoframe_unpack(&frame->spd, buffer);
        break;
    case HDMI_INFOFRAME_TYPE_AUDIO:
        ret = hdmi_audio_infoframe_unpack(&frame->audio, buffer);
        break;
    case HDMI_INFOFRAME_TYPE_VENDOR:
        ret = hdmi_vendor_any_infoframe_unpack(&frame->vendor, buffer);
        break;
    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

bool drm_mode_equal(const struct base_drm_display_mode *mode1,
            const struct drm_display_mode *mode2)
{
    unsigned int flags_mask =
        DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_PHSYNC |
        DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_PVSYNC |
        DRM_MODE_FLAG_NVSYNC;

    if (mode1->clock == mode2->clock &&
        mode1->hdisplay == mode2->hdisplay &&
        mode1->hsync_start == mode2->hsync_start &&
        mode1->hsync_end == mode2->hsync_end &&
        mode1->htotal == mode2->htotal &&
        mode1->vdisplay == mode2->vdisplay &&
        mode1->vsync_start == mode2->vsync_start &&
        mode1->vsync_end == mode2->vsync_end &&
        mode1->vtotal == mode2->vtotal &&
        (mode1->flags & flags_mask) == (mode2->flags & flags_mask)) {
        return true;
    }

    return false;
}

/**
 * drm_mode_sort - sort mode list
 * @edid_data: modes structures to sort
 *
 * Sort @edid_data by favorability, moving good modes to the head of the list.
 */
void drm_mode_sort(struct hdmi_edid_data *edid_data)
{
    struct drm_display_mode *a, *b;
    struct drm_display_mode c;
    int diff, i, j;

    for (i = 0; i < (edid_data->modes - 1); i++) {
        a = &edid_data->mode_buf[i];
        for (j = i + 1; j < edid_data->modes; j++) {
            b = &edid_data->mode_buf[j];
            diff = ((b->type & DRM_MODE_TYPE_PREFERRED) != 0) -
                ((a->type & DRM_MODE_TYPE_PREFERRED) != 0);
            if (diff) {
                if (diff > 0) {
                    c = *a;
                    *a = *b;
                    *b = c;
                }
                continue;
            }

            diff = b->hdisplay * b->vdisplay
                - a->hdisplay * a->vdisplay;
            if (diff) {
                if (diff > 0) {
                    c = *a;
                    *a = *b;
                    *b = c;
                }
                continue;
            }

            diff = b->vrefresh - a->vrefresh;
            if (diff) {
                if (diff > 0) {
                    c = *a;
                    *a = *b;
                    *b = c;
                }
                continue;
            }

            diff = b->clock - a->clock;
            if (diff > 0) {
                c = *a;
                *a = *b;
                *b = c;
            }
        }
    }
    edid_data->preferred_mode = &edid_data->mode_buf[0];
}

/**
 * drm_mode_prune_invalid - remove invalid modes from mode list
 * @edid_data: structure store mode list
 * Returns:
 * Number of valid modes.
 */
int drm_mode_prune_invalid(struct hdmi_edid_data *edid_data)
{
    int i, j;
    int num = edid_data->modes;
    int len = sizeof(struct drm_display_mode);
    struct drm_display_mode *mode_buf = edid_data->mode_buf;

    for (i = 0; i < num; i++) {
        if (mode_buf[i].invalid) {
            /* If mode is invalid, delete it. */
            for (j = i; j < num - 1; j++)
                memcpy(&mode_buf[j], &mode_buf[j + 1], len);

            num--;
            i--;
        }
    }
    /* Clear redundant modes of mode_buf. */
    memset(&mode_buf[num], 0, len * (edid_data->modes - num));

    edid_data->modes = num;
    return num;
}

/**
 * drm_do_probe_ddc_edid() - get EDID information via I2C
 * @adap: ddc adapter
 * @buf: EDID data buffer to be filled
 * @block: 128 byte EDID block to start fetching from
 * @len: EDID data buffer length to fetch
 *
 * Try to fetch EDID information by calling I2C driver functions.
 *
 * Return: 0 on success or -1 on failure.
 */
static int
drm_do_probe_ddc_edid(struct ddc_adapter *adap, u8 *buf, unsigned int block,
              size_t len)
{
    unsigned char start = block * HDMI_EDID_BLOCK_SIZE;
    unsigned char segment = block >> 1;
    unsigned char xfers = segment ? 3 : 2;
    int ret, retries = 5;

    do {
        struct i2c_msg msgs[] = {
            {
                .addr   = DDC_SEGMENT_ADDR,
                .flags  = 0,
                .len    = 1,
                .buf    = &segment,
            }, {
                .addr   = DDC_ADDR,
                .flags  = 0,
                .len    = 1,
                .buf    = &start,
            }, {
                .addr   = DDC_ADDR,
                .flags  = I2C_M_RD,
                .len    = len,
                .buf    = buf,
            }
        };

        ret = adap->ddc_xfer(adap, &msgs[3 - xfers], xfers);

    } while (ret != xfers && --retries);

    /* All msg transfer successfully. */
    return ret == xfers ? 0 : -1;
}

int drm_do_get_edid(struct ddc_adapter *adap, u8 *edid)
{
    int i, j, block_num, block = 0;
    bool edid_corrupt;
#ifdef DEBUG
    u8 *buff;
#endif

    /* base block fetch */
    for (i = 0; i < 4; i++) {
        if (drm_do_probe_ddc_edid(adap, edid, 0, HDMI_EDID_BLOCK_SIZE))
            goto err;
        if (drm_edid_block_valid(edid, 0, true,
                     &edid_corrupt))
            break;
        if (i == 0 && drm_edid_is_zero(edid, HDMI_EDID_BLOCK_SIZE)) {
            printf("edid base block is 0, get edid failed\n");
            goto err;
        }
    }

    if (i == 4)
        goto err;

    block++;
    /* get the number of extensions */
    block_num = edid[0x7e];

    for (j = 1; j <= block_num; j++) {
        for (i = 0; i < 4; i++) {
            if (drm_do_probe_ddc_edid(adap, &edid[0x80 * j], j,
                          HDMI_EDID_BLOCK_SIZE))
                goto err;
            if (drm_edid_block_valid(&edid[0x80 * j], j,
                         true, NULL))
                break;
        }

        if (i == 4)
            goto err;
        block++;
    }

#ifdef DEBUG
    printf("RAW EDID:\n");
    for (i = 0; i < block_num + 1; i++) {
        buff = &edid[0x80 * i];
        for (j = 0; j < HDMI_EDID_BLOCK_SIZE; j++) {
            if (j % 16 == 0)
                printf("\n");
            printf("0x%02x, ", buff[j]);
        }
        printf("\n");
    }
#endif

    return 0;

err:
    printf("can't get edid block:%d\n", block);
    /* clear all read edid block, include invalid block */
    memset(edid, 0, HDMI_EDID_BLOCK_SIZE * (block + 1));
    return -EFAULT;
}

static ssize_t hdmi_ddc_read(struct ddc_adapter *adap, u16 addr, u8 offset,
                 void *buffer, size_t size)
{
    struct i2c_msg msgs[2] = {
        {
            .addr = addr,
            .flags = 0,
            .len = 1,
            .buf = &offset,
        }, {
            .addr = addr,
            .flags = I2C_M_RD,
            .len = size,
            .buf = buffer,
        }
    };

    return adap->ddc_xfer(adap, msgs, ARRAY_SIZE(msgs));
}

static ssize_t hdmi_ddc_write(struct ddc_adapter *adap, u16 addr, u8 offset,
                  const void *buffer, size_t size)
{
    struct i2c_msg msg = {
        .addr = addr,
        .flags = 0,
        .len = 1 + size,
        .buf = NULL,
    };
    void *data;
    int err;

    data = malloc(1 + size);
    if (!data)
        return -ENOMEM;

    msg.buf = data;

    memcpy(data, &offset, sizeof(offset));
    memcpy(data + 1, buffer, size);

    err = adap->ddc_xfer(adap, &msg, 1);

    free(data);

    return err;
}

/**
 * drm_scdc_readb - read a single byte from SCDC
 * @adap: ddc adapter
 * @offset: offset of register to read
 * @value: return location for the register value
 *
 * Reads a single byte from SCDC. This is a convenience wrapper around the
 * drm_scdc_read() function.
 *
 * Returns:
 * 0 on success or a negative error code on failure.
 */
u8 drm_scdc_readb(struct ddc_adapter *adap, u8 offset,
          u8 *value)
{
    return hdmi_ddc_read(adap, SCDC_I2C_SLAVE_ADDRESS, offset, value,
                 sizeof(*value));
}

/**
 * drm_scdc_writeb - write a single byte to SCDC
 * @adap: ddc adapter
 * @offset: offset of register to read
 * @value: return location for the register value
 *
 * Writes a single byte to SCDC. This is a convenience wrapper around the
 * drm_scdc_write() function.
 *
 * Returns:
 * 0 on success or a negative error code on failure.
 */
u8 drm_scdc_writeb(struct ddc_adapter *adap, u8 offset,
           u8 value)
{
    return hdmi_ddc_write(adap, SCDC_I2C_SLAVE_ADDRESS, offset, &value,
                  sizeof(value));
}
