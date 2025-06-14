// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2015 Google, Inc
 */

#define LOG_CATEGORY UCLASS_VIDEO

#include <bloblist.h>
#include <console.h>
#include <cpu_func.h>
#include <cyclic.h>
#include <dm.h>
#include <log.h>
#include <malloc.h>
#include <mapmem.h>
#include <spl.h>
#include <stdio_dev.h>
#include <video.h>
#include <video_console.h>
#include <asm/cache.h>
#include <asm/global_data.h>
#include <dm/lists.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <dm/uclass-internal.h>
#ifdef CONFIG_SANDBOX
#include <asm/sdl.h>
#endif
#include "vidconsole_internal.h"
#include <video_eswin.h>
#include <splash.h>
#include <env.h>

/*
 * Theory of operation:
 *
 * Before relocation each device is bound. The driver for each device must
 * set the @align and @size values in struct video_uc_plat. This
 * information represents the requires size and alignment of the frame buffer
 * for the device. The values can be an over-estimate but cannot be too
 * small. The actual values will be suppled (in the same manner) by the bind()
 * method after relocation. Additionally driver can allocate frame buffer
 * itself by setting plat->base.
 *
 * This information is then picked up by video_reserve() which works out how
 * much memory is needed for all devices. This is allocated between
 * gd->video_bottom and gd->video_top.
 *
 * After relocation the same process occurs. The driver supplies the same
 * @size and @align information and this time video_post_bind() checks that
 * the drivers does not overflow the allocated memory.
 *
 * The frame buffer address is actually set (to plat->base) in
 * video_post_probe(). This function also clears the frame buffer and
 * allocates a suitable text console device. This can then be used to write
 * text to the video device.
 */
DECLARE_GLOBAL_DATA_PTR;

struct cyclic_info;

/**
 * struct video_uc_priv - Information for the video uclass
 *
 * @video_ptr: Current allocation position of the video framebuffer pointer.
 *	While binding devices after relocation, this points to the next
 *	available address to use for a device's framebuffer. It starts at
 *	gd->video_top and works downwards, running out of space when it hits
 *	gd->video_bottom.
 * @cyc: handle for cyclic-execution function, or NULL if none
 */
struct video_uc_priv {
	ulong video_ptr;
	bool cyc_active;
	struct cyclic_info cyc;
};

/** struct vid_rgb - Describes a video colour */
struct vid_rgb {
	u32 r;
	u32 g;
	u32 b;
};

void video_set_flush_dcache(struct udevice *dev, bool flush)
{
	struct video_priv *priv = dev_get_uclass_priv(dev);

	priv->flush_dcache = flush;
}

static ulong alloc_fb_(ulong align, ulong size, ulong *addrp)
{
	ulong base;

	align = align ? align : 1 << 20;
	base = *addrp - size;
	base &= ~(align - 1);
	size = *addrp - base;
	*addrp = base;

	return size;
}

static ulong alloc_fb(struct udevice *dev, ulong *addrp)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	ulong size;

	if (!plat->size) {
		if (IS_ENABLED(CONFIG_VIDEO_COPY) && plat->copy_size) {
			size = alloc_fb_(plat->align, plat->copy_size, addrp);
			plat->copy_base = *addrp;
			return size;
		}

		return 0;
	}

	/* Allow drivers to allocate the frame buffer themselves */
	if (plat->base)
		return 0;

	size = alloc_fb_(plat->align, plat->size, addrp);
	plat->base = *addrp;

	return size;
}

int video_reserve(ulong *addrp)
{
	struct udevice *dev;
	ulong size;

	if (IS_ENABLED(CONFIG_SPL_VIDEO_HANDOFF) && xpl_phase() == PHASE_BOARD_F)
		return 0;

	gd->video_top = *addrp;
	for (uclass_find_first_device(UCLASS_VIDEO, &dev);
	     dev;
	     uclass_find_next_device(&dev)) {
		size = alloc_fb(dev, addrp);
		debug("%s: Reserving %lx bytes at %lx for video device '%s'\n",
		      __func__, size, *addrp, dev->name);
	}

	/* Allocate space for PCI video devices in case there were not bound */
	if (*addrp == gd->video_top)
		*addrp -= CONFIG_VAL(VIDEO_PCI_DEFAULT_FB_SIZE);

	gd->video_bottom = *addrp;
	debug("Video frame buffers from %lx to %lx\n", gd->video_bottom,
	      gd->video_top);

	return 0;
}

ulong video_get_fb(void)
{
	struct udevice *dev;

	uclass_find_first_device(UCLASS_VIDEO, &dev);
	if (dev) {
		const struct video_uc_plat *uc_plat = dev_get_uclass_plat(dev);

		return uc_plat->base;
	}

	return 0;
}

int video_fill_part(struct udevice *dev, int xstart, int ystart, int xend,
		    int yend, u32 colour)
{
	struct video_priv *priv = dev_get_uclass_priv(dev);
	void *start, *line;
	int pixels = xend - xstart;
	int row, i;

	start = priv->fb + ystart * priv->line_length;
	start += xstart * VNBYTES(priv->bpix);
	line = start;
	for (row = ystart; row < yend; row++) {
		switch (priv->bpix) {
		case VIDEO_BPP8: {
			u8 *dst = line;

			if (IS_ENABLED(CONFIG_VIDEO_BPP8)) {
				for (i = 0; i < pixels; i++)
					*dst++ = colour;
			}
			break;
		}
		case VIDEO_BPP16: {
			u16 *dst = line;

			if (IS_ENABLED(CONFIG_VIDEO_BPP16)) {
				for (i = 0; i < pixels; i++)
					*dst++ = colour;
			}
			break;
		}
		case VIDEO_BPP32: {
			u32 *dst = line;

			if (IS_ENABLED(CONFIG_VIDEO_BPP32)) {
				for (i = 0; i < pixels; i++)
					*dst++ = colour;
			}
			break;
		}
		default:
			return -ENOSYS;
		}
		line += priv->line_length;
	}

	video_damage(dev, xstart, ystart, xend - xstart, yend - ystart);

	return 0;
}

int video_draw_box(struct udevice *dev, int x0, int y0, int x1, int y1,
		   int width, u32 colour)
{
	struct video_priv *priv = dev_get_uclass_priv(dev);
	int pbytes = VNBYTES(priv->bpix);
	void *start, *line;
	int pixels = x1 - x0;
	int row;

	start = priv->fb + y0 * priv->line_length;
	start += x0 * pbytes;
	line = start;
	for (row = y0; row < y1; row++) {
		void *ptr = line;
		int i;

		for (i = 0; i < width; i++)
			fill_pixel_and_goto_next(&ptr, colour, pbytes, pbytes);
		if (row < y0 + width || row >= y1 - width) {
			for (i = 0; i < pixels - width * 2; i++)
				fill_pixel_and_goto_next(&ptr, colour, pbytes,
							 pbytes);
		} else {
			ptr += (pixels - width * 2) * pbytes;
		}
		for (i = 0; i < width; i++)
			fill_pixel_and_goto_next(&ptr, colour, pbytes, pbytes);
		line += priv->line_length;
	}
	video_damage(dev, x0, y0, x1 - x0, y1 - y0);

	return 0;
}

int video_reserve_from_bloblist(struct video_handoff *ho)
{
	if (!ho->fb || ho->size == 0)
		return -ENOENT;

	gd->video_bottom = ho->fb;
	gd->video_top = ho->fb + ho->size;
	debug("%s: Reserving %lx bytes at %08x as per bloblist received\n",
	      __func__, (unsigned long)ho->size, (u32)ho->fb);

	return 0;
}

int video_fill(struct udevice *dev, u32 colour)
{
	struct video_priv *priv = dev_get_uclass_priv(dev);

	switch (priv->bpix) {
	case VIDEO_BPP16:
		if (CONFIG_IS_ENABLED(VIDEO_BPP16)) {
			u16 *ppix = priv->fb;
			u16 *end = priv->fb + priv->fb_size;

			while (ppix < end)
				*ppix++ = colour;
			break;
		}
		fallthrough;
	case VIDEO_BPP32:
		if (CONFIG_IS_ENABLED(VIDEO_BPP32)) {
			u32 *ppix = priv->fb;
			u32 *end = priv->fb + priv->fb_size;

			while (ppix < end)
				*ppix++ = colour;
			break;
		}
		fallthrough;
	default:
		memset(priv->fb, colour, priv->fb_size);
		break;
	}

	video_damage(dev, 0, 0, priv->xsize, priv->ysize);

	return video_sync(dev, false);
}

int video_clear(struct udevice *dev)
{
	struct video_priv *priv = dev_get_uclass_priv(dev);
	int ret;

	ret = video_fill(dev, priv->colour_bg);
	if (ret)
		return ret;

	return 0;
}

static const struct vid_rgb colours[VID_COLOUR_COUNT] = {
	{ 0x00, 0x00, 0x00 },  /* black */
	{ 0xc0, 0x00, 0x00 },  /* red */
	{ 0x00, 0xc0, 0x00 },  /* green */
	{ 0xc0, 0x60, 0x00 },  /* brown */
	{ 0x00, 0x00, 0xc0 },  /* blue */
	{ 0xc0, 0x00, 0xc0 },  /* magenta */
	{ 0x00, 0xc0, 0xc0 },  /* cyan */
	{ 0xc0, 0xc0, 0xc0 },  /* light gray */
	{ 0x80, 0x80, 0x80 },  /* gray */
	{ 0xff, 0x00, 0x00 },  /* bright red */
	{ 0x00, 0xff, 0x00 },  /* bright green */
	{ 0xff, 0xff, 0x00 },  /* yellow */
	{ 0x00, 0x00, 0xff },  /* bright blue */
	{ 0xff, 0x00, 0xff },  /* bright magenta */
	{ 0x00, 0xff, 0xff },  /* bright cyan */
	{ 0xff, 0xff, 0xff },  /* white */

	/* an extra one for menus */
	{ 0x40, 0x40, 0x40 },  /* dark gray */
};

u32 video_index_to_colour(struct video_priv *priv, enum colour_idx idx)
{
	switch (priv->bpix) {
	case VIDEO_BPP16:
		if (CONFIG_IS_ENABLED(VIDEO_BPP16)) {
			return ((colours[idx].r >> 3) << 11) |
			       ((colours[idx].g >> 2) <<  5) |
			       ((colours[idx].b >> 3) <<  0);
		}
		break;
	case VIDEO_BPP32:
		if (CONFIG_IS_ENABLED(VIDEO_BPP32)) {
			switch (priv->format) {
			case VIDEO_X2R10G10B10:
				return (colours[idx].r << 22) |
				       (colours[idx].g << 12) |
				       (colours[idx].b <<  2);
			case VIDEO_RGBA8888:
				return (colours[idx].r << 24) |
				       (colours[idx].g << 16) |
				       (colours[idx].b << 8) | 0xff;
			default:
				return (colours[idx].r << 16) |
				       (colours[idx].g <<  8) |
				       (colours[idx].b <<  0);
			}
		}
		break;
	default:
		break;
	}

	/*
	 * For unknown bit arrangements just support
	 * black and white.
	 */
	if (idx)
		return 0xffffff; /* white */

	return 0x000000; /* black */
}

void video_set_default_colors(struct udevice *dev, bool invert)
{
	struct video_priv *priv = dev_get_uclass_priv(dev);
	int fore, back;

	if (priv->white_on_black) {
		/* White is used when switching to bold, use light gray here */
		fore = VID_LIGHT_GRAY;
		back = VID_BLACK;
	} else {
		fore = VID_BLACK;
		back = VID_WHITE;
	}
	if (invert) {
		int temp;

		temp = fore;
		fore = back;
		back = temp;
	}
	priv->fg_col_idx = fore;
	priv->bg_col_idx = back;
	priv->colour_fg = video_index_to_colour(priv, fore);
	priv->colour_bg = video_index_to_colour(priv, back);
}

/* Notify about changes in the frame buffer */
#ifdef CONFIG_VIDEO_DAMAGE
void video_damage(struct udevice *vid, int x, int y, int width, int height)
{
	struct video_priv *priv = dev_get_uclass_priv(vid);
	int xend = x + width;
	int yend = y + height;

	if (x > priv->xsize)
		return;

	if (y > priv->ysize)
		return;

	if (xend > priv->xsize)
		xend = priv->xsize;

	if (yend > priv->ysize)
		yend = priv->ysize;

	/* Span a rectangle across all old and new damage */
	priv->damage.xstart = min(x, priv->damage.xstart);
	priv->damage.ystart = min(y, priv->damage.ystart);
	priv->damage.xend = max(xend, priv->damage.xend);
	priv->damage.yend = max(yend, priv->damage.yend);
}
#endif

static void video_flush_dcache(struct udevice *vid, bool use_copy)
{
	struct video_priv *priv = dev_get_uclass_priv(vid);
	ulong fb = use_copy ? (ulong)priv->copy_fb : (ulong)priv->fb;
	uint cacheline_size = 32;

#ifdef CONFIG_SYS_CACHELINE_SIZE
	cacheline_size = CONFIG_SYS_CACHELINE_SIZE;
#endif

	if (CONFIG_IS_ENABLED(SYS_DCACHE_OFF))
		return;

	if (!priv->flush_dcache)
		return;

	if (!IS_ENABLED(CONFIG_VIDEO_DAMAGE)) {
		flush_dcache_range(fb, ALIGN(fb + priv->fb_size,
					     cacheline_size));

		return;
	}

	if (priv->damage.xend && priv->damage.yend) {
		int lstart = priv->damage.xstart * VNBYTES(priv->bpix);
		int lend = priv->damage.xend * VNBYTES(priv->bpix);
		int y;

		for (y = priv->damage.ystart; y < priv->damage.yend; y++) {
			ulong start = fb + (y * priv->line_length) + lstart;
			ulong end = start + lend - lstart;

			start = ALIGN_DOWN(start, cacheline_size);
			end = ALIGN(end, cacheline_size);

			flush_dcache_range(start, end);
		}
	}
}

static void video_flush_copy(struct udevice *vid)
{
	struct video_priv *priv = dev_get_uclass_priv(vid);

	if (!priv->copy_fb)
		return;

	if (priv->damage.xend && priv->damage.yend) {
		int lstart = priv->damage.xstart * VNBYTES(priv->bpix);
		int lend = priv->damage.xend * VNBYTES(priv->bpix);
		int y;

		for (y = priv->damage.ystart; y < priv->damage.yend; y++) {
			ulong offset = (y * priv->line_length) + lstart;
			ulong len = lend - lstart;

			memcpy(priv->copy_fb + offset, priv->fb + offset, len);
		}
	}
}

/* Flush video activity to the caches */
int video_sync(struct udevice *vid, bool force)
{
	struct video_priv *priv = dev_get_uclass_priv(vid);
	struct video_ops *ops = video_get_ops(vid);
	int ret;

	if (IS_ENABLED(CONFIG_VIDEO_COPY))
		video_flush_copy(vid);

	if (ops && ops->video_sync) {
		ret = ops->video_sync(vid);
		if (ret)
			return ret;
	}

	if (CONFIG_IS_ENABLED(CYCLIC) && !force &&
	    get_timer(priv->last_sync) < CONFIG_VIDEO_SYNC_MS)
		return 0;

	video_flush_dcache(vid, false);

	if (IS_ENABLED(CONFIG_VIDEO_COPY))
		video_flush_dcache(vid, true);

#if defined(CONFIG_VIDEO_SANDBOX_SDL)
	/* to see the copy framebuffer, use priv->copy_fb */
	sandbox_sdl_sync(priv->fb);
#endif
	priv->last_sync = get_timer(0);

	if (IS_ENABLED(CONFIG_VIDEO_DAMAGE)) {
		priv->damage.xstart = priv->xsize;
		priv->damage.ystart = priv->ysize;
		priv->damage.xend = 0;
		priv->damage.yend = 0;
	}

	return 0;
}

void video_sync_all(void)
{
	struct udevice *dev;
	int ret;

	for (uclass_find_first_device(UCLASS_VIDEO, &dev);
	     dev;
	     uclass_find_next_device(&dev)) {
		if (device_active(dev)) {
			ret = video_sync(dev, true);
			if (ret)
				dev_dbg(dev, "Video sync failed\n");
		}
	}
}

bool video_is_active(void)
{
	struct udevice *dev;

	/* Assume video to be active if SPL passed video hand-off to U-boot */
	if (IS_ENABLED(CONFIG_SPL_VIDEO_HANDOFF) && xpl_phase() > PHASE_SPL)
		return true;

	for (uclass_find_first_device(UCLASS_VIDEO, &dev);
	     dev;
	     uclass_find_next_device(&dev)) {
		if (device_active(dev))
			return true;
	}

	return false;
}

int video_get_xsize(struct udevice *dev)
{
	struct video_priv *priv = dev_get_uclass_priv(dev);

	return priv->xsize;
}

int video_get_ysize(struct udevice *dev)
{
	struct video_priv *priv = dev_get_uclass_priv(dev);

	return priv->ysize;
}

#define SPLASH_DECL(_name) \
	extern u8 __splash_ ## _name ## _begin[]; \
	extern u8 __splash_ ## _name ## _end[]

#define SPLASH_START(_name)	__splash_ ## _name ## _begin

SPLASH_DECL(u_boot_logo);

void *video_get_u_boot_logo(void)
{
	return SPLASH_START(u_boot_logo);
}

static int show_splash(struct udevice *dev)
{
	u8 *data = SPLASH_START(u_boot_logo);
	int ret;

	ret = video_bmp_display(dev, map_to_sysmem(data), -4, 4, true);

	return 0;
}

int video_default_font_height(struct udevice *dev)
{
	struct vidconsole_priv *vc_priv = dev_get_uclass_priv(dev);

	if (IS_ENABLED(CONFIG_CONSOLE_TRUETYPE))
		return IF_ENABLED_INT(CONFIG_CONSOLE_TRUETYPE,
				      CONFIG_CONSOLE_TRUETYPE_SIZE);

	return vc_priv->y_charsize;
}

static void video_idle(struct cyclic_info *cyc)
{
	video_sync_all();
}

void video_set_white_on_black(struct udevice *dev, bool white_on_black)
{
	struct video_priv *priv = dev_get_uclass_priv(dev);

	if (priv->white_on_black != white_on_black) {
		priv->white_on_black = white_on_black;
		video_set_default_colors(dev, false);

		video_clear(dev);
	}
}

int die1_bmp_display(struct udevice *dev)
{
	int x = 0, y = 0, ret;
	char *s;
	ulong addr;

	s = env_get("splashimage");
	if (!s)
		return -EINVAL;
	addr = hextoul(s, NULL);

	splash_get_pos(&x, &y);

	if (CONFIG_IS_ENABLED(BMP))
		ret = eswin_bmp_display(dev, addr, x, y);
	else
		return -ENOSYS;

	return ret;
}

/* Set up the display ready for use */
static int video_post_probe(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct video_uc_priv *uc_priv = uclass_get_priv(dev->uclass);
	struct video_priv *priv = dev_get_uclass_priv(dev);
	char name[30], drv[15], *str;
	const char *drv_name = drv;
	struct udevice *cons;
	int ret;
	u32 double_screen = 0;

	/* Set up the line and display size */
	priv->fb = map_sysmem(plat->base, plat->size);
	if (!priv->line_length)
		priv->line_length = priv->xsize * VNBYTES(priv->bpix);

	priv->fb_size = priv->line_length * priv->ysize;

	/*
	 * Set up video handoff fields for passing video blob to next stage
	 * NOTE:
	 * This assumes that reserved video memory only uses a single framebuffer
	 */
	if (xpl_phase() == PHASE_SPL && CONFIG_IS_ENABLED(BLOBLIST)) {
		struct video_handoff *ho;

		ho = bloblist_add(BLOBLISTT_U_BOOT_VIDEO, sizeof(*ho), 0);
		if (!ho)
			return log_msg_ret("blf", -ENOENT);
		ho->fb = gd->video_bottom;
		/* Fill aligned size here as calculated in video_reserve() */
		ho->size = gd->video_top - gd->video_bottom;
		ho->xsize = priv->xsize;
		ho->ysize = priv->ysize;
		ho->line_length = priv->line_length;
		ho->bpix = priv->bpix;
		ho->format = priv->format;
	}

	if (IS_ENABLED(CONFIG_VIDEO_COPY) && plat->copy_base)
		priv->copy_fb = map_sysmem(plat->copy_base, plat->size);

	priv->white_on_black = CONFIG_IS_ENABLED(SYS_WHITE_ON_BLACK);

	/* Set up colors  */
	video_set_default_colors(dev, false);

	if (!CONFIG_IS_ENABLED(NO_FB_CLEAR))
		video_clear(dev);

	/*
	 * Create a text console device. For now we always do this, although
	 * it might be useful to support only bitmap drawing on the device
	 * for boards that don't need to display text. We create a TrueType
	 * console if enabled, a rotated console if the video driver requests
	 * it, otherwise a normal console.
	 *
	 * The console can be override by setting vidconsole_drv_name before
	 * probing this video driver, or in the probe() method.
	 *
	 * TrueType does not support rotation at present so fall back to the
	 * rotated console in that case.
	 */
	if (!priv->rot && IS_ENABLED(CONFIG_CONSOLE_TRUETYPE)) {
		snprintf(name, sizeof(name), "%s.vidconsole_tt", dev->name);
		strcpy(drv, "vidconsole_tt");
	} else {
		snprintf(name, sizeof(name), "%s.vidconsole%d", dev->name,
			 priv->rot);
		snprintf(drv, sizeof(drv), "vidconsole%d", priv->rot);
	}

	str = strdup(name);
	if (!str)
		return -ENOMEM;
	if (priv->vidconsole_drv_name)
		drv_name = priv->vidconsole_drv_name;
	ret = device_bind_driver(dev, drv_name, str, &cons);
	if (ret) {
		debug("%s: Cannot bind console driver\n", __func__);
		return ret;
	}

	ret = device_probe(cons);
	if (ret) {
		debug("%s: Cannot probe console driver\n", __func__);
		return ret;
	}

	if (CONFIG_IS_ENABLED(VIDEO_LOGO) &&
	    !CONFIG_IS_ENABLED(SPLASH_SCREEN) && !plat->hide_logo) {
		ret = show_splash(dev);
		if (ret) {
			log_debug("Cannot show splash screen\n");
			return ret;
		}
	}

	ret = dev_read_u32(dev, "double-screen", &double_screen);
	if (ret) {
		printf("failed to get double-screen, ret %d\n", ret);
		return 0;
	}

	if (double_screen == 1) {
		ret = die1_bmp_display(dev);
	}

	/* register cyclic as soon as the first video device is probed */
	if (CONFIG_IS_ENABLED(CYCLIC) && (gd->flags && GD_FLG_RELOC) &&
	    !uc_priv->cyc_active) {
		uint ms = CONFIG_IF_ENABLED_INT(CYCLIC, VIDEO_SYNC_CYCLIC_MS);

		cyclic_register(&uc_priv->cyc, video_idle, ms * 1000,
				"video_init");
		uc_priv->cyc_active = true;
	}

	return 0;
};

/* Post-relocation, allocate memory for the frame buffer */
static int video_post_bind(struct udevice *dev)
{
	struct video_uc_priv *uc_priv;
	ulong addr;
	ulong size;

	/* Before relocation there is nothing to do here */
	if (!(gd->flags & GD_FLG_RELOC))
		return 0;

	/* Set up the video pointer, if this is the first device */
	uc_priv = uclass_get_priv(dev->uclass);
	if (!uc_priv->video_ptr)
		uc_priv->video_ptr = gd->video_top;

	/* Allocate framebuffer space for this device */
	addr = uc_priv->video_ptr;
	size = alloc_fb(dev, &addr);
	if (addr < gd->video_bottom) {
		/*
		 * Device tree node may need the 'bootph-all' or
		 * 'bootph-some-ram' tag
		 */
		printf("Video device '%s' cannot allocate frame buffer memory "
		       "- ensure the device is set up before relocation\n",
		       dev->name);
		return -ENOSPC;
	}
	debug("%s: Claiming %lx bytes at %lx for video device '%s'\n",
	      __func__, size, addr, dev->name);
	uc_priv->video_ptr = addr;

	return 0;
}

__maybe_unused static int video_destroy(struct uclass *uc)
{
	struct video_uc_priv *uc_priv = uclass_get_priv(uc);

	if (uc_priv->cyc_active) {
		cyclic_unregister(&uc_priv->cyc);
		uc_priv->cyc_active = false;
	}

	return 0;
}

UCLASS_DRIVER(video) = {
	.id		= UCLASS_VIDEO,
	.name		= "video",
	.flags		= DM_UC_FLAG_SEQ_ALIAS,
	.post_bind	= video_post_bind,
	.post_probe	= video_post_probe,
	.priv_auto	= sizeof(struct video_uc_priv),
	.per_device_auto	= sizeof(struct video_priv),
	.per_device_plat_auto	= sizeof(struct video_uc_plat),
	CONFIG_IS_ENABLED(CYCLIC, (.destroy = video_destroy, ))
};
