/* linux/drivers/video/s3c-fb.c
 *
 * Copyright 2008 Openmoko Inc.
 * Copyright 2008-2010 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * Samsung SoC Framebuffer driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#include <media/v4l2-device.h>
#include <media/exynos_mc.h>

#include <mach/exynos5_bus.h>
#include <mach/map.h>
#include <plat/regs-fb-v4.h>
#include <plat/fb.h>

#include <linux/dma-buf.h>
#include <linux/exynos_ion.h>
#include <linux/ion.h>
#include <linux/highmem.h>
#include <linux/memblock.h>
#include <linux/sw_sync.h>
#include <plat/devs.h>
#include <plat/iovmm.h>
#include <plat/sysmmu.h>
#include <mach/sysmmu.h>

#include <video/adf.h>
#include <video/adf_client.h>
#include <video/adf_fbdev.h>
#include <video/adf_format.h>
#include <video/adf_memblock.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

/* note, the previous use of <mach/regs-fb.h> to get platform specific data
 * has been replaced by using the platform device name to pick the correct
 * configuration data for the system.
*/

#ifdef CONFIG_FB_S3C_DEBUG_REGWRITE
#undef writel
#define writel(v, r) do { \
	printk(KERN_DEBUG "%s: %08x => %p\n", __func__, (unsigned int)v, r); \
	__raw_writel(v, r); \
} while (0)
#endif /* FB_S3C_DEBUG_REGWRITE */

#define VSYNC_TIMEOUT_MSEC 50

#define MAX_BW_PER_WINDOW	(2560 * 1600 * 4 * 60)

struct s3c_fb;

extern struct ion_device *ion_exynos;

#define VALID_BPP(x) (1 << ((x) - 1))

#define OSD_BASE(win, variant) ((variant).osd + ((win) * (variant).osd_stride))
#define VIDOSD_A(win, variant) (OSD_BASE(win, variant) + 0x00)
#define VIDOSD_B(win, variant) (OSD_BASE(win, variant) + 0x04)
#define VIDOSD_C(win, variant) (OSD_BASE(win, variant) + 0x08)
#define VIDOSD_D(win, variant) (OSD_BASE(win, variant) + 0x0C)

/**
 * struct s3c_fb_variant - fb variant information
 * @is_2443: Set if S3C2443/S3C2416 style hardware.
 * @nr_windows: The number of windows.
 * @vidtcon: The base for the VIDTCONx registers
 * @wincon: The base for the WINxCON registers.
 * @winmap: The base for the WINxMAP registers.
 * @keycon: The abse for the WxKEYCON registers.
 * @buf_start: Offset of buffer start registers.
 * @buf_size: Offset of buffer size registers.
 * @buf_end: Offset of buffer end registers.
 * @osd: The base for the OSD registers.
 * @palette: Address of palette memory, or 0 if none.
 * @has_prtcon: Set if has PRTCON register.
 * @has_shadowcon: Set if has SHADOWCON register.
 * @has_blendcon: Set if has BLENDCON register.
 * @has_alphacon: Set if has VIDWALPHA register.
 * @has_clksel: Set if VIDCON0 register has CLKSEL bit.
 * @has_fixvclk: Set if VIDCON1 register has FIXVCLK bits.
 */
struct s3c_fb_variant {
	unsigned int	is_2443:1;
	unsigned short	nr_windows;
	unsigned int	vidtcon;
	unsigned short	wincon;
	unsigned short	winmap;
	unsigned short	keycon;
	unsigned short	buf_start;
	unsigned short	buf_end;
	unsigned short	buf_size;
	unsigned short	osd;
	unsigned short	osd_stride;
	unsigned short	palette[S3C_FB_MAX_WIN];

	unsigned int	has_prtcon:1;
	unsigned int	has_shadowcon:1;
	unsigned int	has_blendcon:1;
	unsigned int	has_alphacon:1;
	unsigned int	has_clksel:1;
	unsigned int	has_fixvclk:1;
};

/**
 * struct s3c_fb_win_variant
 * @has_osd_c: Set if has OSD C register.
 * @has_osd_d: Set if has OSD D register.
 * @has_osd_alpha: Set if can change alpha transparency for a window.
 * @palette_sz: Size of palette in entries.
 * @palette_16bpp: Set if palette is 16bits wide.
 * @osd_size_off: If != 0, supports setting up OSD for a window; the appropriate
 *                register is located at the given offset from OSD_BASE.
 * @valid_bpp: 1 bit per BPP setting to show valid bits-per-pixel.
 *
 * valid_bpp bit x is set if (x+1)BPP is supported.
 */
struct s3c_fb_win_variant {
	unsigned int	has_osd_c:1;
	unsigned int	has_osd_d:1;
	unsigned int	has_osd_alpha:1;
	unsigned int	palette_16bpp:1;
	unsigned short	osd_size_off;
	unsigned short	palette_sz;
	u32		valid_bpp;
};

/**
 * struct s3c_fb_driverdata - per-device type driver data for init time.
 * @variant: The variant information for this driver.
 * @win: The window information for each window.
 */
struct s3c_fb_driverdata {
	struct s3c_fb_variant	variant;
	struct s3c_fb_win_variant *win[S3C_FB_MAX_WIN];
};

struct s3c_reg_data {
	u32			shadowcon;
	u32			wincon[S3C_FB_MAX_WIN];
	u32			win_rgborder[S3C_FB_MAX_WIN];
	u32			winmap[S3C_FB_MAX_WIN];
	u32			vidosd_a[S3C_FB_MAX_WIN];
	u32			vidosd_b[S3C_FB_MAX_WIN];
	u32			vidosd_c[S3C_FB_MAX_WIN];
	u32			vidosd_d[S3C_FB_MAX_WIN];
	u32			vidw_alpha0[S3C_FB_MAX_WIN];
	u32			vidw_alpha1[S3C_FB_MAX_WIN];
	u32			blendeq[S3C_FB_MAX_WIN - 1];
	u32			vidw_buf_start[S3C_FB_MAX_WIN];
	u32			vidw_buf_end[S3C_FB_MAX_WIN];
	u32			vidw_buf_size[S3C_FB_MAX_WIN];
	dma_addr_t		dma_addr[S3C_FB_MAX_WIN];
};

/**
 * struct s3c_fb_win - per window private data for each framebuffer.
 * @windata: The platform data supplied for the window configuration.
 * @parent: The hardware that this window is part of.
 * @varint: The variant information for this window.
 * @index: The window number of this window.
 */
struct s3c_fb_win {
	struct s3c_fb_pd_win	*windata;
	struct s3c_fb		*parent;
	struct s3c_fb_win_variant variant;

	unsigned int		 index;
};

#ifdef CONFIG_DEBUG_FS
#define S3C_FB_DEBUG_FIFO_TIMESTAMPS 32
#define S3C_FB_DEBUG_REGS_SIZE 0x0280

struct s3c_fb_debug {
	ktime_t		fifo_timestamps[S3C_FB_DEBUG_FIFO_TIMESTAMPS];
	unsigned int	num_timestamps;
	unsigned int	first_timestamp;
	u8		regs_at_underflow[S3C_FB_DEBUG_REGS_SIZE];
};
#endif

/**
 * struct s3c_fb - overall hardware state of the hardware
 * @slock: The spinlock protection for this data sturcture.
 * @dev: The device that we bound to, for printing, etc.
 * @bus_clk: The clk (hclk) feeding our interface and possibly pixclk.
 * @lcd_clk: The clk (sclk) feeding pixclk.
 * @regs: The mapped hardware registers.
 * @variant: Variant information for this hardware.
 * @pdata: The platform configuration data passed with the device.
 * @windows: The hardware windows that have been claimed.
 * @irq_no: IRQ line number
 */
struct s3c_fb {
	struct adf_device base;
	struct adf_overlay_engine eng;
	struct adf_interface intf;
	struct adf_fbdev fbdev;

	spinlock_t		slock;
	struct device		*dev;
	struct clk		*bus_clk;
	struct clk		*lcd_clk;
	void __iomem		*regs;
	struct s3c_fb_variant	 variant;

	struct s3c_fb_platdata	*pdata;
	struct s3c_fb_win	*windows[S3C_FB_MAX_WIN];

	int			 irq_no;

	struct ion_client	*fb_ion_client;

#ifdef CONFIG_DEBUG_FS
	struct dentry		*debug_dentry;
	struct s3c_fb_debug	debug_data;
#endif
	struct exynos5_bus_mif_handle *fb_mif_handle;
	struct exynos5_bus_int_handle *fb_int_handle;

	struct s3c_reg_data *deferred_update;
};

#define adf_device_to_s3c_fb(p) \
	container_of((p), struct s3c_fb, base)

#define adf_overlay_engine_to_s3c_fb(p) \
	container_of((p), struct s3c_fb, eng)

#define adf_interface_to_s3c_fb(p) \
	container_of((p), struct s3c_fb, intf)

static void s3c_fb_dump_registers(struct s3c_fb *sfb)
{
#ifdef CONFIG_FB_EXYNOS_FIMD_V8
	pr_err("dumping registers\n");
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4, sfb->regs,
			0x0280, false);
	pr_err("...\n");
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
			sfb->regs + SHD_VIDW_BUF_START(0), 0x74, false);
	pr_err("...\n");
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
			sfb->regs + 0x20000, 0x20, false);
#endif
}

static bool s3c_fb_validate_x_alignment(struct s3c_fb *sfb, int x, u32 w,
		u32 bits_per_pixel)
{
	uint8_t pixel_alignment = 32 / bits_per_pixel;

	if (x % pixel_alignment) {
		dev_err(sfb->dev, "left X coordinate not properly aligned to %u-pixel boundary (bpp = %u, x = %u)\n",
				pixel_alignment, bits_per_pixel, x);
		return 0;
	}
	if ((x + w) % pixel_alignment) {
		dev_err(sfb->dev, "right X coordinate not properly aligned to %u-pixel boundary (bpp = %u, x = %u, w = %u)\n",
				pixel_alignment, bits_per_pixel, x, w);
		return 0;
	}

	return 1;
}

/**
 * s3c_fb_calc_pixclk() - calculate the divider to create the pixel clock.
 * @sfb: The hardware state.
 * @pixclock: The pixel clock wanted, in kHz.
 *
 * Given the specified pixel clock, work out the necessary divider to get
 * close to the output frequency.
 */
static int s3c_fb_calc_pixclk(struct s3c_fb *sfb, unsigned int pixclk)
{
	unsigned long clk;

	if (sfb->variant.has_clksel)
		clk = clk_get_rate(sfb->bus_clk);
	else
		clk = clk_get_rate(sfb->lcd_clk);

	return clk / pixclk / 1000;
}

/**
 * shadow_protect_win() - disable updating values from shadow registers at vsync
 *
 * @win: window to protect registers for
 * @protect: 1 to protect (disable updates)
 */
static void shadow_protect_win(struct s3c_fb_win *win, bool protect)
{
	struct s3c_fb *sfb = win->parent;
	u32 reg;

	if (protect) {
		if (sfb->variant.has_prtcon) {
			writel(PRTCON_PROTECT, sfb->regs + PRTCON);
		} else if (sfb->variant.has_shadowcon) {
			reg = readl(sfb->regs + SHADOWCON);
			writel(reg | SHADOWCON_WINx_PROTECT(win->index),
				sfb->regs + SHADOWCON);
		}
	} else {
		if (sfb->variant.has_prtcon) {
			writel(0, sfb->regs + PRTCON);
		} else if (sfb->variant.has_shadowcon) {
			reg = readl(sfb->regs + SHADOWCON);
			writel(reg & ~SHADOWCON_WINx_PROTECT(win->index),
				sfb->regs + SHADOWCON);
		}
	}
}

static inline u32 vidw_buf_size(u32 xres, u32 line_length, u32 bits_per_pixel)
{
	u32 pagewidth = (xres * bits_per_pixel) >> 3;
	return VIDW_BUF_SIZE_OFFSET(line_length - pagewidth) |
	       VIDW_BUF_SIZE_PAGEWIDTH(pagewidth) |
	       VIDW_BUF_SIZE_OFFSET_E(line_length - pagewidth) |
	       VIDW_BUF_SIZE_PAGEWIDTH_E(pagewidth);
}

static inline u32 vidosd_a(int x, int y)
{
	return VIDOSDxA_TOPLEFT_X(x) |
			VIDOSDxA_TOPLEFT_Y(y) |
			VIDOSDxA_TOPLEFT_X_E(x) |
			VIDOSDxA_TOPLEFT_Y_E(y);
}

static inline u32 vidosd_b(int x, int y, u32 xres, u32 yres)
{
	return VIDOSDxB_BOTRIGHT_X(x + xres - 1) |
		VIDOSDxB_BOTRIGHT_Y(y + yres - 1) |
		VIDOSDxB_BOTRIGHT_X_E(x + xres - 1) |
		VIDOSDxB_BOTRIGHT_Y_E(y + yres - 1);
}

static inline u32 vidosd_c(u8 r0, u8 g0, u8 b0, u8 r1, u8 g1, u8 b1)
{
	return VIDOSDxC_ALPHA0_R_H(r0) |
		VIDOSDxC_ALPHA0_G_H(g0) |
		VIDOSDxC_ALPHA0_B_H(b0) |
		VIDOSDxC_ALPHA1_R_H(r1) |
		VIDOSDxC_ALPHA1_G_H(g1) |
		VIDOSDxC_ALPHA1_B_H(b1);
}

static inline u32 vidw_alpha(bool has_osd_alpha, u8 r, u8 g, u8 b)
{
	if (has_osd_alpha)
		return VIDWxALPHAx_R_L(r) |
			VIDWxALPHAx_G_L(g) |
			VIDWxALPHAx_B_L(b);
	else
		return VIDWxALPHAx_R(r) |
			VIDWxALPHAx_G(g) |
			VIDWxALPHAx_B(b);
}

static inline u32 wincon(u32 bits_per_pixel, u32 transp_length, u32 red_length)
{
	u32 data = 0;

	switch (bits_per_pixel) {
	case 1:
		data |= WINCON0_BPPMODE_1BPP;
		data |= WINCONx_BITSWP;
		data |= WINCONx_BURSTLEN_4WORD;
		break;
	case 2:
		data |= WINCON0_BPPMODE_2BPP;
		data |= WINCONx_BITSWP;
		data |= WINCONx_BURSTLEN_8WORD;
		break;
	case 4:
		data |= WINCON0_BPPMODE_4BPP;
		data |= WINCONx_BITSWP;
		data |= WINCONx_BURSTLEN_8WORD;
		break;
	case 8:
		if (transp_length != 0)
			data |= WINCON1_BPPMODE_8BPP_1232;
		else
			data |= WINCON0_BPPMODE_8BPP_PALETTE;
		data |= WINCONx_BURSTLEN_8WORD;
		data |= WINCONx_BYTSWP;
		break;
	case 16:
		if (transp_length == 1)
			data |= WINCON1_BPPMODE_16BPP_A1555
				| WINCON1_BLD_PIX;
		else if (transp_length == 4)
			data |= WINCON1_BPPMODE_16BPP_A4444
				| WINCON1_BLD_PIX;
		else
			data |= WINCON0_BPPMODE_16BPP_565;
		data |= WINCONx_HAWSWP;
		data |= WINCONx_BURSTLEN_16WORD;
		break;
	case 24:
	case 32:
		if (red_length == 6) {
			if (transp_length != 0)
				data |= WINCON1_BPPMODE_19BPP_A1666;
			else
				data |= WINCON1_BPPMODE_18BPP_666;
		} else if (transp_length == 1)
			data |= WINCON1_BPPMODE_25BPP_A1888
				| WINCON1_BLD_PIX;
		else if ((transp_length == 4) ||
			(transp_length == 8))
			data |= WINCON1_BPPMODE_28BPP_A4888
				| WINCON1_BLD_PIX;
		else
			data |= WINCON0_BPPMODE_24BPP_888;

		data |= WINCONx_WSWP;
		data |= WINCONx_BURSTLEN_16WORD;
		break;
	}

	if (transp_length != 1)
		data |= WINCON1_ALPHA_SEL;

	return data;
}

static inline u32 blendeq(enum s3c_fb_blending blending, u8 transp_length)
{
	u8 a, b;

	if (transp_length == 1 && blending == S3C_FB_BLENDING_PREMULT)
		blending = S3C_FB_BLENDING_COVERAGE;

	switch (blending) {
	case S3C_FB_BLENDING_NONE:
		a = BLENDEQ_COEF_ONE;
		b = BLENDEQ_COEF_ZERO;
		break;

	case S3C_FB_BLENDING_PREMULT:
		a = BLENDEQ_COEF_ONE;
		b = BLENDEQ_COEF_ONE_MINUS_ALPHA_A;
		break;

	case S3C_FB_BLENDING_COVERAGE:
		a = BLENDEQ_COEF_ALPHA_A;
		b = BLENDEQ_COEF_ONE_MINUS_ALPHA_A;
		break;

	default:
		return 0;
	}

	return BLENDEQ_A_FUNC(a) |
			BLENDEQ_B_FUNC(b) |
			BLENDEQ_P_FUNC(BLENDEQ_COEF_ZERO) |
			BLENDEQ_Q_FUNC(BLENDEQ_COEF_ZERO);
}

static int s3c_fb_modeset(struct adf_interface *intf,
		struct drm_mode_modeinfo *win_mode)
{
	struct s3c_fb *sfb = adf_interface_to_s3c_fb(intf);
	int clkdiv = s3c_fb_calc_pixclk(sfb, win_mode->clock);
	u32 data = sfb->pdata->vidcon0;
	data &= ~(VIDCON0_CLKVAL_F_MASK | VIDCON0_CLKDIR);
	if (clkdiv > 1)
		data |= VIDCON0_CLKVAL_F(clkdiv-1) | VIDCON0_CLKDIR;
	else
		data &= ~VIDCON0_CLKDIR;

	/* write the timing data to the panel */
	if (sfb->variant.is_2443)
		data |= (1 << 5);

	data |= VIDCON0_ENVID | VIDCON0_ENVID_F;
	writel(data, sfb->regs + VIDCON0);
	data = readl(sfb->regs + VIDCON2);
	data &= ~(VIDCON2_RGB_ORDER_E_MASK | VIDCON2_RGB_ORDER_O_MASK);
	data |= VIDCON2_RGB_ORDER_E_BGR | VIDCON2_RGB_ORDER_O_BGR;
	writel(data, sfb->regs + VIDCON2);

	/* Set alpha value width */
	if (sfb->variant.has_blendcon) {
		data = readl(sfb->regs + BLENDCON);
		data &= ~BLENDCON_NEW_MASK;
		data |= BLENDCON_NEW_8BIT_ALPHA_VALUE;
		writel(data, sfb->regs + BLENDCON);
	}

	data = VIDTCON0_VBPD(win_mode->vtotal - win_mode->vsync_end - 1)
			| VIDTCON0_VFPD(win_mode->vsync_start -
					win_mode->vdisplay - 1)
			| VIDTCON0_VSPW(win_mode->vsync_end -
					win_mode->vsync_start - 1);
	writel(data, sfb->regs + sfb->variant.vidtcon);
	data = VIDTCON1_HBPD(win_mode->htotal - win_mode->hsync_end - 1)
			| VIDTCON1_HFPD(win_mode->hsync_start -
					win_mode->hdisplay - 1)
			| VIDTCON1_HSPW(win_mode->hsync_end -
					win_mode->hsync_start - 1);
	/* VIDTCON1 */
	writel(data, sfb->regs + sfb->variant.vidtcon + 4);
	data = VIDTCON2_LINEVAL(win_mode->vdisplay - 1)
			| VIDTCON2_HOZVAL(win_mode->hdisplay - 1)
			| VIDTCON2_LINEVAL_E(win_mode->vdisplay - 1)
			| VIDTCON2_HOZVAL_E(win_mode->hdisplay - 1);
	/* VIDTCON2 */
	writel(data, sfb->regs + sfb->variant.vidtcon + 8);

	return 0;
}

static unsigned int s3c_fb_calc_bandwidth(u32 w, u32 h, u32 bits_per_pixel, int fps)
{
	unsigned int bw = w * h;

	bw *= DIV_ROUND_UP(bits_per_pixel, 8);
	bw *= fps;

	return bw;
}

static int s3c_fb_screen_size(struct adf_interface *intf, u16 *width_mm,
		u16 *height_mm)
{
	struct s3c_fb *sfb = adf_interface_to_s3c_fb(intf);
	*width_mm = sfb->pdata->win[0]->width;
	*width_mm = sfb->pdata->win[0]->height;
	return 0;
}

static int s3c_fb_enable(struct s3c_fb *sfb);
static int s3c_fb_disable(struct s3c_fb *sfb);

/**
 * s3c_fb_blank() - blank or unblank the given window
 * @blank_mode: The blank state from FB_BLANK_*
 * @info: The framebuffer to blank.
 *
 * Framebuffer layer request to change the power state.
 */
static int s3c_fb_blank(struct adf_interface *intf, u8 dpms_state)
{
	struct s3c_fb *sfb = adf_interface_to_s3c_fb(intf);
	int ret = 0;

	dev_dbg(sfb->dev, "dpms state %d\n", dpms_state);

	pm_runtime_get_sync(sfb->dev);

	switch (dpms_state) {
	case DRM_MODE_DPMS_OFF:
		ret = s3c_fb_disable(sfb);
		break;

	case DRM_MODE_DPMS_ON:
		ret = s3c_fb_enable(sfb);
		break;

	default:
		ret = -EINVAL;
	}

	pm_runtime_put_sync(sfb->dev);

	if (ret == 0) {
		struct fb_event event;
		int fb_state = (dpms_state == DRM_MODE_DPMS_OFF) ?
				FB_BLANK_POWERDOWN : FB_BLANK_UNBLANK;

		event.info = NULL;
		event.data = &fb_state;
		fb_notifier_call_chain(FB_EVENT_BLANK, &event);
	}

	return ret;
}

/**
 * s3c_fb_enable_irq() - enable framebuffer interrupts
 * @sfb: main hardware state
 */
static void s3c_fb_enable_irq(struct s3c_fb *sfb)
{
	void __iomem *regs = sfb->regs;
	u32 irq_ctrl_reg;

	pm_runtime_get_sync(sfb->dev);
	irq_ctrl_reg = readl(regs + VIDINTCON0);

	irq_ctrl_reg |= VIDINTCON0_INT_ENABLE;
	irq_ctrl_reg |= VIDINTCON0_INT_FRAME;
#ifdef CONFIG_DEBUG_FS
	irq_ctrl_reg &= ~VIDINTCON0_FIFOLEVEL_MASK;
	irq_ctrl_reg |= VIDINTCON0_FIFOLEVEL_EMPTY;
	irq_ctrl_reg |= VIDINTCON0_INT_FIFO;
	irq_ctrl_reg |= VIDINTCON0_FIFIOSEL_WINDOW0;
	irq_ctrl_reg |= VIDINTCON0_FIFIOSEL_WINDOW1;
	irq_ctrl_reg |= VIDINTCON0_FIFIOSEL_WINDOW2;
	irq_ctrl_reg |= VIDINTCON0_FIFIOSEL_WINDOW3;
	irq_ctrl_reg |= VIDINTCON0_FIFIOSEL_WINDOW4;
#endif

	irq_ctrl_reg &= ~VIDINTCON0_FRAMESEL0_MASK;
	irq_ctrl_reg |= VIDINTCON0_FRAMESEL0_VSYNC;
	irq_ctrl_reg &= ~VIDINTCON0_FRAMESEL1_MASK;
	irq_ctrl_reg |= VIDINTCON0_FRAMESEL1_NONE;

	writel(irq_ctrl_reg, regs + VIDINTCON0);
	pm_runtime_put_sync(sfb->dev);
}

/**
 * s3c_fb_disable_irq() - disable framebuffer interrupts
 * @sfb: main hardware state
 */
static void s3c_fb_disable_irq(struct s3c_fb *sfb)
{
	void __iomem *regs = sfb->regs;
	u32 irq_ctrl_reg;

	pm_runtime_get_sync(sfb->dev);
	irq_ctrl_reg = readl(regs + VIDINTCON0);

#ifdef CONFIG_DEBUG_FS
	irq_ctrl_reg &= VIDINTCON0_INT_FIFO;
#endif
	irq_ctrl_reg &= ~VIDINTCON0_INT_FRAME;
	irq_ctrl_reg &= ~VIDINTCON0_INT_ENABLE;

	writel(irq_ctrl_reg, regs + VIDINTCON0);
	pm_runtime_put_sync(sfb->dev);
}

static void s3c_fb_set_vsync(struct adf_interface *intf, bool enabled)
{
	struct s3c_fb *sfb = adf_interface_to_s3c_fb(intf);
	if (enabled)
		s3c_fb_enable_irq(sfb);
	else
		s3c_fb_disable_irq(sfb);
}

static bool s3c_fb_supports_event(struct adf_obj *obj, enum adf_event_type type)
{
	return obj->type == ADF_OBJ_INTERFACE && type == ADF_EVENT_VSYNC;
}

static void s3c_fb_set_event(struct adf_obj *obj, enum adf_event_type type,
		bool enabled)
{
	s3c_fb_set_vsync(adf_obj_to_interface(obj), enabled);
}

/**
 * sfb_fb_log_fifo_underflow_locked - log a FIFO underflow event.  Caller must
 * hold the driver's spin lock while calling.
 *
 * @sfb: main hardware state
 * @timestamp: timestamp of the FIFO underflow event
 */
void s3c_fb_log_fifo_underflow_locked(struct s3c_fb *sfb, ktime_t timestamp)
{
#ifdef CONFIG_DEBUG_FS
	unsigned int idx = sfb->debug_data.num_timestamps %
			S3C_FB_DEBUG_FIFO_TIMESTAMPS;
	sfb->debug_data.fifo_timestamps[idx] = timestamp;
	sfb->debug_data.num_timestamps++;
	if (sfb->debug_data.num_timestamps > S3C_FB_DEBUG_FIFO_TIMESTAMPS) {
		sfb->debug_data.first_timestamp++;
		sfb->debug_data.first_timestamp %= S3C_FB_DEBUG_FIFO_TIMESTAMPS;
	}

	memcpy(sfb->debug_data.regs_at_underflow, sfb->regs,
			sizeof(sfb->debug_data.regs_at_underflow));
#endif
}

static irqreturn_t s3c_fb_irq(int irq, void *dev_id)
{
	struct s3c_fb *sfb = dev_id;
	void __iomem  *regs = sfb->regs;
	u32 irq_sts_reg;
	ktime_t timestamp = ktime_get();

	irq_sts_reg = readl(regs + VIDINTCON1);

	if (irq_sts_reg & VIDINTCON1_INT_FRAME) {

		/* VSYNC interrupt, accept it */
		writel(VIDINTCON1_INT_FRAME, regs + VIDINTCON1);

		adf_vsync_notify(&sfb->intf, timestamp);
	}
	if (irq_sts_reg & VIDINTCON1_INT_FIFO) {
		writel(VIDINTCON1_INT_FIFO, regs + VIDINTCON1);
		spin_lock(&sfb->slock);
		s3c_fb_log_fifo_underflow_locked(sfb, timestamp);
		spin_unlock(&sfb->slock);
	}

	return IRQ_HANDLED;
}

static u32 s3c_fb_red_length(u32 format)
{
	switch (format) {
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_BGRA8888:
		return 8;

	case DRM_FORMAT_RGBA5551:
		return 5;

	case DRM_FORMAT_RGB565:
		return 5;

	default:
		pr_warn("s3c-fb: unrecognized pixel format %u\n", format);
		return 0;
	}
}

static u32 s3c_fb_transp_length(u32 format)
{
	switch (format) {
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_BGRA8888:
		return 8;

	case DRM_FORMAT_RGBA5551:
		return 1;

	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_RGB565:
		return 0;

	default:
		pr_warn("s3c-fb: unrecognized pixel format %u\n", format);
		return 0;
	}
}

static u32 s3c_fb_rgborder(u32 format)
{
	switch (format) {
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBA5551:
		return WIN_RGB_ORDER_RGB;

	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGRA8888:
		return WIN_RGB_ORDER_BGR;

	default:
		pr_warn("s3c-fb: unrecognized pixel format %u\n", format);
		return 0;
	}

}

static int s3c_fb_validate_buffer(struct s3c_fb *sfb, struct s3c_fb_win *win,
		struct adf_buffer *fb, struct adf_buffer_mapping *mapping,
		struct s3c_fb_win_config *win_config, struct s3c_reg_data *regs)
{
	unsigned short win_no = win->index;
	size_t window_size;
	u8 alpha0, alpha1;

	u8 bpp = adf_format_bpp(fb->format);
	u32 red_length = s3c_fb_red_length(fb->format);
	u32 transp_length = s3c_fb_transp_length(fb->format);

	if (win_config->blending >= S3C_FB_BLENDING_MAX) {
		dev_err(sfb->dev, "unknown blending %u\n",
				win_config->blending);
		return -EINVAL;
	}

	if (win_no == 0 && win_config->blending != S3C_FB_BLENDING_NONE) {
		dev_err(sfb->dev, "blending not allowed on window 0\n");
		return -EINVAL;
	}

	if (fb->pitch[0] < 128) {
		dev_err(sfb->dev, "window must be at least 128 bytes wide (pitch = %u)\n",
				fb->pitch[0]);
		return -EINVAL;
	}

	if (!s3c_fb_validate_x_alignment(sfb, win_config->x, fb->w,
			bpp)) {
		return -EINVAL;
	}

	window_size = fb->pitch[0] * (fb->h + 1);
	if (fb->offset[0] + window_size > fb->dma_bufs[0]->size) {
		dev_err(sfb->dev, "window goes past end of buffer (width = %u, height = %u, pitch = %u, offset = %u, buf_size = %u)\n",
				fb->w, fb->h, fb->pitch[0], fb->offset[0],
				fb->dma_bufs[0]->size);
		return -EINVAL;
	}

	regs->dma_addr[win_no] = iovmm_map(&s5p_device_fimd1.dev,
			mapping->sg_tables[0]->sgl, 0, fb->dma_bufs[0]->size);
	if (!regs->dma_addr[win_no] || IS_ERR_VALUE(regs->dma_addr[win_no])) {
		dev_err(sfb->dev, "iovmm_map() failed: %d\n",
				regs->dma_addr[win_no]);
		regs->dma_addr[win_no] = 0;
		return -ENOMEM;
	}

	regs->win_rgborder[win_no] = s3c_fb_rgborder(fb->format);
	regs->vidw_buf_start[win_no] = regs->dma_addr[win_no] + fb->offset[0];
	regs->vidw_buf_end[win_no] = regs->vidw_buf_start[win_no] + window_size;
	regs->vidw_buf_size[win_no] = vidw_buf_size(fb->w, fb->pitch[0], bpp);

	regs->vidosd_a[win_no] = vidosd_a(win_config->x, win_config->y);
	regs->vidosd_b[win_no] = vidosd_b(win_config->x, win_config->y, fb->w,
			fb->h);

	if (transp_length == 1 &&
			win_config->blending == S3C_FB_BLENDING_NONE) {
		alpha0 = 0xff;
		alpha1 = 0xff;
	}
	else {
		alpha0 = 0;
		alpha1 = 0xff;
	}

	if (win->variant.has_osd_alpha)
		regs->vidosd_c[win_no] = vidosd_c(alpha0, alpha0, alpha0,
				alpha1, alpha1, alpha1);
	regs->vidw_alpha0[win_no] = vidw_alpha(win->variant.has_osd_alpha,
			alpha0, alpha0, alpha0);
	regs->vidw_alpha1[win_no] = vidw_alpha(win->variant.has_osd_alpha,
			alpha1, alpha1, alpha1);

	if (win->variant.osd_size_off) {
		u32 size = fb->w * fb->h;
		if (win->variant.has_osd_alpha)
			regs->vidosd_d[win_no] = size;
		else
			regs->vidosd_c[win_no] = size;
	}

	regs->shadowcon |= SHADOWCON_CHx_ENABLE(win_no);

	regs->wincon[win_no] = wincon(bpp, transp_length, red_length);
	if (win_no)
		regs->blendeq[win_no - 1] = blendeq(win_config->blending,
				transp_length);

	return 0;
}

static void s3c_fb_state_free(struct adf_device *dev, void *driver_data)
{
	struct s3c_fb *sfb = adf_device_to_s3c_fb(dev);
	struct s3c_reg_data *regs = driver_data;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(regs->dma_addr); i++)
		if (regs->dma_addr[i])
			iovmm_unmap(sfb->dev, regs->dma_addr[i]);
	kfree(regs);
}

static int s3c_fb_validate(struct adf_device *dev, struct adf_post *cfg,
		void **driver_data)
{
	struct s3c_fb *sfb = adf_device_to_s3c_fb(dev);
	struct s3c_fb_win_config_data *win_config;
	int ret = 0;
	unsigned short i;
	struct s3c_reg_data *regs;
	unsigned int bw = 0;

	if (cfg->n_bufs > sfb->variant.nr_windows) {
		dev_err(sfb->dev, "post config has %u buffers, but hardware has only %u windows\n",
				cfg->n_bufs, sfb->variant.nr_windows);
		return -EINVAL;
	}

	if (cfg->custom_data_size != sizeof(*win_config)) {
		dev_err(sfb->dev, "expected %u bytes of custom post data, got %d bytes\n",
				sizeof(*win_config), cfg->custom_data_size);
		return -EINVAL;
	}
	win_config = cfg->custom_data;

	regs = kzalloc(sizeof(struct s3c_reg_data), GFP_KERNEL);
	if (!regs) {
		dev_err(sfb->dev, "could not allocate s3c_reg_data\n");
		return -ENOMEM;
	}

	for (i = 0; i < sfb->variant.nr_windows && !ret; i++) {
		struct s3c_fb_win_config *config = &win_config->config[i];
		struct s3c_fb_win *win = sfb->windows[i];
		struct adf_buffer *buf = NULL;
		struct adf_buffer_mapping *mapping;

		bool enabled = 0;
		u32 color_map = WINxMAP_MAP | WINxMAP_MAP_COLOUR(0);

		switch (config->state) {
		case S3C_FB_WIN_STATE_DISABLED:
			break;
		case S3C_FB_WIN_STATE_COLOR:
			enabled = 1;
			color_map |= WINxMAP_MAP_COLOUR(config->color);
			regs->vidosd_a[i] = vidosd_a(config->x, config->y);
			regs->vidosd_b[i] = vidosd_b(config->x, config->y,
					config->w, config->h);
			break;
		case S3C_FB_WIN_STATE_BUFFER:
			if (config->fb >= cfg->n_bufs) {
				dev_err(sfb->dev, "window %u refers to fb %u but config only contains %u bufs\n",
						i, config->fb, cfg->n_bufs);
				ret = -EINVAL;
				break;
			}
			buf = &cfg->bufs[config->fb];
			mapping = &cfg->mappings[config->fb];
			ret = s3c_fb_validate_buffer(sfb, win, buf, mapping,
					config, regs);
			if (!ret) {
				enabled = 1;
				color_map = 0;
			}
			break;
		default:
			dev_warn(sfb->dev, "unrecognized window state %u",
					config->state);
			ret = -EINVAL;
			break;
		}

		if (enabled)
			regs->wincon[i] |= WINCONx_ENWIN;
		else
			regs->wincon[i] &= ~WINCONx_ENWIN;
		regs->winmap[i] = color_map;

		if (enabled && buf) {
			bw += s3c_fb_calc_bandwidth(buf->w, buf->h,
					adf_format_bpp(buf->format),
					sfb->intf.current_mode.vrefresh);
		}
	}

	dev_dbg(sfb->dev, "Total BW = %d Mbits, Max BW per window = %d Mbits\n",
			bw / (1024 * 1024), MAX_BW_PER_WINDOW / (1024 * 1024));

	if (bw > MAX_BW_PER_WINDOW)
		exynos5_mif_multiple_windows(true);
	else
		exynos5_mif_multiple_windows(false);

	if (ret)
		s3c_fb_state_free(dev, regs);
	else
		*driver_data = regs;

	return ret;
}

static void __s3c_fb_update_regs(struct s3c_fb *sfb, struct s3c_reg_data *regs)
{
	unsigned short i;

	for (i = 0; i < sfb->variant.nr_windows; i++)
		shadow_protect_win(sfb->windows[i], 1);

	for (i = 0; i < sfb->variant.nr_windows; i++) {
		writel(regs->wincon[i], sfb->regs + WINCON(i));
		writel(regs->win_rgborder[i], sfb->regs + WIN_RGB_ORDER(i));
		writel(regs->winmap[i], sfb->regs + WINxMAP(i));
		writel(regs->vidosd_a[i],
				sfb->regs + VIDOSD_A(i, sfb->variant));
		writel(regs->vidosd_b[i],
				sfb->regs + VIDOSD_B(i, sfb->variant));
		if (sfb->windows[i]->variant.has_osd_c)
			writel(regs->vidosd_c[i],
					sfb->regs + VIDOSD_C(i, sfb->variant));
		if (sfb->windows[i]->variant.has_osd_d)
			writel(regs->vidosd_d[i],
					sfb->regs + VIDOSD_D(i, sfb->variant));
		writel(regs->vidw_alpha0[i],
				sfb->regs + VIDW_ALPHA0(i));
		writel(regs->vidw_alpha1[i],
				sfb->regs + VIDW_ALPHA1(i));
		writel(regs->vidw_buf_start[i],
				sfb->regs + VIDW_BUF_START(i));
		writel(regs->vidw_buf_end[i],
				sfb->regs + VIDW_BUF_END(i));
		writel(regs->vidw_buf_size[i],
				sfb->regs + VIDW_BUF_SIZE(i));
		if (i)
			writel(regs->blendeq[i - 1], sfb->regs + BLENDEQ(i));
	}
	if (sfb->variant.has_shadowcon)
		writel(regs->shadowcon, sfb->regs + SHADOWCON);

	for (i = 0; i < sfb->variant.nr_windows; i++)
		shadow_protect_win(sfb->windows[i], 0);
}

static void s3c_fb_update_regs(struct s3c_fb *sfb, struct s3c_reg_data *regs)
{
	bool wait_for_vsync;
	int count = 100;
	int i;

	pm_runtime_get_sync(sfb->dev);

	do {
		__s3c_fb_update_regs(sfb, regs);
		adf_vsync_wait(&sfb->intf, VSYNC_TIMEOUT_MSEC);
		wait_for_vsync = false;

		for (i = 0; i < sfb->variant.nr_windows; i++) {
			u32 new_start = regs->vidw_buf_start[i];
			u32 shadow_start = readl(sfb->regs +
					SHD_VIDW_BUF_START(i));
			if (unlikely(new_start != shadow_start)) {
				wait_for_vsync = true;
				break;
			}
		}
	} while (wait_for_vsync && count--);
	if (wait_for_vsync) {
		pr_err("%s: failed to update registers\n", __func__);
		for (i = 0; i < sfb->variant.nr_windows; i++)
			pr_err("window %d new value %08x register value %08x\n",
				i, regs->vidw_buf_start[i],
				readl(sfb->regs + SHD_VIDW_BUF_START(i)));
	}

	pm_runtime_put_sync(sfb->dev);
}

static void s3c_fb_post(struct adf_device *dev, struct adf_post *post,
		void *driver_data)
{
	struct s3c_fb *sfb = adf_device_to_s3c_fb(dev);
	struct s3c_reg_data *regs = driver_data;

	if (sfb->intf.dpms_state == DRM_MODE_DPMS_OFF) {
		/* s3c-fb registers can't be updated with the display off, so
		   defer the actual update until unblank */
		sfb->deferred_update = regs;
	} else {
		sfb->deferred_update = NULL;
		s3c_fb_update_regs(sfb, regs);
	}
}

/**
 * s3c_fb_missing_pixclock() - calculates pixel clock
 * @mode: The video mode to change.
 *
 * Calculate the pixel clock when none has been given through platform data.
 */
static void __devinit s3c_fb_missing_pixclock(struct fb_videomode *mode)
{
	u64 pixclk = 1000000000000ULL;
	u32 div;

	div  = mode->left_margin + mode->hsync_len + mode->right_margin +
	       mode->xres;
	div *= mode->upper_margin + mode->vsync_len + mode->lower_margin +
	       mode->yres;
#if defined(CONFIG_LCD_MIPI_S6E8AB0) /* this define will be delete after mipi lcd supports 60Hz */
	div *= mode->refresh ? : 40;
#else
	div *= mode->refresh ? : 60;
#endif
	do_div(pixclk, div);

	mode->pixclock = pixclk;
}

/**
 * s3c_fb_alloc_simple_buffer() - allocate display memory for framebuffer window
 * @sfb: The base resources for the hardware.
 * @win: The window to initialise memory for.
 *
 * Allocate memory for the given framebuffer.
 */
static int __devinit s3c_fb_alloc_simple_buffer(struct adf_interface *intf,
		u16 w, u16 h, u32 format, struct dma_buf **dma_buf, u32 *offset,
		u32 *pitch)
{
	struct s3c_fb *sfb = adf_interface_to_s3c_fb(intf);
	size_t size;
	struct ion_handle *handle;

	dev_dbg(sfb->dev, "allocating memory for display\n");

	*pitch = w * adf_format_bpp(format) / 8;
	size = PAGE_ALIGN(*pitch * (h + 1));

	dev_dbg(sfb->dev, "want %u bytes\n", size);

	handle = ion_alloc(sfb->fb_ion_client, (size_t)size, 0,
					EXYNOS_ION_HEAP_EXYNOS_MASK, 0);
	if (IS_ERR(handle)) {
		dev_err(sfb->dev, "failed to ion_alloc\n");
		return PTR_ERR(handle);
	}

	*dma_buf = ion_share_dma_buf(sfb->fb_ion_client, handle);
	ion_free(sfb->fb_ion_client, handle);
	if (IS_ERR_OR_NULL(*dma_buf)) {
		dev_err(sfb->dev, "ion_share_dma_buf() failed\n");
		if (IS_ERR(*dma_buf))
			return PTR_ERR(*dma_buf);
		else
			return -ENOMEM;
	}

	*offset = 0;
	return 0;
}

/**
 * s3c_fb_release_win() - release resources for a framebuffer window.
 * @win: The window to cleanup the resources for.
 *
 * Release the resources that where claimed for the hardware window,
 * such as the framebuffer instance and any memory claimed for it.
 */
static void s3c_fb_release_win(struct s3c_fb *sfb, struct s3c_fb_win *win)
{
	u32 data;

	if (sfb->variant.has_shadowcon) {
		data = readl(sfb->regs + SHADOWCON);
		data &= ~SHADOWCON_CHx_ENABLE(win->index);
		data &= ~SHADOWCON_CHx_LOCAL_ENABLE(win->index);
		writel(data, sfb->regs + SHADOWCON);
	}
	kfree(win);
}

/**
 * s3c_fb_probe_win() - register an hardware window
 * @sfb: The base resources for the hardware
 * @variant: The variant information for this window.
 * @res: Pointer to where to place the resultant window.
 *
 * Allocate and do the basic initialisation for one of the hardware's graphics
 * windows.
 */
static int __devinit s3c_fb_probe_win(struct s3c_fb *sfb, unsigned int win_no,
				      struct s3c_fb_win_variant *variant,
				      struct s3c_fb_win **res)
{
	struct s3c_fb_pd_win *windata;
	struct s3c_fb_win *win;

	dev_dbg(sfb->dev, "probing window %d, variant %p\n", win_no, variant);

	windata = sfb->pdata->win[win_no];

	WARN_ON(windata->max_bpp == 0);

	win = kzalloc(sizeof(*win), GFP_KERNEL);
	if (!win)
		return -ENOMEM;
	*res = win;
	win->variant = *variant;
	win->parent = sfb;
	win->windata = windata;
	win->index = win_no;

	return 0;
}

/**
 * s3c_fb_clear_win() - clear hardware window registers.
 * @sfb: The base resources for the hardware.
 * @win: The window to process.
 *
 * Reset the specific window registers to a known state.
 */
static void s3c_fb_clear_win(struct s3c_fb *sfb, int win)
{
	void __iomem *regs = sfb->regs;
	u32 reg;

	writel(0, regs + sfb->variant.wincon + (win * 4));
	writel(0, regs + VIDOSD_A(win, sfb->variant));
	writel(0, regs + VIDOSD_B(win, sfb->variant));
	writel(0, regs + VIDOSD_C(win, sfb->variant));
	if (sfb->variant.has_shadowcon) {
		reg = readl(sfb->regs + SHADOWCON);
		reg &= ~SHADOWCON_CHx_ENABLE(win);
		reg &= ~SHADOWCON_WINx_PROTECT(win);
		writel(reg, sfb->regs + SHADOWCON);
	}
	reg = readl(sfb->regs + WINCON(win));
	reg &= ~WINCONx_ENWIN;
	writel(reg, sfb->regs + WINCON(win));
}

static int s3c_fb_register_mc_subdev_nodes(struct s3c_fb *sfb)
{
	int ret;
	struct exynos_md *md;

	md = (struct exynos_md *)module_name_to_driver_data(MDEV_MODULE_NAME);

	if (!md) {
		dev_err(sfb->dev, "failed to get output media device\n");
		return -ENODEV;
	}

	/* This function is for exposing sub-devices nodes to user space
	 * in case of marking with V4L2_SUBDEV_FL_HAS_DEVNODE flag.
	 *
	 * And it depends on probe sequence
	 * because v4l2_dev ptr is shared all of output devices below
	 *
	 * probe sequence of output devices
	 * output media device -> gscaler -> window in fimd
	 */
	ret = v4l2_device_register_subdev_nodes(&md->v4l2_dev);
	if (ret) {
		dev_err(sfb->dev, "failed to make nodes for subdev\n");
		return ret;
	}

	dev_dbg(sfb->dev, "Register V4L2 subdev nodes for FIMD\n");

	return 0;
}

static int s3c_fb_describe_simple_post(struct adf_interface *intf,
		struct adf_buffer *buf, void *data, size_t *size)
{
	struct s3c_fb_win_config_data *win_config = data;

	win_config->config[0].state = S3C_FB_WIN_STATE_BUFFER;
	win_config->config[0].fb = 0;
	win_config->config[0].blending = S3C_FB_BLENDING_NONE;
	win_config->config[0].x = 0;
	win_config->config[0].y = 0;
	*size = sizeof(*win_config);

	return 0;
}

/*------------------------------------------------------------------ */

int s3c_fb_sysmmu_fault_handler(struct device *dev,
		enum exynos_sysmmu_inttype itype, unsigned long pgtable_base,
		unsigned long fault_addr)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s3c_fb *sfb = platform_get_drvdata(pdev);

	pr_err("FIMD1 PAGE FAULT occurred at 0x%lx (Page table base: 0x%lx)\n",
			fault_addr, pgtable_base);

	s3c_fb_dump_registers(sfb);

	pr_err("Generating Kernel OOPS... because it is unrecoverable.\n");

	BUG();

	return 0;
}

static int __devinit s3c_fb_export_bootloader_logo(struct s3c_fb *sfb,
		struct platform_device *pdev,
		struct drm_mode_modeinfo *panel_mode, struct adf_buffer *buf)
{
	struct resource *res;
	struct dma_buf *dma_buf;
	int ret = 0;

	if (IS_ENABLED(CONFIG_FRAMEBUFFER_CONSOLE))
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res || !res->start || !resource_size(res)) {
		dev_warn(&pdev->dev, "failed to find bootloader framebuffer\n");
		return -ENOENT;
	}

	dma_buf = adf_memblock_export(res->start, resource_size(res), O_RDONLY);
	if (IS_ERR(dma_buf)) {
		dev_warn(&pdev->dev, "failed to export bootloader logo memblock: %ld\n",
				PTR_ERR(dma_buf));
		ret = PTR_ERR(dma_buf);
		goto err;
	}

	memset(buf, 0, sizeof(*buf));
	buf->overlay_engine = &sfb->eng;
	buf->w = panel_mode->hdisplay;
	buf->h = panel_mode->vdisplay;
	buf->format = DRM_FORMAT_RGBX8888;
	buf->dma_bufs[0] = dma_buf;
	buf->offset[0] = 0;
	buf->pitch[0] = panel_mode->hdisplay * 4;
	buf->n_planes = 1;
	return 0;

err:
	if (memblock_free(res->start, resource_size(res)))
		dev_warn(&pdev->dev, "failed to free bootloader framebuffer memblock\n");

	return ret;
}

static const u32 fimd_supported_formats[] = {
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_RGBA5551,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGRA8888,
};

static const struct adf_device_ops s3c_fb_dev_ops = {
	.owner = THIS_MODULE,

	.validate = s3c_fb_validate,
	.post = s3c_fb_post,
	.state_free = s3c_fb_state_free,
};

static const struct adf_overlay_engine_ops s3c_fb_eng_ops = {
	.supported_formats = fimd_supported_formats,
	.n_supported_formats = ARRAY_SIZE(fimd_supported_formats),
};

static const struct adf_interface_ops s3c_fb_intf_ops = {
	.base = {
		.supports_event = s3c_fb_supports_event,
		.set_event = s3c_fb_set_event,
	},
	.blank = s3c_fb_blank,
	.alloc_simple_buffer = s3c_fb_alloc_simple_buffer,
	.describe_simple_post = s3c_fb_describe_simple_post,
	.modeset = s3c_fb_modeset,
	.screen_size = s3c_fb_screen_size,
};

static struct fb_ops s3c_fb_ops = {
	.owner = THIS_MODULE,

	.fb_open = adf_fbdev_open,
	.fb_release = adf_fbdev_release,
	.fb_check_var = adf_fbdev_check_var,
	.fb_set_par = adf_fbdev_set_par,
	.fb_blank = adf_fbdev_blank,
	.fb_pan_display = adf_fbdev_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = adf_fbdev_mmap,
};

#ifdef CONFIG_DEBUG_FS

static int s3c_fb_debugfs_show(struct seq_file *f, void *offset)
{
	struct s3c_fb *sfb = f->private;
	struct s3c_fb_debug *debug_data = kzalloc(sizeof(struct s3c_fb_debug),
			GFP_KERNEL);

	if (!debug_data) {
		seq_printf(f, "kmalloc() failed; can't generate file\n");
		return -ENOMEM;
	}

	spin_lock(&sfb->slock);
	memcpy(debug_data, &sfb->debug_data, sizeof(sfb->debug_data));
	spin_unlock(&sfb->slock);

	seq_printf(f, "%u FIFO underflows\n", debug_data->num_timestamps);
	if (debug_data->num_timestamps) {
		unsigned int i;
		unsigned int temp = S3C_FB_DEBUG_FIFO_TIMESTAMPS;
		unsigned int n = min(debug_data->num_timestamps, temp);

		seq_printf(f, "Last %u FIFO underflow timestamps:\n", n);
		for (i = 0; i < n; i++) {
			unsigned int idx = (debug_data->first_timestamp + i) %
					S3C_FB_DEBUG_FIFO_TIMESTAMPS;
			ktime_t timestamp = debug_data->fifo_timestamps[idx];
			seq_printf(f, "\t%lld\n", ktime_to_ns(timestamp));
		}

		seq_printf(f, "Registers at time of last underflow:\n");
		for (i = 0; i < S3C_FB_DEBUG_REGS_SIZE; i += 32) {
			unsigned char buf[128];
			hex_dump_to_buffer(debug_data->regs_at_underflow + i,
					32, 32, 4, buf,
					sizeof(buf), false);
			seq_printf(f, "%.8x: %s\n", i, buf);
		}
	}

	kfree(debug_data);
	return 0;
}

static int s3c_fb_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, s3c_fb_debugfs_show, inode->i_private);
}

static const struct file_operations s3c_fb_debugfs_fops = {
	.owner		= THIS_MODULE,
	.open		= s3c_fb_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int s3c_fb_debugfs_init(struct s3c_fb *sfb)
{
	sfb->debug_dentry = debugfs_create_file("s3c-fb", S_IRUGO, NULL, sfb,
			&s3c_fb_debugfs_fops);
	if (IS_ERR_OR_NULL(sfb->debug_dentry)) {
		sfb->debug_dentry = NULL;
		dev_err(sfb->dev, "debugfs_create_file() failed\n");
		return -EIO;
	}

	return 0;
}

static void s3c_fb_debugfs_cleanup(struct s3c_fb *sfb)
{
	if (sfb->debug_dentry)
		debugfs_remove(sfb->debug_dentry);
}

#else

static int s3c_fb_debugfs_init(struct s3c_fb *sfb) { return 0; }
static void s3c_fb_debugfs_cleanup(struct s3c_fb *sfb) { }

#endif

static int __devinit s3c_fb_probe(struct platform_device *pdev)
{
	const struct platform_device_id *platid;
	struct s3c_fb_driverdata *fbdrv;
	struct device *dev = &pdev->dev;
	struct s3c_fb_platdata *pd;
	struct s3c_fb *sfb;
	struct resource *res;
	struct drm_mode_modeinfo panel_mode;
	struct adf_buffer bootloader_logo;
	struct s3c_fb_pd_win *windata;
	int win;
	int default_win;
	int ret = 0;
	u32 reg;

	platid = platform_get_device_id(pdev);
	fbdrv = (struct s3c_fb_driverdata *)platid->driver_data;

	if (fbdrv->variant.nr_windows > S3C_FB_MAX_WIN) {
		dev_err(dev, "too many windows, cannot attach\n");
		return -EINVAL;
	}

	pd = pdev->dev.platform_data;
	if (!pd) {
		dev_err(dev, "no platform data specified\n");
		return -EINVAL;
	}

	sfb = devm_kzalloc(dev, sizeof(struct s3c_fb), GFP_KERNEL);
	if (!sfb) {
		dev_err(dev, "no memory for framebuffers\n");
		return -ENOMEM;
	}

	dev_dbg(dev, "allocate new framebuffer %p\n", sfb);

	sfb->dev = dev;
	sfb->pdata = pd;
	sfb->variant = fbdrv->variant;

	ret = s3c_fb_debugfs_init(sfb);
	if (ret)
		dev_warn(dev, "failed to initialize debugfs entry\n");

	sfb->bus_clk = clk_get(dev, "lcd");
	if (IS_ERR(sfb->bus_clk)) {
		dev_err(dev, "failed to get bus clock\n");
		return PTR_ERR(sfb->bus_clk);
	}

	clk_enable(sfb->bus_clk);

	if (!sfb->variant.has_clksel) {
		sfb->lcd_clk = clk_get(dev, "sclk_fimd");
		if (IS_ERR(sfb->lcd_clk)) {
			dev_err(dev, "failed to get lcd clock\n");
			ret = PTR_ERR(sfb->lcd_clk);
			goto err_bus_clk;
		}

		clk_enable(sfb->lcd_clk);
	}

	pm_runtime_enable(sfb->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "failed to find registers\n");
		ret = -ENOENT;
		goto err_lcd_clk;
	}

	sfb->regs = devm_request_and_ioremap(dev, res);
	if (!sfb->regs) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_lcd_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "failed to acquire irq resource\n");
		ret = -ENOENT;
		goto err_lcd_clk;
	}
	sfb->irq_no = res->start;
	ret = devm_request_irq(dev, sfb->irq_no, s3c_fb_irq,
			  0, "s3c_fb", sfb);
	if (ret) {
		dev_err(dev, "irq request failed\n");
		goto err_lcd_clk;
	}

	dev_dbg(dev, "got resources (regs %p), probing windows\n", sfb->regs);

	platform_set_drvdata(pdev, sfb);
	pm_runtime_get_sync(sfb->dev);

	/* setup gpio and output polarity controls */

	pd->setup_gpio();

	writel(pd->vidcon1, sfb->regs + VIDCON1);

	/* set video clock running at under-run */
	if (sfb->variant.has_fixvclk) {
		reg = readl(sfb->regs + VIDCON1);
		reg &= ~VIDCON1_VCLK_MASK;
		reg |= VIDCON1_VCLK_RUN;
		writel(reg, sfb->regs + VIDCON1);
	}

	/* zero all windows before we do anything */
	for (win = 0; win < fbdrv->variant.nr_windows; win++)
		if (win != pd->default_win)
			s3c_fb_clear_win(sfb, win);

	/* initialise colour key controls */
	for (win = 0; win < (fbdrv->variant.nr_windows - 1); win++) {
		void __iomem *regs = sfb->regs + sfb->variant.keycon;

		regs += (win * 8);
		writel(0xffffff, regs + WKEYCON0);
		writel(0xffffff, regs + WKEYCON1);
	}
	sfb->fb_ion_client = ion_client_create(ion_exynos, "fimd");
	if (IS_ERR(sfb->fb_ion_client)) {
		dev_err(sfb->dev, "failed to ion_client_create\n");
		goto err_pm_runtime;
	}

	/* setup vmm */
	platform_set_sysmmu(&SYSMMU_PLATDEV(fimd1).dev, &s5p_device_fimd1.dev);
	exynos_sysmmu_set_fault_handler(&s5p_device_fimd1.dev,
			s3c_fb_sysmmu_fault_handler);

	default_win = sfb->pdata->default_win;
	for (win = 0; win < fbdrv->variant.nr_windows; win++) {
		if (!pd->win[win])
			continue;

		if (!pd->win[win]->win_mode.pixclock)
			s3c_fb_missing_pixclock(&pd->win[win]->win_mode);
	}

	/* use platform specified window as the basis for the lcd timings */
	adf_modeinfo_from_fb_videomode(&pd->win[default_win]->win_mode,
			&panel_mode);
	s3c_fb_modeset(&sfb->intf, &panel_mode);

	/* we have the register setup, start allocating framebuffers */
	for (win = 0; win < fbdrv->variant.nr_windows; win++) {
		ret = s3c_fb_probe_win(sfb, win, fbdrv->win[win],
				       &sfb->windows[win]);
		if (ret < 0) {
			dev_err(dev, "failed to create window %d\n", win);
			for (; win >= 0; win--)
				s3c_fb_release_win(sfb, sfb->windows[win]);
			goto err_pm_runtime;
		}
	}

	ret = s3c_fb_register_mc_subdev_nodes(sfb);
	if (ret) {
			dev_err(sfb->dev, "failed to register s3c_fb mc subdev node\n");
			goto err_pm_runtime;
	}

#ifdef CONFIG_S5P_DP
	writel(DPCLKCON_ENABLE, sfb->regs + DPCLKCON);
#endif
	platform_set_drvdata(pdev, sfb);

	ret = adf_device_init(&sfb->base, dev, &s3c_fb_dev_ops, dev_name(dev));
	if (ret < 0)
		goto err_device_init;

	ret = adf_overlay_engine_init(&sfb->eng, &sfb->base, &s3c_fb_eng_ops,
			"FIMD");
	if (ret < 0)
		goto err_engine_init;

	ret = adf_interface_init(&sfb->intf, &sfb->base, ADF_INTF_eDP, 0,
			ADF_INTF_FLAG_PRIMARY, &s3c_fb_intf_ops, "FIMD");
	if (ret < 0)
		goto err_interface_init;
	sfb->intf.dpms_state = DRM_MODE_DPMS_ON;

	ret = adf_attachment_allow(&sfb->base, &sfb->eng, &sfb->intf);
	if (ret < 0)
		goto err_attachment_init;

	memcpy(&sfb->intf.current_mode, &panel_mode, sizeof(panel_mode));
	ret = adf_hotplug_notify_connected(&sfb->intf, &panel_mode, 1);
	if (ret < 0)
		goto err_attachment_init;

	ret = s3c_fb_export_bootloader_logo(sfb, pdev, &panel_mode,
			&bootloader_logo);
	if (ret < 0) {
		s3c_fb_clear_win(sfb, default_win);
	} else {
		struct sync_fence *fence = adf_interface_simple_post(&sfb->intf,
				&bootloader_logo);
		if (IS_ERR(fence)) {
			ret = PTR_ERR(fence);
			dev_warn(sfb->dev, "failed to post bootloader logo: %d\n",
					ret);
			s3c_fb_clear_win(sfb, default_win);
		} else {
			sync_fence_put(fence);
		}
		dma_buf_put(bootloader_logo.dma_bufs[0]);
	}

	adf_vsync_wait(&sfb->intf, 0);
	ret = iovmm_activate(&s5p_device_fimd1.dev);
	if (ret < 0) {
		dev_err(sfb->dev, "failed to activate vmm\n");
		goto err_attachment_init;
	}

	if (!sfb->fb_mif_handle) {
		sfb->fb_mif_handle = exynos5_bus_mif_min(300000);
		if (!sfb->fb_mif_handle)
			dev_err(sfb->dev, "failed to request min_freq for mif \n");
	}

	if (!sfb->fb_int_handle) {
		sfb->fb_int_handle = exynos5_bus_int_min(160000);
		if (!sfb->fb_int_handle)
			dev_err(sfb->dev, "failed to request min_freq for int \n");
	}

	windata = sfb->windows[default_win]->windata;
	ret = adf_fbdev_init(&sfb->fbdev, &sfb->intf, &sfb->eng,
			windata->virtual_x, windata->virtual_y,
			DRM_FORMAT_RGBX8888, &s3c_fb_ops, "%s", dev_name(dev));
	if (ret < 0)
		goto err_attachment_init;

	dev_info(sfb->dev, "window %d: fb %s\n", default_win,
			sfb->base.base.name);

	return 0;

err_attachment_init:
	adf_interface_destroy(&sfb->intf);
err_interface_init:
	adf_overlay_engine_destroy(&sfb->eng);
err_engine_init:
	adf_device_destroy(&sfb->base);
err_device_init:
	if (sfb->fb_mif_handle)
		exynos5_bus_mif_put(sfb->fb_mif_handle);
	if (sfb->fb_int_handle)
		exynos5_bus_int_put(sfb->fb_int_handle);
	iovmm_deactivate(&s5p_device_fimd1.dev);

err_pm_runtime:
	pm_runtime_put_sync(sfb->dev);

err_lcd_clk:
	pm_runtime_disable(sfb->dev);

	if (!sfb->variant.has_clksel) {
		clk_disable(sfb->lcd_clk);
		clk_put(sfb->lcd_clk);
	}

err_bus_clk:
	clk_disable(sfb->bus_clk);
	clk_put(sfb->bus_clk);
	return ret;
}

/**
 * s3c_fb_remove() - Cleanup on module finalisation
 * @pdev: The platform device we are bound to.
 *
 * Shutdown and then release all the resources that the driver allocated
 * on initialisation.
 */
static int __devexit s3c_fb_remove(struct platform_device *pdev)
{
	struct s3c_fb *sfb = platform_get_drvdata(pdev);
	int win;

	pm_runtime_get_sync(sfb->dev);

	adf_fbdev_destroy(&sfb->fbdev);
	adf_interface_destroy(&sfb->intf);
	adf_overlay_engine_destroy(&sfb->eng);
	adf_device_destroy(&sfb->base);

	for (win = 0; win < S3C_FB_MAX_WIN; win++)
		s3c_fb_release_win(sfb, sfb->windows[win]);

	if (!sfb->variant.has_clksel) {
		clk_disable(sfb->lcd_clk);
		clk_put(sfb->lcd_clk);
	}

	clk_disable(sfb->bus_clk);
	clk_put(sfb->bus_clk);

	s3c_fb_debugfs_cleanup(sfb);

	pm_runtime_put_sync(sfb->dev);
	pm_runtime_disable(sfb->dev);

	if (sfb->fb_mif_handle)
		exynos5_bus_mif_put(sfb->fb_mif_handle);
	if (sfb->fb_int_handle)
		exynos5_bus_int_put(sfb->fb_int_handle);

	return 0;
}

static int s3c_fb_disable(struct s3c_fb *sfb)
{
	u32 vidcon0;

	if (sfb->pdata->backlight_off)
		sfb->pdata->backlight_off();

	if (sfb->pdata->lcd_off)
		sfb->pdata->lcd_off();

	vidcon0 = readl(sfb->regs + VIDCON0);

	/* see the note in the framebuffer datasheet about
	 * why you cannot take both of these bits down at the
	 * same time. */

	if (vidcon0 & VIDCON0_ENVID) {
		vidcon0 |= VIDCON0_ENVID;
		vidcon0 &= ~VIDCON0_ENVID_F;
		writel(vidcon0, sfb->regs + VIDCON0);
	} else
		dev_warn(sfb->dev, "ENVID not set while disabling fb");

	if (!sfb->variant.has_clksel)
		clk_disable(sfb->lcd_clk);

	clk_disable(sfb->bus_clk);
	iovmm_deactivate(&s5p_device_fimd1.dev);

	pm_runtime_put_sync(sfb->dev);

	exynos5_mif_multiple_windows(false);
	return 0;
}

/**
 * s3c_fb_enable() - Enable the main LCD output
 * @sfb: The main framebuffer state.
 */
static int s3c_fb_enable(struct s3c_fb *sfb)
{
	struct s3c_fb_platdata *pd = sfb->pdata;
	int win_no;
	int ret;
	u32 reg;

	pm_runtime_get_sync(sfb->dev);

	clk_enable(sfb->bus_clk);

	if (!sfb->variant.has_clksel)
		clk_enable(sfb->lcd_clk);

	/* setup gpio and output polarity controls */
	pd->setup_gpio();
	writel(pd->vidcon1, sfb->regs + VIDCON1);
	writel(REG_CLKGATE_MODE_NON_CLOCK_GATE,
			sfb->regs + REG_CLKGATE_MODE);

	/* set video clock running at under-run */
	if (sfb->variant.has_fixvclk) {
		reg = readl(sfb->regs + VIDCON1);
		reg &= ~VIDCON1_VCLK_MASK;
		reg |= VIDCON1_VCLK_RUN;
		writel(reg, sfb->regs + VIDCON1);
	}

	if (sfb->deferred_update) {
		__s3c_fb_update_regs(sfb, sfb->deferred_update);
		sfb->deferred_update = NULL;
	} else {
		/* zero all windows before we do anything */
		for (win_no = 0; win_no < sfb->variant.nr_windows; win_no++)
			s3c_fb_clear_win(sfb, win_no);
	}

	s3c_fb_modeset(&sfb->intf, &sfb->intf.current_mode);

	ret = iovmm_activate(&s5p_device_fimd1.dev);
	if (ret < 0) {
		dev_err(sfb->dev, "failed to reactivate vmm\n");
		return ret;
	}

#ifdef CONFIG_S5P_DP
	writel(DPCLKCON_ENABLE, sfb->regs + DPCLKCON);
#endif

	reg = readl(sfb->regs + VIDCON0);
	reg |= VIDCON0_ENVID | VIDCON0_ENVID_F;
	writel(reg, sfb->regs + VIDCON0);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int s3c_fb_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s3c_fb *sfb = platform_get_drvdata(pdev);
	int ret = 0;

	if (sfb->intf.dpms_state == DRM_MODE_DPMS_ON) {
		dev_warn(dev, "LCD output on while entering suspend\n");
		ret = -EBUSY;
	}

	return ret;
}

static int s3c_fb_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int s3c_fb_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s3c_fb *sfb = platform_get_drvdata(pdev);

	if (!sfb->variant.has_clksel)
		clk_disable(sfb->lcd_clk);

	clk_disable(sfb->bus_clk);

	if (sfb->fb_mif_handle) {
		if(exynos5_bus_mif_put(sfb->fb_mif_handle))
			dev_err(sfb->dev, "failed to free min_freq for mif \n");
		sfb->fb_mif_handle = NULL;
	}

	if (sfb->fb_int_handle) {
		if(exynos5_bus_int_put(sfb->fb_int_handle))
			dev_err(sfb->dev, "failed to free min_freq for int \n");
		sfb->fb_int_handle = NULL;
	}

	return 0;
}

static int s3c_fb_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s3c_fb *sfb = platform_get_drvdata(pdev);
	struct s3c_fb_platdata *pd = sfb->pdata;

	if (!sfb->fb_mif_handle) {
		sfb->fb_mif_handle = exynos5_bus_mif_min(300000);
		if (!sfb->fb_mif_handle)
			dev_err(sfb->dev, "failed to request min_freq for mif \n");
	}

	if (!sfb->fb_int_handle) {
		sfb->fb_int_handle = exynos5_bus_int_min(160000);
		if (!sfb->fb_int_handle)
			dev_err(sfb->dev, "failed to request min_freq for int \n");
	}

	clk_enable(sfb->bus_clk);

	if (!sfb->variant.has_clksel)
		clk_enable(sfb->lcd_clk);

	/* setup gpio and output polarity controls */
	pd->setup_gpio();
	writel(pd->vidcon1, sfb->regs + VIDCON1);

	return 0;
}
#endif

#define VALID_BPP124 (VALID_BPP(1) | VALID_BPP(2) | VALID_BPP(4))
#define VALID_BPP1248 (VALID_BPP124 | VALID_BPP(8))

static struct s3c_fb_win_variant s3c_fb_data_64xx_wins[] = {
	[0] = {
		.has_osd_c	= 1,
		.osd_size_off	= 0x8,
		.palette_sz	= 256,
		.valid_bpp	= (VALID_BPP1248 | VALID_BPP(16) |
				   VALID_BPP(18) | VALID_BPP(24)),
	},
	[1] = {
		.has_osd_c	= 1,
		.has_osd_d	= 1,
		.osd_size_off	= 0xc,
		.has_osd_alpha	= 1,
		.palette_sz	= 256,
		.valid_bpp	= (VALID_BPP1248 | VALID_BPP(16) |
				   VALID_BPP(18) | VALID_BPP(19) |
				   VALID_BPP(24) | VALID_BPP(25) |
				   VALID_BPP(28)),
	},
	[2] = {
		.has_osd_c	= 1,
		.has_osd_d	= 1,
		.osd_size_off	= 0xc,
		.has_osd_alpha	= 1,
		.palette_sz	= 16,
		.palette_16bpp	= 1,
		.valid_bpp	= (VALID_BPP1248 | VALID_BPP(16) |
				   VALID_BPP(18) | VALID_BPP(19) |
				   VALID_BPP(24) | VALID_BPP(25) |
				   VALID_BPP(28)),
	},
	[3] = {
		.has_osd_c	= 1,
		.has_osd_alpha	= 1,
		.palette_sz	= 16,
		.palette_16bpp	= 1,
		.valid_bpp	= (VALID_BPP124  | VALID_BPP(16) |
				   VALID_BPP(18) | VALID_BPP(19) |
				   VALID_BPP(24) | VALID_BPP(25) |
				   VALID_BPP(28)),
	},
	[4] = {
		.has_osd_c	= 1,
		.has_osd_alpha	= 1,
		.palette_sz	= 4,
		.palette_16bpp	= 1,
		.valid_bpp	= (VALID_BPP(1) | VALID_BPP(2) |
				   VALID_BPP(16) | VALID_BPP(18) |
				   VALID_BPP(19) | VALID_BPP(24) |
				   VALID_BPP(25) | VALID_BPP(28)),
	},
};

static struct s3c_fb_win_variant s3c_fb_data_s5p_wins[] = {
	[0] = {
		.has_osd_c	= 1,
		.osd_size_off	= 0x8,
		.palette_sz	= 256,
		.valid_bpp	= (VALID_BPP1248 | VALID_BPP(13) |
				   VALID_BPP(15) | VALID_BPP(16) |
				   VALID_BPP(18) | VALID_BPP(19) |
				   VALID_BPP(24) | VALID_BPP(25) |
				   VALID_BPP(32)),
	},
	[1] = {
		.has_osd_c	= 1,
		.has_osd_d	= 1,
		.osd_size_off	= 0xc,
		.has_osd_alpha	= 1,
		.palette_sz	= 256,
		.valid_bpp	= (VALID_BPP1248 | VALID_BPP(13) |
				   VALID_BPP(15) | VALID_BPP(16) |
				   VALID_BPP(18) | VALID_BPP(19) |
				   VALID_BPP(24) | VALID_BPP(25) |
				   VALID_BPP(32)),
	},
	[2] = {
		.has_osd_c	= 1,
		.has_osd_d	= 1,
		.osd_size_off	= 0xc,
		.has_osd_alpha	= 1,
		.palette_sz	= 256,
		.valid_bpp	= (VALID_BPP1248 | VALID_BPP(13) |
				   VALID_BPP(15) | VALID_BPP(16) |
				   VALID_BPP(18) | VALID_BPP(19) |
				   VALID_BPP(24) | VALID_BPP(25) |
				   VALID_BPP(32)),
	},
	[3] = {
		.has_osd_c	= 1,
		.has_osd_alpha	= 1,
		.palette_sz	= 256,
		.valid_bpp	= (VALID_BPP1248 | VALID_BPP(13) |
				   VALID_BPP(15) | VALID_BPP(16) |
				   VALID_BPP(18) | VALID_BPP(19) |
				   VALID_BPP(24) | VALID_BPP(25) |
				   VALID_BPP(32)),
	},
	[4] = {
		.has_osd_c	= 1,
		.has_osd_alpha	= 1,
		.palette_sz	= 256,
		.valid_bpp	= (VALID_BPP1248 | VALID_BPP(13) |
				   VALID_BPP(15) | VALID_BPP(16) |
				   VALID_BPP(18) | VALID_BPP(19) |
				   VALID_BPP(24) | VALID_BPP(25) |
				   VALID_BPP(32)),
	},
};

static struct s3c_fb_driverdata s3c_fb_data_64xx = {
	.variant = {
		.nr_windows	= 5,
		.vidtcon	= VIDTCON0,
		.wincon		= WINCON(0),
		.winmap		= WINxMAP(0),
		.keycon		= WKEYCON,
		.osd		= VIDOSD_BASE,
		.osd_stride	= 16,
		.buf_start	= VIDW_BUF_START(0),
		.buf_size	= VIDW_BUF_SIZE(0),
		.buf_end	= VIDW_BUF_END(0),

		.palette = {
			[0] = 0x400,
			[1] = 0x800,
			[2] = 0x300,
			[3] = 0x320,
			[4] = 0x340,
		},

		.has_prtcon	= 1,
		.has_clksel	= 1,
	},
	.win[0]	= &s3c_fb_data_64xx_wins[0],
	.win[1]	= &s3c_fb_data_64xx_wins[1],
	.win[2]	= &s3c_fb_data_64xx_wins[2],
	.win[3]	= &s3c_fb_data_64xx_wins[3],
	.win[4]	= &s3c_fb_data_64xx_wins[4],
};

static struct s3c_fb_driverdata s3c_fb_data_s5pc100 = {
	.variant = {
		.nr_windows	= 5,
		.vidtcon	= VIDTCON0,
		.wincon		= WINCON(0),
		.winmap		= WINxMAP(0),
		.keycon		= WKEYCON,
		.osd		= VIDOSD_BASE,
		.osd_stride	= 16,
		.buf_start	= VIDW_BUF_START(0),
		.buf_size	= VIDW_BUF_SIZE(0),
		.buf_end	= VIDW_BUF_END(0),

		.palette = {
			[0] = 0x2400,
			[1] = 0x2800,
			[2] = 0x2c00,
			[3] = 0x3000,
			[4] = 0x3400,
		},

		.has_prtcon	= 1,
		.has_blendcon	= 1,
		.has_alphacon	= 1,
		.has_clksel	= 1,
	},
	.win[0]	= &s3c_fb_data_s5p_wins[0],
	.win[1]	= &s3c_fb_data_s5p_wins[1],
	.win[2]	= &s3c_fb_data_s5p_wins[2],
	.win[3]	= &s3c_fb_data_s5p_wins[3],
	.win[4]	= &s3c_fb_data_s5p_wins[4],
};

static struct s3c_fb_driverdata s3c_fb_data_s5pv210 = {
	.variant = {
		.nr_windows	= 5,
		.vidtcon	= VIDTCON0,
		.wincon		= WINCON(0),
		.winmap		= WINxMAP(0),
		.keycon		= WKEYCON,
		.osd		= VIDOSD_BASE,
		.osd_stride	= 16,
		.buf_start	= VIDW_BUF_START(0),
		.buf_size	= VIDW_BUF_SIZE(0),
		.buf_end	= VIDW_BUF_END(0),

		.palette = {
			[0] = 0x2400,
			[1] = 0x2800,
			[2] = 0x2c00,
			[3] = 0x3000,
			[4] = 0x3400,
		},

		.has_shadowcon	= 1,
		.has_blendcon	= 1,
		.has_alphacon	= 1,
		.has_clksel	= 1,
		.has_fixvclk	= 1,
	},
	.win[0]	= &s3c_fb_data_s5p_wins[0],
	.win[1]	= &s3c_fb_data_s5p_wins[1],
	.win[2]	= &s3c_fb_data_s5p_wins[2],
	.win[3]	= &s3c_fb_data_s5p_wins[3],
	.win[4]	= &s3c_fb_data_s5p_wins[4],
};

static struct s3c_fb_driverdata s3c_fb_data_exynos4 = {
	.variant = {
		.nr_windows	= 5,
		.vidtcon	= VIDTCON0,
		.wincon		= WINCON(0),
		.winmap		= WINxMAP(0),
		.keycon		= WKEYCON,
		.osd		= VIDOSD_BASE,
		.osd_stride	= 16,
		.buf_start	= VIDW_BUF_START(0),
		.buf_size	= VIDW_BUF_SIZE(0),
		.buf_end	= VIDW_BUF_END(0),

		.palette = {
			[0] = 0x2400,
			[1] = 0x2800,
			[2] = 0x2c00,
			[3] = 0x3000,
			[4] = 0x3400,
		},

		.has_shadowcon	= 1,
		.has_blendcon	= 1,
		.has_alphacon	= 1,
		.has_fixvclk	= 1,
	},
	.win[0]	= &s3c_fb_data_s5p_wins[0],
	.win[1]	= &s3c_fb_data_s5p_wins[1],
	.win[2]	= &s3c_fb_data_s5p_wins[2],
	.win[3]	= &s3c_fb_data_s5p_wins[3],
	.win[4]	= &s3c_fb_data_s5p_wins[4],
};

static struct s3c_fb_driverdata s3c_fb_data_exynos5 = {
	.variant = {
		.nr_windows	= 5,
		.vidtcon	= VIDTCON0,
		.wincon		= WINCON(0),
		.winmap		= WINxMAP(0),
		.keycon		= WKEYCON,
		.osd		= VIDOSD_BASE,
		.osd_stride	= 16,
		.buf_start	= VIDW_BUF_START(0),
		.buf_size	= VIDW_BUF_SIZE(0),
		.buf_end	= VIDW_BUF_END(0),

		.palette = {
			[0] = 0x2400,
			[1] = 0x2800,
			[2] = 0x2c00,
			[3] = 0x3000,
			[4] = 0x3400,
		},
		.has_shadowcon	= 1,
		.has_blendcon	= 1,
		.has_alphacon	= 1,
		.has_fixvclk	= 1,
	},
	.win[0]	= &s3c_fb_data_s5p_wins[0],
	.win[1]	= &s3c_fb_data_s5p_wins[1],
	.win[2]	= &s3c_fb_data_s5p_wins[2],
	.win[3]	= &s3c_fb_data_s5p_wins[3],
	.win[4]	= &s3c_fb_data_s5p_wins[4],
};

/* S3C2443/S3C2416 style hardware */
static struct s3c_fb_driverdata s3c_fb_data_s3c2443 = {
	.variant = {
		.nr_windows	= 2,
		.is_2443	= 1,

		.vidtcon	= 0x08,
		.wincon		= 0x14,
		.winmap		= 0xd0,
		.keycon		= 0xb0,
		.osd		= 0x28,
		.osd_stride	= 12,
		.buf_start	= 0x64,
		.buf_size	= 0x94,
		.buf_end	= 0x7c,

		.palette = {
			[0] = 0x400,
			[1] = 0x800,
		},
		.has_clksel	= 1,
	},
	.win[0] = &(struct s3c_fb_win_variant) {
		.palette_sz	= 256,
		.valid_bpp	= VALID_BPP1248 | VALID_BPP(16) | VALID_BPP(24),
	},
	.win[1] = &(struct s3c_fb_win_variant) {
		.has_osd_c	= 1,
		.has_osd_alpha	= 1,
		.palette_sz	= 256,
		.valid_bpp	= (VALID_BPP1248 | VALID_BPP(16) |
				   VALID_BPP(18) | VALID_BPP(19) |
				   VALID_BPP(24) | VALID_BPP(25) |
				   VALID_BPP(28)),
	},
};

static struct s3c_fb_driverdata s3c_fb_data_s5p64x0 = {
	.variant = {
		.nr_windows	= 3,
		.vidtcon	= VIDTCON0,
		.wincon		= WINCON(0),
		.winmap		= WINxMAP(0),
		.keycon		= WKEYCON,
		.osd		= VIDOSD_BASE,
		.osd_stride	= 16,
		.buf_start	= VIDW_BUF_START(0),
		.buf_size	= VIDW_BUF_SIZE(0),
		.buf_end	= VIDW_BUF_END(0),

		.palette = {
			[0] = 0x2400,
			[1] = 0x2800,
			[2] = 0x2c00,
		},

		.has_blendcon	= 1,
		.has_fixvclk	= 1,
	},
	.win[0] = &s3c_fb_data_s5p_wins[0],
	.win[1] = &s3c_fb_data_s5p_wins[1],
	.win[2] = &s3c_fb_data_s5p_wins[2],
};

static struct platform_device_id s3c_fb_driver_ids[] = {
	{
		.name		= "s3c-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_64xx,
	}, {
		.name		= "s5pc100-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_s5pc100,
	}, {
		.name		= "s5pv210-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_s5pv210,
	}, {
		.name		= "exynos4-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_exynos4,
	}, {
		.name		= "exynos5-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_exynos5,
	}, {
		.name		= "s3c2443-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_s3c2443,
	}, {
		.name		= "s5p64x0-fb",
		.driver_data	= (unsigned long)&s3c_fb_data_s5p64x0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, s3c_fb_driver_ids);

static const struct dev_pm_ops s3cfb_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(s3c_fb_suspend, s3c_fb_resume)
	SET_RUNTIME_PM_OPS(s3c_fb_runtime_suspend, s3c_fb_runtime_resume,
			   NULL)
};

static struct platform_driver s3c_fb_driver = {
	.probe		= s3c_fb_probe,
	.remove		= __devexit_p(s3c_fb_remove),
	.id_table	= s3c_fb_driver_ids,
	.driver		= {
		.name	= "s3c-fb",
		.owner	= THIS_MODULE,
		.pm	= &s3cfb_pm_ops,
	},
};

static int __init s3c_fb_init(void)
{
	return platform_driver_register(&s3c_fb_driver);
}

static void __exit s3c_fb_cleanup(void)
{
	platform_driver_unregister(&s3c_fb_driver);
}

late_initcall(s3c_fb_init);
module_exit(s3c_fb_cleanup);

MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_DESCRIPTION("Samsung S3C SoC Framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:s3c-fb");
