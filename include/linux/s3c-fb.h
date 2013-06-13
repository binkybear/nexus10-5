/* include/linux/s3c-fb.h
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

#ifndef __S3C_FB_H__
#define __S3C_FB_H__

enum s3c_fb_blending {
	S3C_FB_BLENDING_NONE = 0,
	S3C_FB_BLENDING_PREMULT = 1,
	S3C_FB_BLENDING_COVERAGE = 2,
	S3C_FB_BLENDING_MAX = 3,
};

struct s3c_fb_win_config {
	enum {
		S3C_FB_WIN_STATE_DISABLED = 0,
		S3C_FB_WIN_STATE_COLOR,
		S3C_FB_WIN_STATE_BUFFER,
	} state;

	union {
		struct {
			__u32 color;
			__u32 w;
			__u32 h;
		};
		struct {
			__u8			fb;
			enum s3c_fb_blending	blending;
		};
	};

	int	x;
	int	y;
};

/* S3C_FB_MAX_WIN
 * Set to the maximum number of windows that any of the supported hardware
 * can use. Since the platform data uses this for an array size, having it
 * set to the maximum of any version of the hardware can do is safe.
 */
#define S3C_FB_MAX_WIN	(5)

struct s3c_fb_win_config_data {
	struct s3c_fb_win_config config[S3C_FB_MAX_WIN];
};

#endif /* __S3C_FB_H__ */
