/*
 * Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "msm_fb_panel.h"
#include "mipi_hx8357.h"

static struct msm_panel_common_pdata *mipi_hx8357_pdata;
static struct dsi_buf hx8357_tx_buf;
static struct dsi_buf hx8357_rx_buf;
spinlock_t hx8357_spin_lock;

static char HX8357_cmd_set6[] = {0x11};
static char HX8357_cmd_set7[] = {0x29};


// SLPOUT and set 0xB9h = 0xFFh, 0x83h, 0x57h to access the extension commands.
static char HX8357_password_set[] = {
        0xB9, 0xFF, 0x83, 0x57
       };

//b6 set vcom       
static char HX8357_set_vcom[] = {
	0xb6, 0x50
	};

//E9 DITH enable      
static char HX8357_enable_DITH[] = {
	0xe9, 0x80
	};


//B1 set power	
static char HX8357_power_set[] = {
	0xB1, 0x00, 0x14, 0x1e, 
	0x1e, 0x85, 0xaa
	};
	
//C0 set power	
static char HX8357_set_STBA[] = {
	0xC0, 0x36, 0x36, 0x01, 
	0x3c, 0xc8, 0x08
	};	

//B3 set display waveform cycle	
static char HX8357_set_RGB[] = {
	0xB3, 0x40, 0x00, 0x06, 
	0x06
	};

//B4 set display waveform cycle	
static char HX8357_columm_set[] = {
	0xB4, 0x02, 0x40, 0x00, 
	0x2a, 0x2a, 0x0d, 0x8f
	};

//Set internal oscillator (e3h)	
static char HX8357_set_e3[] = {
	0xe3, 0x2f, 0x1f
	};

static char HX8357_cmd9[] = {
	0xcc, 0x05
};


//E0 set gamma curve related setting
static char HX8357_gamma_set[] = {
	0xE0, 0x00, 0x00, 0x00, 
	0x00, 0x02, 0x00, 0x05, 
	0x00, 0x18, 0x00, 0x21, 
	0x00, 0x35, 0x00, 0x41, 
	0x00, 0x4a, 0x00, 0x4e, 
	0x00, 0x47, 0x00, 0x40, 
	0x00, 0x3a, 0x00, 0x30, 
	0x00, 0x2e, 0x00, 0x2c, 
	0x00, 0x07, 0x00, 0x00,
	0x00, 0x02, 0x00, 0x05,
	0x00, 0x18, 0x00, 0x21,
	0x00, 0x35, 0x00, 0x41, 
	0x00, 0x4a, 0x00, 0x4e,
	0x00, 0x47, 0x00, 0x40, 
	0x00, 0x3a, 0x00, 0x30,
	0x00, 0x2e, 0x00, 0x2c,
	0x00, 0x07, 0x00, 0x44
	};

//BA set MIPI mode
static char HX8357_mipi_set[] = {
	0xBA, 0x00, 0x56, 0xd4,
	0x00, 0x0A, 0x00, 0x10,
	0x32, 0x6e, 0x04, 0x05,
	0x9a, 0x14, 0x19, 0x10,
	0x40
	};

//3A set pixel format
static char HX8357_cmd_set2[] = {
	0x3A, 0x70
	};

static char display_bringtness[] = {
	0x51, 0xff,
};

static char crtl_display[] = {
	0x53, 0x24,
};
static char cabc[2] = {
	0x55, 0x02,
};

/*
static char HX8357_Acc_direction[] = {
	0x36, 0xD0,
};

static char HX8357_cmd113[] = {
	0x35, 0x00,
	};
	
static char HX8357_cmd114[] = {
	0x44, 0x03, 0xC0, 
	};
*/

static struct dsi_cmd_desc hx8357_cmd_display_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(HX8357_password_set), HX8357_password_set},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(HX8357_set_vcom), HX8357_set_vcom},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(HX8357_cmd_set2), HX8357_cmd_set2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10, sizeof(HX8357_enable_DITH), HX8357_enable_DITH},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10, sizeof(HX8357_cmd9), HX8357_cmd9},
	//{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(HX8357_Acc_direction), HX8357_Acc_direction},
	//{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(HX8357_cmd113), HX8357_cmd113},
	//{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(HX8357_cmd114), HX8357_cmd114},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,	sizeof(HX8357_set_RGB), HX8357_set_RGB},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,	sizeof(HX8357_power_set), HX8357_power_set},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(HX8357_set_STBA), HX8357_set_STBA},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(HX8357_columm_set), HX8357_columm_set},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(HX8357_set_e3), HX8357_set_e3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(HX8357_mipi_set), HX8357_mipi_set},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(HX8357_gamma_set), HX8357_gamma_set},
	{DTYPE_DCS_WRITE, 1, 0, 0, 150,	sizeof(HX8357_cmd_set6), HX8357_cmd_set6},
	{DTYPE_DCS_WRITE, 1, 0, 0, 30,	sizeof(HX8357_cmd_set7), HX8357_cmd_set7},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(display_bringtness), display_bringtness},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(crtl_display), crtl_display},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(cabc), cabc},
};

static char display_off[2] = {0x28, 0x00};
static char enter_sleep[2] = {0x10, 0x00};

static struct dsi_cmd_desc hx8357_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(enter_sleep), enter_sleep}
};

#define MAX_BL_LEVEL 32
#define LCD_BL_EN 96
static void mipi_hx8357_set_backlight(struct msm_fb_data_type *mfd)
{
	int level = mfd->bl_level;
	int max = mfd->panel_info.bl_max;
	int min = mfd->panel_info.bl_min;
	unsigned long flags;
	int i;

	printk("%s, level = %d\n", __func__, level);

	spin_lock_irqsave(&hx8357_spin_lock, flags); //disable local irq and preemption
	if (level < min)
		level = min;
	if (level > max)
		level = max;

	if (level == 0) {
		gpio_set_value(LCD_BL_EN, 0);
		spin_unlock_irqrestore(&hx8357_spin_lock, flags);
		return;
	}

	for (i = 0; i < (MAX_BL_LEVEL - level + 1); i++) {
		gpio_set_value(LCD_BL_EN, 0);
		udelay(1);
		gpio_set_value(LCD_BL_EN, 1);
		udelay(1);
	}
	spin_unlock_irqrestore(&hx8357_spin_lock, flags);

	return;
}

static int mipi_hx8357_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	printk("kevin %s: Enter\n", __func__);

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_cmds_tx(mfd,&hx8357_tx_buf,
		hx8357_cmd_display_on_cmds,
		ARRAY_SIZE(hx8357_cmd_display_on_cmds));

	printk("%s: Done\n", __func__);
	return 0;
}

static int mipi_hx8357_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	printk("%s: Enter\n", __func__);
	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_cmds_tx(mfd,&hx8357_tx_buf,
		hx8357_display_off_cmds,
		ARRAY_SIZE(hx8357_display_off_cmds));


	printk("%s: Done\n", __func__);
	return 0;
}

static int __devinit mipi_hx8357_lcd_probe(struct platform_device *pdev)
{
	printk("%s: Enter\n", __func__);
	if (pdev->id == 0) {
		mipi_hx8357_pdata = pdev->dev.platform_data;
		return 0;
	}

	spin_lock_init(&hx8357_spin_lock);
	msm_fb_add_device(pdev);

	printk("%s: Done\n", __func__);
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_hx8357_lcd_probe,
	.driver = {
		.name   = "mipi_hx8357",
	},
};

static struct msm_fb_panel_data hx8357_panel_data = {
	.on    = mipi_hx8357_lcd_on,
	.off    = mipi_hx8357_lcd_off,
	.set_backlight    = mipi_hx8357_set_backlight,
};

static int ch_used[3];

int mipi_hx8357_device_register(struct msm_panel_info *pinfo,
                    u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	printk("%s\n", __func__);

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_hx8357", (panel << 8)|channel);
	if (!pdev)
	return -ENOMEM;

	hx8357_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &hx8357_panel_data, sizeof(hx8357_panel_data));
	if (ret) {
		printk(KERN_ERR"%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR"%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

	err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_hx8357_lcd_init(void)
{
	printk("%s\n", __func__);

	mipi_dsi_buf_alloc(&hx8357_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&hx8357_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(mipi_hx8357_lcd_init);

