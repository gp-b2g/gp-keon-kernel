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
#include "mipi_ILI9487.h"

static struct msm_panel_common_pdata *mipi_ILI9487_pdata;
static struct dsi_buf ILI9487_tx_buf;
static struct dsi_buf ILI9487_rx_buf;
spinlock_t ILI9487_spin_lock;

static char ILI9487_command1[] = {
        0xF2, 0x58, 0x10, 0x12, 0X02, 0x92
       };

static char ILI9487_command2[] = {
	0xF7, 0xA9, 0x51, 0x2C, 0x8A
	};
   
static char ILI9487_command3[] = {
	0xFC, 0x00, 0x09
	};

static char ILI9487_command4[] = {
	0x3A, 0x77
	};

static char ILI9487_command5[] = {
	0x36, 0x98
	};	

static char ILI9487_command14[] = {
	0xE9, 0x01
	};
	
static char ILI9487_command15[] = {
	0xB1, 0xA0, 0x19
	};
		
static char ILI9487_command6[] = {
	0xB0, 0x01
	};	

static char ILI9487_command7[] = {
	0xB4, 0x02
	};

static char ILI9487_command8[] = {
	0xB6, 0x00
	};
	
static char ILI9487_command9[] = {
	0xC1, 0x41
	};	

static char ILI9487_command10[] = {
	0xC0, 0x10, 0x10
	};

static char ILI9487_command11[] = {
	0xC5, 0x00, 0x45
	};

static char ILI9487_command12[] = {
	0xE0,0x0F,0x18,0x17,
	0x0C,0x0E,0x06,0x4A,
	0x96,0x3E,0x09,0x15,
	0x08,0x17,0x0D,0x00
	};

static char ILI9487_command13[] = {
	0xE1,0x0F,0x35,0x2E,
	0x07,0x09,0x01,0x45,
	0x53,0x34,0x05,0x0E,
	0x04,0x21,0x21,0x00
};

static char ILI9487_command17[] = {0x11};
static char ILI9487_command18[] = {0x29};
static char ILI9487_command19[] = {0x2C};

static char display_bringtness[] = {
	0x51, 0xff,
};

static char crtl_display[] = {
	0x53, 0x24,
};
static char cabc[2] = {
	0x55, 0x02,
};

static struct dsi_cmd_desc ILI9487_cmd_display_on_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10, sizeof(display_bringtness), display_bringtness},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(crtl_display), crtl_display},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(cabc), cabc},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ILI9487_command1), ILI9487_command1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ILI9487_command2), ILI9487_command2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10, sizeof(ILI9487_command3), ILI9487_command3},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 30, sizeof(ILI9487_command4), ILI9487_command4},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ILI9487_command5), ILI9487_command5},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ILI9487_command6), ILI9487_command6},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ILI9487_command7), ILI9487_command7},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ILI9487_command8), ILI9487_command8},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ILI9487_command9), ILI9487_command9},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 10, sizeof(ILI9487_command10), ILI9487_command10},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ILI9487_command11), ILI9487_command11},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ILI9487_command12), ILI9487_command12},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ILI9487_command13), ILI9487_command13},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ILI9487_command14), ILI9487_command14},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ILI9487_command15), ILI9487_command15},
	{DTYPE_DCS_WRITE, 1, 0, 0, 200, sizeof(ILI9487_command17), ILI9487_command17},
	{DTYPE_DCS_WRITE, 1, 0, 0, 150, sizeof(ILI9487_command18), ILI9487_command18},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(ILI9487_command19), ILI9487_command19},
};

static char display_off[2] = {0x28, 0x00};
static char enter_sleep[2] = {0x10, 0x00};

static struct dsi_cmd_desc ILI9487_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(enter_sleep), enter_sleep}
};

#define MAX_BL_LEVEL 32
#define LCD_BL_EN 96
static void mipi_ILI9487_set_backlight(struct msm_fb_data_type *mfd)
{
	int level = mfd->bl_level;
	int max = mfd->panel_info.bl_max;
	int min = mfd->panel_info.bl_min;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&ILI9487_spin_lock, flags); //disable local irq and preemption
	if (level < min)
		level = min;
	if (level > max)
		level = max;

	if (level == 0) {
		gpio_set_value(LCD_BL_EN, 0);
		spin_unlock_irqrestore(&ILI9487_spin_lock, flags);
		return;
	}

	for (i = 0; i < (MAX_BL_LEVEL - level + 1); i++) {
		gpio_set_value(LCD_BL_EN, 0);
		udelay(1);
		gpio_set_value(LCD_BL_EN, 1);
		udelay(1);
	}
	spin_unlock_irqrestore(&ILI9487_spin_lock, flags);

	return;
}

static int mipi_ILI9487_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
	mipi_set_tx_power_mode(1);
	mipi_dsi_cmds_tx(mfd,&ILI9487_tx_buf,
					 ILI9487_cmd_display_on_cmds,
					 ARRAY_SIZE(ILI9487_cmd_display_on_cmds));
	mipi_set_tx_power_mode(0);

	return 0;
}

static int mipi_ILI9487_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
	mipi_set_tx_power_mode(1);
	mipi_dsi_cmds_tx(mfd,&ILI9487_tx_buf,
		ILI9487_display_off_cmds,
		ARRAY_SIZE(ILI9487_display_off_cmds));
	mipi_set_tx_power_mode(0);

	return 0;
}

static int __devinit mipi_ILI9487_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mipi_ILI9487_pdata = pdev->dev.platform_data;
		return 0;
	}

	spin_lock_init(&ILI9487_spin_lock);
	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_ILI9487_lcd_probe,
	.driver = {
		.name   = "mipi_ILI9487",
	},
};

static struct msm_fb_panel_data ILI9487_panel_data = {
	.on    = mipi_ILI9487_lcd_on,
	.off    = mipi_ILI9487_lcd_off,
	.set_backlight    = mipi_ILI9487_set_backlight,
};

static int ch_used[3];

int mipi_ILI9487_device_register(struct msm_panel_info *pinfo,
                    u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;
	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_ILI9487", (panel << 8)|channel);
	if (!pdev)
	return -ENOMEM;

	ILI9487_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &ILI9487_panel_data, sizeof(ILI9487_panel_data));
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

static int __init mipi_ILI9487_lcd_init(void)
{
	mipi_dsi_buf_alloc(&ILI9487_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&ILI9487_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(mipi_ILI9487_lcd_init);
