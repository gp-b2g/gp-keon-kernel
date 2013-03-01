/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef MIPI_ILI9487_H
#define MIPI_ILI9487_H

int mipi_ILI9487_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel);

//cellon,Zepeng,modify start,2013-02-25,for LCD white screen
#define LCD_BL_EN 96
//cellon,Zepeng,modify end,2013-02-25,for LCD white screen

#endif  /* MIPI_ILI9487_H */
