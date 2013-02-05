/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#ifndef S5K5CAG_H
#define S5K5CAG_H

#include <mach/camera.h>
#include <linux/types.h>

//extern struct mt9t013_reg mt9t013_regs; /* from mt9t013_reg.c */

typedef enum {
  LED_MODE_OFF,
  LED_MODE_AUTO,
  LED_MODE_ON,
  LED_MODE_TORCH,

  /*new mode above should be added above this line*/
  LED_MODE_MAX
} led_mode_t;

struct s5k5cag_sensor_cfg_data {
    struct sensor_cfg_data cfg;
    led_mode_t led_mode;
};

enum s5k5cag_width {
        WORD_LEN,
        BYTE_LEN
};

struct s5k5cag_i2c_reg_conf {
        unsigned short waddr;
        unsigned short wdata;
        enum s5k5cag_width width;
        unsigned short mdelay_time;
};

enum s5k5cag_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum s5k5cag_setting {
	RES_PREVIEW,
	RES_CAPTURE
};

enum s5k5cag_reg_update {
	/* Sensor registers that need to be updated during initialization */
	REG_INIT,
	/* Sensor registers that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

struct s5k5cag_reg {
        const struct s5k5cag_i2c_reg_conf const *config0;
        uint16_t config0_size;
        const struct s5k5cag_i2c_reg_conf const *config1;
        uint16_t config1_size;
        const struct s5k5cag_i2c_reg_conf const *config2;
        uint16_t config2_size;
        const struct s5k5cag_i2c_reg_conf const *config3;
        uint16_t config3_size;
        const struct s5k5cag_i2c_reg_conf const *config4;
        uint16_t config4_size;
        const struct s5k5cag_i2c_reg_conf const *config5;
        uint16_t config5_size;
        const struct s5k5cag_i2c_reg_conf const *config6;
        uint16_t config6_size;
        const struct s5k5cag_i2c_reg_conf const *config7;
        uint16_t config7_size;
        const struct s5k5cag_i2c_reg_conf const *config8;
        uint16_t config8_size;
        const struct s5k5cag_i2c_reg_conf const *config9;
        uint16_t config9_size;
        const struct s5k5cag_i2c_reg_conf const *config10;
        uint16_t config10_size;
        const struct s5k5cag_i2c_reg_conf const *config11;
        uint16_t config11_size;
        const struct s5k5cag_i2c_reg_conf const *config_capture;
        uint16_t config_capture_size;
        const struct s5k5cag_i2c_reg_conf const *config_preview;
        uint16_t config_preview_size;
        const struct s5k5cag_i2c_reg_conf const *config_end;
        uint16_t config_end_size;
};

extern int msm_camera_flash_current_driver(struct msm_camera_sensor_flash_current_driver *current_driver,unsigned led_state);
#endif /* #define S5K5CAG_H */
