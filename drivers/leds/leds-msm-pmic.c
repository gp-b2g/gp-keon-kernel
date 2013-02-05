/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>  //cellon,zhihua,2013-1-4
#define LED_MPP(x)		((x) & 0xFF)
#define LED_CURR(x)		((x) >> 16)

#if 1//cellon,zhihua,start, 2013-1-6, for key backlight
struct pmic_mpp_led_data {
	struct led_classdev cdev;
	int which;
	int type;
	int max;
};
static void pm_mpp_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct pmic_mpp_led_data *led;
	int ret;
       
	//printk(KERN_ERR "zhihua1 pm_mpp_led_set value=%d \n",value);
	led = container_of(led_cdev, struct pmic_mpp_led_data, cdev);

	if (value < LED_OFF || value > led->cdev.max_brightness) {
		dev_err(led->cdev.dev, "Invalid brightness value");
		return;
	}
	if(value > led->max) {
		value = led->max;
	}
    //dev_err(led->cdev.dev, "zhihua1 pm_mpp_led_set\n");
	if(PMIC8029_DRV_TYPE_CUR == led->type) {
		ret = pmic_secure_mpp_config_i_sink(led->which, value,
				value ? PM_MPP__I_SINK__SWITCH_ENA :
					PM_MPP__I_SINK__SWITCH_DIS);

	} else {
		ret = pmic_secure_mpp_control_digital_output(led->which,
			value,
			value ? PM_MPP__DLOGIC_OUT__CTRL_HIGH : PM_MPP__DLOGIC_OUT__CTRL_LOW);
	}
	if (ret)
		dev_err(led_cdev->dev, "can't set mpp led\n");

}

static int pmic_mpp_led_probe(struct platform_device *pdev)
{
	const struct pmic8029_leds_platform_data *pdata = pdev->dev.platform_data;
	struct pmic_mpp_led_data *led, *tmp_led;
	int i, rc;

	//printk("zhihua1 pmic_mpp_led_probe\n");
	if (!pdata) {
		dev_err(&pdev->dev, "platform data not supplied\n");
		return -EINVAL;
	}

	led = kcalloc(pdata->num_leds, sizeof(*led), GFP_KERNEL);
	if (!led) {
		dev_err(&pdev->dev, "failed to alloc memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, led);

	for (i = 0; i < pdata->num_leds; i++) {
		tmp_led	= &led[i];
		tmp_led->cdev.name = pdata->leds[i].name;
		tmp_led->cdev.brightness_set = pm_mpp_led_set;
		tmp_led->cdev.brightness = LED_OFF;
		tmp_led->cdev.max_brightness = LED_FULL;
		tmp_led->which = pdata->leds[i].which;
		tmp_led->type = pdata->leds[i].type;
		if(PMIC8029_DRV_TYPE_CUR == tmp_led->type) {
			tmp_led->max = pdata->leds[i].max.cur;
		} else {
			tmp_led->max = pdata->leds[i].max.vol;
		}

		if (PMIC8029_DRV_TYPE_CUR == tmp_led->type &&
			(tmp_led->max < PM_MPP__I_SINK__LEVEL_5mA ||
			tmp_led->max > PM_MPP__I_SINK__LEVEL_40mA)) {
			dev_err(&pdev->dev, "invalid current\n");
			goto unreg_led_cdev;
		}

		rc = led_classdev_register(&pdev->dev, &tmp_led->cdev);
		if (rc) {
			dev_err(&pdev->dev, "failed to register led\n");
			goto unreg_led_cdev;
		}
	}

	return 0;

unreg_led_cdev:
	while (i)
		led_classdev_unregister(&led[--i].cdev);

	kfree(led);
	return rc;

}

static int __devexit pmic_mpp_led_remove(struct platform_device *pdev)
{
	const struct pmic8029_leds_platform_data *pdata = pdev->dev.platform_data;
	struct pmic_mpp_led_data *led = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < pdata->num_leds; i++)
		led_classdev_unregister(&led[i].cdev);

	kfree(led);

	return 0;
}

static struct platform_driver pmic_mpp_led_driver = {
	.probe		= pmic_mpp_led_probe,
	.remove		= __devexit_p(pmic_mpp_led_remove),
	.driver		= {
		.name	= "pmic-mpp-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init pmic_mpp_led_init(void)
{
	//printk("zhihua1 pmic_mpp_led_init\n");
	return platform_driver_register(&pmic_mpp_led_driver);
}
module_init(pmic_mpp_led_init);

static void __exit pmic_mpp_led_exit(void)
{
	platform_driver_unregister(&pmic_mpp_led_driver);
}
module_exit(pmic_mpp_led_exit);

MODULE_DESCRIPTION("PMIC MPP LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-mpp-leds");
#else
//#define MAX_KEYPAD_BL_LEVEL	16
extern int mpp_config_i_sink(unsigned mpp, unsigned config);

static void msm_keypad_bl_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int ret;
    printk(KERN_ERR "%s zhihua msm_keypad_bl_led_set  value=%d \n",__func__,value);
	dev_err(&pdev->dev, "zhihua msm_keypad_bl_led_set  value=%d \n",value);

#if 1 //cellon,zhihua,2013-1-4,start for key backlight
	if(LED_OFF==value){
		ret=mpp_config_i_sink(PM_MPP_5, 0);
	}
	else{
		ret=mpp_config_i_sink(PM_MPP_5, 4<<16 |1);
	}
#else
	ret = pmic_set_led_intensity(LED_KEYPAD, value / MAX_KEYPAD_BL_LEVEL);
#endif//#if 1 //cellon,zhihua,2013-1-4,end for key backlight

	if (ret)
		dev_err(led_cdev->dev, "can't set keypad backlight\n");
}

static struct led_classdev msm_kp_bl_led = {
	.name			= "keyboard-backlight",
	.brightness_set		= msm_keypad_bl_led_set,
	.brightness		= LED_OFF,
};

static int msm_pmic_led_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &msm_kp_bl_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register led class driver\n");
		return rc;
	}
    printk(KERN_ERR "%s zhihua msm_pmic_led_probe \n",__func__);
	msm_keypad_bl_led_set(&msm_kp_bl_led, LED_OFF);
	return rc;
}

static int __devexit msm_pmic_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&msm_kp_bl_led);

	return 0;
}

#ifdef CONFIG_PM
static int msm_pmic_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&msm_kp_bl_led);

	return 0;
}

static int msm_pmic_led_resume(struct platform_device *dev)
{
	led_classdev_resume(&msm_kp_bl_led);

	return 0;
}
#else
#define msm_pmic_led_suspend NULL
#define msm_pmic_led_resume NULL
#endif

static struct platform_driver msm_pmic_led_driver = {
	.probe		= msm_pmic_led_probe,
	.remove		= __devexit_p(msm_pmic_led_remove),
	.suspend	= msm_pmic_led_suspend,
	.resume		= msm_pmic_led_resume,
	.driver		= {
		.name	= "pmic-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_pmic_led_init(void)
{
    printk(KERN_ERR "%s zhihua msm_pmic_led_init \n",__func__);
	return platform_driver_register(&msm_pmic_led_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	platform_driver_unregister(&msm_pmic_led_driver);
}
module_exit(msm_pmic_led_exit);

MODULE_DESCRIPTION("MSM PMIC LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-leds");
#endif //cellon,zhihua,end, 2013-1-6, for key backlight

