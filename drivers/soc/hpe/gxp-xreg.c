// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "gxp-soclib.h"

#define XREG_SERVER_ID_L	0x01
#define XREG_SERVER_ID_H	0x02
#define XREG_SPEC_VER		0x03
#define XREG_IOP_LEDS		0x04
#define XREG_SIDEBAND_SEL	0x40

struct gxp_xreg_drvdata {
	void __iomem *base;
	struct mutex mutex;
};

static ssize_t server_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + XREG_SERVER_ID_H);
	value = value<<8 | readb(xreg->base + XREG_SERVER_ID_L);
	ret = sprintf(buf, "0x%04x", value);

	mutex_unlock(&xreg->mutex);
	return ret;
}
static DEVICE_ATTR_RO(server_id);

static ssize_t spec_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + XREG_SPEC_VER);
	ret = sprintf(buf, "0x%02x", value);

	mutex_unlock(&xreg->mutex);
	return ret;
}
static DEVICE_ATTR_RO(spec_version);


static ssize_t iop_leds_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + XREG_IOP_LEDS);
	ret = sprintf(buf, "0x%02x", value);

	mutex_unlock(&xreg->mutex);
	return ret;
}

static ssize_t iop_leds_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &value);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&xreg->mutex);

	writeb(value & 0xff, xreg->base + XREG_IOP_LEDS);

	mutex_unlock(&xreg->mutex);
	return count;

}
static DEVICE_ATTR_RW(iop_leds);

static ssize_t uid_mux_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x05);
	switch ((value&0xc0)>>6) {
	case 0x00:
		ret = sprintf(buf, "0x00,not in control");
		break;
	case 0x02:
		ret = sprintf(buf, "0x01,off");
		break;
	case 0x03:
		ret = sprintf(buf, "0x03,on");
		break;
	case 0x01:
	default:
		ret = sprintf(buf, "0x01,reserved");
		break;
	}

	mutex_unlock(&xreg->mutex);
	return ret;
}

static ssize_t uid_mux_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int input;
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&xreg->mutex);
	value =  readb(xreg->base + 0x05);

	switch (input) {
	case 0x00:
	case 0x02:
	case 0x03:
		value = (value & ~0xc0) | (input << 6);
		break;
	case 0x01:
	default:
		return -EINVAL;
	}

	writeb(value & 0xff, xreg->base + 0x05);

	mutex_unlock(&xreg->mutex);
	return count;
}
static DEVICE_ATTR_RW(uid_mux);

static ssize_t uid_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x05);
	ret = sprintf(buf, "%s", (value&0x02) ? "0x01,Host driving" :
						"0x00,Not driven by host");

	mutex_unlock(&xreg->mutex);
	return ret;
}
static DEVICE_ATTR_RO(uid_state);

static ssize_t uid_blink_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x05);
	switch ((value&0x38)>>3) {
	case 0x04:
		ret = sprintf(buf, "0x04,blink 1Hz");
		break;
	case 0x03:
		ret = sprintf(buf, "0x03,blink USB Done");
		break;
	case 0x01:
		ret = sprintf(buf, "0x01,blink USB Busy");
		break;
	case 0x02:
		ret = sprintf(buf, "0x02,blink USB Error");
		break;
	default:
		ret = sprintf(buf, "0x%02x,undefined", (value&0x38)>>3);
		break;
	}

	mutex_unlock(&xreg->mutex);
	return ret;
}

static ssize_t uid_blink_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int input;
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&xreg->mutex);
	value =  readb(xreg->base + 0x05);

	switch (input) {
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x04:
		value = (value & ~0x38) | (input << 3);
		break;
	default:
		return -EINVAL;
	}

	writeb(value & 0xff, xreg->base + 0x05);

	mutex_unlock(&xreg->mutex);
	return count;
}
static DEVICE_ATTR_RW(uid_blink);

static ssize_t health_led_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x0d);

	switch ((value&0x30)>>4) {
	case 0x00:
		ret = sprintf(buf, "off");
		break;
	case 0x01:
		ret = sprintf(buf, "green");
		break;
	case 0x02:
		ret = sprintf(buf, "red");
		break;
	case 0x03:
		ret = sprintf(buf, "amber");
		break;
	default:
		ret = sprintf(buf, "unknown");
		break;
	}

	mutex_unlock(&xreg->mutex);
	return ret;
}
static DEVICE_ATTR_RO(health_led);


static ssize_t health_led_red_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x0D);
	if (value&0x80)
		ret = sprintf(buf, "1");
	else
		ret = sprintf(buf, "0");

	mutex_unlock(&xreg->mutex);
	return ret;
}

static ssize_t health_led_red_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int input;
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&xreg->mutex);

	value =  readb(xreg->base + 0x0D);
	value = (input != 0)?(value | 0x80):(value & ~0x80);
	writeb(value & 0xff, xreg->base + 0x0D);

	mutex_unlock(&xreg->mutex);
	return count;
}
static DEVICE_ATTR_RW(health_led_red);

static ssize_t health_led_amber_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x0D);
	if (value&0x40)
		ret = sprintf(buf, "1");
	else
		ret = sprintf(buf, "0");

	mutex_unlock(&xreg->mutex);
	return ret;
}

static ssize_t health_led_amber_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int input;
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&xreg->mutex);

	value =  readb(xreg->base + 0x0D);
	value = (input != 0)?(value | 0x40):(value & ~0x40);
	writeb(value & 0xff, xreg->base + 0x0D);

	mutex_unlock(&xreg->mutex);
	return count;
}
static DEVICE_ATTR_RW(health_led_amber);

static ssize_t sideband_sel_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x40);
	ret = sprintf(buf, "0x%02x", value&0x03);

	mutex_unlock(&xreg->mutex);
	return ret;
}

static ssize_t sideband_sel_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int input;
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	if (input < 0 || input > 3)
		return -EINVAL;

	mutex_lock(&xreg->mutex);
	value =  readb(xreg->base + 0x40);
	value = (value & ~0x03) | (input);

	writeb(value & 0xff, xreg->base + 0x40);

	mutex_unlock(&xreg->mutex);
	return count;

}
static DEVICE_ATTR_RW(sideband_sel);

static ssize_t ps_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x18);
	ret = sprintf(buf, "0x%02x", value);

	mutex_unlock(&xreg->mutex);
	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int input;
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	if (input < 0 || input > 256)
		return -EINVAL;

	mutex_lock(&xreg->mutex);

	writeb(value & 0xff, xreg->base + 0x18);

	mutex_unlock(&xreg->mutex);
	return count;

}
static DEVICE_ATTR_RW(ps_enable);

static ssize_t ps_inst_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x19);
	ret = sprintf(buf, "0x%02x", value);

	mutex_unlock(&xreg->mutex);
	return ret;
}
static DEVICE_ATTR_RO(ps_inst);

static ssize_t dedicated_inst_n_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x3a);
	ret = sprintf(buf, "%d", (value&0x20)?1:0);

	mutex_unlock(&xreg->mutex);
	return ret;
}
static DEVICE_ATTR_RO(dedicated_inst_n);

static ssize_t alom_inst_n_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x3a);
	ret = sprintf(buf, "%d", (value&0x80)?1:0);

	mutex_unlock(&xreg->mutex);
	return ret;
}
static DEVICE_ATTR_RO(alom_inst_n);

static ssize_t lom_inst_n_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x3a);
	ret = sprintf(buf, "%d", (value&0x40)?1:0);

	mutex_unlock(&xreg->mutex);
	return ret;
}
static DEVICE_ATTR_RO(lom_inst_n);

static ssize_t fw_boot_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int input;
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&xreg->mutex);

	value =  readb(xreg->base + 0x0B);
	value = (input != 0)?(value | 0x80):(value & ~0x80);
	writeb(value & 0xff, xreg->base + 0x0B);

	mutex_unlock(&xreg->mutex);
	return count;
}
static DEVICE_ATTR_WO(fw_boot);

static ssize_t fw_managed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int input;
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&xreg->mutex);

	value =  readb(xreg->base + 0x0B);
	value = (input != 0)?(value | 0x40):(value & ~0x40);
	writeb(value & 0xff, xreg->base + 0x0B);

	mutex_unlock(&xreg->mutex);
	return count;
}
static DEVICE_ATTR_WO(fw_managed);

static ssize_t pwr_on_mask_n_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = readb(xreg->base + 0x06);
	ret = sprintf(buf, "%d", (value&0x04)?1:0);

	mutex_unlock(&xreg->mutex);
	return ret;
}

static ssize_t pwr_on_mask_n_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int input;
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&xreg->mutex);

	value =  readb(xreg->base + 0x06);
	value = (input != 0)?(value | 0x04):(value & ~0x04);
	writeb(value & 0xff, xreg->base + 0x06);

	mutex_unlock(&xreg->mutex);
	return count;

}
static DEVICE_ATTR_RW(pwr_on_mask_n);

static ssize_t fan_inst_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = 0;
	value = readb(xreg->base + 0x27);
	value |= readb(xreg->base + 0x28) << 8;

	ret = sprintf(buf, "0x%04x", value);

	mutex_unlock(&xreg->mutex);
	return ret;
}
static DEVICE_ATTR_RO(fan_inst);

static ssize_t fan_fail_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *xreg = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	mutex_lock(&xreg->mutex);

	value = 0;
	value = readb(xreg->base + 0x29);
	value |= readb(xreg->base + 0x2a) << 8;

	ret = sprintf(buf, "0x%04x", value);

	mutex_unlock(&xreg->mutex);
	return ret;
}
static DEVICE_ATTR_RO(fan_fail);

static struct attribute *xreg_attrs[] = {
	&dev_attr_server_id.attr,
	&dev_attr_spec_version.attr,
	&dev_attr_iop_leds.attr,
	&dev_attr_uid_mux.attr,
	&dev_attr_uid_state.attr,
	&dev_attr_uid_blink.attr,
	&dev_attr_health_led.attr,
	&dev_attr_health_led_red.attr,
	&dev_attr_health_led_amber.attr,
	&dev_attr_sideband_sel.attr,
	&dev_attr_ps_enable.attr,
	&dev_attr_ps_inst.attr,
	&dev_attr_alom_inst_n.attr,
	&dev_attr_lom_inst_n.attr,
	&dev_attr_dedicated_inst_n.attr,
	&dev_attr_fw_boot.attr,
	&dev_attr_fw_managed.attr,
	&dev_attr_pwr_on_mask_n.attr,
	&dev_attr_fan_inst.attr,
	&dev_attr_fan_fail.attr,
	NULL,
};
ATTRIBUTE_GROUPS(xreg);

static int sysfs_register(struct device *parent, struct gxp_xreg_drvdata *xreg)
{
	struct device *dev;

	dev = device_create_with_groups(soc_class, parent, 0,
					xreg, xreg_groups, "xreg");
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	return 0;
}

static const struct of_device_id gxp_xreg_of_match[] = {
	{ .compatible = "hpe,gxp-xreg" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_xreg_of_match);

static int gxp_xreg_probe(struct platform_device *pdev)
{

	struct gxp_xreg_drvdata *drvdata;
	struct resource *res;

	drvdata = devm_kzalloc(&pdev->dev,
				sizeof(struct gxp_xreg_drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	mutex_init(&drvdata->mutex);
	sysfs_register(&pdev->dev, drvdata);

	return 0;
}

static struct platform_driver gxp_xreg_driver = {
	.probe = gxp_xreg_probe,
	.driver = {
		.name = "gxp-xreg",
		.of_match_table = of_match_ptr(gxp_xreg_of_match),
	},
};
module_platform_driver(gxp_xreg_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("HPE GXP Xreg Driver");
