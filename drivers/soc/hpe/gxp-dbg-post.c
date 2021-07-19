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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "gxp-soclib.h"

#define DBG_POST_PORTDATA		0x4

struct gxp_dbg_post_drvdata {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *base;
	struct mutex mutex;
};

static ssize_t dbg_post_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_dbg_post_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned short int value;
	ssize_t ret;

	mutex_lock(&drvdata->mutex);

	value = readl(drvdata->base + DBG_POST_PORTDATA);
	ret = sprintf(buf, "0x%02x", value);

	mutex_unlock(&drvdata->mutex);
	return ret;
}

static DEVICE_ATTR_RW(dbg_post);

static struct attribute *dbg_post_attrs[] = {
	&dev_attr_dbg_post.attr,
	NULL,
};
ATTRIBUTE_GROUPS(dbg_post);

static int sysfs_register(struct device *parent,
			struct gxp_dbg_post_drvdata *drvdata)
{
	struct device *dev;

	dev = device_create_with_groups(soc_class, parent, 0,
					drvdata, dbg_post_groups, "dbg_post");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	drvdata->dev = dev;
	return 0;
}

static int gxp_dbg_post_probe(struct platform_device *pdev)
{
	struct gxp_dbg_post_drvdata *drvdata;
	struct resource *res;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_dbg_post_drvdata),
				GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->pdev = pdev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	mutex_init(&drvdata->mutex);

	return sysfs_register(&pdev->dev, drvdata);
}

static const struct of_device_id gxp_dbg_post_of_match[] = {
	{ .compatible = "hpe,gxp-dbg-post" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_dbg_post_of_match);

static struct platform_driver gxp_dbg_post_driver = {
	.probe = gxp_dbg_post_probe,
	.driver = {
		.name = "gxp-dbg-post",
		.of_match_table = of_match_ptr(gxp_dbg_post_of_match),
	},
};
module_platform_driver(gxp_dbg_post_driver);

MODULE_AUTHOR("Jean-Marie Verdun <jean-marie.verdun@hpe.com>");
MODULE_DESCRIPTION("HPE GXP DBG POST Driver");
