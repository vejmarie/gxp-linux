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

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/init.h>

#include "gxp-soclib.h"

#define DBG_POST_PORTDATA		0x4
#define DBG_POST_CSR			0x1E

struct gxp_dbg_post_drvdata {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *base;
	struct mutex mutex;
        int irq;
};

static struct class *post_class;

static dev_t first;
static struct cdev c_dev; 
static struct class *cl; 

unsigned int state=0;

static int post_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations post_fops = {
        .owner          = THIS_MODULE,
        .open           = post_open,
};


static ssize_t dbg_post_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_dbg_post_drvdata *drvdata = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&drvdata->mutex);

	ret = sprintf(buf, "%d", state);

	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t dbg_post_store(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
        struct gxp_dbg_post_drvdata *drvdata = dev_get_drvdata(dev);
        unsigned int input;
        unsigned short int value;
        int rc;

        rc = kstrtouint(buf, 0, &input);
        if (rc < 0)
                return -EINVAL;
	if (input != 0 && input != 1)
		return -EINVAL;

        mutex_lock(&drvdata->mutex);

	state = input;
	if (state == 1)
	{
		value = readw(drvdata->base + DBG_POST_CSR);
	        printk(KERN_INFO "DBG: base csr value %02x\n", value);
	        value = value | 0xf;
	        writew( value, drvdata->base + DBG_POST_CSR);
	}

        mutex_unlock(&drvdata->mutex);
        return count;
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
	printk(KERN_INFO "registering dbg_post into sysfs\n");
	dev = device_create_with_groups(soc_class, parent, 0,
					drvdata, dbg_post_groups, "dbg_post");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	drvdata->dev = dev;
	return 0;
}

unsigned short int postvalue=0;
unsigned long  interruptnb=0;

static irqreturn_t gxp_dbg_post_irq(int irq, void *_drvdata)
{
	unsigned short int value;
	struct gxp_dbg_post_drvdata *drvdata = (struct gxp_dbg_post_drvdata *)_drvdata;
	interruptnb++;
	// For the moment let's printk a message
	mutex_lock(&drvdata->mutex);

        value = readl(drvdata->base + DBG_POST_PORTDATA);
	if (postvalue != value ) {
        	printk(KERN_INFO "DBG_POST: Interrupt update 0x%02x \n", value);
		postvalue = value;
	}
	/*
	if (interruptnb % 10000 == 0) {
		printk(KERN_INFO "DBG_POST: Inerrupt number %ld\n", interruptnb);
	}
	*/
	// update CSR
	value = readw(drvdata->base + DBG_POST_CSR);
	writew( value | 0xc, drvdata->base + DBG_POST_CSR);
        mutex_unlock(&drvdata->mutex);
	return IRQ_HANDLED;
}

static int gxp_dbg_post_probe(struct platform_device *pdev)
{
	struct gxp_dbg_post_drvdata *drvdata;
	struct resource *res;
	int ret;
	unsigned short int value;
	int err = -ENODEV;

	printk(KERN_INFO "Initializing dbg_post driver\n");
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
	// let's retrieve the driver irq
        ret = platform_get_irq(pdev, 0);
        if (ret < 0) {
                dev_err(&pdev->dev, "unable to obtain IRQ number\n");
                return ret;
        }
        drvdata->irq = ret;
	// Let's attach to the irq
        ret = devm_request_irq(&pdev->dev,
				drvdata->irq,
				gxp_dbg_post_irq,
				IRQF_SHARED,
				"gxp-dbg-post",
				drvdata);

	// Let's start the irq
	printk(KERN_INFO "DBG: base csr address %04x\n", drvdata->base + DBG_POST_CSR);
	value = readw(drvdata->base + DBG_POST_CSR);
	printk(KERN_INFO "DBG: base csr value %02x\n", value);
	value = value | 0xf;
	writew( value, drvdata->base + DBG_POST_CSR);

	mutex_init(&drvdata->mutex);


	// Let's create the character device for the output
	//

	if ((ret = alloc_chrdev_region(&first, 0, 1, "gxp-dbg-post")) < 0)
	{
	        return ret;
	}
	if (IS_ERR(cl = class_create(THIS_MODULE, "chardrv")))
	{
		unregister_chrdev_region(first, 1);
		return PTR_ERR(cl);
	}
	if (IS_ERR(dev_ret = device_create(cl, NULL, first, NULL, "postcode")))
	{
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return PTR_ERR(dev_ret);
	}

	cdev_init(&c_dev, &post_fops);
	if ((ret = cdev_add(&c_dev, first, 1)) < 0)
	{
		device_destroy(cl, first);
		class_destroy(cl);
		unregister_chrdev_region(first, 1);
		return ret;
	}

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
