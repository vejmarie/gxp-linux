// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2021 Hewlett-Packard Development Company, L.P.
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
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include "gxp-soclib.h"

#define DBG_POST_PORTDATA		0x4
#define DBG_POST_CSR			0x1E

struct gxp_dbg_drvdata {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *base;
	struct mutex mutex;
        int irq;
	unsigned int state;
	unsigned short int postcode;
	unsigned short int previouspostcode;
	unsigned short int initialvalue;
	unsigned short int count;
	dev_t postcodedev;
	struct cdev postcode_c_dev; 
	struct class *postcode_cl; 
};

struct gxp_dbg_drvdata *drvdata=NULL;

static DECLARE_WAIT_QUEUE_HEAD(wq);

static int post_open(struct inode *inode, struct file *file)
{
	unsigned short int value;
	printk("Device open\n");
        value = readw(drvdata->base + DBG_POST_CSR);
        printk(KERN_INFO "DBG: base csr value %02x\n", value);
	// We need to wait for the interrupt to be launched if state
	// is null. or let it go if postcode value is not null after reading it
       	mutex_lock(&drvdata->mutex);
	if ( drvdata->count > 0 )
	{
		mutex_unlock(&drvdata->mutex);
		return -EBUSY;
	}
        drvdata->postcode = readl(drvdata->base + DBG_POST_PORTDATA);
	drvdata->previouspostcode = 0xFF;
	drvdata->initialvalue = 0x00;
	drvdata->count++;
	mutex_unlock(&drvdata->mutex);
	return 0;
}

static int post_release(struct inode *inode, struct file *filp)
{
	mutex_lock(&drvdata->mutex);
	drvdata->count--;
	mutex_unlock(&drvdata->mutex);
	return 0;
}

static int post_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
        printk(KERN_INFO "DBG_POST: seeking next value 0x%02x 0x%02x\n", drvdata->postcode, drvdata->previouspostcode);
	// whatever happens we need to return an initial value even if postcode == 0 and previouspostcode == 0
	// this case is happening when the system is offline
	printk(KERN_INFO "DBG_POST: Wait for postcode != previouspostcode \n");
	wait_event_interruptible(wq, drvdata->postcode != drvdata->previouspostcode);
	drvdata->previouspostcode = drvdata->postcode;
	if (copy_to_user(buf, &drvdata->postcode, 2)) {
        	return -EFAULT;
    	}
        return 1;
}

static const struct file_operations post_fops = {
        .owner          = THIS_MODULE,
        .open           = post_open,
	.read		= post_read,
	.release	= post_release,
};


static ssize_t postcode_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	ssize_t ret;

//	mutex_lock(&drvdata->mutex);

	ret = sprintf(buf, "%d", drvdata->state);

//	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t postcode_enable_store(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
        unsigned int input;
        unsigned short int value;
        int rc;

        rc = kstrtouint(buf, 0, &input);
        if (rc < 0)
                return -EINVAL;
	if (input != 0 && input != 1)
		return -EINVAL;

        mutex_lock(&drvdata->mutex);

	drvdata->state = input;
	if (drvdata->state == 1)
	{
		value = readw(drvdata->base + DBG_POST_CSR);
	        printk(KERN_INFO "DBG: base csr value %02x\n", value);
	        value = value | 0xf;
	        writew( value, drvdata->base + DBG_POST_CSR);
	}

        mutex_unlock(&drvdata->mutex);
        return count;
}

static DEVICE_ATTR_RW(postcode_enable);

static struct attribute *dbg_attrs[] = {
	&dev_attr_postcode_enable.attr,
	NULL,
};
ATTRIBUTE_GROUPS(dbg);

static int sysfs_register(struct device *parent)
{
	struct device *dev;
	printk(KERN_INFO "registering dbg_post into sysfs\n");
	dev = device_create_with_groups(soc_class, parent, 0,
					drvdata, dbg_groups, "dbg");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	drvdata->dev = dev;
	return 0;
}

static irqreturn_t gxp_dbg_post_irq(int irq, void *_drvdata)
{
	unsigned short int value;
//	mutex_lock(&drvdata->mutex);

        value = readl(drvdata->base + DBG_POST_PORTDATA);
	if (drvdata->postcode != value ) {
        	printk(KERN_INFO "DBG_POST: Interrupt update 0x%02x \n", value);
		drvdata->previouspostcode = drvdata->postcode;
		drvdata->postcode = value;
	}
	// update CSR
	value = readw(drvdata->base + DBG_POST_CSR);
	writew( value | 0xc, drvdata->base + DBG_POST_CSR);
//        mutex_unlock(&drvdata->mutex);
	wake_up_interruptible(&wq);
	return IRQ_HANDLED;
}

static int gxp_dbg_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	unsigned short int value;
	struct device *dev_ret;

	printk(KERN_INFO "Initializing dbg_post driver\n");
	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_dbg_post_drvdata),
				GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->pdev = pdev;
	platform_set_drvdata(pdev, drvdata);

	drvdata->state=0;
	drvdata->postcode =  0x00;
	drvdata->previouspostcode = 0xFF;
	drvdata->initialvalue = 0x00;
	drvdata->count = 0x00;

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

	mutex_init(&drvdata->mutex);


	// Let's create the character device for the output

	if ((ret = alloc_chrdev_region(&drvdata->postcodedev, 0, 1, "gxp-dbg-post")) < 0)
	{
	        return ret;
	}
	if (IS_ERR(drvdata->postcode_cl = class_create(THIS_MODULE, "chardrv")))
	{
		unregister_chrdev_region(drvdata->postcodedev, 1);
		return PTR_ERR(drvdata->postcode_cl);
	}
	if (IS_ERR(dev_ret = device_create(drvdata->postcode_cl, NULL, drvdata->postcodedev, NULL, "postcode")))
	{
		class_destroy(drvdata->postcode_cl);
		unregister_chrdev_region(drvdata->postcodedev, 1);
		return PTR_ERR(dev_ret);
	}

	cdev_init(&drvdata->postcode_c_dev, &post_fops);
	if ((ret = cdev_add(&drvdata->postcode_c_dev, drvdata->postcodedev, 1)) < 0)
	{
		device_destroy(drvdata->postcode_cl, drvdata->postcodedev);
		class_destroy(drvdata->postcode_cl);
		unregister_chrdev_region(drvdata->postcodedev, 1);
		return ret;
	}

	// register driver control through sysfs

	return sysfs_register(&pdev->dev);
}

static const struct of_device_id gxp_dbg_of_match[] = {
	{ .compatible = "hpe,gxp-dbg" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_dbg_of_match);

static struct platform_driver gxp_dbg_driver = {
	.probe = gxp_dbg_probe,
	.driver = {
		.name = "gxp-dbg",
		.of_match_table = of_match_ptr(gxp_dbg_of_match),
	},
};
module_platform_driver(gxp_dbg_driver);

MODULE_AUTHOR("Jean-Marie Verdun <jean-marie.verdun@hpe.com>");
MODULE_DESCRIPTION("HPE GXP DBG Driver");
