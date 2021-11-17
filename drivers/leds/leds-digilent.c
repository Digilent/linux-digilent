/*  pwm-rgb-led.c - The simplest kernel module.

* Copyright (C) 2013 - 2016 Xilinx, Inc
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License along
*   with this program. If not, see <http://www.gnu.org/licenses/>.

*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include<linux/module.h>
#include<linux/moduleparam.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR
    ("Digilent Inc.");
MODULE_DESCRIPTION
    ("prl(pwm-rgb-led) - module to control up to 8 pwm rgb leds");
//prl = PWM RGB LED
#define DRIVER_NAME "prl"
#define DEVICE_NAME "prl"
#define CLASS_NAME "prl"

//Pwm Rgb Led device data
#define PRL_MAX_MINORS 8
#define PRL_COLOR_SIZE 9

static u8 prl_count;
struct prl_device_data {
	dev_t devno;
	struct cdev cdev;
	struct device *dev;
	u32 *reg; // addres of the register to write color to
	char color_str[PRL_COLOR_SIZE];
	struct semaphore sem;
};

static dev_t prl_devno;
static struct prl_device_data prl_cdev[PRL_MAX_MINORS];
static struct class *prl_class;

static int prl_open(struct inode *, struct file *);
static int prl_release(struct inode *, struct file *);
static ssize_t prl_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t prl_write(struct file *, const char __user *, size_t, loff_t *);

static struct file_operations prl_fops =
{
	.owner = THIS_MODULE,
	.open = prl_open,
	.read = prl_read,
	.write = prl_write,
	.release = prl_release,
};

struct prl_local {
	unsigned long mem_start;
	unsigned long mem_end;
	void __iomem *base_addr;
};


static int prl_open(struct inode *inodep, struct file *filep){

	struct prl_device_data *dev;
	dev = container_of(inodep->i_cdev, struct prl_device_data, cdev);
	filep->private_data = dev;
	return 0;
};

static int prl_release(struct inode *inodep, struct file *filep){
	printk(KERN_INFO "Led Device successfully closed\n");
	return 0;
}

static ssize_t prl_read(struct file *filep, char __user *buffer, size_t size, loff_t *offset){
	printk(KERN_INFO "Led Device nothing to send\n");
	return 0;
};

static ssize_t prl_write(struct file *filep, const char __user *buffer, size_t size, loff_t *offset) {
	long int color;
	int ret;
	struct prl_device_data *dev = filep->private_data;
	if (down_interruptible(&dev->sem))
		return -ERESTARTSYS;

	ret = copy_from_user(dev->color_str, buffer,PRL_COLOR_SIZE); //copy maximum first color
	if(ret == 0 ) {
		printk(KERN_INFO "Led Device %zu characters from user\n",strlen(dev->color_str));
		ret = kstrtol(dev->color_str, 16, &color);
		if(ret == 0) {
			printk(KERN_INFO "Led color: 0x%08x",color );
			*(dev->reg) = color;
		} else {
			printk(KERN_INFO "Led Device failed to convert input:%s\n",strlen(dev->color_str));
		}
	} else {
		printk(KERN_ALERT "Led Device failed to copy %zu characters from user\n",ret);
		goto out;
	}
	ret = size;
out:
	up(&dev->sem);
	return ret;
}

static int prl_probe(struct platform_device *pdev)
{
	//struct resource *r_irq; /* Interrupt resources */
	struct resource *r_mem; /* IO mem resources */
	struct device *dev = &pdev->dev;
	struct prl_local *lp = NULL;
	dev_t curr_dev;

	int rc = 0;
	int i, err;
	dev_info(dev, "Device Tree Probing\n");
	/* Get iospace for the device */
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		dev_err(dev, "invalid address\n");
		return -ENODEV;
	}
	lp = (struct prl_local *) kmalloc(sizeof(struct prl_local), GFP_KERNEL);
	if (!lp) {
		dev_err(dev, "Cound not allocate pwm-rgb-led device\n");
		return -ENOMEM;
	}
	dev_set_drvdata(dev, lp);
	lp->mem_start = r_mem->start;
	lp->mem_end = r_mem->end;

	if (!request_mem_region(lp->mem_start,
				lp->mem_end - lp->mem_start + 1,
				DRIVER_NAME)) {
		dev_err(dev, "Couldn't lock memory region at %p\n",
			(void *)lp->mem_start);
		rc = -EBUSY;
		goto error1;
	}

	lp->base_addr = ioremap(lp->mem_start, lp->mem_end - lp->mem_start + 1);
	if (!lp->base_addr) {
		dev_err(dev, "pwm-rgb-led: Could not allocate iomem\n");
		rc = -EIO;
		goto error2;
	}

	dev_info(dev,"pwm-rgb-led at 0x%08x mapped to 0x%08x",
		(unsigned int __force)lp->mem_start,
		(unsigned int __force)lp->base_addr);

	i = 0;
	prl_cdev[i].reg = (u32 *) lp->base_addr;
	prl_count = (*(prl_cdev[i].reg))>>24;
	dev_info(dev,"count: 0x%02x ", prl_count);
	for ( i = 1; i < prl_count; i++ ) {
		prl_cdev[i].reg = prl_cdev[i - 1].reg + 1;
	}

	err = alloc_chrdev_region( &prl_devno, 0, prl_count, "prl_driver");
	if ( err < 0 ) {
		dev_alert(dev, "Led: failed to allocate device region\n");
		goto error2;
	}

	prl_class = class_create(THIS_MODULE, "prl_class");
	if ( prl_class == NULL) {
		dev_alert(dev, "Led: failed to create device class\n");
		goto r_class;
	}

	dev_info(dev, "Led: device major: %d\n",MAJOR(prl_devno));

	for ( i = 0 ; i < prl_count; i++ ) {
		cdev_init(&prl_cdev[i].cdev, &prl_fops);
		prl_cdev[i].devno = MKDEV(MAJOR(prl_devno), MINOR(prl_devno) + i);
		prl_cdev[i].dev = device_create(prl_class, NULL, prl_cdev[i].devno, NULL, "led%d", i);
		if (IS_ERR(prl_cdev[i].dev)){
			rc = ERR_PTR(prl_cdev[i].dev);
			dev_alert(dev, "Led: chrdev create error 0x%08x\n",rc);
			goto r_device;
		}
		rc = cdev_add(&prl_cdev[i].cdev, prl_cdev[i].devno, 1);
		if (rc < 0 ){
			dev_alert(dev, "Led: cdev add error %d\n",i);
			goto r_device;
		}
		sema_init(&prl_cdev[i].sem,1);
		dev_info(dev, "Led %d: cdev add success\n",i);
	}

	return 0;

r_device:
	for(; i >= 0; i--){
		device_destroy(prl_class, prl_cdev[i].devno);
		cdev_del(&prl_cdev[i].cdev);
	}
	class_unregister(prl_class);
	class_destroy(prl_class);
r_class:
	unregister_chrdev_region(prl_devno, prl_count);
error2:
	release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
error1:
	kfree(lp);
	dev_set_drvdata(dev, NULL);
	return rc;
}

static int prl_remove(struct platform_device *pdev)
{
	int i;
	for( i = 0; i < prl_count; i++ ) {
		device_destroy(prl_class, prl_cdev[i].devno);
		cdev_del(&prl_cdev[i].cdev);
	}
	class_unregister(prl_class);
	class_destroy(prl_class);
	unregister_chrdev_region(prl_devno, prl_count);

	struct device *dev = &pdev->dev;
	struct prl_local *lp = dev_get_drvdata(dev);
	iounmap(lp->base_addr);
	release_mem_region(lp->mem_start, lp->mem_end - lp->mem_start + 1);
	kfree(lp);
	dev_set_drvdata(dev, NULL);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id prl_of_match[] = {
	{ .compatible = "digilent,prl", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, prl_of_match);
#else
# define prl_of_match
#endif

static struct platform_driver prl_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= prl_of_match,
	},
	.probe		= prl_probe,
	.remove		= prl_remove,
};

static int __init prl_init(void)
{
	printk("Digilent pwm rgb led module loaded.\n");
	return platform_driver_register(&prl_driver);
}

static void __exit prl_exit(void)
{
	platform_driver_unregister(&prl_driver);
	printk(KERN_ALERT "Digilent pwm rgb led module unloaded.\n");
}

module_init(prl_init);
module_exit(prl_exit);
