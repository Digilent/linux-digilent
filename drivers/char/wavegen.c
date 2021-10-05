/*  wavegen.c - Wavegen kernel module.
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/regmap.h>

#define WAVEGEN_GET_CLK_RATE _IOR('q', 1, ulong *)
#define WAVEGEN_SET_CLK_RATE _IOWR('q', 2, ulong *)
#define WAVEGEN_READ_REG _IOWR('q', 3, uint32_t *)
#define WAVEGEN_WRITE_REG _IOWR('q', 4, uint32_t *)

#define REG_OFFSET 0
#define REG_VALUE  1

#define MAX_STR_SIZE 30
#define MAX_DEV 1

static int wavegen_open(struct inode *inode, struct file *file);
static int wavegen_release(struct inode *inode, struct file *file);
static long wavegen_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t wavegen_read(struct file *file, char __user *buf, size_t count, loff_t *offset);
static ssize_t wavegen_write(struct file *file, const char __user *buf, size_t count, loff_t *offset);

static const struct file_operations wavegen_fops = {
    .owner      = THIS_MODULE,
    .open       = wavegen_open,
    .release    = wavegen_release,
    .unlocked_ioctl = wavegen_ioctl,
    .read       = wavegen_read,
    .write       = wavegen_write
};

static const struct regmap_config wavegen_regmap_config = {
    .reg_bits = 32,
    .val_bits = 32,
    .reg_stride = 4,
};

struct wavegen_data {
    struct cdev cdev;
    struct clk *clk;
    struct device *dev;

    struct regmap *regmap;
    void __iomem *base;

};

static int dev_major = 0;
static struct class *wavegen_class = NULL;
static struct wavegen_data wavegen[MAX_DEV];

static uint8_t data[MAX_STR_SIZE] = "Hello from the kernel world!\n";

static int wavegen_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

static int __init wavegen_init(void)
{
    int err, i;
    dev_t dev;
    //platform driver register
    err = alloc_chrdev_region(&dev, 0, MAX_DEV, "wavegen");
    if ( err < 0 ) {
      dev_alert(dev, "wavegen: failed to allocate device region\n");
      return err;
    }
    dev_major = MAJOR(dev);

    wavegen_class = class_create(THIS_MODULE, "wavegen");
    wavegen_class->dev_uevent = wavegen_uevent;

    for (i = 0; i < MAX_DEV; i++) {
        cdev_init(&wavegen[i].cdev, &wavegen_fops);
        wavegen[i].cdev.owner = THIS_MODULE;

        cdev_add(&wavegen[i].cdev, MKDEV(dev_major, i), 1);

        device_create(wavegen_class, NULL, MKDEV(dev_major, i), NULL, "wavegen-%d", i);
    }

    return 0;
}

static void __exit wavegen_exit(void)
{
    int i;

    for (i = 0; i < MAX_DEV; i++) {
        device_destroy(wavegen_class, MKDEV(dev_major, i));
    }

    class_unregister(wavegen_class);
    class_destroy(wavegen_class);

    unregister_chrdev_region(MKDEV(dev_major, 0), MINORMASK);
}

static int wavegen_open(struct inode *inode, struct file *file)
{
    printk("wavegen: Device open\n");
    return 0;
}

static int wavegen_release(struct inode *inode, struct file *file)
{
    printk("wavegen: Device close\n");
    return 0;
}

static long wavegen_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    size_t ncopied;
    int ret;
    ulong target_clk_rate = 0;
    ulong actual_clk_rate = 0;
	uint32_t reg_buffer[2];
    printk("wavegen: Device ioctl\n");
    switch (cmd)
    {
        case WAVEGEN_GET_CLK_RATE:
            target_clk_rate = clk_get_rate(wavegen[0].clk);
            if (copy_to_user((ulong *)arg, &target_clk_rate, sizeof(target_clk_rate)))
            {
                return -EACCES;
            }
            printk("wavegen: GET: Copied Data from the kernel space to user space: %lu [%ld bytes]", target_clk_rate, sizeof(ulong));
            break;
        case WAVEGEN_SET_CLK_RATE:
            ncopied = copy_from_user(&target_clk_rate, (ulong *)arg, sizeof(target_clk_rate));
            if (ncopied == 0) {
                // FIXME?: Currently the xilinx driver for the clocking wizard 
                // returns the same clock rate when clk_round_rate is called (2021.1) for fractionary dividers (clkout[0])
                // in the future it may be appropriate to use the following:
                // actual_clk_rate = clk_round_rate(wavegen[0].clk, target_clk_rate); 
                
                printk("wavegen: SET: Copied Data from the userspace to kernel space: %lu [%ld bytes]", target_clk_rate, sizeof(ulong));
                
                ret = clk_set_rate(wavegen[0].clk, target_clk_rate);  
                if (ret) {
                    printk("wavegen: Failed to set clk rate\n");
                    return ret;
                }        
                actual_clk_rate = clk_get_rate(wavegen[0].clk);
                if (copy_to_user((ulong *)arg, &actual_clk_rate, sizeof(actual_clk_rate)))
                {
                    return -EACCES;
                }      
                printk("wavegen: Target clk rate: %lu, Actual clk rate: %lu\n", target_clk_rate, actual_clk_rate);
            } else {
                printk("wavegen: Could't copy %zd bytes from the userspace\n", ncopied);
                return -EACCES;
            }
            break;
        case WAVEGEN_READ_REG:
			ncopied = copy_from_user(&reg_buffer[REG_OFFSET], (uint32_t *)arg, sizeof(reg_buffer[REG_OFFSET]));
			if (ncopied == 0) {
				ret = regmap_read(wavegen[0].regmap,reg_buffer[REG_OFFSET], &reg_buffer[REG_VALUE]);
				if (ret) {
					dev_err(wavegen[0].dev, "failed to read register: %d\n", ret);
					return ret;
				}
				printk("wavegen: Register found at %x contains %x\n", reg_buffer[REG_OFFSET], reg_buffer[REG_VALUE]);

				if (copy_to_user((uint32_t *)arg, &reg_buffer[REG_VALUE], sizeof(reg_buffer[REG_VALUE])))
				{
					return -EACCES;
				}
				printk("wavegen: GET: Copied Data from the kernel space to user space: %lu [%ld bytes]", reg_buffer[REG_VALUE], sizeof(uint32_t));
			} else {
                printk("wavegen: Could't copy %zd bytes from the userspace\n", ncopied);
                return -EACCES;
            }
            break;
        case WAVEGEN_WRITE_REG:
			ncopied = copy_from_user(&reg_buffer, (uint32_t *)arg, 2 * sizeof(uint32_t));
			if (ncopied == 0) {
				uint32_t reg_copy = 0;
				ret = regmap_read(wavegen[0].regmap, reg_buffer[REG_OFFSET], &reg_copy);
				if (ret) {
					dev_err(wavegen[0].dev, "failed to read register: %d\n", ret);
					return ret;
				}
				printk("wavegen: Register found at %x contained %x\n", reg_buffer[REG_OFFSET], reg_copy);
				
				ret = regmap_write(wavegen[0].regmap, reg_buffer[REG_OFFSET], reg_buffer[REG_VALUE]);
				if (ret) {
					dev_err(wavegen[0].dev, "failed to write register: %d\n", ret);
					return ret;
				}
				printk("wavegen: Register found at %x contains %x\n",reg_buffer[REG_OFFSET], reg_buffer[REG_VALUE]);

				if (copy_to_user((uint32_t *)arg, &reg_buffer, 2 * sizeof(uint32_t)))
				{
					return -EACCES;
				}
			} else {
                printk("wavegen: Could't copy %zd bytes from the userspace\n", ncopied);
                return -EACCES;
            }
            break;
        default:
            return -EINVAL;
    }
    return 0;
}

static ssize_t wavegen_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{

    printk("wavegen: Reading device: %d\n", MINOR(file->f_path.dentry->d_inode->i_rdev));

    if (count > MAX_STR_SIZE) {
        count = MAX_STR_SIZE;
    }

    if (copy_to_user(buf, &data, count)) {
        return -EFAULT;
    }

    return count;
}

static ssize_t wavegen_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    size_t maxdatalen = MAX_STR_SIZE, ncopied;
    uint8_t databuf[maxdatalen];

    printk("wavegen: Writing device: %d\n", MINOR(file->f_path.dentry->d_inode->i_rdev));

    if (count < MAX_STR_SIZE) {
        maxdatalen = count;
    }

    ncopied = copy_from_user(databuf, buf, maxdatalen);

    if (ncopied == 0) {
        printk("wavegen: Copied %zd bytes from the user\n", maxdatalen);
    } else {
        printk("wavegen: Could't copy %zd bytes from the user\n", ncopied);
    }

    databuf[maxdatalen] = 0;

    printk("wavegen: Data from the user: %s\n", databuf);

    return count;
}


static int wavegen_parse_dt(struct wavegen_data* wavegen)
{
	struct device *dev = wavegen->dev;
	int ret;
	wavegen->clk = clk_get(dev, "fs_clk");
	if (IS_ERR(wavegen->clk)) {
		ret = PTR_ERR(wavegen->clk);
		dev_err(dev, "failed to get wavegen clock: %d\n", ret);
		return ret;
	}
    clk_prepare_enable(wavegen->clk);
    printk("wavegen: clk-wiz rate: %lu", clk_get_rate(wavegen->clk));

    return 0;
}

/* Initialize the wavegen device driver module.
 */
static int wavegen_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "wavegen:Platform device initialized!\n");

	struct device *dev = &pdev->dev;
    struct resource *mem;
	int ret;

	if (!(&wavegen[0])) {
		ret = -ENOMEM;
		dev_err(dev, "failed to allocate: %d\n", ret);
		return ret;
	}

    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    wavegen[0].base = devm_ioremap(dev, mem->start, resource_size(mem));
    if (!wavegen[0].base)
		return -ENOMEM;

	wavegen[0].dev = dev;
    
    wavegen[0].regmap = devm_regmap_init_mmio(wavegen[0].dev, wavegen[0].base, &wavegen_regmap_config);
    if (IS_ERR(wavegen[0].regmap)) {
        ret = PTR_ERR(wavegen[0].regmap);
        dev_err(dev, "failed to init regmap: %d\n", ret);
        return ret;
    }

	ret = wavegen_parse_dt(&wavegen[0]);
	if (ret) {
		dev_err(dev, "failed to parse device tree: %d\n", ret);
		return ret;
	}
    platform_set_drvdata(pdev, &wavegen[0]);

    printk("wavegen:Done probing!");
	return 0;
}
 
/* Exit the wavegen device driver module.
 */
static int wavegen_remove(struct platform_device *pdev)
{

	printk(KERN_INFO "wavegen:Platform device exited!\n");

	return 0;
}

static const struct of_device_id wavegen_of_match[] = {
	{ .compatible = "digilent,wavegen"},
	{}
};

static struct platform_driver wavegen_driver = {
	.driver = {
		.name = "wavegen_driver",
		.owner = THIS_MODULE,
		.of_match_table = wavegen_of_match,
	},
	.probe = wavegen_probe,
	.remove = wavegen_remove,
};

MODULE_DEVICE_TABLE(of, wavegen_of_match);
module_platform_driver(wavegen_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eliza Balas <ana-maria.balas@digilent.com>");
MODULE_AUTHOR("Eduard Nita <eduard-marian.nita@digilent.com>");

module_init(wavegen_init);
module_exit(wavegen_exit);
