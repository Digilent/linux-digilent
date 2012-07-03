#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <asm/uaccess.h>

#define DRIVER_NAME "pmodoled"
#define DISPLAY_BUF_SZ	512 /* 32 x 128 bit monochrome  == 512 bytes        */
#define MAX_LINE_LEN	16 /* 128 bits wide and current char width is 8 bit */
#define MAX_ROW			4
#define OLED_MAX_PG_CNT      4 /* number of display pages in OLED controller */
#define OLED_CONTROLLER_PG_SZ	128
#define OLED_CONTROLLER_CMD	0
#define OLED_CONTROLLER_DATA	1

/* commands for the OLED display controller	*/
#define OLED_SET_PG_ADDR		0x22
#define OLED_DISPLAY_OFF		0xAE
#define OLED_DISPLAY_ON			0xAF
#define OLED_CONTRAST_CTRL		0x81
#define OLED_SET_PRECHARGE_PERIOD	0xD9
#define OLED_SET_SEGMENT_REMAP		0xA1
#define OLED_SET_COM_DIR			0xC8
#define OLED_SET_COM_PINS		0xDA

dev_t pmodoled_dev_id = 0;
static unsigned int device_num = 0;
static unsigned int cur_minor = 0;
struct mutex minor_mutex;
static struct class *pmodoled_class = NULL;

struct pmodoled_device {
	char *name;
	/* R/W Mutex Lock */
	struct mutex mutex;
	/* Display Buffers */
	uint8_t disp_on;
	uint8_t *disp_buf;
	/* Pin Assignment */
	unsigned long iVBAT;
	unsigned long iVDD;
	unsigned long iRES;
	unsigned long iDC;
	unsigned long iCLK;
	unsigned long iMOSI;
	unsigned long iCS;
	/* SPI Info */
	uint32_t spi_id;
	/* platform device structures */
	struct platform_device *pdev;
	/* Char Device */
	struct cdev cdev;
	struct spi_device *spi;
	dev_t dev_id;
};

/**
 * screen_buf_to_display - 
 * @screen_buf -
 * @dev -
 *
 */
static int screen_buf_to_display(uint8_t *screen_buf, struct pmodoled_device *dev) 
{
	uint32_t pg;
	int status;
	uint8_t lower_start_column = 0x00;
	uint8_t upper_start_column = 0x10;
	uint8_t wr_buf[10];

	for(pg = 0; pg < OLED_MAX_PG_CNT; pg++) {
		wr_buf[0] = OLED_SET_PG_ADDR;
		wr_buf[1] = pg;
		wr_buf[2] = lower_start_column;
		wr_buf[3] = upper_start_column;
		gpio_set_value(dev->iDC, OLED_CONTROLLER_CMD);
		status = spi_write(dev->spi, wr_buf, 4);
		if(status) {
			printk(KERN_INFO DRIVER_NAME "screen_buf_to_display: Error writing to SPI\n");
			break;
		}
		
		gpio_set_value(dev->iDC, OLED_CONTROLLER_DATA);
		status = spi_write(dev->spi, (uint8_t *) (screen_buf + 
						(pg*OLED_CONTROLLER_PG_SZ)), OLED_CONTROLLER_PG_SZ);
		if(status) {
			printk(KERN_INFO DRIVER_NAME "screen_buf_to_display: Error writing to SPI\n");
			break;
		}
	}
	return status;
}

/**
 * A basic open function. It exists mainly to save the id of
 * the OLED and some other basic information.
 */
static int 	pmodoled_open(struct inode *inode, struct file *fp) 
{
	struct pmodoled_device *dev;

	dev = container_of(inode->i_cdev, struct pmodoled_device, cdev);
	fp->private_data = dev;

	return 0;
}

static int 	pmodoled_close(struct inode *inode, struct file *fp)
{
	return 0;
}

/**
 * Driver write function
 *
 * This function uses a generic SPI write to send values to the Pmod device
 * It takes a raw data array from the app in the buffer, copied it into 
 * device dispay buffer, and finally sends the buffer to the OLED using SPI
 */
static ssize_t	pmodoled_write(struct file *fp, const char __user *buffer, size_t length, loff_t *offset) 
{
	ssize_t retval = 0;
	struct pmodoled_device *dev;
	unsigned int minor_id;
	int cnt;
	int status;

	dev = fp->private_data;
	minor_id = MINOR(dev->dev_id);

	if(mutex_lock_interruptible(&dev->mutex)) {
		retval = -ERESTARTSYS;
		goto write_lock_err;
	}

	if (buffer == NULL) {
		printk(KERN_ALERT "oled_write: ERROR: invalid buffer address: 0x%08x\n",
					(unsigned int) buffer);
		retval = -EINVAL;
		goto quit_write;
	}

	if(length > DISPLAY_BUF_SZ) {
		cnt = DISPLAY_BUF_SZ;
	}
	else {
		cnt = length;
	}

	printk(KERN_INFO "oled_write: data length %d\n", cnt);
	if(copy_from_user(dev->disp_buf, buffer, cnt)) {
		printk(KERN_ALERT "oled_write: copy_from_user failed\n");
		retval = -EFAULT;
		goto quit_write;
	}
	else {
		retval = cnt;
	}

	status = screen_buf_to_display(dev->disp_buf, dev);
	if(status) {
		printk(KERN_ALERT "oled_write: Error sending string to display\n");
		retval = -EFAULT;
		goto quit_write;
	}
	
quit_write:
	mutex_unlock(&dev->mutex);
write_lock_err:
	return retval;
}

/**
 * Driver Read Function
 *
 * This function does not actually read the Pmod as it is a write-only device. Instead 
 * It returns data in the buffer generated for the display that was used when the OLED
 * was last programmed.
 */
static ssize_t	pmodoled_read(struct file *fp, char __user *buffer, size_t length, loff_t *offset)
{
	ssize_t retval = 0;
	struct pmodoled_device *dev;
	unsigned int minor_id;
	int cnt;
	
	dev = fp->private_data;
	minor_id = MINOR(dev->dev_id);

	if(mutex_lock_interruptible(&dev->mutex)) {
		retval = -ERESTARTSYS;
		goto read_lock_err;
	}

	if (buffer == NULL) {
		dev_err(&dev->spi->dev, "OLED_read: ERROR: invalid buffer "
				"address: 0x%08X\n", (unsigned int)buffer);
		retval = -EINVAL;
		goto quit_read;
	}

	if (length > DISPLAY_BUF_SZ)
		cnt = DISPLAY_BUF_SZ;
 	else
		cnt = length;
	retval = copy_to_user((void *)buffer, dev->disp_buf, cnt);
	if (!retval)
		retval = cnt; /* copy success, return amount in buffer */

quit_read:
	mutex_unlock(&dev->mutex);
read_lock_err:
	return(retval);
}

struct file_operations pmodoled_cdev_fops = {
	.owner = THIS_MODULE,
	.write = pmodoled_write,
	.read = pmodoled_read,
	.open = pmodoled_open,
	.release = pmodoled_close,
};

static const struct of_device_id pmodoled_of_match[] = {
	{ .compatible = "dglnt,pmodoled-1.00.a", },
	{},
};

static int __init add_pmodoled_device_to_bus(struct pmodoled_device* dev) {
	struct spi_master *spi_master;
	struct spi_device *spi_device;
	int status = 0;

	spi_master = spi_busnum_to_master(dev->spi_id);
	if(!spi_master) {
		printk(KERN_ALERT "spi_busnum_to_master(%d) returned NULL\n", dev->spi_id);
		return -ENOSYS;
	}

	spi_device = spi_alloc_device(spi_master);
	if(!spi_device) {
		put_device(&spi_master->dev);
		printk(KERN_ALERT "spi_alloc_device() failed\n");
		return -ENOMEM;
	}

	spi_device->chip_select = 0;
	spi_device->max_speed_hz = 4000000;
	spi_device->mode = SPI_MODE_0;
	spi_device->bits_per_word = 8;
	spi_device->controller_data = (void *) dev->iCS;
	spi_device->dev.platform_data = dev;
	strlcpy(spi_device->modalias, DRIVER_NAME, sizeof(DRIVER_NAME));

	status = spi_add_device(spi_device);
	if(status < 0) {
		spi_dev_put(spi_device);
		printk(KERN_ALERT "spi_add_device() failed %d\n", status);
		return status;
	}
	dev->spi = spi_device;

	put_device(&spi_master->dev);

	return status;
}

/** 
 * pmodoled_of_probe - Probe method for ZED on-board OLED device.
 * @np: pointer to device tree node
 *
 * This function probes the OLED device in the device tree. It initializes the
 * OLED driver data structure. It returns 0, if the driver is bound to the OLED
 * device, or a negative value if there is an error.
 */
static int __init pmodoled_of_probe(struct device_node *np)
{
	struct pmodoled_device *pmodoled_dev;
	struct platform_device *pmodoled_pdev;
	struct spi_gpio_platform_data *pmodoled_pdata;
	const u32* tree_info;
	int status = 0;

	/* Alloc Space for platform device structure */
	pmodoled_dev = (struct pmodoled_device*) kzalloc(sizeof(*pmodoled_dev), GFP_KERNEL);
	if(!pmodoled_dev) {
		status = -ENOMEM;
		goto dev_alloc_err;
	}
	
	/* Alloc Graphic Buffer for device */
	pmodoled_dev->disp_buf = (uint8_t*) kmalloc(DISPLAY_BUF_SZ, GFP_KERNEL);
	if(!pmodoled_dev->disp_buf) {
		status = -ENOMEM;
		printk(KERN_INFO DRIVER_NAME "Device Display data buffer allocation failed: %d\n", status);
		goto disp_buf_alloc_err;
	}

	/* Get CS for SPI_GPIO */
	tree_info = of_get_property(np, "spi-chip-select", NULL);
	if(tree_info) {
		pmodoled_dev->iCS = be32_to_cpup((tree_info));
	}
	else {
		pmodoled_dev->iCS = SPI_GPIO_NO_CHIPSELECT;
	}

	/* Get the Pin index */
	tree_info = of_get_property(np, "gpio-pins", NULL);
	if(tree_info) {
		pmodoled_dev->iVBAT = be32_to_cpup((tree_info));
		pmodoled_dev->iVDD = be32_to_cpup((tree_info + 1));
		pmodoled_dev->iRES = be32_to_cpup((tree_info + 2));
		pmodoled_dev->iDC = be32_to_cpup((tree_info + 3));
		pmodoled_dev->iCLK = be32_to_cpup((tree_info + 4));
		pmodoled_dev->iMOSI = be32_to_cpup((tree_info + 5));
	}

	/* Get SPI Related Params */
	tree_info = of_get_property(np, "spi-bus-num", NULL);
	if(tree_info) {
		pmodoled_dev->spi_id = be32_to_cpup((tree_info));
#ifdef CONFIG_PMODS_DEBUG
		printk(KERN_INFO "pmodoled: BUS_ID\t%x\n", pmodoled_dev->spi_id);
#endif
	}

	/* Alloc Space for platform data structure */
	pmodoled_pdata = (struct spi_gpio_platform_data*) kzalloc(sizeof(*pmodoled_pdata), GFP_KERNEL);
	if(!pmodoled_pdata) {
		status = -ENOMEM;
		goto pdata_alloc_err;
	}

	/* Fill up Platform Data Structure */
	pmodoled_pdata->sck = pmodoled_dev->iCLK;
	pmodoled_pdata->miso = SPI_GPIO_NO_MISO;
	pmodoled_pdata->mosi = pmodoled_dev->iMOSI;
	pmodoled_pdata->num_chipselect = 1;
	
	/* Alloc Space for platform data structure */
	pmodoled_pdev = (struct platform_device*) kzalloc(sizeof(*pmodoled_pdev), GFP_KERNEL);
	if(!pmodoled_pdev) {
		status = -ENOMEM;
		goto pdev_alloc_err;
	}
	
	/* Fill up Platform Device Structure */	
	pmodoled_pdev->name = "spi_gpio";
	pmodoled_pdev->id = pmodoled_dev->spi_id;
	pmodoled_pdev->dev.platform_data = pmodoled_pdata;
	pmodoled_dev->pdev = pmodoled_pdev;

	/* Register spi_gpio master */
	status = platform_device_register(pmodoled_dev->pdev);
	if(status < 0) {
		printk(KERN_ALERT "platform_device_register failed: %d\n", status);
		goto pdev_reg_err;
	}
	/* Fill up Board Info for SPI device */
	status = add_pmodoled_device_to_bus(pmodoled_dev);
	if(status < 0) {
		printk(KERN_ALERT "add_pmodoled_device_to_bus failed: %d\n", status);
		goto spi_add_err;
	}

	/* Point device node data to pmodoled_device structure */
	if(np->data == NULL)
		np->data = pmodoled_dev;

	pmodoled_dev->name = np->name;	
	device_num ++;	
	return status;

spi_add_err:
	platform_device_unregister(pmodoled_dev->pdev);
pdev_reg_err:
	kfree(pmodoled_pdev);
pdev_alloc_err:
	kfree(pmodoled_pdata);
pdata_alloc_err:
	kfree(pmodoled_dev->disp_buf);
disp_buf_alloc_err:
	kfree(pmodoled_dev);
dev_alloc_err:
	return status;
}

/** 
 * pmodoled_of_remove - Remove method for ZED on-board OLED device.
 * @np: pointer to device tree node
 *
 * This function removes the OLED device in the device tree. It frees the
 * OLED driver data structure. It returns 0, if the driver is successfully
 * removed, or a negative value if there is an error.
 */
static int __exit pmodoled_of_remove(struct device_node *np) 
{
	struct pmodoled_device *pmodoled_dev;
	
	if(np->data == NULL) {
		printk(KERN_ERR "pmodoled %s: ERROR: No pmodoled_device structure found!\n", np->name);
		return -ENOSYS;
	}
	pmodoled_dev = (struct pmodoled_device*) (np->data);

	if(pmodoled_dev->disp_buf != NULL) {
		kfree(pmodoled_dev->disp_buf);
	}
	
	if(pmodoled_dev->pdev != NULL) {
		platform_device_unregister(pmodoled_dev->pdev);
	}
	
	np->data = NULL;
	
	return 0;
}

/** 
 * pmodoled_setup_cdev - Setup Char Device for ZED on-board OLED device.
 * @dev: pointer to device tree node
 * @dev_id: pointer to device major and minor number
 * @spi: pointer to spi_device structure
 *
 * This function initializes char device for OLED device, and add it into 
 * kernel device structure. It returns 0, if the cdev is successfully
 * initialized, or a negative value if there is an error.
 */
static int pmodoled_setup_cdev(struct pmodoled_device *dev, dev_t *dev_id, struct spi_device *spi) 
{
	int status = 0;
	struct device *device;
	
	cdev_init(&dev->cdev, &pmodoled_cdev_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &pmodoled_cdev_fops;
	dev->spi = spi;

	*dev_id = MKDEV(MAJOR(pmodoled_dev_id), cur_minor++);
	status = cdev_add(&dev->cdev, *dev_id, 1);
	if(status < 0) {
		return status;
	}
	
	/* Add Device node in system */
	device = device_create(pmodoled_class, NULL,
					*dev_id, NULL,
					"%s.%d", dev->name, (cur_minor - 1));
	if(IS_ERR(device)) {
		status = PTR_ERR(device);
		printk(KERN_WARNING "failed to create device node %s.%d, err %d\n",
				dev->name, (cur_minor-1), status);
		cdev_del(&dev->cdev);
	}
	
	return status;
}

/**
 * pmodoled_init_gpio - Initialize GPIO for ZED Onboard OLED
 * @dev - pmodoled_device
 *
 * Initializes OLED GPIO Control Pins.
 * It returns 0, if the gpio pins are successfully
 * initialized, or a negative value if there is an error.
 */
static int pmodoled_init_gpio(struct pmodoled_device *dev) 
{

	struct gpio pmodoled_ctrl[] = {
		{dev->iVBAT, GPIOF_OUT_INIT_HIGH, "OLED VBat"},
		{dev->iVDD, GPIOF_OUT_INIT_HIGH, "OLED VDD"},
		{dev->iRES, GPIOF_OUT_INIT_HIGH, "OLED_RESET"},
		{dev->iDC, GPIOF_OUT_INIT_HIGH, "OLED_D/C"},
	};
	int status;
	int i;

	for (i = 0; i < ARRAY_SIZE(pmodoled_ctrl); i++) {
		status = gpio_is_valid(pmodoled_ctrl[i].gpio);
		if(!status) {
			printk(KERN_INFO DRIVER_NAME "!! gpio_is_valid for GPIO %d, %s FAILED!, status: %d\n",
					pmodoled_ctrl[i].gpio, pmodoled_ctrl[i].label, status);
			goto gpio_invalid;
		}
	}

	status = gpio_request_array(pmodoled_ctrl, ARRAY_SIZE(pmodoled_ctrl));
	if(status) {
		printk(KERN_INFO DRIVER_NAME "!!  gpio_request_array FAILED!\n");
		printk(KERN_INFO DRIVER_NAME "          status is: %d\n", status);
		gpio_free_array(pmodoled_ctrl, 4);
		goto gpio_invalid;
	}

gpio_invalid:
	return status;
}

/**
 * pmodoled_disp_init -
 * @dev:
 *
 */
static void pmodoled_disp_init(struct pmodoled_device *dev) 
{
	int status;
	uint8_t wr_buf[20];
	
	// We are going to be sending commands
	// so clear the data/cmd bit
	gpio_set_value(dev->iDC, OLED_CONTROLLER_CMD);

	// Start by turning VDD on and wait for the power to come up
	gpio_set_value(dev->iVDD, 0);
	msleep(1);

	// Display off Command
	wr_buf[0] = OLED_DISPLAY_OFF;
	status = spi_write(dev->spi, wr_buf, 1);
	
	/* Bring Reset Low and then High */
	gpio_set_value(dev->iRES, 1);
	msleep(1);
	gpio_set_value(dev->iRES, 0);
	msleep(1);
	gpio_set_value(dev->iRES, 1);

	// Send the set charge pump and set precharge period commands
	wr_buf[0] = 0x8D;
	wr_buf[1] = 0x14;
	wr_buf[2] = OLED_SET_PRECHARGE_PERIOD;
	wr_buf[3] = 0xF1;

	status = spi_write(dev->spi, wr_buf, 4);

	/* Turn on VCC and wait 100ms */
	gpio_set_value(dev->iVBAT, 0);
	msleep(100);

	/* Set Display COntrast */
	wr_buf[0] = OLED_CONTRAST_CTRL;
	wr_buf[1] = 0x0F;
	
	/* Invert the display */
	wr_buf[2] = OLED_SET_SEGMENT_REMAP; // Remap Columns
	wr_buf[3] = OLED_SET_COM_DIR;	// Remap Rows

	// Select sequential COM configuration
	wr_buf[4] = OLED_SET_COM_PINS;
	wr_buf[5] = 0x00;
	wr_buf[6] = 0xC0;
	wr_buf[7] = 0x20;
	wr_buf[8] = 0x00;

	// Turn on Display
	wr_buf[9] = OLED_DISPLAY_ON;
	
	status = spi_write(dev->spi, wr_buf, 10);
}


/**
 * SPI hardware probe. Sets correct SPI mode, attempts
 * to obtain memory needed by the driver, and performs
 * a simple initialization of the device.
 */
static int pmodoled_spi_probe(struct spi_device *spi) {
	int status = 0;
	struct pmodoled_device *pmodoled_dev;

	printk(KERN_INFO DRIVER_NAME "oled_spi_probe: Probing %s ..........\n", DRIVER_NAME);
	
	/* We rely on full duplex transfers, mostly to reduce
	 * per transfer overheads (by making few transfers).
	 */
	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
		status = -EINVAL;
		printk(KERN_INFO DRIVER_NAME "SPI settings incorrect: %d\n", status);
		goto spi_err;
	}

	/* We must use SPI_MODE_0 */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	status = spi_setup(spi);
	if(status < 0) {
		printk(KERN_INFO DRIVER_NAME "needs SPI mode %02x, %d KHz; %d\n",
				spi->mode, spi->max_speed_hz / 1000,
				status);
		goto spi_err;
	}

	/* Get pmodoled_device structure */
	pmodoled_dev = (struct pmodoled_device*) spi->dev.platform_data;
	if(pmodoled_dev == NULL) {
		printk(KERN_INFO DRIVER_NAME "Cannot get pmodoled_device.\n");
		goto spi_platform_data_err;
	}
	
	/* Setup char driver */
	status = pmodoled_setup_cdev(pmodoled_dev, &(pmodoled_dev->dev_id), spi);	
	if (status) {
		printk(KERN_INFO DRIVER_NAME "oled_spi_probe: Error adding %s device: %d\n", DRIVER_NAME, status);
		goto cdev_add_err;
	}
	
	/* Initialize Mutex */
	mutex_init(&pmodoled_dev->mutex);

	/**
 	 * It is important to the OLED's longevity that the lines that 
 	 * control it's power are carefully controlled. This is a good
 	 * time to ensure that the device is ot turned on until it is
 	 * instructed to do so.
 	 */
	status = pmodoled_init_gpio(pmodoled_dev);
	if(status) {
		printk(KERN_INFO DRIVER_NAME "oled_spi_probe: Error initializing GPIO\n");
		goto oled_init_error;
	}

	pmodoled_disp_init(pmodoled_dev);

	memset(pmodoled_dev->disp_buf, 0x00, DISPLAY_BUF_SZ);

	status = screen_buf_to_display(pmodoled_dev->disp_buf, pmodoled_dev);
	if(status) {
		printk(KERN_INFO DRIVER_NAME "oled_spi_probe: Error sending initial Display String\n");
		goto oled_init_error;
	}
	return status;

oled_init_error:
	if (&pmodoled_dev->cdev)
		cdev_del(&pmodoled_dev->cdev);
cdev_add_err:
spi_platform_data_err:
spi_err:
	return(status);
}

static int __devexit pmodoled_spi_remove(struct spi_device *spi) 
{
	int status;
	struct pmodoled_device *dev;
	uint8_t wr_buf[10];

	dev = (struct pmodoled_device*) spi->dev.platform_data;

	/* Clear Display */
	memset(dev->disp_buf, 0, DISPLAY_BUF_SZ);
	status = screen_buf_to_display(dev->disp_buf, dev);
	
	/* Turn off display */
	wr_buf[0] = OLED_DISPLAY_OFF;
	status = spi_write(spi, wr_buf, 1);
	if(status) {
		dev_err(&spi->dev, "oled_spi_remove: Error writing to SPI device\n");
	}
	
	/* Turn off VCC (VBAT) */
	gpio_set_value(dev->iVBAT, 1);
	msleep(100);
	/* TUrn off VDD Power */
	gpio_set_value(dev->iVDD, 1);
	
{	
	struct gpio pmodoled_ctrl[] = {
		{dev->iVBAT, GPIOF_OUT_INIT_HIGH, "OLED VBat"},
		{dev->iVDD, GPIOF_OUT_INIT_HIGH, "OLED VDD"},
		{dev->iRES, GPIOF_OUT_INIT_HIGH, "OLED_RESET"},
		{dev->iDC, GPIOF_OUT_INIT_HIGH, "OLED_D/C"},
	};

	gpio_free_array(pmodoled_ctrl, 4);
}
	
	if(&dev->cdev) {
		device_destroy(pmodoled_class, dev->dev_id);
		cdev_del(&dev->cdev);
	}
	
	return status;
}

static struct spi_driver pmodoled_spi_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = pmodoled_spi_probe,
	.remove = __devexit_p(pmodoled_spi_remove),
};

/**
 * pmodoled_init - Initial driver registration call
 */

static int __init pmodoled_init(void)
{
	int status;
#ifdef CONFIG_OF
	struct device_node *np;

	for_each_matching_node(np, pmodoled_of_match)
		pmodoled_of_probe(np);
#else
	printk(KERN_INFO "pmodoled: CONFIG_OF needs to be enabled!\n");
	return 0;
#endif
	
	/* Alloc Major & Minor number for char device */
	status = alloc_chrdev_region(&pmodoled_dev_id, 0, device_num, DRIVER_NAME);
	if(status) {
		printk(KERN_INFO DRIVER_NAME "Character device region not allocated correctly: %d\n", status);
		return status;
	}

	pmodoled_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(pmodoled_class)) {
		status = PTR_ERR(pmodoled_class);
		return status;
	}

	return spi_register_driver(&pmodoled_spi_driver);
}

/**
 * pmodoled_exit - Clean up function when module gets removed
 */

static void __exit pmodoled_exit(void)
{
#ifdef CONFIG_OF
{
	struct device_node *np;
	
	for_each_matching_node(np, pmodoled_of_match)
		pmodoled_of_remove(np);
}
#endif

	spi_unregister_driver(&pmodoled_spi_driver);

	if(pmodoled_class) {
		class_destroy(pmodoled_class);
	}
	
	unregister_chrdev_region(pmodoled_dev_id, device_num);
	
	return;
}

module_init(pmodoled_init);
module_exit(pmodoled_exit);

MODULE_AUTHOR("Digilent, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME": ZED On-board OLED display driver");
MODULE_ALIAS("spi:"DRIVER_NAME);
