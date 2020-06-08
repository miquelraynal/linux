/*
 * Silvaco I3C Autonomous Slave
 *
 * Linux Device Driver
 *
 * Copyright (C) 2019 Silvaco Inc.
 *  
 * 
 * crw-------    1 root     root      244,   0 Nov 25 10:35 svc-auts0 
 * crw-------    1 root     root      244,   1 Nov 25 10:35 svc-auts1 
 * crw-------    1 root     root      244,   2 Nov 25 10:35 svc-auts2
 * 
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i3c/master.h>
#include <linux/i3c/device.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/regmap.h> 
#include <linux/fs.h> 
#include <linux/uaccess.h> 
#include <linux/cdev.h> 
#include <linux/string.h>
#include "svc-auts.h"

#define SVC_I3C_AUTS_DEVICE_NAME "svc-auts"
#define SVC_I3C_AUTS_MINORS 12 // number of support svc i3c auts devices this driver will support.

//#define AUTS_DEBUG_MESSAGES

static LIST_HEAD(svc_i3c_auts_list); // list of svc_i3c_auts devices in this driver.
static DEFINE_SPINLOCK(svc_i3c_auts_list_lock); // lock for the list of svc_i3c_auts devices

static dev_t dev_no; // First device number
static struct class *svc_i3c_auts_class; // class for svc auts devices

/**
 * struct svc_i3c_auts_dev
 * 
 * structure to hold an autonomous slave device and associated data structures
 * 
 * ----------------- conor TODO -----------------
 * do we need to add in a per-device lock 
 * as part of the device structure ???
 * ----------------------------------------------
 */
static struct svc_i3c_auts_dev {
	struct list_head list;
	struct i3c_device *i3c_device;
    // <per-device> lock here ?
    struct cdev cdev;
    struct regmap *regmap;
	dev_t device_number;
    u8 target_reg;
};


static struct svc_i3c_auts_dev *svc_i3c_auts_get_by_minor_locked(unsigned index)
{
	struct svc_i3c_auts_dev *svc_i3c_auts_dev;

	list_for_each_entry(svc_i3c_auts_dev, &svc_i3c_auts_list, list) {
		if (MINOR(svc_i3c_auts_dev->device_number) == index)
            goto found;
	}
	svc_i3c_auts_dev = NULL;
found:
	return svc_i3c_auts_dev;
}


/* 
 * Search for the first available minor device number starting from 0.
 * 
 * Returns:
 * 1: are no additional slots available.
 * 0: success
 * 
 * Note -- the svc_i3c_auts_list must be locked before calling this function.
 */
static int svc_i3c_auts_get_free_minor_locked(int *device_minor_number)
{
	struct svc_i3c_auts_dev *svc_i3c_auts_dev;
	int minor_index = 0; // negative return value indicates no free minor slots.
	int found_minor = 0;

	while (minor_index < SVC_I3C_AUTS_MINORS) {
		found_minor = 0;
		
		// FOR LOOP:
		list_for_each_entry(svc_i3c_auts_dev, &svc_i3c_auts_list, list){
			if (MINOR(svc_i3c_auts_dev->device_number) == minor_index) {
				// this minor number is already in use.
				found_minor = 1;
				break; // break out of the for loop that's iterating through the list.
			}
		}

		// DONE ITERATING THROUGH LIST...
		if(found_minor){
			minor_index++;
		}else{
			// device minor number was not found in the list, thus use this minor_index
			*device_minor_number = minor_index;
			return 0; // successfully found an available device minor number.
		}  
    }
    printk("AUTS: Failed to find an available minor number\n");
	return 1; // failed to find an available minor number.
}

/*---------------------------------- 
  ------- REGMAP READ CONFIG ------- 
  ----------------------------------*/
/* registers (0x00 - 0xFF) are readable */
static const struct regmap_range svc_i3c_auts_rd_yes_ranges[] = {
    regmap_reg_range(0x00, 0xFF),
};
static const struct regmap_access_table svc_i3c_auts_rd_table = {
    .yes_ranges = svc_i3c_auts_rd_yes_ranges,
    .n_yes_ranges = 1,
};

/*---------------------------------- 
  ------- REGMAP WRITE CONFIG ------- 
  ----------------------------------*/
/* registers (0x00 - 0x0F) are non-writable */
static const struct regmap_range svc_i3c_auts_wr_no_ranges[] = {
    regmap_reg_range(0x00, 0x0F),
};
/* registers (0x10 - 0xFF) are writable */
static const struct regmap_range svc_i3c_auts_wr_yes_ranges[] = {
    regmap_reg_range(0x10, 0xFF),
};
static const struct regmap_access_table svc_i3c_auts_wr_table = {
    .yes_ranges = svc_i3c_auts_wr_yes_ranges,
    .n_yes_ranges = 1,
    .no_ranges = svc_i3c_auts_wr_no_ranges,
    .n_no_ranges = 1,
};

struct regmap_config svc_i3c_auts_regmap_config = {
    .reg_bits = 8, 
    .val_bits = 8, 
    .max_register = 0xFF,
    .wr_table = &svc_i3c_auts_wr_table,
    .rd_table = &svc_i3c_auts_rd_table,
};


/* Silvaco I3C Autonomous Slave:
 * <ManufID>: 0x011B
 * <PartID>: 0xBEEF
 * I3C_DEVICE(<manuf>, <part>, <pdata *unused*>)
 */
static const struct i3c_device_id svc_i3c_auts_ids[] = {
    I3C_DEVICE(0x011B, 0xBEEF, (void*)0x0),
    { /* sentinel */ },
};
MODULE_DEVICE_TABLE(i3c, svc_i3c_auts_ids);


/*
 * /dev/ file open()
 * 
 */
static int svc_i3c_auts_file_open(struct inode *inode, struct file *file)
{
    unsigned int minor = iminor(inode);
    struct svc_i3c_auts_dev *svc_dev;
    
    // search the list of devices for the device with svc_i3c_auts_dev with matching minor number
    spin_lock(&svc_i3c_auts_list_lock);
    svc_dev = svc_i3c_auts_get_by_minor_locked(minor);
    spin_unlock(&svc_i3c_auts_list_lock);

    /* set the file's private_data to point at the svc_i3c_auts_dev
    this will make it possible to access the svc_i3c_auts_dev later with other file_operations calls
    such as auts_file_read() / auts_file_write() / auts_file_unlocked_ioctl()
    */
    file->private_data = svc_dev;
    
    #ifdef AUTS_DEBUG_MESSAGES
    printk(KERN_INFO "SVC AUTS DRIVER: file open()\n");
    #endif

    return 0;
}


/*
 * /dev/ file close()
 *
 */
static int svc_i3c_auts_file_release(struct inode *inode, struct file *file)
{
    
    #ifdef AUTS_DEBUG_MESSAGES
    printk(KERN_INFO "SVC AUTS DRIVER: file close()\n");
    #endif

    return 0;
}

/*
 * /dev/ file read()
 * 
 */
static ssize_t svc_i3c_auts_file_read(struct file *filp, char __user *buf, size_t length, loff_t *offset)
{
    ssize_t ret = 0;

    if (length == 0) {
        printk("[svc-auts.c] svc_i3c_auts_file_read() error arg length: %zd\n", length);
        return -EINVAL;
    }

    struct svc_i3c_auts_dev *svc_dev = filp->private_data;
    u8 reg_buf[256]; // temp kernel space storage for the register data to be filled by regmap-i3c

    // success is a return value of 0.
    ret = regmap_bulk_read(svc_dev->regmap, svc_dev->target_reg, (void*)reg_buf, length);
    if (ret) {
        printk(KERN_INFO "[svc-auts.c] regmap_bulk_read error ret: %zd\n", ret);
        return -EAGAIN;
    }

    ret = copy_to_user(buf, reg_buf, length);
    if (ret) {
        printk(KERN_INFO "[svc-auts.c] copy_to_user could not copy ret: %zd (number of bytes)\n", ret);
        return -EAGAIN;
    }

    return length; // SUCCESS
}

/*
 * /dev/ file write()
 */
static ssize_t svc_i3c_auts_file_write(struct file *filp, const char __user *buf, size_t length, loff_t *offset)
{
    ssize_t ret = 0;

    if (length == 0) {
        printk("[svc-auts.c] svc_i3c_auts_file_write() error arg length: %zd\n", length);
        return -EINVAL;
    }

    struct svc_i3c_auts_dev *svc_dev = filp->private_data;
    u8 reg_buf[256]; // temp kernel space storage for the register data to be filled by regmap-i3c

    ret = copy_from_user(reg_buf, buf, length);
    if (ret) {
        // failed to copy bytes from user-space
        printk("[svc-auts.c] copy_from_user error could not copy ret: %zd (number of bytes)\n", ret);
        return -EAGAIN;
    }

    ret = regmap_bulk_write(svc_dev->regmap, svc_dev->target_reg, (void*)reg_buf, length);
    if (ret) {
        // failed to read from regmap-i3c
        printk(KERN_INFO "[svc-auts.c]regmap_bulk_write error ret: %zd\n", ret);
        return -EAGAIN;
    }

    return length; // SUCCESS
}

/*
 * /dev/ file ioctl()
 * 
 * 
 * Set the target register address.
 */
static ssize_t svc_i3c_auts_file_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    #ifdef AUTS_DEBUG_MESSAGES
    printk(KERN_INFO "SVC AUTS DRIVER: file ioctl()\n");
    #endif

    // direct the ioctl call based on the method....
    struct svc_i3c_auts_dev *svc_dev = filp->private_data; // get the svc_dev from the file
    u8 target_reg;

    switch(cmd) {
    case SVC_I3C_IOCTL_TARGET_REG: {
        target_reg = (u8)arg;
        svc_dev->target_reg = target_reg;
        
        #ifdef AUTS_DEBUG_MESSAGES
        printk(KERN_INFO "SVC_I3C_IOCTL_TARGET_REG target_reg:0x%02X\n", target_reg); // svc-auts0, svc-auts1, svc-auts2
        #endif

        break;
    }
    case SVC_I3c_IOCTL_POLL_IBI: {
        // - TODO -
        break;
    }
    default:
        return -ENOTTY;
    }
   
    return 0;
}


// add ioctl functionality here for configuring the transfer params.
// specifically, we need to state stuff like... sdr, or hdr transfer, 
// register address to begin xfer at
// num registers to sequentially read including the beginning register.
// this essentially sets-up an i3c-priv-xfer structure and passes to the /i3c/device.c subsystem
static const struct file_operations fops = {
    .read = svc_i3c_auts_file_read,
    .write = svc_i3c_auts_file_write,
    .open = svc_i3c_auts_file_open,
    .release = svc_i3c_auts_file_release,
    .unlocked_ioctl = svc_i3c_auts_file_unlocked_ioctl,
};



// IBI handler:
void svc_i3c_auts_ibi_handler(struct i3c_device *dev, const struct i3c_ibi_payload *payload)
{
    u8 payload_data;
    // assuming 1 byte of payload data
    if(payload->len == 1){
        payload_data = *((u8*)payload->data);

        //#ifdef AUTS_DEBUG_MESSAGES
        printk(KERN_INFO "svc-auts recieved ibi, length:%d data: 0x%02X\n", payload->len, payload_data);
        //#endif

    }else{

        //#ifdef AUTS_DEBUG_MESSAGES
        printk(KERN_INFO "svc-auts recieved ibi, length:%d data: <none>\n", payload->len);
        //#endif

    }
    
}

// IBI Setup:
static struct i3c_ibi_setup svc_i3c_auts_ibi_setup = {
    .max_payload_len = 1, // 1 byte payload data.
    .num_slots = 1, // 1 ibi slot.
    .handler =  &svc_i3c_auts_ibi_handler, // IBI handler callback function
};


/*
 * svc_i3c_auts_probe()
 * 
 * called when this i3c device driver is "probed" with an i3c_device.
 * The purpose is to check the i3c_device for compatibility
 * Then the driver will initialize some private data and attach 
 * itself to the underlying struct device.
 */
static int svc_i3c_auts_probe(struct i3c_device *i3cdev)
{
    
    int err = 0;
    struct svc_i3c_auts_dev *svc_dev;
    struct regmap *regmap;
    struct device *dev_ret;
    char buf[2048];

	const struct i3c_device_id *id = NULL;
    id = i3c_device_match_id(i3cdev, svc_i3c_auts_ids);

    if(!id){
        printk(KERN_ERR "svc-i3c-auts-driver failed to match id\n");
        return -ENODEV; // unsupported device
    }

    regmap = devm_regmap_init_i3c(i3cdev, &svc_i3c_auts_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&i3cdev->dev, "Failed to register i3c regmap %d\n",
			(int)ERR_PTR(regmap));
		return ERR_PTR(regmap);
	}    
    
    
    /* 
     * Allocate memory for the slave
     * devm - is "device managed memory", so when the device is removed, 
     * the memory that was allocated is automatically freed.
     * 
     * kzalloc is kernel zero allocation, 0's memory out after alloc.
     */
    svc_dev = devm_kzalloc(&i3cdev->dev, sizeof(*svc_dev), GFP_KERNEL);
    if (!svc_dev){
        return ERR_PTR(-ENOMEM); // failed to allocate memory
    }
    svc_dev->regmap = regmap;

    dev_set_drvdata(&i3cdev->dev, (void*)svc_dev); 

    // setup IBI
    err = i3c_device_request_ibi(i3cdev, &svc_i3c_auts_ibi_setup);
    if(err){
        
        printk(KERN_INFO "svc_i3c_auts: Failed to request_ibi\n");

        return err;
    }

    #ifdef AUTS_DEBUG_MESSAGES
    printk(KERN_INFO "svc_i3c_auts: setup ibi request\n");
    #endif

    // enable IBI
    err = i3c_device_enable_ibi(i3cdev);
    if(err){

        printk(KERN_INFO "svc_i3c_auts: Failed to enable_ibi\n");

        return err;
    }

    #ifdef AUTS_DEBUG_MESSAGES
    printk(KERN_INFO "svc_i3c_auts: enabled ibi\n");
    #endif



    // get the device's unique identifier.
    struct i3c_device_info dev_info;
    i3c_device_get_info(i3cdev, &dev_info);
    u16 manuf, part, ext;
    int len;

    
    int minor_number;

    // ---- lock the bus ----
    spin_lock(&svc_i3c_auts_list_lock); // acquire spin lock for the device list.
    
    err = svc_i3c_auts_get_free_minor_locked(&minor_number);
    if (err) {
        
        printk(KERN_INFO "svc_i3c_auts: failed to get free minor\n");

        return -ENODEV; // return an error to abort.
    }
    
    // setup cdev.
    dev_t auts_device_number = MKDEV(MAJOR(dev_no), minor_number);
    svc_dev->device_number = auts_device_number; // record the device number the cdev
    snprintf(buf, sizeof(buf), "svc-auts%u", minor_number); // svc-auts0, svc-auts1, svc-auts2

    // ------------------ CDEV -----------------------------
    cdev_init(&svc_dev->cdev, &fops);
    svc_dev->cdev.owner = THIS_MODULE;
    
    err = cdev_add(&svc_dev->cdev, auts_device_number, 1);
    if (err) {
		goto error_cdev;
	}
    
    



    // ------------------ DEVICE_CREATE -----------------------------
    /*
     * Create a new device as part of the svc_i3c_auts_class.
     * it has no parent, NULL..
     * it will use the auts_device_number as the device number (major, minor)
     * it will not have any attached drvdata... 
     * it will have the name that's stored in auts->buf. (which is built to identify the unique svc i3c auts... for now)
     */
    dev_ret = device_create(svc_i3c_auts_class, NULL, auts_device_number, NULL, buf);
    if (IS_ERR(dev_ret))
	{
        
        printk(KERN_INFO "svc_i3c_auts: device_create failed!\n");

		return PTR_ERR(dev_ret);
	}

    // add to list of devices.
    list_add_tail(&svc_dev->list, &svc_i3c_auts_list);
    spin_unlock(&svc_i3c_auts_list_lock);

    return 0;

error_cdev:
    // destroy the device we created earlier.
    device_destroy(svc_i3c_auts_class, svc_dev->device_number);
    spin_unlock(&svc_i3c_auts_list_lock);
    return err;
}

static int svc_i3c_auts_remove(struct i3c_device *i3cdev)
{    
    int err;
    // get the auts structure associated with the i3c_device.
    struct svc_i3c_auts_dev * svc_dev = (struct svc_i3c_auts_dev*)dev_get_drvdata(&i3cdev->dev);

    // disable IBI
    err = i3c_device_disable_ibi(i3cdev);
    if(err){
        
        printk(KERN_INFO "svc_i3c_auts: Failed to disable_ibi\n");

        return err;
    }

    // free any reserved ibi slots
    i3c_device_free_ibi(i3cdev);
    
    

    // remove the virtual device from userspace.
    device_destroy(svc_i3c_auts_class, svc_dev->device_number);

    // remove the cdev (character device)
    cdev_del(&svc_dev->cdev);

    // lock the list of devices and remove the list entry.
    spin_lock(&svc_i3c_auts_list_lock);

    /* Remove the device from the list of devices.
       effectively recycling the device MINOR number.*/
    list_del(&svc_dev->list);

    // unlock the list of devices now that the device entry is freed.
    spin_unlock(&svc_i3c_auts_list_lock);

    return 0;
}



/* ------------------------------------------------------------------------- */

static struct i3c_driver svc_i3c_auts_driver = {
    .driver = {
        .name = "svc-i3c-auts-driver",
    },
    .probe = svc_i3c_auts_probe,
    .remove = svc_i3c_auts_remove,
    .id_table = svc_i3c_auts_ids,
};

/* ------------------------------------------------------------------------- */

/*
 * module load/unload record keeping
 */

static int __init svc_i3c_auts_driver_init(void)
{
	int err = 0;    

    /* 
     * Allocated device numbers
     * device major number is dynamically allocated by kernel, 
     * 12 sequential minor numbers are allocated starting with minor (0)
     */
	err = alloc_chrdev_region(&dev_no, 0, SVC_I3C_AUTS_MINORS, SVC_I3C_AUTS_DEVICE_NAME);
	if (err)
		goto out;

    svc_i3c_auts_class = class_create(THIS_MODULE, SVC_I3C_AUTS_DEVICE_NAME);
	if (IS_ERR(svc_i3c_auts_class)) {
		err = PTR_ERR(svc_i3c_auts_class);
		goto out_unreg_chrdev;
	}

    /* register as an i3c driver ready to handle i3c devices */
    err = i3c_driver_register(&svc_i3c_auts_driver);
	if(err)
		goto out_unreg_class;
	
	return 0;

out_unreg_class:
	class_destroy(svc_i3c_auts_class);
out_unreg_chrdev:
	unregister_chrdev_region(dev_no, SVC_I3C_AUTS_MINORS);
out:
	
    printk(KERN_ERR "%s: Driver Initialization failed\n", __FILE__);

	return err;
}

static void __exit svc_i3c_auts_driver_exit(void)
{
    i3c_driver_unregister(&svc_i3c_auts_driver);
	class_destroy(svc_i3c_auts_class);
	unregister_chrdev_region(dev_no,SVC_I3C_AUTS_MINORS);
}

module_init(svc_i3c_auts_driver_init);
module_exit(svc_i3c_auts_driver_exit);

MODULE_AUTHOR("Conor Culhane <conor.culhane@silvaco.com>");
MODULE_AUTHOR("Ron Werner <ron.werner@silvaco.com>");
MODULE_DESCRIPTION("Silvaco I3C autonomous slave driver");
MODULE_LICENSE("GPL v2");