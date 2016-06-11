/*
 * rover.c
 *
 * Rover microcontroller 0 (wheels, distance sensor) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * Copyright (C) 2016 Tomasz Chadzynski
 */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/input-polldev.h>
#include <misc/rover.h>

/*
 * TODO:
 * Properly comment the code
 */

struct uc0_device {
    struct miscdevice misc_dev;
    struct i2c_client *i2c_dev;
    struct input_polled_dev *distance_polled_dev;
    struct uc0_wheel_state wheel_state; 
};

   
/*
 * Module parameters
 */


static unsigned poll_interval = 1000; //msec 
module_param(poll_interval, uint, S_IRUGO);

/*
 * Device send/read functions
 */

inline static int uc0_send_command(struct i2c_client *uc0_dev, enum command cmd, int16_t arg)
{
    volatile int16_t cmdBuff[2] = {cmd, arg};
	i2c_master_send(uc0_dev, (char*)cmdBuff, 4);

    printk(KERN_DEBUG "Rover[uc0]: CMD send: %i %i \n", cmd, arg);
	
    return 0;
}

inline static int uc0_read_property(struct i2c_client *uc0_dev, enum command cmd, int16_t *arg)
{
    volatile int16_t buffer[2] = {0,0};

    switch(cmd) {
    case REQ_LEFT_SPEED:
        uc0_send_command(uc0_dev, REQ_LEFT_SPEED, 0);
        i2c_master_recv(uc0_dev, (char*)buffer, 4);

        if(buffer[0] == REQ_LEFT_SPEED) {
            *arg = buffer[1];
        } else {
            printk(KERN_ALERT "Rover[uc0]: io error for req arg %i \n", cmd);
            return -EIO;
        }
        break;
    case REQ_RIGHT_SPEED:
        uc0_send_command(uc0_dev, REQ_RIGHT_SPEED, 0);
        i2c_master_recv(uc0_dev, (char*)buffer, 4);

        if(buffer[0] == REQ_RIGHT_SPEED) {
            *arg = buffer[1];
        } else {
            printk(KERN_ALERT "Rover[uc0]: io error for req arg %i \n", cmd);
            return -EIO;
        }
        break;
    case REQ_DISTANCE:
        uc0_send_command(uc0_dev, REQ_DISTANCE, 0);
        i2c_master_recv(uc0_dev, (char*)buffer, 4);

        if(buffer[0] == REQ_DISTANCE) {
            *arg = buffer[1];
        } else {
            printk(KERN_INFO "Rover[uc0]: io error for req arg %i buff %i %i \n", cmd, buffer[0], buffer[1]);
            return -EIO;
        }
        break;
    default:
        printk(KERN_ALERT "Rover[uc0]: invalid REQ argument %i \n", cmd); 
        return -EINVAL;
    }
    return 0;
}

/*
 * Misc device framework fops
 */

static int roveruc0_fopen(struct inode *i, struct file *f)
{
    printk(KERN_DEBUG "Rover[uc0]: Driver file open \n");
    return 0;
}


static int roveruc0_fclose(struct inode *i, struct file *f)
{
    printk(KERN_DEBUG "Rover[uc0]: Driver file closed \n");
    return 0;
}

static ssize_t roveruc0_fread(struct file *f, char __user *buf, size_t len, loff_t *off)
{
    struct uc0_device *uc0 = container_of(f->private_data, struct uc0_device, misc_dev);

    size_t state_size;
    size_t data_left;
    size_t bytes_to_read;

    state_size = sizeof(struct uc0_wheel_state);
    if(*off >= state_size) {
        return 0;
    }
    data_left = state_size - *off;
    if(len > data_left) {
        bytes_to_read = data_left;
    } else {
        bytes_to_read = len;
    }

    if(copy_to_user(buf, (&(uc0->wheel_state)+(*off)), bytes_to_read) != 0) {
        printk(KERN_ALERT "Rover[uc0]: Failed copy to user !!! \n");
        return -EFAULT;
    }

    *off += bytes_to_read;
    return bytes_to_read;
}

static ssize_t roveruc0_fwrite(struct file *f, const char __user *buf, size_t len, loff_t *off)
{
/*
 * TODO:
 * figure out better way to handle non writable drv char dev
 */
    printk(KERN_DEBUG "Rover[uc0]: Driver file write attempt \n");
    return len;
}

static loff_t roveruc0_fllseek (struct file *file, loff_t offset, int whence) 
{
    size_t max_size =  sizeof(struct uc0_wheel_state);
    if(whence == SEEK_SET) {
        if(offset >=0 && offset <= max_size) {
            file->f_pos = offset;
            return offset;
        } else {
            return -EINVAL;
        }
    } else {
        return -EINVAL;
    }
}

static long roveruc0_ioctl (struct file * f, unsigned int cmd, unsigned long arg)
{
    struct uc0_device *uc0 = container_of(f->private_data, struct uc0_device, misc_dev);
    struct i2c_client *client = uc0->i2c_dev;

    int16_t __user *argp = (int16_t __user *)arg;
    switch(cmd) {
    case SET_WHEEL_SPEED:
        uc0->wheel_state.left_wheel_speed = argp[0];
        uc0->wheel_state.right_wheel_speed = argp[1];
        uc0_send_command(client, SET_LEFT_SPEED, argp[0]);
        uc0_send_command(client, SET_RIGHT_SPEED, argp[1]);
        break;
    case STOP:
        if(uc0->wheel_state.left_wheel_speed || uc0->wheel_state.right_wheel_speed) {
            uc0->wheel_state.left_wheel_speed =0;
            uc0->wheel_state.right_wheel_speed =0;
            uc0_send_command(client, WHEELS_STOP,0);
        }
        break;
    };

    return 0;
}

/*
 * Pooled device part
 */


static void roveruc0_distance_poll ( struct input_polled_dev *dev)
{

    struct i2c_client *client = ((struct uc0_device*)dev->private)->i2c_dev;

    uint16_t dist;
    if(uc0_read_property(client, REQ_DISTANCE, &dist)) {
        printk(KERN_ALERT "Rover[uc0]: Error reading distance property \n");
        return;
    }

    input_event(dev->input, EV_MSC, MSC_RAW, dist);
    //input_sync(polled_device->input);
}

/*
 * Probe
 */


static int roveruc0_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    volatile int16_t buffer[2];
    struct uc0_device *uc0;
    char *devname_str;
    struct file_operations *fops;
    struct input_polled_dev *polled_device;

    printk(KERN_INFO "Rover[uc0]: Probing device at %i \n", client->addr);

    buffer[0] = INIT_CONTROL_STATE; buffer[1] = 0x0;
    uc0_send_command(client, INIT_CONTROL_STATE,0);
    i2c_master_recv(client, (char*)buffer, 4);
    printk(KERN_INFO "Rover[uc0]: Probe result %i:%i\n", buffer[0], buffer[1]);
    
    if(buffer[0] != INIT_CONTROL_STATE || buffer[1] != 0x5AAA) {
        dev_err(&client->dev, "Rover[uc0]: Probing failed expected %i:%i\n",INIT_CONTROL_STATE, 0x5AAA);
        return -EIO;
    }

    uc0 = devm_kzalloc(&client->dev, sizeof(struct uc0_device), GFP_KERNEL);
    devname_str = devm_kasprintf(&client->dev, GFP_KERNEL, "roveruc0");
    fops = devm_kzalloc(&client->dev, sizeof(struct file_operations), GFP_KERNEL);
    if(!uc0 || !devname_str || !fops) {
        dev_err(&client->dev, "Rover[uc0]:Failed to allocate kernel memory !\n");
        return -ENOMEM;
    }

    fops->owner = THIS_MODULE;
    fops->open = roveruc0_fopen;
    fops->release = roveruc0_fclose;
    fops->read = roveruc0_fread;
    fops->write = roveruc0_fwrite;
    fops->unlocked_ioctl = roveruc0_ioctl;
    fops->llseek = roveruc0_fllseek;

    uc0->wheel_state.left_wheel_speed = 0;
    uc0->wheel_state.right_wheel_speed = 0;
    uc0->wheel_state.wheel_min_speed = 0;
    uc0->wheel_state.wheel_max_speed = 255;
    
    polled_device = input_allocate_polled_device();
    if(!polled_device) {
        printk(KERN_CRIT "Rover[uc0]: Error allocating resources for polled device\n");
        return -EIO;
    }

    polled_device->poll = roveruc0_distance_poll;
    polled_device->poll_interval = poll_interval;
    polled_device->input->name = "Rover uc0 distance sensor";
    polled_device->input->id.bustype = BUS_I2C;

    set_bit(EV_MSC, polled_device->input->evbit);
    set_bit(MSC_RAW, polled_device->input->mscbit);
    

    if(input_register_polled_device(polled_device)) {
        printk(KERN_CRIT "Rover[uc0]: Error error registering polled device\n");
        input_free_polled_device(polled_device);
        return -EIO;
    }

    uc0->i2c_dev = client;
    uc0->distance_polled_dev = polled_device;

    polled_device->private = uc0;
    polled_device->input->dev.parent = &client->dev;

    uc0->misc_dev.minor = MISC_DYNAMIC_MINOR;
    uc0->misc_dev.name = devname_str;
    uc0->misc_dev.fops = fops;

    if(misc_register(&uc0->misc_dev)) {
        printk(KERN_INFO "Rover[uc0]: Error error registering misc device\n");
        input_unregister_polled_device(polled_device);
        input_free_polled_device(polled_device);
        return -EIO;
    }

	i2c_set_clientdata(client, uc0);

	return 0;
}

static int roveruc0_remove(struct i2c_client *client)
{
	struct uc0_device *uc0 = i2c_get_clientdata(client);

    input_unregister_polled_device(uc0->distance_polled_dev);
    input_free_polled_device(uc0->distance_polled_dev);
    misc_deregister(&uc0->misc_dev);

    printk(KERN_DEBUG "Rover[uc0]: Driver removed \n");

	return 0;
}


static const struct i2c_device_id roveruc0_id[] = {
	{ "roveruc0", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, roveruc0_id);

#ifdef CONFIG_OF
static const struct of_device_id roveruc0_id_ids[] = {
	{ .compatible = "rover,roveruc0",},
	{}
};
MODULE_DEVICE_TABLE(of, roveruc0_id_ids);
#endif

static struct i2c_driver roveruc0_driver = {
	.probe = roveruc0_probe,
	.remove = roveruc0_remove,
	.id_table = roveruc0_id,
	.driver = {
		.name = "roveruc0-drv",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(roveruc0_id_ids),
	},
};


/*
 * Lifecycle functions
 */
 
static int __init drv_init(void) 
{
    printk(KERN_INFO "Rover[uc0]: Registering roveruc0 driver\n");
    i2c_add_driver(&roveruc0_driver);

    return 0;
}
 
static void __exit drv_exit(void) 
{
    printk(KERN_INFO "Rover[uc0]: Unregister roveruc0 driver\n");
    i2c_del_driver(&roveruc0_driver);

}
 
module_init(drv_init);
module_exit(drv_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomasz Chadzynski");
MODULE_DESCRIPTION("Rover uc0 driver v. 0.1");


