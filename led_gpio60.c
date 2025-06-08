#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#define DEVICE          5
#define AUTHOR          "DATCR7"
#define DESC            "Character device driver using GPIO60 (P9_12)"
#define DEVICE_NAME     "led_gpio60"

#define MAGIC_NO        100
#define SEND_DATA_CMD   _IOW(MAGIC_NO, 1, char*)
#define IOCTL_DATA_LEN  1024

// GPIO60 = GPIO1_28
#define GPIO_ADDR_BASE  0x4804C000
#define ADDR_SIZE       0x1000
#define GPIO_SETDATAOUT_OFFSET  0x194
#define GPIO_CLEARDATAOUT_OFFSET 0x190
#define DATA_IN_REG     0x138
#define GPIO_OE_OFFSET  0x134

#define GPIO_BIT        (1 << 28) // bit của GPIO60

static dev_t dev_num;
struct class *my_class;
struct device *my_dev;
struct cdev my_cdev;

char config_data[IOCTL_DATA_LEN];
bool first_oper;
void __iomem *base_addr;
uint32_t reg_data, old_pin_mode;

static int my_open(struct inode *inode, struct file *file) {
    printk(KERN_INFO "DATCR7: Device opened\n");
    first_oper = true;
    return 0;
}

static int my_close(struct inode *inode, struct file *file) {
    printk(KERN_INFO "DATCR7: Device closed\n");
    return 0;
}

static ssize_t my_read(struct file *flip, char __user *user_buf, size_t len, loff_t *offs) {
    char *data;
    reg_data = readl_relaxed(base_addr + DATA_IN_REG);

    data = kmalloc(len, GFP_KERNEL);
    if (!data) return -ENOMEM;
    memset(data, 0, len);
    data[0] = ((reg_data & GPIO_BIT) ? '1' : '0');

    if (copy_to_user(user_buf, data, len)) {
        kfree(data);
        return -EFAULT;
    }
    kfree(data);

    if (first_oper) {
        first_oper = false;
        return 1;
    }
    return 0;
}

static ssize_t my_write(struct file *flip, const char __user *user_buf, size_t len, loff_t *offs) {
    char cmd;
    if (copy_from_user(&cmd, user_buf, 1)) return -EFAULT;

    switch (cmd) {
        case '1':
            writel_relaxed(GPIO_BIT, base_addr + GPIO_SETDATAOUT_OFFSET);
            break;
        case '0':
            writel_relaxed(GPIO_BIT, base_addr + GPIO_CLEARDATAOUT_OFFSET);
            break;
        default:
            printk(KERN_INFO "Unknown write command: %c\n", cmd);
    }
    return len;
}

static long my_ioctl(struct file *filep, unsigned int cmd, unsigned long arg) {
    if (cmd == SEND_DATA_CMD) {
        memset(config_data, 0, IOCTL_DATA_LEN);
        if (copy_from_user(config_data, (char*)arg, IOCTL_DATA_LEN))
            return -EFAULT;
        printk(KERN_INFO "DATCR7: IOCTL received: %s\n", config_data);
    } else {
        return -ENOTTY;
    }
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = my_open,
    .release = my_close,
    .read = my_read,
    .write = my_write,
    .unlocked_ioctl = my_ioctl,
};

static int __init func_init(void) {
    int ret;

    ret = alloc_chrdev_region(&dev_num, 0, DEVICE, DEVICE_NAME);
    if (ret < 0) return ret;

    my_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(my_class)) return PTR_ERR(my_class);

    cdev_init(&my_cdev, &fops);
    cdev_add(&my_cdev, dev_num, 1);
    my_dev = device_create(my_class, NULL, dev_num, NULL, DEVICE_NAME);

    base_addr = ioremap(GPIO_ADDR_BASE, ADDR_SIZE);
    if (!base_addr) {
        printk(KERN_ERR "DATCR7: Failed to ioremap GPIO\n");
        return -ENOMEM;
    }

    // Set pin to output
    reg_data = readl_relaxed(base_addr + GPIO_OE_OFFSET);
    old_pin_mode = reg_data;
    reg_data &= ~GPIO_BIT;  // clear bit -> set output
    writel_relaxed(reg_data, base_addr + GPIO_OE_OFFSET);

    printk(KERN_INFO "DATCR7: Driver loaded for GPIO60 (P9_12)\n");
    return 0;
}

static void __exit func_exit(void) {
    // Restore original pin mode
    writel_relaxed(old_pin_mode, base_addr + GPIO_OE_OFFSET);
    iounmap(base_addr);
    device_destroy(my_class, dev_num);
    class_destroy(my_class);
    cdev_del(&my_cdev);
    unregister_chrdev_region(dev_num, DEVICE);
    printk(KERN_INFO "DATCR7: Driver unloaded\n");
}

module_init(func_init);
module_exit(func_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(AUTHOR);
MODULE_DESCRIPTION(DESC);
MODULE_VERSION("0.02");

