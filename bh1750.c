/**
 * BH1750 I2C Driver for BeagleBone Black using bit-banging
 * 
 * This driver implements I2C protocol using bit-banging on GPIO pins
 * to communicate with the BH1750 light sensor.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define DEVICE_NAME "bh1750"
#define CLASS_NAME "bh1750_class"

/* BH1750 I2C Address */
#define BH1750_ADDR 0x23  // ADDR pin is tied to GND

/* BH1750 Commands */
#define BH1750_POWER_DOWN      0x00
#define BH1750_POWER_ON        0x01
#define BH1750_RESET           0x07
#define BH1750_CONT_H_RES_MODE 0x10  // Continuously H-Resolution Mode

/* BeagleBone Black GPIO Base Address */
#define GPIO1_BASE 0x4804C000
#define GPIO_SIZE  0x1000

/* GPIO Register offsets */
#define GPIO_OE        0x134
#define GPIO_DATAIN    0x138
#define GPIO_DATAOUT   0x13C
#define GPIO_SETDATAOUT 0x194
#define GPIO_CLEARDATAOUT 0x190

#define SCL_PIN 17  // P9_23 - GPIO1_17
#define SDA_PIN 16  // P9_15 - GPIO1_16


/* GPIO Bit Masks */
#define SCL_MASK (1 << SCL_PIN)
#define SDA_MASK (1 << SDA_PIN)

/* Module Parameters */
static int major;
static struct class *bh1750_class = NULL;
static struct device *bh1750_device = NULL;
static struct cdev bh1750_cdev;
static void __iomem *gpio_base;

/* I2C Bit-banging timing (in microseconds) */
#define I2C_DELAY 5

/* Light sensor data structure */
struct bh1750_data {
    unsigned int lux_value;
};

static struct bh1750_data sensor_data;

/* I2C Bit-banging Functions */
static inline void set_scl(int value)
{
    if (value)
        iowrite32(SCL_MASK, gpio_base + GPIO_SETDATAOUT);
    else
        iowrite32(SCL_MASK, gpio_base + GPIO_CLEARDATAOUT);
    udelay(I2C_DELAY);
}

static inline void set_sda(int value)
{
    if (value)
        iowrite32(SDA_MASK, gpio_base + GPIO_SETDATAOUT);
    else
        iowrite32(SDA_MASK, gpio_base + GPIO_CLEARDATAOUT);
    udelay(I2C_DELAY);
}

static inline int read_sda(void)
{
    u32 val;
    u32 oe_val;
    
    /* Set SDA pin as input */
    oe_val = ioread32(gpio_base + GPIO_OE);
    iowrite32(oe_val | SDA_MASK, gpio_base + GPIO_OE);
    
    /* Read SDA value */
    val = ioread32(gpio_base + GPIO_DATAIN);
    
    /* Set SDA pin as output again */
    iowrite32(oe_val, gpio_base + GPIO_OE);
    
    return (val & SDA_MASK) ? 1 : 0;
}

/* Set SDA as input or output */
static inline void sda_dir_in(void)
{
    u32 reg_val = ioread32(gpio_base + GPIO_OE);
    iowrite32(reg_val | SDA_MASK, gpio_base + GPIO_OE);
    udelay(I2C_DELAY);
}

static inline void sda_dir_out(void)
{
    u32 reg_val = ioread32(gpio_base + GPIO_OE);
    iowrite32(reg_val & ~SDA_MASK, gpio_base + GPIO_OE);
    udelay(I2C_DELAY);
}

/* I2C Protocol Implementation */
static void i2c_start(void)
{
    sda_dir_out();
    set_sda(1);
    set_scl(1);
    set_sda(0);
    set_scl(0);
}

static void i2c_stop(void)
{
    sda_dir_out();
    set_sda(0);
    set_scl(1);
    set_sda(1);
}

static int i2c_write_byte(unsigned char byte)
{
    int i;
    int ack;
    
    sda_dir_out();
    
    /* Send 8 bits */
    for (i = 7; i >= 0; i--) {
        set_sda((byte >> i) & 1);
        set_scl(1);
        set_scl(0);
    }
    
    /* Read ACK */
    sda_dir_in();
    set_scl(1);
    ack = !read_sda();  // ACK = 0, NACK = 1
    set_scl(0);
    
    return ack;
}

static unsigned char i2c_read_byte(int ack)
{
    int i;
    unsigned char byte = 0;
    
    sda_dir_in();
    
    /* Read 8 bits */
    for (i = 7; i >= 0; i--) {
        set_scl(1);
        if (read_sda())
            byte |= (1 << i);
        set_scl(0);
    }
    
    /* Send ACK/NACK */
    sda_dir_out();
    set_sda(!ack);  // ACK = 0, NACK = 1
    set_scl(1);
    set_scl(0);
    
    return byte;
}

/* BH1750 Communication Functions */
static int bh1750_write_command(unsigned char cmd)
{
    int ret;
    
    i2c_start();
    ret = i2c_write_byte(BH1750_ADDR << 1);  // Write address
    if (!ret) {
        i2c_stop();
        printk(KERN_ERR "BH1750: No ACK on address write\n");
        return -EIO;
    }
    
    ret = i2c_write_byte(cmd);
    if (!ret) {
        i2c_stop();
        printk(KERN_ERR "BH1750: No ACK on command write\n");
        return -EIO;
    }
    
    i2c_stop();
    return 0;
}

static int bh1750_read_value(unsigned short *value)
{
    unsigned char msb, lsb;
    int ret;
    
    i2c_start();
    ret = i2c_write_byte((BH1750_ADDR << 1) | 1);  // Read address
    if (!ret) {
        i2c_stop();
        printk(KERN_ERR "BH1750: No ACK on address read\n");
        return -EIO;
    }
    
    msb = i2c_read_byte(1);  // Read MSB with ACK
    lsb = i2c_read_byte(0);  // Read LSB with NACK
    
    i2c_stop();
    
    *value = (msb << 8) | lsb;
    return 0;
}

static int bh1750_init_sensor(void)
{
    int ret;
    
    /* Power on the sensor */
    ret = bh1750_write_command(BH1750_POWER_ON);
    if (ret < 0)
        return ret;
        
    /* Reset data register */
    ret = bh1750_write_command(BH1750_RESET);
    if (ret < 0)
        return ret;
        
    /* Set continuous high resolution mode */
    ret = bh1750_write_command(BH1750_CONT_H_RES_MODE);
    if (ret < 0)
        return ret;
        
    /* Wait for first measurement to be ready */
    msleep(180);
    
    return 0;
}

static int bh1750_read_lux(unsigned int *lux)
{
    unsigned short raw_value;
    int ret;

    // Gửi lệnh khởi động cảm biến
    ret = bh1750_write_command(BH1750_POWER_ON);
    if (ret < 0)
        return ret;
    msleep(10);

    // Gửi lệnh đo liên tục ở độ phân giải cao
    ret = bh1750_write_command(BH1750_CONT_H_RES_MODE);
    if (ret < 0)
        return ret;
    msleep(180);  // BH1750 cần khoảng 120~180ms để đo xong

    // Đọc dữ liệu cảm biến
    ret = bh1750_read_value(&raw_value);
    if (ret < 0)
        return ret;

    // Chuyển đổi sang đơn vị lux
    *lux = (raw_value * 5) / 6;

    return 0;
}


/* Driver File Operations */
static int bh1750_open(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "BH1750: Device opened\n");
    return 0;
}

static int bh1750_release(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "BH1750: Device closed\n");
    return 0;
}

static ssize_t bh1750_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    unsigned int lux;
    int ret;
    char lux_str[20];
    int len;
    
    if (*offset > 0)
        return 0;  // EOF
    
    /* Read lux value from sensor */
    ret = bh1750_read_lux(&lux);
    if (ret < 0)
        return ret;
    
    /* Update stored sensor data */
    sensor_data.lux_value = lux;
    
   len = snprintf(lux_str, sizeof(lux_str), "%u lux\n", lux);

    
    /* Copy to user space */
    if (copy_to_user(buf, lux_str, len))
        return -EFAULT;
    
    *offset += len;
    return len;
}

static ssize_t bh1750_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    char kbuf[64];
    unsigned long cmd;
    
    if (count >= sizeof(kbuf))
        return -EINVAL;
    
    if (copy_from_user(kbuf, buf, count))
        return -EFAULT;
    
    kbuf[count] = '\0';
    
    if (kstrtoul(kbuf, 0, &cmd))
        return -EINVAL;
    
    /* Handle commands from user space */
    switch (cmd) {
    case 0:  // Power down
        bh1750_write_command(BH1750_POWER_DOWN);
        break;
    case 1:  // Power on and reset
        bh1750_write_command(BH1750_POWER_ON);
        bh1750_write_command(BH1750_RESET);
        break;
    case 2:  // High resolution mode
        bh1750_write_command(BH1750_CONT_H_RES_MODE);
        break;
    default:
        return -EINVAL;
    }
    
    return count;
}

/* File operations structure */
static const struct file_operations bh1750_fops = {
    .owner = THIS_MODULE,
    .open = bh1750_open,
    .release = bh1750_release,
    .read = bh1750_read,
    .write = bh1750_write,
};

/* GPIO Setup Function */
static int setup_gpio(void)
{
    u32 reg_val;
    
    /* Map GPIO registers */
    gpio_base = ioremap(GPIO1_BASE, GPIO_SIZE);
    if (!gpio_base) {
        printk(KERN_ERR "BH1750: Failed to map GPIO registers\n");
        return -ENOMEM;
    }
    
    /* Configure SCL and SDA pins as outputs */
    reg_val = ioread32(gpio_base + GPIO_OE);
    reg_val &= ~(SCL_MASK | SDA_MASK);  // Clear bits to set as output
    iowrite32(reg_val, gpio_base + GPIO_OE);
    
    /* Initialize SCL and SDA to high */
    iowrite32(SCL_MASK | SDA_MASK, gpio_base + GPIO_SETDATAOUT);
    
    return 0;
}

/* Module Initialization */
static int __init bh1750_init(void)
{
    int ret;
    dev_t dev = 0;
    
    printk(KERN_INFO "BH1750: Initializing driver\n");
    
    /* Dynamically allocate major/minor numbers */
    ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        printk(KERN_ERR "BH1750: Failed to allocate device numbers\n");
        return ret;
    }
    major = MAJOR(dev);
    
    /* Initialize character device */
    cdev_init(&bh1750_cdev, &bh1750_fops);
    bh1750_cdev.owner = THIS_MODULE;
    ret = cdev_add(&bh1750_cdev, dev, 1);
    if (ret < 0) {
        printk(KERN_ERR "BH1750: Failed to add character device\n");
        goto fail_cdev;
    }
    
    /* Create device class */
    bh1750_class = class_create(CLASS_NAME);
    if (IS_ERR(bh1750_class)) {
        printk(KERN_ERR "BH1750: Failed to create device class\n");
        ret = PTR_ERR(bh1750_class);
        goto fail_class;
    }
    
    /* Create device file */
    bh1750_device = device_create(bh1750_class, NULL, dev, NULL, DEVICE_NAME);
    if (IS_ERR(bh1750_device)) {
        printk(KERN_ERR "BH1750: Failed to create device\n");
        ret = PTR_ERR(bh1750_device);
        goto fail_device;
    }
    
    /* Setup GPIO pins */
    ret = setup_gpio();
    if (ret < 0) {
        printk(KERN_ERR "BH1750: Failed to setup GPIO\n");
        goto fail_gpio;
    }
    
    /* Initialize BH1750 sensor */
    ret = bh1750_init_sensor();
    if (ret < 0) {
        printk(KERN_ERR "BH1750: Failed to initialize sensor\n");
        goto fail_sensor;
    }
    
    printk(KERN_INFO "BH1750: Driver initialized successfully (major: %d)\n", major);
    return 0;
    
fail_sensor:
    iounmap(gpio_base);
fail_gpio:
    device_destroy(bh1750_class, MKDEV(major, 0));
fail_device:
    class_destroy(bh1750_class);
fail_class:
    cdev_del(&bh1750_cdev);
fail_cdev:
    unregister_chrdev_region(MKDEV(major, 0), 1);
    return ret;
}

/* Module Cleanup */
static void __exit bh1750_exit(void)
{
    printk(KERN_INFO "BH1750: Removing driver\n");
    
    /* Power down the sensor */
    bh1750_write_command(BH1750_POWER_DOWN);
    
    /* Unmap GPIO registers */
    iounmap(gpio_base);
    
    /* Remove device and class */
    device_destroy(bh1750_class, MKDEV(major, 0));
    class_destroy(bh1750_class);
    
    /* Remove character device and release major/minor numbers */
    cdev_del(&bh1750_cdev);
    unregister_chrdev_region(MKDEV(major, 0), 1);
    
    printk(KERN_INFO "BH1750: Driver removed\n");
}

module_init(bh1750_init);
module_exit(bh1750_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dat");
MODULE_DESCRIPTION("BH1750 I2C Driver using GPIO bit-banging for BeagleBone Black");
MODULE_VERSION("1.0");