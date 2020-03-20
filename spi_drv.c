#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <asm/current.h>
#include <linux/kern_levels.h>
#include <linux/printk.h>
#include <linux/sched/signal.h>
#include <linux/spi/spi.h>

#include "spi.h"
#include "spi_drv.h"

#define DRIVER_NAME "spi_device"
#define MAJOR_NUMBER	156
#define MAX_MINOR_NUMBER 64

static int buf_depth = 64;
module_param(buf_depth, int, S_IRUGO | S_IWUSR);

static LIST_HEAD(spi_device_dev_list);
static DEFINE_MUTEX(spi_device_dev_list_lock);
static DEFINE_IDR(spi_device_idr);
static DEFINE_SPINLOCK(spi_device_idr_lock);

typedef struct spi_dev {
	unsigned char *control_regs;
	unsigned char *status_regs;
	unsigned char *data_regs;
}spi_dev_t;


struct spi_drv {
        dev_t dev_num;                             //present device number
	int id;
	unsigned int open_cnt;
	struct  list_head device_entry;
	struct class *spi_device_class;
	struct device *dev;
	struct cdev *vcdev;
	struct spi_dev_t *spi_hw;
	struct spi_device *slave_device;
};

struct spi_device_message {
	void  __iomem *tx;
	u32 tx_actual_length;
	void  __iomem *rx;
	u32 rx_actual_length;
	u8 mode;
	u32 bits_per_word;
	u32 buf_depth;
	u32 max_speed;
	u8 lsb_first;

	wait_queue_head_t wait;
	spinlock_t wait_lock;
	struct mutex msg_lock;
};

struct spi_device {
	struct device dev;
	struct spi_device_message *msg;
	void *spi_device_gadget;

	int (*transfer_msg)(struct spi_device *slave_device);
	void (*clear_msg)(struct spi_device *slave_device);
};

/*
 * con trỏ hàm open
 * chức năng: khởi tạo thiết bị nếu cần, lưu lại minor number…
 * tham số đầu vào:
 *     *inode [I]: địa chỉ của cấu trúc inode. Cấu trúc này dùng để mô tả
 *                 các file trong hệ thống.
 *     *filp  [I]: địa chỉ của cấu trúc file. Cấu trúc file dùng để mô tả
 *                 một file đang mở.
 * giá trị trả về:
 *      Hàm này trả về 0 để thông báo mở device file thành công.
 *      Ngược lại, trả về một số khác 0 để thông báo mở device file
 *      thất bại.
 */
static int spi_device_open(struct inode *inode, struct file *filp)
{
	struct spi_drv *drv;
	struct spi_device *slave_device = drv->slave_device;
        struct spi_device_message *msg;
        int ret = -ENXIO;

        pr_info("%s: function: open\n", DRIVER_NAME);

	//mutex_lock(&spi_device_dev_list_lock);

        list_for_each_entry(drv, &spi_device_dev_list, device_entry) {
                if (drv->dev_num == inode->i_rdev) {
                        ret = 0;
                        break;
                }
        }

        if (ret) {
               // mutex_unlock(&spi_device_dev_list_lock);
                return ret;
        }
	
	drv->open_cnt++;
	printk("OK_1");
	if(drv->open_cnt > 1)
		return -EBUSY;

	filp->private_data = drv;
        nonseekable_open(inode, filp);

        msg = spi_device_msg_alloc(slave_device);
        if (!msg)
                ret = -ENOMEM;
        msg->buf_depth = buf_depth;

        //mutex_unlock(&spi_device_dev_list_lock);


	return 0;

}

/*
 * con trỏ hàm release
 * chức năng: khi một tiến trình trên user space gọi system call close
 *            để đóng device file tương ứng với char driver này, thì
 *            hàm này sẽ được gọi để thực hiện một số việc như tắt
 *            thiết bị, hoặc làm cho thiết bị ngừng hoạt động…
 * tham số đầu vào: giống như hàm open
 * giá trị trả về:
 *      Hàm này trả về 0 để thông báo đóng device file thành công.
 *      Ngược lại, trả về một số khác 0 để thông báo đóng device file
 *      thất bại.
 */
static int spi_device_release(struct inode *inode, struct file *filp)
{
	struct spi_drv *drv = filp->private_data;
        struct spi_device *slave_device = drv->slave_device;
        struct spi_device_message *msg = slave_device->msg;

        pr_info("%s: function: release\n", DRIVER_NAME);
	
	mutex_lock(&spi_device_dev_list_lock);

        kfree(msg->tx);
        kfree(msg->rx);

        spi_device_msg_remove(slave_device);

        drv->open_cnt--;
        mutex_unlock(&spi_device_dev_list_lock);


	return 0;
}

/*
 * con trỏ hàm write
 * chức năng: sao chép dữ liệu từ user buffer vào trong kernel buffer,
 *            sau đó ghi dữ liệu từ kernel buffer vào trong buffer
 *            của char device.
 * tham số đầu vào:
 *     *filp [I]: địa chỉ của cấu trúc file. Cấu trúc này mô tả một
 *                device file đang mở.
 *     *buff [I]: địa chỉ của user buffer.
 *      size [I]: số lượng byte dữ liệu mà tiến trình cần đọc.
 *     *off  [I]: địa chỉ của cấu trúc loff_t. Cấu trúc này cho biết vị trí
 *                trong buffer của char device mà dữ liệu bắt đầu được ghi vào.
 * giá trị trả về:
 *      Trả về một số dương thể hiện số byte đã được ghi vào thiết bị.
 *      Trong nhiều trường hợp, nếu số này nhỏ hơn tham số [size], thì
 *      tiến trình sẽ tiếp tục gọi system call write cho tới khi nào
 *      tổng lượng dữ liệu ghi vào bằng tham số [size], hoặc cho tới khi
 *      buffer của char device đầy, không thể ghi được nữa.
 *
 *      Trả về 0 thể hiện rằng buffer của char device đã đầy, không thể
 *      ghi vào được nữa.
 *
 *      Trả về một số âm nếu có lỗi.
 */

static ssize_t spi_device_write(struct file *filp, const char __user *buf,
                              size_t count, loff_t *f_pos)
{
        ssize_t status = 0;
        int ret = 0;
        unsigned long missing;
        struct spi_drv *drv  = filp->private_data;
        struct spi_device *slave_device = drv->slave_device;
        struct spi_device_message *msg = slave_device->msg;

        pr_info("%s: function: write\n", DRIVER_NAME);

	//mutex_lock(&msg->msg_lock);

        if (!msg->tx) {
                msg->tx = kzalloc(msg->buf_depth, GFP_KERNEL);
                if (!msg->tx)
                        return -ENOMEM;
        } else
                memset(msg->tx, 0, msg->buf_depth);

        if (!msg->rx) {
                msg->rx = kzalloc(msg->buf_depth, GFP_KERNEL);
                if (!msg->rx)
                        return -ENOMEM;
        }

        if (count > msg->buf_depth)
                return -EMSGSIZE;

        missing = copy_from_user(msg->tx, buf, count);

        if (missing == 0)
                status = count;
        else
                status = -EFAULT;

        //mutex_unlock(&msg->msg_lock);

        msg->mode |= SPI_DEVICE_TRM;
        msg->mode &= ~SPI_DEVICE_TM;

        ret = spi_device_transfer_msg(slave_device);
        if (ret < 0) {
                status = -EFAULT;
                pr_info("%s: function: write - status:%d", DRIVER_NAME, status);
        }

        return status;
}


/*
 * con trỏ hàm read
 * chức năng: đọc dữ liệu từ buffer của char device vào kernel buffer,
 *            sau đó, sao chép dữ liệu từ kernel buffer vào trong
 *            user buffer của tiến trình.
 * tham số đầu vào:
 *     *filp [I]: địa chỉ của cấu trúc file. Cấu trúc này mô tả một
 *                device file đang mở.
 *     *buff [I]: địa chỉ của user buffer.
 *      size [I]: số lượng byte dữ liệu mà tiến trình cần đọc.
 *     *off  [I]: địa chỉ của cấu trúc loff_t. Cấu trúc này cho biết vị trí
 *                trên buffer của char device mà dữ liệu bắt đầu được đọc ra.
 * giá trị trả về:
 *      Trả về một số dương thể hiện số byte đã đọc được từ thiết bị.
 *      Trong nhiều trường hợp, nếu số này nhỏ hơn tham số [size], thì
 *      tiến trình sẽ tiếp tục gọi system call read cho tới khi nào
 *      tổng lượng dữ liệu đọc được bằng tham số [size], hoặc cho tới
 *      khi toàn bộ dữ liệu trong buffer của char device được đọc hết.
 *
 *      Trả về 0 thể hiện rằng toàn bộ dữ liệu trong buffer của char device
 *      đã được đọc hết.
 *
 *      Trả về một số âm nếu có lỗi.
 */

static ssize_t spi_device_read(struct file *filp, char __user *buf, size_t count,
                             loff_t *f_pos)
{
	struct spi_drv *drv = filp->private_data;
        struct spi_device *slave_device = drv->slave_device;
        struct spi_device_message *msg = slave_device->msg;
        ssize_t status;
        unsigned long missing;
        DECLARE_WAITQUEUE(wait, current);
        unsigned long flags;
        int ret = 0;

        msg->mode |= SPI_DEVICE_TRM;
        msg->mode &= ~SPI_DEVICE_RM;

        pr_info("%s: function: read\n", DRIVER_NAME);

        ret = spi_device_transfer_msg(slave_device);
        if (ret < 0)
                status = -EFAULT;

        spin_lock_irqsave(&msg->wait_lock, flags);

        if (filp->f_flags & O_NONBLOCK) {
                spin_unlock_irqrestore(&msg->wait_lock, flags);
                return -EAGAIN;
        }

        add_wait_queue(&msg->wait, &wait);
        for (;;) {
                set_current_state(TASK_INTERRUPTIBLE);
                if (signal_pending(current))
                        break;

                spin_unlock_irqrestore(&msg->wait_lock, flags);
                schedule();
                spin_lock_irqsave(&msg->wait_lock, flags);
        }
        set_current_state(TASK_RUNNING);
        remove_wait_queue(&msg->wait, &wait);
        spin_unlock_irqrestore(&msg->wait_lock, flags);

        mutex_lock(&msg->msg_lock);

        if (count > msg->buf_depth)
                return -EMSGSIZE;

        if (!msg->rx) {
                msg->rx = kzalloc(msg->buf_depth, GFP_KERNEL);
                if (!msg->rx)
                        return -ENOMEM;
        }

        status = count;
        missing = copy_to_user(buf, msg->rx, msg->rx_actual_length);
        if (missing == status)
                status = -EFAULT;
        else
                status = status - missing;

        mutex_unlock(&msg->msg_lock);

        return status;

}


static long spi_device_ioctl(struct file *filp, unsigned int cmd,
                           unsigned long arg)
{
        struct spi_drv *drv = filp->private_data;
        struct spi_device *slave_device = drv->slave_device;
        struct spi_device_message *msg = slave_device->msg;

        struct spi_device_ioctl_transfer *ioctl_msg;
        u32 size_msg;

        int ret = 0;

        pr_info("%s: function: ioctl\n", DRIVER_NAME);

        mutex_lock(&msg->msg_lock);

        /*
         * FIXME: check arg and cmd argument, if cmd is ioctl type
         */

        /*
         * FIXME: check all ioctl inputs and outputs
         */

        switch (cmd) {
        case SPI_DEVICE_RD_TX_ACTUAL_LENGTH:
                ret = __put_user(msg->tx_actual_length, (__u32 __user *)arg);
                break;

        case SPI_DEVICE_RD_RX_ACTUAL_LENGTH:
                ret = __put_user(msg->rx_actual_length, (__u32 __user *)arg);
                break;

        case SPI_DEVICE_RD_BITS_PER_WORD:
                ret = __put_user(msg->bits_per_word, (__u32 __user *)arg);
                break;

        case SPI_DEVICE_RD_MODE:
                ret = __put_user(msg->mode, (__u32 __user *)arg);
                break;

        case SPI_DEVICE_RD_MAX_SPEED:
                ret = __put_user(msg->max_speed, (__u32 __user *)arg);
                break;

        case SPI_DEVICE_WR_BITS_PER_WORD:
                ret = __get_user(msg->bits_per_word, (__u32 __user *)arg);
                break;

        case SPI_DEVICE_WR_MODE:
                ret = __get_user(msg->mode, (__u32 __user *)arg);
                break;

        case SPI_DEVICE_WR_MAX_SPEED:
                ret = __get_user(msg->max_speed, (__u32 __user *)arg);
                break;

        default:

                /*
                 * TODO: add support for full-duplex transfer
                 * message uses own structures which located in
                 * spi-slave_device-dev.h,
                 *
                 * need translate ioctl message to spi slave_device message standards
                 */

                size_msg = _IOC_SIZE(cmd);
                pr_info("%s: ioctl size_msg:%d/n", DRIVER_NAME, size_msg);

                ioctl_msg = kzalloc(size_msg, GFP_KERNEL);

                if (__copy_from_user(ioctl_msg,
                                    (struct spi_device_ioctl_transfer __user *)arg,
                                    size_msg)) {
                        kfree(ioctl_msg);
                        return -EFAULT;
                }

                pr_info("%s: ioctl msg: mode:%d\n", DRIVER_NAME,
                        ioctl_msg->mode);
                pr_info("%s: ioctl max_speed:%d\n", DRIVER_NAME,
                        ioctl_msg->max_speed);
                pr_info("%s: ioctl bits_per_word:%d\n", DRIVER_NAME,
                        ioctl_msg->bits_per_word);
                pr_info("%s: ioctl tx actual_length:%d\n", DRIVER_NAME,
                        ioctl_msg->tx_actual_length);
                pr_info("%s: ioctl rx actual_length:%d\n", DRIVER_NAME,
                        ioctl_msg->rx_actual_length);

                break;
        }

        mutex_unlock(&msg->msg_lock);
        return ret;
}

static unsigned int spi_device_event_poll(struct file *filp,
                                        struct poll_table_struct *wait)
{
        struct spi_drv *drv = filp->private_data;
        struct spi_device *slave_device = drv->slave_device;
        struct spi_device_message *msg = slave_device->msg;

        pr_info("%s: function: poll\n", DRIVER_NAME);

        poll_wait(filp, &msg->wait, wait);
        if (msg->rx_actual_length > 0)
                return POLLIN | POLLRDNORM;

        return 0;
}

static const struct file_operations spi_device_fops = {
        .owner          = THIS_MODULE,
        .open           = spi_device_open,
        .read           = spi_device_read,
        .write          = spi_device_write,
        .release        = spi_device_release,
        .unlocked_ioctl = spi_device_ioctl,
        .poll           = spi_device_event_poll,
};

static int spi_device_probe(struct spi_dev_device *spi)
{
        int ret = 0;
        struct spi_drv *drv;
        struct spi_device *slave_device;
        struct device *dev;
        struct device_node *node;

        pr_info("%s: function: probe\n", DRIVER_NAME);

        //data = &spi->dev.plaformdata
        drv = kzalloc(sizeof(*drv), GFP_KERNEL);
        if (!drv)
                return -ENODEV;

        slave_device = spi->slave_device;
        spi->slave_device = slave_device;
        node = spi->dev.of_node;

        if (!slave_device) {
                ret = -ENODEV;
                goto err_out;
        }

        INIT_LIST_HEAD(&data->device_entry);

        mutex_lock(&spi_device_dev_list_lock);
        spin_lock(&spi_device_idr_lock);

        ret = idr_alloc(&spi_device_idr, data, 0, MAX_MINOR_NUMBER,
                             GFP_KERNEL);

        spin_unlock(&spi_device_idr_lock);


        list_add(&data->device_entry, &spi_device_dev_list);
        spi_device_set_drv_data(spi, data);
        mutex_unlock(&spi_device_dev_list_lock);


err_out:
        kfree(data);
        mutex_unlock(&spi_device_dev_list_lock);
        return ret;

        return 0;
}


static int spi_device_remove(struct spi_dev_device *spi)
{
        struct spi_drv *drv = spi_device_get_drv_data(spi);

        pr_info("%s: function: remove\n", DRIVER_NAME);

        drv->slave_device = NULL;

        mutex_lock(&spi_device_dev_list_lock);
        list_del(&drv->device_entry);
        mutex_unlock(&spi_device_dev_list_lock);

        device_destroy(drv->spi_device_class, drv->dev_num);

        kfree(drv);

        return 0;
}

static const struct of_device_id spi_device_of_match[] = {
        { .compatible = "linux,spi_device"},
        {},
};
MODULE_DEVICE_TABLE(of, spi_device_of_match);

static const struct spi_device_id  spi_device_table[] = {
	{"spi_device", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, spi_device_table);


static struct spi_driver slave_device_driver = {
        .driver = {
                .name = DRIVER_NAME,
                .of_match_table = spi_device_of_match,
        },
        .probe = spi_device_probe,
        .remove = spi_device_remove,
	.id_table = spi_device_id,
};
module_spi_driver(spi_device_driver);


static int spi_hw_init(spi_dev_t *hw)
{
	char *buf;
	buf = kzalloc( NUM_DEV_REGS * REG_SIZE, GFP_KERNEL);
	if(!buf) {
		return -ENOMEM;
	}

	hw->control_regs = buf;
	hw->status_regs = hw->control_regs +NUM_CTRL_REGS;
	hw->data_regs = hw->status_regs + NUM_STS_REGS;

	hw->control_regs[CONTROL_ACCESS_REG] = 0x03;
	hw->status_regs[DEVICE_STATUS_REG] = 0x03;

	return 0;

}

void spi_hw_exit(spi_dev_t *hw)
{
	kfree(hw->control_regs);

}

/* ham khoi tao driver */
static int __init spi_device_init(void)
{
	pr_info("%s: function: init\n", DRIVER_NAME);

        int ret = 0;

        /* cap phat device number */

        spi_drv.id = ret;
        spi_drv.open_cnt = 0;
        spi_drv.dev_num = MKDEV(MAJOR_NUMBER, spi_drv.id);
        ret = register_chrdev_region(spi_drv.dev_num, 1, DRIVER_NAME);
        if (ret < 0) {
                printk("Registering the char device failed with %d\n", ret);
                goto failed_register_devnum;
        }

	/* tao device file */

	spi_drv.spi_device_class = class_create(THIS_MODULE, "class_spi_device");
        if(spi_drv.spi_device_class == NULL) {
                printk("failed to create a device class\n");
                goto failed_create_class;
        }

        spi_drv.dev = device_create(spi_drv.spi_device_class, NULL, spi_drv.dev_num, NULL, DRIVER_NAME);
	if(IS_ERR(spi_drv.dev)) {
		printk("failed to create a device\n");
                goto failed_create_device;
	}

	/* cap phat bo nho cho cac cau truc du lieu cua driver va khoi tao */
	spi_drv.spi_hw = kzalloc(sizeof(spi_dev_t), GFP_KERNEL);
	if(!spi_drv.spi_hw) {
		printk("failed to allocate data structure of the driver\n");
		ret = -ENOMEM;
		goto failed_allocate_structure;
	}
	/* khoi tao thiet bi vat ly */
	ret = 0;
	if(ret<0) {
		printk("failed to intialize a virtual charater device\n");
		goto failed_init_hw;
	}
	/* dang ky cac entry point voi kernel */
	spi_drv.vcdev=cdev_alloc();
	if(spi_drv.vcdev == NULL) {
		printk("failed to allocate cdev structure\n");
		goto failed_allocate_cdev;
	}
	cdev_init(spi_drv.vcdev,&spi_device_fops);
	ret = cdev_add(spi_drv.vcdev,spi_drv.dev_num, 1);
	if( ret < 0) {
		printk("failed to add a char device to the system\n");
		goto failed_allocate_cdev;
	}

	/* dang ky ham xu ly ngat */
	 idr_init(&spi_device_idr);	

	printk("Initialize vchar driver successfully\n");
        return 0;
failed_allocate_cdev:
	return ret;
       // spi_hw_exit(spi_drv->spi_hw);
failed_init_hw:
	return ret;
	//kfree(spi_drv.spi_hw);
failed_allocate_structure:
	device_destroy(spi_drv.spi_device_class, spi_drv.dev_num);
failed_create_device:
        class_destroy(spi_drv.spi_device_class);
failed_create_class:
        unregister_chrdev_region(spi_drv.dev_num, 1);
failed_register_devnum:
        return ret;
}

static void __exit spi_device_exit(void)
{

	/* huy dang ky xu ly ngat */
	idr_destroy(&spi_device_idr);

	/* huy dang ky entry point voi kernel */
	cdev_del(spi_drv.vcdev);

	/* giai phong thiet bi vat ly */
	spi_hw_exit(spi_drv.spi.hw);
	/* giai phong bo nho da cap phat cau truc du lieu cua driver */
	kfree(spi_drv.spi_hw);
	/* xoa bo device file */
        device_destroy(spi_drv.spi_device_class, spi_drv.dev_num);
        class_destroy(spi_drv.spi_device_class);


	/* giai phong device number */
        unregister_chrdev_region(spi_drv.dev_num,1);


	printk("Exit vchar driver\n");
}



module_init(spi_device_init);
module_exit(spi_device_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tinh Nguyen Van");
MODULE_DESCRIPTION("SPI device driver");
