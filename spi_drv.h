#define SPI_DRV_H
#define SPI_DRV_H

#include <linux/types.h>

#define SPI_DEVICE_NAME_SIZE 32

#define SPI_DEVICE_IOCTL_MAGIC		'k'

#define SPI_DEVICE_SLAVE		0x01
#define SPI_DEVICE_CPHA			0x02
#define SPI_DEVICE_CPOL			0x04
#define SPI_DEVICE_NO_CS		0x08
#define SPI_DEVICE_CS_HIGH		0x10
#define SPI_DEVICE_LSB_FIRST		0x20
#define SPI_DEVICE_RM			0x40
#define SPI_DEVICE_TM			0x80
#define SPI_DEVICE_TRM			0xC0


#define SPI_DEVICE_MSGSIZE(N) \
	((((N) * (sizeof(struct spi_device_ioctl_transfer))) \
	< (1 << _IOC_SIZEBITS)) \
	? ((N) * (sizeof(struct spi_device_ioctl_transfer))) : 0)

#define SPI_DEVICE_MESSAGE(N)		__IOC(SPI_DEVICE_IOCTL_MAGIC, 0, \
						char[SPI_DEVICE_MSGSIZE(N)])

#define SPI_DEVICE_RD_BITS_PER_WORD	_IOR(SPI_DEVICE_IOCTL_MAGIC, 1, __u8)
#define SPI_DEVICE_WR_BITS_PER_WORD	_IOW(SPI_DEVICE_IOCTL_MAGIC, 1, __u8)

#define SPI_DEVICE_RD_MODE		_IOR(SPI_DEVICE_IOCTL_MAGIC, 2, __u32)
#define SPI_DEVICE_WR_MODE		_IOW(SPI_DEVICE_IOCTL_MAGIC, 2, __u32)

#define SPI_DEVICE_RD_MAX_SPEED		_IOR(SPI_DEVICE_IOCTL_MAGIC, 3, __u32)
#define SPI_DEVICE_WR_MAX_SPEED		_IOW(SPI_DEVICE_IOCTL_MAGIC, 3, __u32)

#define SPI_DEVICE_RD_TX_ACTUAL_LENGTH	_IOR(SPI_DEVICE_IOCTL_MAGIC, 4, __u32)
#define SPI_DEVICE_RD_RX_ACTUAL_LENGTH	_IOR(SPI_DEVICE_IOCTL_MAGIC, 5, __u32)


struct spi_device_ioctl_transfer {
	__u64	tx_buf;
	__u64	rx_buf;

	__u32	tx_actual_length;
	__u32	rx_actual_length;

	__u32	mode;
	__u32	max_speed;
	__u8	bits_per_word;
};


/* struct spi_device_message {
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
}; */

struct spi_device_id {
	char name[SPI_DEVICE_NAME_SIZE];
	kernel_ulong_t driver_data;
};

struct spi_dev_device {
	struct device dev;
	struct spi_device *slave_device;
	char modalias[SPI_DEVICE_NAME_SIZE];
};

struct spi_device_driver {
	const struct spi_device_id *id_table;
	int (*probe)(struct spi_dev_device *spi);
	int (*remove)(struct spi_dev_device *spi);
	struct device_driver driver;
};

extern struct spi_device_message *spi_device_msg_alloc(struct spi_device *slave_device);
extern void spi_device_msg_remove(struct spi_device *slave_device);
extern int spi_device_transfer_msg(struct spi_device *slave_device);

extern int spi_device_register_driver(struct spi_device_driver *sdrv);
extern void spi_device_unregister_driver(struct spi_device_driver *sdrv);
extern int devm_spi_device_register_slave_device(struct device *dev,
					struct spi_device *slave_device);
extern void spi_device_unregister_device(struct spi_dev_device *dev);

static inline void *spi_device_get_drv_data(struct spi_dev_device *sdev)
{
	return dev_get_drvdata(&sdev->dev);
}

static inline void spi_device_set_drv_data(struct spi_dev_device *sdev,
					    void *data)
{
	return dev_set_drvdata(&sdev->dev, data);
}

static inline void *spi_device_get_slave_device_data(struct spi_device *slave_device)
{
	return dev_get_drvdata(&slave_device->dev);
}

static inline void spi_device_set_slave_device_data(struct spi_device *slave_device,
					    void *data)
{
	return dev_set_drvdata(&slave_device->dev, data);
}

extern struct spi_device *spi_device_alloc_slave_device(struct device *dev,
					      unsigned int size);
static inline struct spi_dev_device *to_spi_device_dev(struct device *dev)
{
	return container_of(dev, struct spi_dev_device, dev);
}

static inline struct spi_device_driver *to_spi_device_drv(struct device_driver *drv)
{
	return container_of(drv, struct spi_device_driver, driver);
}


};
