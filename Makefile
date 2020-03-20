PWD := $(shell pwd)

src-file = ./spi_drv.c
obj-m += spi_drv.o

CROSS := /home/boss/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
KERNEL := /home/boss/linux/

all:
	make ARCH=arm CROSS_COMPILE=$(CROSS) -C $(KERNEL) SUBDIRS=$(PWD) modules
clean:
	make -C $(KERNEL) SUBDIRS=$(PWD) clean

