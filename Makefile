# Modified from:
# Dr. Derek Molloy, School of Electronic Engineering, Dublin City University,
# Ireland. URL: http://derekmolloy.ie/writing-a-linux-kernel-module-part-1-introduction/
PWD=$(shell pwd)
KERNEL_BUILD=/lib/modules/6.1.21-v7l+/build

obj-m+=gpiod_driver.o

all:
	make -C /lib/modules/6.1.21-v7l+/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/6.1.21-v7l+/build/ M=$(PWD) clean
