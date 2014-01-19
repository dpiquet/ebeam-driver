obj-m += ebeam.o

ccflags-y += -DCONFIG_INPUT_EBEAM_USB

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
