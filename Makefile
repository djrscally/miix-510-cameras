obj-m += ov2680.o
obj-m += ov5648.o
obj-m += tps68470-regulator.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

