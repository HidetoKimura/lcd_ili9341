obj-m := lcd-ili9341.o
lcd-ili9341-y = ili9341_spi.o ezfont.o

all:
	make -C $(KERNEL_SRC) M=$(shell pwd) modules

clean:
	rm -rf *.o *.ko *.mod *.mod.c *.order *.symvers .*.cmd
