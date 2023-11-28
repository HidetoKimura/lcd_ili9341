# lcd_ili9341

# build kernel

In pc
~~~~
sudo apt install crossbuild-essential-arm64
git clone --depth=1 --branch rpi-6.1.y https://github.com/raspberrypi/linux
cd linux
export KERNEL=kernel8
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu- 
export INSTALL_MOD_PATH=../
make bcm2711_defconfig
make -j4 Image modules dtbs
make modules_install
~~~~

# build LKM

In pc
~~~~
export KERNEL_SRC=<kernel path>
git clone https://github.com/HidetoKimura/lcd_ili9341.git
cd lcd_ili9341
make
scp  lcd-ili9341.ko rpi@raspberrypi.local:
~~~~

In rpi
~~~~
insmod lcd-ili9341.ko 
rmmod lcd-ili9341 
~~~~

# build DTB

In pc
~~~~
cd dts
dtc -O dtb -o lcd-ili9341.dtbo lcd-ili9341.dts 
scp lcd-ili9341.dtbo rpi@raspberrypi.local:
~~~~

In rpi
~~~~
cp lcd-ili9341.dtbo /boot/overlay/
sync
~~~~

# modify /boot/config.txt

In rpi
~~~~
dtparam=spi=on
dtoverlay=lcd-ili9341
~~~~
