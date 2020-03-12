install stlink:

1.git clone https://github.com/vdudouyt/stm8flash.git
2.cd stm8flash
3.make
#error:
        pgm.h:7:11: fatal error: libusb.h: No such file or directory

sudo apt-get install libusb-1.0-0-dev
sudo apt install pkg-config
re-run make

4.sudo make install
5.stm8flash -c stlinkv2 -p stm8s103 -w blinky.ihx

stlink usb permision rule:

sudo vi /etc/udev/rules.d/45-usb-stlink-v2.rules

#FT232
ATTRS{idProduct}=="6014", ATTRS{idVendor}=="0403", MODE="666", GROUP="plugdev"

#FT2232
ATTRS{idProduct}=="6010", ATTRS{idVendor}=="0403", MODE="666", GROUP="plugdev"

#FT230X
ATTRS{idProduct}=="6015", ATTRS{idVendor}=="0403", MODE="666", GROUP="plugdev"

#STLINK V1
ATTRS{idProduct}=="3744", ATTRS{idVendor}=="0483", MODE="666", GROUP="plugdev"

#STLINK V2
ATTRS{idProduct}=="3748", ATTRS{idVendor}=="0483", MODE="666", GROUP="plugdev"

After that you can reboot or try

sudo service udev restart
