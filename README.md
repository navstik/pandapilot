PandaPilot
==========

OpenSource code for autopilot application. It makes use of ArduPilot and is a fork of PX4 code. PandaPilot currently supports NavStik hardware (based on STM32F4).


Setup and Compilation instructions
==================================

1. Install the following packages :
-----------------------------------
$ sudo apt-get install python-serial python-argparse openocd flex bison libncurses5-dev autoconf texinfo build-essential libftdi-dev libtool zlib1g-dev genromfs git-core wget gawk minicom libusb-1.0-0-dev


2. Install toolchain :
----------------------
$ wget https://launchpad.net/gcc-arm-embedded/4.6/4.6-2012-q2-update/+download/gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2
$ tar -vjxf gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2

Add your toolchain path to environmental variable PATH; either manually or by adding this line to ~/.bashrc or /etc/bash.bashrc
export PATH=/your/path/gcc-arm-none-eabi-4_6-2012q2/bin:\$PATH


3. Configure build environment :
--------------------------------

~/firmware/nuttx/tools$ ./configure.sh navstik/nsh

~/firmware$ make configure_navstik

Your ardupilot/config.mk file should look like this (replace paths with your actual paths) :
~/ArduPlane-path$ cat ../config.mk

# Select 'mega' for the 1280 APM1, 'mega2560' otherwise
BOARD = navstik

# HAL_BOARD determines default HAL target.
HAL_BOARD ?= HAL_BOARD_NAVSTIK

# The communication port used to communicate with the APM.
PORT = /dev/ttyACM0

# NAVSTIK app build: fill in the path to PandaPilot Firmware repository:
NAVSTIK_ROOT = /home/amit/work/navstik/pandapilot/firmware/

APPDIR = /home/amit/work/navstik/pandapilot/firmware/apps


4. Compile the code :
---------------------
~/ArduPlane-path$ make navstik
You should see navstik.bin in nuttx/Images directory.


5. Download the code :
----------------------
Download latest version of dfu-utils with this command : 
$ git clone git://gitorious.org/dfu-util/dfu-util.git
Follow the usual configure, make, make install cycle to compile and install dfu-util.

You will notice two small buttons and two jumper switches next to each other on the interface board.
From the jumpers, put number 2 to 'ON' position and press the buttone named 'NS'. This will put the board into DFU mode.
Issue the following command : 
$ sudo dfu-util --device 0483:df11 -a0 --dfuse-address 0x8000000 -D path/to/navstik-nuttx/Images/navstik.bin

Once the code is downloaded, bring switch 2 to OFF position and reset the board.

BTW, you need to insert a micro-SD card to run any of the sketches. Else it will not be able to create the .stg file, and will stop executing the script further.

Happy hacking! :)

[![githalytics.com alpha](https://cruel-carlota.pagodabox.com/3491608aa7e8d99999785bf74fa3b1a5 "githalytics.com")](http://githalytics.com/navstik/pandapilot)
