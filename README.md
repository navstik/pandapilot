navstik-pilot
=============

Ardupilot port for NavStik.


Setup and Compilation instructions
==================================

1. Install the following packages :
-----------------------------------
$ sudo apt-get install python-serial python-argparse openocd flex bison libncurses5-dev autoconf texinfo build-essential libftdi-dev libtool zlib1g-dev genromfs git-core wget


2. Install toolchain :
----------------------
$ wget https://launchpad.net/gcc-arm-embedded/4.6/4.6-2012-q2-update/+download/gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2
$ tar -vjxf gcc-arm-none-eabi-4_6-2012q2-20120614.tar.bz2

Add your toolchain path to environmental variable PATH; either manually or by adding this line to ~/.bashrc or /etc/bash.bashrc
export PATH=/your/path/gcc-arm-none-eabi-4_6-2012q2/bin:\$PATH


3. Configure build environment :
--------------------------------
~/nuttx-path/nuttx/tools$ ./configure.sh navstik/nsh
~/nuttx-path$ make configure_navstik

Your ardupilot/config.mk file should look like this (replace paths with your actual paths) :
~/ArduPlane-path$ cat ../config.mk

# Select 'mega' for the 1280 APM1, 'mega2560' otherwise
BOARD = navstik

# HAL_BOARD determines default HAL target.
HAL_BOARD ?= HAL_BOARD_NAVSTIK

# The communication port used to communicate with the APM.
PORT = /dev/ttyACM0

# PX4 app build: uncomment and fill in the path to PX4 Firmware repository:
# PX4_ROOT = /home/amit/work/navstik/PX4-firmware

# NAVSTIK app build: uncomment and fill in the path to Navstik Firmware repository:
NAVSTIK_ROOT = /home/amit/work/navstik/navstik-pilot/navstik-pilot/navstik-nuttx/

APPDIR = /home/amit/work/navstik/navstik-pilot/navstik-pilot/navstik-nuttx/apps

#ARDUINO = /home/amit/work/navstik/arduino-1.0.3


4. Compile the code :
---------------------

~/ArduPlane-path$ make navstik
You should see navstik.bin in nuttx/Images directory.

Happy hacking! :)
