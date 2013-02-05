
# Select 'mega' for the 1280 APM1, 'mega2560' otherwise
BOARD = navstik

# HAL_BOARD determines default HAL target.
HAL_BOARD ?= HAL_BOARD_NAVSTIK

# The communication port used to communicate with the APM.
PORT = /dev/ttyACM0

# PX4 app build: uncomment and fill in the path to PX4 Firmware repository:
# PX4_ROOT = /home/amit/work/navstik/PX4-firmware

# NAVSTIK app build: uncomment and fill in the path to Navstik Firmware repository:
NAVSTIK_ROOT = /home/amit/work/navstik/navstik-pilot/navstik-nuttx/

APPDIR = /home/amit/work/navstik/navstik-pilot/navstik-nuttx/apps

#ARDUINO = /path/to/arduino-1.0.3
