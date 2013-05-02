#
# Top-level Makefile for building Navstik firmware images.
#
# Ideally we should generate Navstik link kits via the NuttX
# 'make export' mechanism.
#
#

#
# Some useful paths.
#
export NAVSTIKBASE		= $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
export NUTTX_SRC	 = $(NAVSTIKBASE)/nuttx
export NUTTX_APPS	 = $(NAVSTIKBASE)/apps
export MAVLINK_SRC	 = $(NAVSTIKBASE)/mavlink
export ROMFS_SRC	 = $(NAVSTIKBASE)/ROMFS
export IMAGE_DIR	 = $(NAVSTIKBASE)/Images

#
# Tools
#
MKFW			 = $(NAVSTIKBASE)/Tools/navstik_mkfw.py
UPLOADER		 = $(NAVSTIKBASE)/Tools/px_uploader.py

#
# What are we currently configured for?
#
CONFIGURED		 = $(NAVSTIKBASE)/.configured
ifneq ($(wildcard $(CONFIGURED)),)
export TARGET		:= $(shell cat $(CONFIGURED))
endif

#
# What we will build
#
FIRMWARE_BUNDLE		 = $(IMAGE_DIR)/$(TARGET).bundle
FIRMWARE_BINARY		 = $(IMAGE_DIR)/$(TARGET).bin
FIRMWARE_PROTOTYPE	 = $(IMAGE_DIR)/$(TARGET).prototype

#
# Debugging
#
MQUIET			 = --no-print-directory
#MQUIET			 = --print-directory

all:			$(FIRMWARE_BUNDLE)

#
# Generate a wrapped bundle file from the built binary
#
$(FIRMWARE_BUNDLE):	$(FIRMWARE_BINARY) $(MKFW) $(FIRMWARE_PROTOTYPE)
	@echo Generating $@
	@$(MKFW) --prototype $(FIRMWARE_PROTOTYPE) \
		--git_identity $(NAVSTIKBASE) \
		--image $(FIRMWARE_BINARY) > $@

#
# Build the firmware binary.
#
.PHONY:			$(FIRMWARE_BINARY)
$(FIRMWARE_BINARY):	setup_$(TARGET) configure-check
	@echo Building $@ for $(TARGET)
	@make -C $(NUTTX_SRC) -r $(MQUIET) all
	@cp $(NUTTX_SRC)/nuttx.bin $@

#
# The 'configure' targets select one particular firmware configuration
# and makes it current.
#
configure_navstik:
	@echo Configuring for Navstik
	@make -C $(NAVSTIKBASE) distclean
	@cd $(NUTTX_SRC)/tools && /bin/sh configure.sh navstik/nsh
	@echo navstik > $(CONFIGURED)

configure-check:
ifeq ($(wildcard $(CONFIGURED)),)
	@echo
	@echo "Not configured - use 'make configure_navstik' first"
	@echo
	@exit 1
endif


#
# Per-configuration additional targets
#
.PHONY:			navstik_setup
setup_navstik:
	@echo Generating ROMFS
	@make -C $(ROMFS_SRC) all

# fake target to make configure-check happy if TARGET is not set
setup_:

#
# Firmware uploading.
#

# serial port defaults by operating system.
SYSTYPE			 = $(shell uname)
ifeq ($(SYSTYPE),Darwin)
SERIAL_PORTS		?= "/dev/tty.usbmodemPX1,/dev/tty.usbmodemPX2,/dev/tty.usbmodemPX3,/dev/tty.usbmodemNAVSTIK,/dev/tty.usbmodem1,/dev/tty.usbmodem2,/dev/tty.usbmodem3,/dev/tty.usbmodem4"
endif
ifeq ($(SYSTYPE),Linux)
SERIAL_PORTS		?= "/dev/ttyACM5,/dev/ttyACM4,/dev/ttyACM3,/dev/ttyACM2,/dev/ttyACM1,/dev/ttyACM0"
endif
ifeq ($(SERIAL_PORTS),)
SERIAL_PORTS		 = "\\\\.\\COM32,\\\\.\\COM31,\\\\.\\COM30,\\\\.\\COM29,\\\\.\\COM28,\\\\.\\COM27,\\\\.\\COM26,\\\\.\\COM25,\\\\.\\COM24,\\\\.\\COM23,\\\\.\\COM22,\\\\.\\COM21,\\\\.\\COM20,\\\\.\\COM19,\\\\.\\COM18,\\\\.\\COM17,\\\\.\\COM16,\\\\.\\COM15,\\\\.\\COM14,\\\\.\\COM13,\\\\.\\COM12,\\\\.\\COM11,\\\\.\\COM10,\\\\.\\COM9,\\\\.\\COM8,\\\\.\\COM7,\\\\.\\COM6,\\\\.\\COM5,\\\\.\\COM4,\\\\.\\COM3,\\\\.\\COM2,\\\\.\\COM1,\\\\.\\COM0"
endif

upload:		$(FIRMWARE_BUNDLE) $(UPLOADER)
	@python -u $(UPLOADER) --port $(SERIAL_PORTS) $(FIRMWARE_BUNDLE)

#
# JTAG firmware uploading with OpenOCD
#
ifeq ($(JTAGCONFIG),)
JTAGCONFIG=interface/olimex-jtag-tiny.cfg
endif

upload-jtag-navstik:
	@echo Attempting to flash Navstik board via JTAG
	@openocd -f $(JTAGCONFIG) -f ../Bootloader/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx/nuttx" -c "flash write_image erase ../Bootloader/navstik_bl.elf" -c "reset run" -c shutdown

#
# Hacks and fixups
#

ifeq ($(SYSTYPE),Darwin)
# PATH inherited by Eclipse may not include toolchain install location 
export PATH			 := $(PATH):/usr/local/bin
endif

#
# Cleanup targets.  'clean' should remove all built products and force
# a complete re-compilation, 'distclean' should remove everything 
# that's generated leaving only files that are in source control.
#
.PHONY:	clean upload-jtag-navstik
clean:
	@make -C $(NUTTX_SRC) -r $(MQUIET) distclean
	@make -C $(ROMFS_SRC) -r $(MQUIET) clean

.PHONY:	distclean
distclean:
	@rm -f $(CONFIGURED)
	@make -C $(NUTTX_SRC) -r $(MQUIET) distclean
	@make -C $(ROMFS_SRC) -r $(MQUIET) distclean

