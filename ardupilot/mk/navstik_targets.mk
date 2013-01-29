# NAVSTIK build is via external build system

ifneq ($(NAVSTIK_ROOT),)

# install rc.APM from the AP_HAL_NAVSTIK/scripts directory as /etc/init.d/rc.APM
HAL_NAVSTIK_DIR = $(realpath $(MK_DIR)/../libraries/AP_HAL_NAVSTIK)
NAVSTIK_EXTERNAL_SCRIPTS = $(HAL_NAVSTIK_DIR)/scripts/rc.APM~init.d/rc.APM

NAVSTIK_EXTERNAL = EXTERNAL_APPS=$(PWD) EXTERNAL_SCRIPTS=$(NAVSTIK_EXTERNAL_SCRIPTS)
NAVSTIK_MAKE = make -C $(NAVSTIK_ROOT) $(NAVSTIK_EXTERNAL) CONFIG_APM=y


navstik:
	$(NAVSTIK_MAKE)

navstik-clean: clean
	$(NAVSTIK_MAKE) clean
	$(NAVSTIK_MAKE) configure_navstik

navstik-upload:
	$(NAVSTIK_MAKE) upload

else

navstik:
	$(error ERROR: You need to add NAVSTIK_ROOT to your config.mk)

navstik-clean: navstik

navstik-upload: navstik

endif
