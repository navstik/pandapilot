# find the mk/ directory, which is where this makefile fragment
# lives. (patsubst strips the trailing slash.)
MK_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

include ../config.mk

ifeq ($(BOARD),navstik)
####################
# Navstik build
include $(MK_DIR)/navstik_core.mk
include $(MK_DIR)/navstik_targets.mk

else ifeq ($(BOARD),PX4)
####################
# PX4 build
include $(MK_DIR)/px4_core.mk
include $(MK_DIR)/px4_targets.mk

else
####################
# AVR and SITL build
include $(MK_DIR)/Arduino.mk
include $(MK_DIR)/targets.mk


endif

