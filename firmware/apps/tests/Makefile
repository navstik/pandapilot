APPNAME   	 = tests
PRIORITY    = SCHED_PRIORITY_MAX-5
STACKSIZE    = 4096

CSRCS			 = tests_main.c \
			   test_barometer.c \
			   test_battery.c \
			   test_gpio.c \
			   test_gyrometer.c \
			   test_led.c \
			   test_magnetometer.c \
			   test_accelerometer.c \
			   test_pwren.c \
			   test_usb.c \
			   test_pwm.c 

INCLUDES	 = $(TOPDIR)/arch/arm/src/stm32 $(TOPDIR)/arch/arm/src/common
include $(APPDIR)/mk/app.mk
