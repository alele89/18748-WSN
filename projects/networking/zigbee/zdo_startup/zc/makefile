# Platform name  cc2420DK, firefly2_1, firefly2_2, micaZ, stk600_128rfa1
PLATFORM = firefly3


# Target file name (without extension).
TARGET = zdo_start_zc

# Set the Port that you programmer is connected to 
PROGRAMMING_PORT = /dev/ttyUSB0 # programmer connected to serial device

# Set this such that the nano-RK directory is the base path
ROOT_DIR = ../../../../../

# Set platform specific defines 
# The following will be defined based on the PLATFORM variable:
# PROG_TYPE  (e.g. avrdude, or uisp)
# MCU (e.g. atmega32, atmega128, atmega1281) 
# RADIO (e.g. cc2420)
include $(ROOT_DIR)/include/platform.mk


SRC = $(TARGET).c
SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM_TYPE)/source/ff_basic_sensor.c
SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM_TYPE)/source/twi_base_calls.c

# Add extra source files. 
# For example:
#SRC += $(ROOT_DIR)/src/net/bmac/$(RADIO)/bmac.c


# Add extra includes files. 
# For example:
EXTRAINCDIRS =
#EXTRAINCDIRS += $(ROOT_DIR)/src/net/bmac
#EXTRAINCDIRS += $(ROOT_DIR)/include/zb_include

# The following defines are for ZBOSS stack compilation
ZB_PLATFORM_DEFINES=  -DZB_PLATFORM_FIREFLY3 -DZB_LIMITED_FEATURES -DZB_LIMITED_FEATURES2 

# This is where the ZBOSS sources and include directories are added
include $(ROOT_DIR)/include/platform/$(PLATFORM)/zb_common.mk

#  This is where the final compile and download happens
include $(ROOT_DIR)/include/platform/$(PLATFORM)/common.mk
