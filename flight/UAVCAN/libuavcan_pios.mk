#
# Rules to (help) build the UAVCAN support.
#

#
# Directory containing this makefile
#
PIOS_UAVCAN_MOD	:=	$(dir $(lastword $(MAKEFILE_LIST)))

LIBUAVCAN_DIR := $(PIOS_UAVCAN_MOD)/libuavcan
PIOS_DRIVER_DIR := $(PIOS_UAVCAN_MOD)pios_driver

#
# Library sources
#
LIBUAVCAN_SRC := $(shell find $(LIBUAVCAN_DIR)/src -type f -name '*.cpp')
PIOSUAVCAN_SRC := $(shell find $(PIOS_DRIVER_DIR) -type f -name '*.cpp')

#LIBUAVCAN_INC := $(LIBUAVCAN_DIR)/include
#PIOSUAVCAN_INC := $(PIOS_UAVCAN_MOD)/pios_driver

CPPSRC  += $(LIBUAVCAN_SRC) 
CPPSRC  += $(PIOSUAVCAN_SRC)

EXTRAINCDIRS += $(LIBUAVCAN_DIR)/include
EXTRAINCDIRS += $(LIBUAVCAN_DIR)/include/dsdlc_generated
EXTRAINCDIRS += $(PIOS_UAVCAN_MOD)／pios_driver

CPPFLAGS += -DUAVCAN_CPP_VERSION=UAVCAN_CPP11 -pedantic -std=c++11 -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit

#
# DSDL compiler executable
#
#LIBUAVCAN_DSDLC := $(LIBUAVCAN_DIR)/dsdl_compiler/libuavcan_dsdlc

#
# Standard DSDL definitions
#
#UAVCAN_DSDL_DIR := $(UAVCAN_DIR)/dsdl/uavcan


#
# PIOS device library source and includes
#
#EXTRAINCDIRS		+=	$(PIOS_DEVLIB)/inc

