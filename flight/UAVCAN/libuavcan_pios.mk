#
# Rules to (help) build the UAVCAN support.
#

#
# Directory containing this makefile
#
PIOS_UAVCAN_LIB	:=	$(dir $(lastword $(MAKEFILE_LIST)))

LIBUAVCAN_DIR := $(PIOS_UAVCAN_LIB)/libuavcan
PIOS_DRIVER_DIR := $(PIOS_UAVCAN_LIB)/pios_driver

#
# Library sources
#
LIBUAVCAN_SRC := $(shell find $(LIBUAVCAN_DIR)/src -type f -name '*.cpp')
PIOSUAVCAN_SRC := $(shell find $(PIOS_DRIVER_DIR) -type f -name '*.cpp')

#LIBUAVCAN_INC := $(LIBUAVCAN_DIR)/include
#PIOSUAVCAN_INC := $(PIOS_UAVCAN_LIB)/pios_driver

CPPSRC  += $(LIBUAVCAN_SRC) 
CPPSRC  += $(PIOSUAVCAN_SRC)

EXTRAINCDIRS += $(LIBUAVCAN_DIR)/include
EXTRAINCDIRS += $(PIOS_DRIVER_DIR) 

CPPFLAGS += -DUAVCAN_CPP_VERSION=UAVCAN_CPP11 -std=c++11 -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit

#
# DSDL compiler executable
#
LIBUAVCAN_DSDLC := $(LIBUAVCAN_DIR)/dsdl_compiler/libuavcan_dsdlc

#
# Standard DSDL definitions
#
UAVCAN_DSDL_DIR := $(PIOS_UAVCAN_LIB)/dsdl/uavcan
UAVCAN_PIXCAR_DIR := $(PIOS_UAVCAN_LIB)/pixcar

# Invoke DSDL compiler and add its default output directory to the include search path
UAVCAN_TYPES_INC := $(PIOS_UAVCAN_LIB)/uavcantypes
$(info $(shell $(LIBUAVCAN_DSDLC) -O $(UAVCAN_TYPES_INC) $(UAVCAN_DSDL_DIR)))
$(info $(shell $(LIBUAVCAN_DSDLC) -O $(UAVCAN_TYPES_INC) $(UAVCAN_PIXCAR_DIR)))
EXTRAINCDIRS += $(UAVCAN_TYPES_INC)

#
# PIOS device library source and includes
#
#EXTRAINCDIRS		+=	$(PIOS_DEVLIB)/inc

