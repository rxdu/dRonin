#
# Rules to (help) build the UAVCAN support.
#

#
# Directory containing this makefile
#
PIOS_UAVCAN_MOD	:=	$(dir $(lastword $(MAKEFILE_LIST)))
LIBUAVCAN_DIR := $(PIOS_UAVCAN_MOD)/libuavcan

#
# Library sources
#
LIBUAVCAN_SRC := $(shell find $(LIBUAVCAN_DIR)/src -type f -name '*.cpp')
PIOSUAVCAN_SRC := $(shell find $(PIOS_UAVCAN_MOD)/pios_driver/ -type f -name "*.cpp")

LIBUAVCAN_INC := $(LIBUAVCAN_DIR)/include
PIOSUAVCAN_INC := $(LIBUAVCAN_DIR)

CPPSRC  += $(LIBUAVCAN_SRC) $(PIOSUAVCAN_SRC)
UINCDIR += $(LIBUAVCAN_INC) $(PIOSUAVCAN_INC)

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

