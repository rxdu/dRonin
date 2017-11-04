#
# Directory containing this makefile
#
MOD_UAVCAN_NODE	:=	$(dir $(lastword $(MAKEFILE_LIST)))

#
# Library sources
#
MOD_UAVCAN_NODE_SRC := $(shell find $(MOD_UAVCAN_NODE) -type f -name '*.cpp')

CPPSRC  += $(MOD_UAVCAN_NODE_SRC) 

EXTRAINCDIRS += $(MOD_UAVCAN_NODE)

