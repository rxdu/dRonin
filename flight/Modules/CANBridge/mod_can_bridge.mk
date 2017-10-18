#
# Directory containing this makefile
#
MOD_CAN_BRIDGE	:=	$(dir $(lastword $(MAKEFILE_LIST)))

#
# Library sources
#
MOD_CAN_BRIDGE_SRC := $(shell find $(MOD_CAN_BRIDGE) -type f -name '*.cpp')

CPPSRC  += $(MOD_CAN_BRIDGE_SRC) 

EXTRAINCDIRS += $(MOD_CAN_BRIDGE)

