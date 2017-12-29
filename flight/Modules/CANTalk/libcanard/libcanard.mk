#
# Rules to (help) build the UAVCANARD support.
#

#
# Directory containing this makefile
#
LIBCANARD_DIR	:=	$(dir $(lastword $(MAKEFILE_LIST)))

#
# Library sources
#
LIBCANARD_SRC := $(shell find $(LIBCANARD_DIR) -type f -name '*.c')

SRC  += $(LIBCANARD_SRC) 

EXTRAINCDIRS += $(LIBCANARD_DIR)
