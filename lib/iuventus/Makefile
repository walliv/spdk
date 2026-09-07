#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) Vladislav Válek
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

SO_VER := 1
SO_MINOR := 0

C_SRCS = iuventus.c
LIBNAME = iuventus

# The card is reached through the nfb device tree, so both libraries are hard requirements.
LOCAL_SYS_LIBS = -lnfb -lfdt

SPDK_MAP_FILE = $(abspath $(CURDIR)/spdk_iuventus.map)

include $(SPDK_ROOT_DIR)/mk/spdk.lib.mk
