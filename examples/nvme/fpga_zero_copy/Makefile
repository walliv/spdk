#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2015 Intel Corporation.
#  All rights reserved.
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../../..)

APP=fpga_zero_copy
LDFLAGS+=-lfdt -lnfb

include $(SPDK_ROOT_DIR)/mk/nvme.libtest.mk

# ublk (io_uring-based userspace block device) is the only host-filesystem
# front-end.  Requires liburing and a Linux kernel >= 6.0 with
# CONFIG_BLK_DEV_UBLK=y.
SPDK_LIB_LIST += bdev ublk thread
CFLAGS         += -I$(SPDK_ROOT_DIR)/lib/ublk
SYS_LIBS       += -luring
