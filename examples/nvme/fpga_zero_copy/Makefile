#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2015 Intel Corporation.
#  All rights reserved.
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk
include $(SPDK_ROOT_DIR)/mk/spdk.modules.mk

APP = fpga_zero_copy

C_SRCS := $(APP:%=%.c)

# The app framework brings up bdev_nvme and the ublk target for -U from an in-memory JSON config.
SPDK_LIB_LIST = $(ALL_MODULES_LIST) event event_bdev event_ublk

# The card bring-up, the NVMe queue pairs that live in its BARs and the teardown ordering all
# come from the iuventus library; libnfb and libfdt are its own dependencies, named here because
# a static library carries no link line of its own.
SPDK_LIB_LIST  += iuventus
SYS_LIBS       += -lnfb -lfdt -luring

include $(SPDK_ROOT_DIR)/mk/spdk.app.mk
