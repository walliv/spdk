#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) Vladislav Válek
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../../..)

APP=groupby

include $(SPDK_ROOT_DIR)/mk/nvme.libtest.mk

# The card bring-up, the NVMe queue pairs that live in its BARs and the teardown ordering all
# come from the iuventus library; libnfb and libfdt are its own dependencies, named here because
# a static library carries no link line of its own. json is only needed for --verify's sidecar
# parsing but is small and dependency-free enough to always link.
SPDK_LIB_LIST  += iuventus json
SYS_LIBS       += -lnfb -lfdt
