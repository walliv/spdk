#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) Vladislav Válek

# rpc.py plugin for the RPCs fpga_zero_copy registers. With this directory on PYTHONPATH:
#   scripts/rpc.py --plugin fzc_rpc_plugin iuventus_reset_controllers

from spdk.rpc.client import print_json


def iuventus_reset_controllers(args):
    print_json(args.client.call('iuventus_reset_controllers'))


def spdk_rpc_plugin_initialize(subparsers):
    p = subparsers.add_parser('iuventus_reset_controllers',
                              help='NVMe Controller Reset of every drive the card queues are bound to')
    p.set_defaults(func=iuventus_reset_controllers)
