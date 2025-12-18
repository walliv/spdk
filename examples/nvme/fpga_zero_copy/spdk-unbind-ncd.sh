#!/usr/bin/env sh

echo "0000:61:00.1" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/unbind
sudo setpci -s61:00.1 COMMAND=0x00
