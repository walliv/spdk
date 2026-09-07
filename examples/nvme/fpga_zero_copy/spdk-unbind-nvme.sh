#!/usr/bin/env sh

echo "0000:23:00.0" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/unbind
echo "0000:2d:00.0" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/unbind
echo "0000:63:00.0" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/unbind
echo "0000:64:00.0" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/unbind
