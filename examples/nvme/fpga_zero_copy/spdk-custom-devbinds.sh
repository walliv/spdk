#!/usr/bin/env sh

echo "18ec c020" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/new_id
echo "8086 f1a5" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/new_id
echo "144d a80c" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/new_id
echo "1c5c 1639" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/new_id
#echo 0000:61:00.1 | sudo tee /sys/bus/pci/drivers/uio_pci_generic/bind
#echo 0000:01:00.1 | sudo tee /sys/bus/pci/drivers/uio_pci_generic/bind
echo 0000:41:00.1 | sudo tee /sys/bus/pci/drivers/uio_pci_generic/bind
#sudo setpci -s61:00.1 COMMAND=0x406
#sudo setpci -s01:00.1 COMMAND=0x406
sudo setpci -s41:00.1 COMMAND=0x406

echo '0000:23:00.0' | sudo tee /sys/bus/pci/devices/0000\:23\:00.0/driver/unbind
echo "0000:23:00.0" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/bind
echo '0000:2d:00.0' | sudo tee /sys/bus/pci/devices/0000\:2d\:00.0/driver/unbind
echo "0000:2d:00.0" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/bind
#echo '0000:63:00.0' | sudo tee /sys/bus/pci/devices/0000\:63\:00.0/driver/unbind
#echo "0000:63:00.0" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/bind
#echo '0000:64:00.0' | sudo tee /sys/bus/pci/devices/0000\:64\:00.0/driver/unbind
#echo "0000:64:00.0" | sudo tee /sys/bus/pci/drivers/uio_pci_generic/bind
sudo setpci -s23:00.0 COMMAND=0x406
sudo setpci -s2d:00.0 COMMAND=0x406
#sudo setpci -s63:00.0 COMMAND=0x406
#sudo setpci -s64:00.0 COMMAND=0x406
