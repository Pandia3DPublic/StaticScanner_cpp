#!/bin/bash
[ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"

echo -n "Run script? (y/n) "
read answer
if [ "$answer" == "${answer#[Yy]}" ] ;then
    echo "Aborting"
    exit
fi

# see https://importgeek.wordpress.com/2017/02/26/increase-usbfs-memory-limit-in-ubuntu/
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"/GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"/g' /etc/default/grub
sudo update-grub

