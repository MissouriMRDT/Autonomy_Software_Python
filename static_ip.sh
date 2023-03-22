#!/bin/bash

NETWORK=/etc/network/interfaces

if [ $1 = "on" ]; then
	echo "Turning On - Static IP"

	echo "# interfaces(5) file used by ifup(8) and ifdown(8)" > "${NETWORK}"
	echo "# Include files from /etc/network/interfaces.d:" >> "${NETWORK}"
	echo "source-directory /etc/network/interface" >> "${NETWORK}"
	echo "" >> "${NETWORK}"
	echo "# Set static ip on ethernet interface of 192.168.1.139" >> "${NETWORK}"
	echo "allow-hotplug eth0" >> "${NETWORK}"
	echo "iface eth0 inet static" >> "${NETWORK}"
	echo "        address 192.168.1.139" >> "${NETWORK}"
	echo "        netmask 255.255.255.0" >> "${NETWORK}"
	echo "        gateway 192.168.0.1" >> "${NETWORK}"
	echo "        dns-nameservers 4.4.4.4" >> "${NETWORK}"
	echo "        dns-nameservers 8.8.8.8" >> "${NETWORK}"
	echo "" >> "${NETWORK}"

	echo "Static IP has been turned ON"
elif [ $1 = "off" ]; then
	echo "Turning Off -  Static IP"

        echo "# interfaces(5) file used by ifup(8) and ifdown(8)" > "${NETWORK}"
        echo "# Include files from /etc/network/interfaces.d:" >> "${NETWORK}"
        echo "source-directory /etc/network/interface" >> "${NETWORK}"
        echo "" >> "${NETWORK}"
        echo "# Set static ip on ethernet interface of 192.168.1.139" >> "${NETWORK}"
        echo "#allow-hotplug eth0" >> "${NETWORK}"
        echo "#iface eth0 inet static" >> "${NETWORK}"
        echo "#        address 192.168.1.139" >> "${NETWORK}"
        echo "#        netmask 255.255.255.0" >> "${NETWORK}"
        echo "#        gateway 192.168.0.1" >> "${NETWORK}"
        echo "#        dns-nameservers 4.4.4.4" >> "${NETWORK}"
        echo "#        dns-nameservers 8.8.8.8" >> "${NETWORK}"
        echo "" >> "${NETWORK}"

	echo "Static IP has been turned OFF"
	echo "Make sure to disconnect from the network before trying to reach the internet!"
fi

if [ $1 = "on" ] || [ $1 = "off" ]; then
	echo "Restarting network-manager"
	service network-manager restart
	echo "Restarting networking"
	service networking restart
	echo "Restart Complete"
else
	echo "Error: Static Preference not given"
	echo "Example On: sudo ./static_ip.sh on"
	echo "Example Off: sudo ./static_ip.sh off"
fi
