#!/bin/bash

#sets the ground computer in ac-hoc mode
#external adapter is set as default

#put your settings here...
adapter_internal="wlan0"
adapter_external="wlx503eaab7033d"
network_name="swarmmesh"
#cell_id=C6:32:96:BC:58:29 	#comment out to randomly assign cell id
network_key=1234567890
network_channel=3
host_id=117
tx_power=2000

sudo service network-manager stop
sudo ip link set $adapter_internal down

if [ "$1" == "external" ]; then
    echo "Setting the external Wi-Fi adapter in ad-hoc mode..."
    adapter_to_use=$adapter_external
else
    echo "Setting the internal Wi-Fi adapter in ad-hoc mode..."
    adapter_to_use=$adapter_internal
    sudo ip link set $adapter_to_use down
fi

sudo iwconfig $adapter_to_use mode ad-hoc
sudo iwconfig $adapter_to_use channel $network_channel
sudo iwconfig $adapter_to_use essid $network_name
if [ $cell_id ]; then
	echo "Setting hard coded cell id..."
	sudo iwconfig $adapter_to_use ap $cell_id
else
    echo "Setting random cell id..."
    sudo iwconfig $adapter_to_use ap any
fi
sudo iwconfig $adapter_to_use key $network_key

sudo ip link set $adapter_to_use up

sudo ip addr add 192.168.4.$host_id/24 dev $adapter_to_use
sudo iw dev $adapter_to_use set txpower fixed $tx_power

sudo route add -net 224.0.0.0 netmask 255.255.255.0 $adapter_to_use

echo "Wi-Fi adapter set into ad-hoc mode."

bash set_adhoc_ground2.sh
