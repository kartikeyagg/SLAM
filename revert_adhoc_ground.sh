#!/bin/bash

#sets the ground computer in managed (station) mode
#external adapter is set as default

#put your settings here...
adapter_internal="wlan0"
adapter_external="wlx503eaab7033d"
tx_power=2000

sudo service network-manager stop
sudo ip link set $adapter_internal down

if [ "$1" == "external" ]; then
    echo "Reverting external Wi-Fi adapter to default settings..."
    adapter_to_use=$adapter_external
else
    echo "Reverting internal Wi-Fi adapter to default settings..."
    adapter_to_use=$adapter_internal
    sudo ip link set $adapter_to_use down
fi

sudo iwconfig $adapter_to_use mode managed

sudo ip link set $adapter_to_use up

sudo iw dev $adapter_to_use set txpower fixed $tx_power

sudo service network-manager start

echo "Wi-Fi adapter reverted to default settings."
