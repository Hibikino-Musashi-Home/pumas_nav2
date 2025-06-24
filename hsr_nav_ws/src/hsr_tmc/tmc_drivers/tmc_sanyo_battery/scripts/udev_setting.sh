#!/bin/bash

function get_device_name() {
    ls /dev/ | grep ttyUSB | while read DEVICE_NAME
    do
        RESULT=`rosrun tmc_sanyo_battery sanyo_battery_status -d /dev/${DEVICE_NAME} 2>/dev/null`
        if [ "$RESULT" != "" ]; then
            echo $DEVICE_NAME
            break
        fi
    done
}

function get_device_info() {
    IFS_BACKUP=$IFS
    IFS=$'\n'
    udevadm info --query=all --name=/dev/`get_device_name` | while read line
    do
        if [ $(echo $line | grep -e 'SUBSYSTEM') ]; then
            SUBSYSTEM=`echo ${line} | sed -e 's/.*SUBSYSTEM=//'`
            echo 'SUBSYSTEM=="'$SUBSYSTEM'", '
        fi
        if [ $(echo $line | grep -e 'ID_VENDOR_ID') ]; then
            VENDOR_ID=`echo ${line} | sed -e 's/.*ID_VENDOR_ID=//'`
            echo 'ATTRS{idVendor}=="'$VENDOR_ID'", '
        fi
        if [ $(echo $line | grep -e 'ID_MODEL_ID') ]; then
            PRODUCT_ID=`echo ${line} | sed -e 's/.*ID_MODEL_ID=//'`
            echo 'ATTRS{idProduct}=="'$PRODUCT_ID'", '
        fi
    done
    IFS=$IFS_BACKUP
}

if [ "`id | grep sudo`" = "" ]; then
    echo -e "\033[0;31m[$0] please execute it by sudo user.\033[0;39m"
    exit 1
fi

DEVICE_INFO=`get_device_info`
if [ "$DEVICE_INFO" = "" ]; then
    echo -e "\033[0;31m[$0] cannot get device information.\033[0;39m"
    exit 1
fi

echo $DEVICE_INFO'OWNER="root", GROUP="dialout", MODE="0666", SYMLINK+="sanyo-battery"' > 10-sanyo-battery.rules
sudo cp 10-sanyo-battery.rules /etc/udev/rules.d/
rm 10-sanyo-battery.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
