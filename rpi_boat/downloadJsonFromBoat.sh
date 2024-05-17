#!/bin/sh
if [ $# -ne 1 ]; then
    echo "Usage: downloadJsonFromBoat <boat>"
    exit 1
fi
sshpass -p "mtecsail" rsync -avhu -e ssh ubuntu@$1:/home/ubuntu/APMboat/log/*.json /home/developer/Code/rpi_boat/logs/$1
