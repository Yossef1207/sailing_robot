#!/bin/sh
if [ $# -ne 1 ]; then
    echo "Usage: connectToBoat <boat>"
    exit 1
fi
sshpass -p "mtecsail" ssh ubuntu@192.168.188.$1 
