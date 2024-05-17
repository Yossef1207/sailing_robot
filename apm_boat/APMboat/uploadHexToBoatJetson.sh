#!/bin/sh
if [ $# -ne 1 ]; then
    echo "Usage: uploadHexToBoatjetson <boat>"
    exit 1
fi
sshpass -p "jetson" scp /tmp/APMboat.build/APMboat.hex nano@$1:/home/nano/APMboat/APMHexFile/APMboat.hex
sshpass -p "jetson" ssh nano@$1 /home/nano/APMboat/APMHexFile/upload.sh
