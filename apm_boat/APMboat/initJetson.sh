#!/bin/sh
if [ $# -ne 1 ]; then
    echo "Usage: initJetson <boat>"
    exit 1
fi
sshpass -p "jetson" ssh nano@$1 mkdir /home/nano/APMboat
sshpass -p "jetson" ssh nano@$1 mkdir /home/nano/APMboat/APMHexFile
sshpass -p "jetson" scp uploadJetson.sh nano@$1:/home/nano/APMboat/APMHexFile/upload.sh
