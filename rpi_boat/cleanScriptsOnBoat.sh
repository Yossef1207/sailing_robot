#!/bin/sh
if [ $# -ne 1 ]; then
    echo "Usage: cleanScriptsOnBoat <boat>"
    exit 1
fi
sshpass -p "mtecsail" ssh ubuntu@$1 'rm -r /home/ubuntu/APMboat/scripts'
