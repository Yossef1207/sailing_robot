#!/bin/sh
if [ $# -ne 1 ]; then
    echo "Usage: uploadScriptsToBoat <boat>"
    exit 1
fi
sshpass -p "mtecsail" scp -r ./init/APMboat ubuntu@$1:/home/ubuntu
