#!/bin/sh
if [ $# -ne 1 ]; then
    echo "Usage: uploadScriptsToBoat <boat>"
    exit 1
fi
sshpass -p "mtecsail" scp -r ./scripts ubuntu@$1:/home/ubuntu/APMboat
