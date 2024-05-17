sshpass -p "jetson" ssh nano@$1 mkdir /home/nano/APMboat
sshpass -p "jetson" ssh nano@$1 mkdir /home/nano/APMboat/APMHexFile
sshpass -p "jetson" scp uploadJetson.sh nano@$1:/home/nano/APMboat/APMHexFile/upload.sh
