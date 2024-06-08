#!/bin/bash
readonly TARGET_IP="$1"
readonly PROGRAM="$2"
readonly APPDIR="$3"
readonly TARGET_DIR="/home/root"


# Must match startsPattern in tasks.json
echo "Deploying to target"

# kill gdbserver on target and delete old binary
 ssh -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa root@${TARGET_IP} "sh -c '/usr/bin/killall -q gdbserver; rm -rf ${TARGET_DIR}/${PROGRAM}  exit 0'"

# Send additional files
#ssh -q root@${TARGET_IP} [[ ! -f "labels.txt" ]] && scp labels.txt root@${TARGET_IP}:${TARGET_DIR} || echo "File Exits";
#ssh -q root@${TARGET_IP} [[ ! -d "yolov2_cam" ]] && scp yolov2_cam.tgz root@${TARGET_IP}:${TARGET_DIR} || echo "File Exits";
#ssh -q root@${TARGET_IP} [[ ! -d "yolov2_cam" ]] && ssh root@${TARGET_IP} "sh -c 'tar xvf yolov2_cam.tgz; rm -rfd yolov2_cam.tgz'" || echo "File Exits"
#ssh -q root@${TARGET_IP} [[ ! -f "/usr/lib64/libtvm_runtime.so" ]] && scp ./runtime/libtvm_runtime.so root@${TARGET_IP}:/usr/lib64;

# send the program to the target
ssh -q root@${TARGET_IP} [[ ! -d ${APPDIR} ]] && ssh root@${TARGET_IP} "sh -c 'mkdir ${APPDIR}'" || echo "Directory ${APPDIR} Exits";
scp ./build/${APPDIR} root@${TARGET_IP}:${TARGET_DIR}/${APPDIR}/${PROGRAM}

# Must match endsPattern in tasks.json
echo "Starting GDB Server on Target"

# start gdbserver on target
ssh -o HostKeyAlgorithms=+ssh-rsa -o PubkeyAcceptedAlgorithms=+ssh-rsa -t root@${TARGET_IP} "sh -c 'cd ${TARGET_DIR}; gdbserver localhost:3000 ${APPDIR}/${PROGRAM}'"