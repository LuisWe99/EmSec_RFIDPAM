#!/bin/bash

sudo cp -rf sshd /etc/pam.d/sshd

sudo cp lib/MFRC522.h /usr/include

gcc -fPIC -fno-stack-protector -c pam_rfid.c -lbcm2835 -lpam

sudo ld -x --shared -o /lib/aarch64-linux-gnu/security/pam_rfid.so pam_rfid.o

rm pam_rfid.o
