Purpose: 
The purpose of this programm is to read a single RFID tag, that will be used for Two-Factor-Authorisation (2FA).
The first factor of the 2FA is the password and the second is a unique RFID Tag.
Program will create a pluggable authentication module (PAM). The PAM will be used whenever a remote SSH connection is being established.

Prerequisites:
sudo apt-get install libpam0g-dev
http://www.airspayce.com/mikem/bcm2835/bcm2835-1.71.tar.gz

Usage:
./buildPam.sh
Executing this Shellscript will setup your machine, it will override the file in /etc/pam.d/sshd and copy neccesary files to /lib/aarch64.../security

Environment:
Raspberry PI4 using Raspberry Pi OS
RFID Reader with MFRC522 chip

Helpful repositories + authors:
https://github.com/beatgammit/simple-pam
https://github.com/paguz/RPi-RFID
