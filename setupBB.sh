#!/bin/bash

# Copyright (c) 2017,2018 Peter Budd. All rights reserved
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
# associated documentation files (the "Software"), to deal in the Software without restriction, 
# including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
# subject to the following conditions:
#     * The above copyright notice and this permission notice shall be included in all copies or substantial 
#       portions of the Software.
#     * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
#       BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
#       IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
#       WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
#       SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. THE AUTHORS AND COPYRIGHT HOLDERS, HOWEVER, 
#       ACCEPT LIABILITY FOR DEATH OR PERSONAL INJURY CAUSED BY NEGLIGENCE AND FOR ALL MATTERS LIABILITY 
#       FOR WHICH MAY NOT BE LAWFULLY LIMITED OR EXCLUDED UNDER ENGLISH LAW

# The script below takes a stock installation of Arch Linux Arm (for the Raspberry Pi) and adds all scripts
# and installs all required packages to turn the Pi into a Bell-Boy device. General instructions are: 
#    (a)    Install Arch Linux to an SD card https://archlinuxarm.org/platforms/armv6/raspberry-pi;
#    (b)    Copy this script to the FAT partition (for convenience)
#    (c)    Pop the SD card into your Raspberry Pi and boot it up with an internet connection (usb/ethernet adapter)
#    (d)    SSH into the Pi hostname alarmpi (or use IP address set up by your router), log in
#           (username alarm, password alarm), change to root user (su root, password root) 
#    (e)    run this script - /boot/setupBB.sh (assuming you placed the script into the FAT partition.

PORTABLEWIFINETWORK=Bell-Boy
HOSTNAME=Bell-Boy

if [ $(id -u) -ne 0 ]; then
    echo "You appear not to be running as root user (an assumption this script makes).  Try 'su root'.  Exiting"
    exit 1
fi

if [ $(cat /boot/config.txt | grep 'dtparam=i2c_arm=on' | wc -l) -eq 0 ]; then
  echo -e "\ndtparam=i2c_arm=on\ndtparam=i2c_arm_baudrate=400000\n" | tee -a /boot/config.txt
fi

#if [ $(cat /boot/config.txt | grep 'dtoverlay=pi3-disable-wifi' | wc -l) -eq 0 ]; then
#  echo -e "\ndtoverlay=pi3-disable-wifi\n" | tee -a /boot/config.txt
#fi

if [ $(cat /boot/config.txt | grep 'dtoverlay=pi3-disable-bt' | wc -l) -eq 0 ]; then
  echo -e "\ndtoverlay=pi3-disable-bt\n" | tee -a /boot/config.txt
fi

#if [ $(cat /boot/config.txt | grep 'dtoverlay=i2c-gpio' | wc -l) -eq 0 ]; then
#  echo -e "\ndtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=0\n" | tee -a /boot/config.txt
#fi

#if [ $(cat /boot/config.txt | grep 'dtoverlay=gpio-poweroff' | wc -l) -eq 0 ]; then
#  echo -e "\ndtoverlay=gpio-poweroff,gpiopin=18,active_low\n" | tee -a /boot/config.txt
#fi

#test to see if we should have BNO080 in reset
#if [ $(cat /boot/config.txt | grep 'gpio=24=pd' | wc -l) -eq 0 ]; then
#  echo -e "\ngpio=24=pd\n" | tee -a /boot/config.txt
#fi

#if [ $(cat /boot/config.txt | grep 'gpio=17=pd' | wc -l) -eq 0 ]; then
#  echo -e "\ngpio=17=pd\n" | tee -a /boot/config.txt
#fi

if [ $(cat /etc/modules-load.d/raspberrypi.conf | grep 'i2c-dev' | wc -l) -eq 0 ]; then
  echo -e "\ni2c-dev\n" | tee -a /etc/modules-load.d/raspberrypi.conf
fi

#this may fail on more recent firmwares but does not affect script
if [ $(cat /etc/modules-load.d/raspberrypi.conf | grep '\#snd-bcm2835' | wc -l) -eq 0 ]; then
  sed -i 's:snd-bcm2835:#snd-bcm2835:' /etc/modules-load.d/raspberrypi.conf
fi

if [ $(cat /etc/ssh/sshd_config | grep 'PermitRootLogin yes' | wc -l) -eq 0 ]; then
  echo -e "\nPermitRootLogin yes\n" | tee -a /etc/ssh/sshd_config
fi

timedatectl set-timezone Europe/London
hostnamectl set-hostname ${HOSTNAME}

pacman-key --init
pacman-key --populate archlinuxarm
pacman-key --refresh
pacman -Syu --noconfirm || { echo "Failed pacman -Syu.  Exiting"; exit 1; }
sync
pacman -S --noconfirm --needed wget samba bzip2 screen tmux hostapd crda samba base-devel i2c-tools unzip dhcp git || { echo "Failed downloading packages.  Exiting"; exit 1; }

echo -e '\nWIRELESS_REGDOM="GB"\n' >> /etc/conf.d/wireless-regdom

tee /etc/dhcpd.conf << HDHD 
ddns-update-style none;
default-lease-time 6000;
max-lease-time 28800;
authoritative;
subnet 10.0.0.0 netmask 255.255.255.0 {
range 10.0.0.2 10.0.0.10;
option broadcast-address 10.0.0.255;
option routers 10.0.0.1;
default-lease-time 6000;
max-lease-time 28800;
option domain-name "Bell-Boy";
}
HDHD

tee /etc/hostapd/hostapd.conf << HDHD || { echo "Unable to write hostapd config - exiting"; exit 1; }
interface=wlan0
ssid=Bell-Boy
country_code=GB
channel=11
hw_mode=g
wmm_enabled=1
ieee80211d=1
ieee80211n=1
HDHD

tee /etc/systemd/system/ap0.service <<HDHD || { echo "Unable to write dhcpd4 service file - exiting"; exit 1; }
[Unit]
Description=IPv4 DHCP server
After=network.target
ConditionPathExists=!/sys/class/net/wlan1

[Service]
Type=forking
PIDFile=/run/ap.pid
ExecStartPre=-/opt/vc/bin/tvservice -o
ExecStartPre=-/usr/bin/sed -i s:wlan1:wlan0: /etc/hostapd/hostapd.conf
ExecStartPre=-/usr/bin/ip link set dev wlan0 down
ExecStartPre=-/usr/bin/ip link set dev wlan0 up
ExecStartPre=-/usr/bin/iw dev wlan0 set power_save off
ExecStartPre=/usr/bin/sh -c "/usr/bin/ip addr add 10.0.0.1/24 broadcast 10.0.0.255 dev wlan0"
ExecStartPre=/usr/bin/systemctl start hostapd
ExecStart=/usr/bin/dhcpd -4 -q -cf /etc/dhcpd.conf -pf /run/ap.pid wlan0
ExecStopPost=/usr/bin/systemctl stop hostapd
ExecStopPost=-/usr/bin/ip link set dev wlan0 down
KillSignal=SIGINT

[Install]
WantedBy=multi-user.target
HDHD

tee /etc/systemd/system/ap0.timer <<HDHD || { echo "Unable to write dhcpd4 service file - exiting"; exit 1; }
[Unit]
Description=IPv4 DHCP server timer

[Timer]
OnBootSec=40

[Install]
WantedBy=multi-user.target
HDHD

## BindsTo=sys-subsystem-net-devices-wlp4s0.device
## After=sys-subsystem-net-devices-wlp4s0.device

tee /etc/systemd/system/ap1.service <<HDHD || { echo "Unable to write dhcpd4 service file - exiting"; exit 1; }
[Unit]
Description=IPv4 DHCP server
After=network.target
ConditionPathExists=/sys/class/net/wlan1

[Service]
Type=forking
PIDFile=/run/ap.pid
ExecStartPre=-/opt/vc/bin/tvservice -o
ExecStartPre=-/usr/bin/sed -i s:wlan0:wlan1: /etc/hostapd/hostapd.conf
ExecStartPre=-/usr/bin/ip link set dev wlan1 down
ExecStartPre=-/usr/bin/ip link set dev wlan1 up
ExecStartPre=-/usr/bin/iw dev wlan1 set power_save off
ExecStartPre=/usr/bin/sh -c "/usr/bin/ip addr add 10.0.0.1/24 broadcast 10.0.0.255 dev wlan1"
ExecStartPre=/usr/bin/systemctl start hostapd
ExecStart=/usr/bin/dhcpd -4 -q -cf /etc/dhcpd.conf -pf /run/ap.pid wlan1
ExecStopPost=/usr/bin/systemctl stop hostapd
ExecStopPost=-/usr/bin/ip link set dev wlan1 down
KillSignal=SIGINT

[Install]
WantedBy=multi-user.target
HDHD

tee /etc/systemd/system/ap1.timer <<HDHD || { echo "Unable to write dhcpd4 service file - exiting"; exit 1; }
[Unit]
Description=IPv4 DHCP server timer

[Timer]
OnBootSec=40

[Install]
WantedBy=multi-user.target
HDHD

wget https://github.com/joewalnes/websocketd/releases/download/v0.3.0/websocketd-0.3.0-linux_arm.zip
unzip websocketd-0.3.0-linux_arm.zip || { echo "Unable to extract websocketd executable. Exiting"; exit 1; }
mv ./websocketd /srv/http/ || { echo "Could not find websocketd executable. Exiting"; exit 1; }

tee /etc/systemd/system/bellboy.service <<HDHD
[Unit]
Description= Bellboy websocket service file to start on boot
After=network.target
[Service]
User=root
Type=forking
ExecStartPre=/usr/bin/sh -c "/srv/http/powermonitor.sh &"
ExecStart=/usr/bin/screen -S wrad -d -m sh -c "(/srv/http/websocketd --port=80 --staticdir=/srv/http/ /srv/http/grabber 2>&1) | tee -a /var/log/bellboy.log"
ExecStop=/usr/bin/screen -S wrad -X wrad
KillMode = control-group
TimeoutStopSec=0
Restart=always
RestartSec=5
[Install]
WantedBy=multi-user.target
HDHD

tee /etc/systemd/system/32kHz.service <<HDHD
[Unit]
Description= 32Khz PWM service file
[Service]
User=root
Type=oneshot
ExecStart=/root/32kHz
[Install]
WantedBy=multi-user.target
HDHD

cd ~

git clone https://github.com/BBUK/Bell-Boy.git || { echo "Unable to download Bell-Boy code"; exit 1; }
cd Bell-Boy
cp * /srv/http

mkdir -p /data/samples
mv samples/* /data/samples/
chown -R nobody.nobody /data
chmod -R 777 /data

# setup bcm2835
cd ~
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.57.tar.gz || { echo "Unable to download BCM2835 library. Exiting"; exit 1; }
tar xzvf bcm2835-1.57.tar.gz
cd bcm2835-1.57
./configure
make
make check
make install  || { echo "Unable to install BCM2835 library. Exiting"; exit 1; }

cd ~/Bell-Boy/grabber
gcc grabber.c -o grabber -lm -lbcm2835 || { echo "Unable to compile grabber.  Exiting."; exit 1; }
mv grabber /srv/http/
gcc 32kHz.c -o 32kHz -lbcm2835 || { echo "Unable to compile 32kHz. Exiting"; exit 1; }
mv 32kHz /root/

cd ~/Bell-Boy
mv images/Mounting.png /srv/http/
mv images/BBtaped.png /srv/http/
mv images/MainScreenGuide.png /srv/http/
mv images/Playback.png /srv/http/
mv images/FileDownload.png /srv/http/
mv images/StrokeByStroke.png /srv/http/
mv images/StrokeComparison.png /srv/http/
mv images/Settings.png /srv/http/

cd ~

tee /etc/samba/smb.conf <<HDHD
[global]
# workgroup = NT-Domain-Name or Workgroup-Name
workgroup = WORKGROUP

server string = Samba Server

printcap name = /etc/printcap
load printers = no
log file = /var/log/samba/%m.log
max log size = 50
map to guest = Bad User
security = user
dns proxy = no 

[BELL SAMPLES]
comment = Bell-Boy Recorded Samples
path = /data/samples
writeable = yes
guest ok = yes
create mask = 0777
directory mask = 0777
read only = no
browseable = yes
force user = root
public = yes
HDHD

systemctl enable bellboy
systemctl enable ap0.timer
systemctl enable ap1.timer
systemctl enable smb nmb
systemctl enable 32kHz

cp ~/Bell-Boy/images/imagepack.zip /srv/http
cd /srv/http
echo "Type password for the image pack.  If you don't have the password, just press enter (no images will be installed)"
unzip -q imagepack.zip

sync && sync && poweroff
#systemctl disable serial-getty@ttyAMA0.service 


