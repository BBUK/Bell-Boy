#!/bin/bash

# Copyright (c) 2017 Peter Budd. All rights reserved
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
#    (e)    run this script - /boot/setup.sh (assuming you placed the script into the FAT partition.

# This script also configures some GPIO pins of the Pi.  GPIO 17 (pulled down by the DTS compiled below)
# is used to detect whether the power button has been pressed on the device (i.e. when it has been switched on)
# so that it can be shut down.  The GPIO power down overlay is configured to use GPIO 18 to indicate power
# up.  The standard PCB for the device wires things up to these GPIOs.   

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

#if [ $(cat /boot/config.txt | grep 'dtoverlay=pi3-miniuart-bt' | wc -l) -eq 0 ]; then
#  echo -e "\ndtoverlay=pi3-miniuart-bt\n" | tee -a /boot/config.txt
#fi

#if [ $(cat /boot/config.txt | grep 'dtoverlay=i2c-gpio' | wc -l) -eq 0 ]; then
#  echo -e "\ndtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=0\n" | tee -a /boot/config.txt
#fi

if [ $(cat /boot/config.txt | grep 'dtoverlay=gpio-poweroff' | wc -l) -eq 0 ]; then
  echo -e "\ndtoverlay=gpio-poweroff,gpiopin=18,active_low\n" | tee -a /boot/config.txt
fi

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

#cp /etc/netctl/examples/wireless-wpa /etc/netctl/
#sed -i "s/MyNetwork/${MAINWIFINETWORK}/" /etc/netctl/wireless-wpa
#sed -i "s/'WirelessKey'/${MAINWIFIPASSWORD}/" /etc/netctl/wireless-wpa

timedatectl set-timezone Europe/London
hostnamectl set-hostname ${HOSTNAME}

#netctl start wireless-wpa

pacman-key --init
pacman -Syu --noconfirm || { echo "Failed pacman -Syu.  Exiting"; exit 1; }
sync
pacman -S --noconfirm --needed wget dtc samba bzip2 screen tmux hostapd crda samba base-devel i2c-tools unzip dhcp git || { echo "Failed downloading packages.  Exiting"; exit 1; }

#Load up CRDA, makes wifi more reliable in the first min or two
echo -e '\nWIRELESS_REGDOM="GB"\n' >> /etc/conf.d/wireless-regdom

tee /etc/dhcpd.conf << HDHD 
ddns-update-style none;
default-lease-time 600;
max-lease-time 7200;
authoritative;
subnet 10.0.0.0 netmask 255.255.255.0 {
range 10.0.0.2 10.0.0.10;
option broadcast-address 10.0.0.255;
option routers 10.0.0.1;
default-lease-time 600;
max-lease-time 7200;
option domain-name "Bell-Boy";
}
HDHD

tee /etc/hostapd/hostapd.conf << HDHD || { echo "Unable to write hostapd config - exiting"; exit 1; }
interface=wlan0
ssid=Bell-Boy
hw_mode=g
channel=6
auth_algs=1
wmm_enabled=0
HDHD

tee /etc/systemd/system/ap.service <<HDHD || { echo "Unable to write dhcpd4 service file - exiting"; exit 1; }
[Unit]
Description=IPv4 DHCP server
After=network.target

[Service]
Type=forking
PIDFile=/run/ap.pid
ExecStartPre=-/usr/bin/tvservice -o
ExecStartPre=-/usr/bin/ip link set dev wlan0 down
#ip addr flush dev wlan0
ExecStartPre=-/usr/bin/ip link set dev wlan0 up
ExecStartPre=-/usr/bin/iw dev wlan0 set power_save off
ExecStartPre=/usr/bin/sh -c "sleep 5 && /usr/bin/ip addr add 10.0.0.1/24 broadcast 10.0.0.255 dev wlan0"
ExecStartPre=/usr/bin/systemctl start hostapd
ExecStart=/usr/bin/dhcpd -4 -q -cf /etc/dhcpd.conf -pf /run/ap.pid
ExecStopPost=/usr/bin/systemctl stop hostapd
ExecStopPost=-/usr/bin/ip link set dev wlan0 down
KillSignal=SIGINT

[Install]
WantedBy=multi-user.target
HDHD

wget https://github.com/joewalnes/websocketd/releases/download/v0.3.0/websocketd-0.3.0-linux_arm.zip
unzip websocketd-0.3.0-linux_arm.zip || { echo "Unable to extract websocketd executable. Exiting"; exit 1; }
mv ./websocketd /srv/http/ || { echo "Could not find websocketd executable. Exiting"; exit 1; }

#git clone https://github.com/google/pywebsocket.git || { echo "Unable to download pywebsocket.  Exiting"; exit 1; }

#cd pywebsocket

#python2 setup.py build
#python2 setup.py install || { echo "Unable to install Pywebsocket.  Exiting"; exit 1; }

#cp build/lib/mod_pywebsocket/standalone.py /srv/http/ || { echo "Could not find standalone.py.  Exiting"; exit 1; }

tee /etc/systemd/system/bellboy.service <<HDHD
[Unit]
Description= Websocket script to start on boot
After=network.target
[Service]
User=root
Type=forking
ExecStartPre=/usr/bin/sh -c "/srv/http/powermonitor.sh &"
ExecStart=/usr/bin/screen -S wrad -d -m sh -c "(/srv/http/websocketd --port=80 --staticdir=/srv/http/ /srv/http/bb_dcmimu 200 500 4 2>&1) | tee -a /var/log/bellboy.log"
ExecStop=/usr/bin/screen -S wrad -X wrad
KillMode = control-group
TimeoutStopSec=0
Restart=always
RestartSec=5
[Install]
WantedBy=multi-user.target
HDHD

#cd ~
#git clone https://github.com/RTIMULib/RTIMULib2.git
#cd RTIMULib2/Linux/python
#python2 setup.py build || { echo "Unable to install RTIMULib2 files"; exit 1; }
#python2 setup.py install

#cd ../RTIMULibCal
#make || { echo "Unable to make RTIMULibCal"; exit 1; }
#make install || { echo "Unable to install RTIMULibCal"; exit 1; }

cd ~

git clone https://github.com/BBUK/Bell-Boy.git || { echo "Unable to download Bell-Boy code"; exit 1; }
cd Bell-Boy
cp * /srv/http

mkdir -p /data/samples
mv samples/* /data/samples/

cd bb_dcmimu
gcc bb_dcmimu.c -o bb_dcmimu -lm || { echo "Unable compile bb_dcmimu.  Exiting."; exit 1; }
mv bb_dcmimu /srv/http
gcc bb_calibrate.c -o bb_calibrate
mv bb_calibrate /srv/http/

#cd python
#python2 setup.py build_ext --inplace || { echo "Unable to build dcmimu module"; exit 1; }
#cp dcmimu.so /srv/http/

cd ~
tee gpio_pull-overlay.dts <<HDHD
/*
* Overlay for enabling gpio's to pull at boot time
* this overlay uses pincctrl to initialize the pull-up register for the the listed gpios
* the compatible="gpio-leds" forces a module probe so the pinctrl does something
*
* To use this dts:
* copy this to a file named gpio_pull-overlay.dts
* modify the brcm,pins, brcm,function, and brcm,pull values
* apt-get install device-tree-compiler
* dtc -@ -I dts -O dtb -o gpio_pull-overlay.dtb gpio_pull-overlay.dts
* sudo cp gpio_pull-overlay.dtb /boot/overlays
* add this line to the end config.txt: dtoverlay=gpio_pull
* reboot
*/

/dts-v1/;
/plugin/;
/ {
  compatible = "brcm,bcm2835", "brcm,bcm2708";
  fragment@0 {
    target = <&gpio>;
    __overlay__ {
       gpio_pins: gpio_pins {
          brcm,pins = <17>; /* list of gpio(n) pins to pull */
          brcm,function = <0>; /* boot up direction:in=0 out=1 */
          brcm,pull = <1>; /* pull direction: none=0, 1 = down, 2 = up */
       };
    };
  };
  fragment@1 {
    target-path = "/soc";
    __overlay__ {
       gpiopull:gpiopull {
          compatible = "gpio-leds";
          pinctrl-names = "default";
          pinctrl-0 = <&gpio_pins>;
          status = "okay";
       };
    };
  };
  __overrides__ {
     gpio_pull = <&gpiopull>,"status";
  };
};
HDHD

dtc -Wno-unit_address_vs_reg -@ -I dts -O dtb -o gpio_pull.dtbo gpio_pull-overlay.dts || { echo "Unable to compile dts"; exit 1; }
cp gpio_pull.dtbo /boot/overlays/

if [ $(cat /boot/config.txt | grep 'dtoverlay=gpio_pull' | wc -l) -eq 0 ]; then
  echo -e "\ndtoverlay=gpio_pull\n" | tee -a /boot/config.txt
fi

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
systemctl enable ap
systemctl enable smbd nmbd

sync && sync && poweroff
#systemctl disable serial-getty@ttyAMA0.service 


