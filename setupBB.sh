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

# This script forms part of the Bell-Boy project to measure the force applied by a bell ringer to a tower
# bell rope.  The Bell-Boy uses tangential acceleration of the bell as a proxy for force applied.  The
# hardware is currently a Pi Zero running Arch Linux, two MPU6050 breakout boards and the official Pi Wifi dongle
# The MPU6050 breakout boards are mounted on opposite sides of the bell axle such that gravity pulls
# on MPU6050s in the same direction but so that the tangential acceleration applied by the ringer 
# works in opposite directions on each sensor.  This makes it easy to separate out tangential acceleration 
# due to gravity and tangential acceleration due to force applied.

# The script below takes a stock installation of Arch Linux Arm (for the Raspberry Pi) and adds all scripts
# and installs all required packages to turn the Pi into a Bell-Boy device. General instructions are: 
#    (a)    Install Arch Linux to an SD card https://archlinuxarm.org/platforms/armv6/raspberry-pi or 
#           https://archlinuxarm.org/platforms/armv7/broadcom/raspberry-pi-2;
#    (b)    Copy this script to the FAT partition (for convenience)
#    (c)    Pop the SD card into your Raspberry Pi and boot it up with an internet connection (use ethernet
#           port or usb/ethernet adapter)
#    (d)    SSH into the Pi (username alarm, password alarm), change to root user (su root, password root) 
#    (e)    run this script - it will take a while
# Note that for first time usage, the MPU6050s should be calibrated and that the MPU6050 clocks need to be
# chosen in order to sync the two MPU6050s as far as is reasonably possible.


PORTABLEWIFINETWORK=Bell-Boy
HOSTNAME=Bell-Boy_D

if [ $(id -u) -ne 0 ]; then
    echo "You appear not to be running as root user (an assumption this script makes).  Try 'su root'.  Exiting"
    exit 1
fi

if [ $(cat /boot/config.txt | grep 'dtparam=i2c_arm=on' | wc -l) -eq 0 ]; then
  echo -e "\ndtparam=i2c_arm=on\ndtparam=i2c_arm_baudrate=400000\n" | tee -a /boot/config.txt
fi

if [ $(cat /boot/config.txt | grep 'dtoverlay=pi3-disable-wifi' | wc -l) -eq 0 ]; then
  echo -e "\ndtoverlay=pi3-disable-wifi\n" | tee -a /boot/config.txt
fi

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
  echo -e "\ndtoverlay=gpio-poweroff,gpiopin=5,active_low\n" | tee -a /boot/config.txt
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
pacman -S --noconfirm --needed wget dtc samba bzip2 screen python2 python2-numpy tmux hostapd crda samba base-devel i2c-tools unzip dhcp git || { echo "Failed downloading packages.  Exiting"; exit 1; }

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

#wget --no-check-certificate http://www.dropbox.com/s/rcxkbidv7a3t9kk/hostapd-rtl8192cu-0.8-1.src.tar.gz { echo "Failed download hostapd.  Exiting"; exit 1; }
#tar -zxvf hostapd-rtl8192cu-0.8-1.src.tar.gz
#cd hostapd*
#su alarm -c "makepkg" || { echo "Failed to build hostapd.  Exiting"; exit 1; }
#pacman -U --noconfirm *.xz

tee /etc/hostapd/hostapd.conf << HDHD || { echo "Unable to write hostapd config - exiting"; exit 1; }
interface=wlan0
ssid=Bell-Boy
hw_mode=g
channel=6
auth_algs=1
wmm_enabled=0
HDHD

tee /usr/lib/systemd/system/dhcpd4.service <<HDHD || { echo "Unable to write dhcpd4 service file - exiting"; exit 1; }
[Unit]
Description=IPv4 DHCP server
After=network.target

[Service]
Type=forking
PIDFile=/run/dhcpd4.pid
ExecStartPre=-/usr/bin/tvservice -o
ExecStartPre=-/usr/bin/ip link set dev wlan0 down
#ip addr flush dev wlan0
ExecStartPre=-/usr/bin/ip link set dev wlan0 up
ExecStartPre=-/usr/bin/iw dev wlan0 set power_save off
ExecStartPre=/usr/bin/sh -c "sleep 5 && /usr/bin/ip addr add 10.0.0.1/24 broadcast 10.0.0.255 dev wlan0"
ExecStartPre=/usr/bin/systemctl start hostapd
ExecStart=/usr/bin/dhcpd -4 -q -cf /etc/dhcpd.conf -pf /run/dhcpd4.pid
ExecStopPost=/usr/bin/systemctl stop hostapd
ExecStopPost=-/usr/bin/ip link set dev wlan0 down
KillSignal=SIGINT

[Install]
WantedBy=multi-user.target
HDHD

git clone https://github.com/google/pywebsocket.git || { echo "Unable to download pywebsocket.  Exiting"; exit 1; }

cd pywebsocket

python2 setup.py build
python2 setup.py install || { echo "Unable to install Pywebsocket.  Exiting"; exit 1; }

cp build/lib/mod_pywebsocket/standalone.py /srv/http/ || { echo "Could not find standalone.py.  Exiting"; exit 1; }

tee /etc/systemd/system/bellboy.service <<HDHD
[Unit]
Description= Websocket script to start on boot
After=network.target
[Service]
User=root
Type=forking
ExecStart=/usr/bin/screen -S wrad -d -m sh -c "(/usr/bin/python2 /srv/http/standalone.py -p 80 -w /srv/http/ -d /srv/http/ 2>&1) | tee -a /var/log/bellboy.log"
ExecStop=/usr/bin/screen -S wrad -X wrad
KillMode = control-group
TimeoutStopSec=0
Restart=always
RestartSec=5
[Install]
WantedBy=multi-user.target
HDHD

#cd ~
#
#git clone https://github.com/adafruit/Adafruit_Python_BNO055.git
#cd Adafruit_Python_BNO055
#curl https://github.com/johnweber/Adafruit_Python_BNO055/commit/424916e703a20b413d120ca8e99ade2c70e7f56f.patch | patch -f -p1 -i -
#python2 setup.py install || { echo "Unable to install BNO055 files"; exit 1; }

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
mv * /srv/http

mkdir -p /data/samples
mv samples/* /data/samples/

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
          brcm,pins = <6 13>; /* list of gpio(n) pins to pull */
          brcm,function = <0 0>; /* boot up direction:in=0 out=1 */
          brcm,pull = <2 1>; /* pull direction: none=0, 1 = down, 2 = up */
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

dtc -@ -I dts -O dtb -o gpio_pull.dtbo gpio_pull-overlay.dts || { echo "Unable to compile dts"; exit 1; }
cp gpio_pull.dtbo /boot/overlays/

if [ $(cat /boot/config.txt | grep 'dtoverlay=gpio_pull' | wc -l) -eq 0 ]; then
  echo -e "\ndtoverlay=gpio_pull\n" | tee -a /boot/config.txt
fi

if [ ! -f /etc/samba/smb.conf ]; then
  cp /etc/samba/smb.conf.default /etc/samba/smb.conf
fi

comp=`cat /etc/samba/smb.conf | grep '\[BELL SAMPLES\]'`
if [ ! "$comp" = "[BELL SAMPLES]" ]; then
  tee -a /etc/samba/smb.conf <<HDHD
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
  sed -i '
/security = user/ i\
    map to guest = Bad User
' /etc/samba/smb.conf
  sed -i 's/MYGROUP/WORKGROUP/' etc/samba/smb.conf
fi

systemctl enable bellboy
systemctl enable dhcpd4
systemctl enable smbd nmbd

#sync && reboot
#systemctl disable serial-getty@ttyAMA0.service 


