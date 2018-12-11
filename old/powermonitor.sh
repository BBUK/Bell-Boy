#!/bin/bash 
echo "4" > /sys/class/gpio/export
echo "17" > /sys/class/gpio/export
while true; do
  sleep 2
  if [ $(cat /sys/class/gpio/gpio4/value | grep '0' | wc -l) -eq 1 ]; then
    shutdown +15
    exit 0
  fi
  if [ $(cat /sys/class/gpio/gpio17/value | grep '1' | wc -l) -eq 1 ]; then
    sleep 2
    if [ $(cat /sys/class/gpio/gpio17/value | grep '1' | wc -l) -eq 1 ]; then
      sync && shutdown now
    fi
  fi 
done

