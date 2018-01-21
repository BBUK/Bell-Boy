#!/bin/bash 
echo "6" > /sys/class/gpio/export
while true; do
  dd if=/dev/urandom count=20000 | bzip2 -9 >> /dev/null
  sleep 2
  if [ $(cat /sys/class/gpio/gpio6/value | grep '0' | wc -l) -eq 1 ]; then
    echo "low battery warning"
    date
    sync && poweroff
  fi
  date 
done

