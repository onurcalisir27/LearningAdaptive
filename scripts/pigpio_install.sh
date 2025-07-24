#!/bin/bash
set -e

# installing pigpio
mkdir -p /$HOME/tmp
cd /$HOME/tmp
sudo apt install -y python-setuptools python3-setuptools
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install

