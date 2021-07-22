#!/bin/bash

# Dependencies
sudo apt install python3-pip
sudo apt install -y python3-venv
curl -sL https://deb.nodesource.com/setup_16.x | sudo -E bash -
sudo apt install -y nodejs
sudo npm install -g yarn

# Paths
HOME=/home/$USER
AGROBOT=$HOME/Agrobot
AGROBOT_ENV=$AGROBOT/env

# Folders
rm -rf $AGROBOT
mkdir $AGROBOT/

rm -rf $AGROBOT_ENV
mkdir $AGROBOT_ENV

# Virtual Env
python3 -m venv $AGROBOT_ENV/


# Post install
clear
echo DONE