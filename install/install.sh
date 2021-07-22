#!/bin/bash

# Dependencies
sudo apt install python3-pip
sudo apt install -y python3-venv
curl -sL https://deb.nodesource.com/setup_16.x | sudo -E bash -
sudo apt install -y nodejs
sudo npm install -g yarn

# Paths
HOME=/home/$USER
BASHRC=$HOME/.bashrc
ZSHRC=$HOME/.zshrc
AGROBOT=$HOME/Agrobot
AGROBOT_ENV=$AGROBOT/env
AGROBOT_ENV_BIN=$AGROBOT_ENV/bin

# Folders
rm -rf $AGROBOT
mkdir $AGROBOT/

rm -rf $AGROBOT_ENV
mkdir $AGROBOT_ENV

# Virtual Env
python3 -m venv $AGROBOT_ENV/
echo "source /opt/ros/noetic/setup.zsh 2>/dev/null" >> $AGROBOT_ENV_BIN/activate
echo "source /opt/ros/noetic/setup.bash 2>/dev/null" >> $AGROBOT_ENV_BIN/activate
echo "alias exit_agrobot=deactivate" >> $AGROBOT_ENV_BIN/activate

AGROBOT_ALIAS="alias agrobot='source $AGROBOT_ENV_BIN/activate'"
AGROBOT_ALIAS_GREP="$(grep 'alias agrobot' $BASHRC)"

if [ "$AGROBOT_ALIAS_GREP" == "$AGROBOT_ALIAS" ]
then
    echo "agrobot alias already registered"
else
    echo "alias agrobot='source $AGROBOT_ENV_BIN/activate'" >> $BASHRC
    echo "alias agrobot='source $AGROBOT_ENV_BIN/activate'" >> $ZSHRC
fi

# Post install
clear
echo DONE