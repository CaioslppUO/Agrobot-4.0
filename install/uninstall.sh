#!/bin/bash

# Paths
HOME=/home/$USER
AGROBOT=$HOME/Agrobot
BASHRC=$HOME/.bashrc
ZSHRC=$HOME/.zshrc

# Folders
rm -rf $AGROBOT

# Profiles
grep -v "alias agrobot" $BASHRC > temp && mv temp $BASHRC # Remove agrobot alias from .bashrc file
grep -v "alias agrobot" $ZSHRC > temp && mv temp $ZSHRC

# Post uninstall
clear
echo DONE