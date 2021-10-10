#!/bin/bash

# Color Constants
PURPLE='\033[1;35m'
RED='\033[0;31m'
NC='\033[0m'

# Dependencies
{ ## Ubuntu
    sudo apt install python3-pip &&
    sudo apt install -y python3-venv &&
    curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash - &&
    sudo apt install -y nodejs &&
    sudo npm install -g yarn &&
    printf "${PURPLE}Instaled dependencies for Ubuntu Linux${NC}\n"
} || {
    ## Arch
    sudo pacman -S --noconfirm python-pip &&
    sudo pacman -S --noconfirm python-virtualenv &&
    sudo pacman -S --noconfirm nodejs &&
    sudo npm install -g yarn &&
    printf "${PURPLE}Instaled dependencies for Arch Linux${NC}\n"
} || {
    printf "${RED}Could not install dependencies for Arch or Ubuntu Linux${NC}\n"
}


# Paths
## User
LOCAL_FOLDER=$PWD
HOME=/home/$USER
BASHRC=$HOME/.bashrc
ZSHRC=$HOME/.zshrc
## Virtualenv
AGROBOT_FOLDER=$HOME/Agrobot
AGROBOT_ENV=$AGROBOT_FOLDER/agrobot_env

AGROBOT_ENV_BIN=$AGROBOT_ENV/bin
## Catkin
CATKIN=$AGROBOT_FOLDER/catkin_ws
CATKIN_SRC=$CATKIN/src
CATKIN_DEVEL=$CATKIN/devel
## project
AGROBOT=$CATKIN_SRC/agrobot
AGROBOT_SRC=$AGROBOT/src
AGROBOT_MSG=$AGROBOT/msg

# Folders
rm -rf $AGROBOT_FOLDER
mkdir $AGROBOT_FOLDER/

mkdir $AGROBOT_ENV
mkdir -p $CATKIN/src/

# Virtual Env
python3 -m venv $AGROBOT_ENV/
source "$AGROBOT_ENV_BIN/activate"
packages=$(cat "$LOCAL_FOLDER/req")
for package in $packages
do
    pip3 install $package
done
deactivate
echo "source /opt/ros/noetic/setup.zsh 2>/dev/null" >> $AGROBOT_ENV_BIN/activate
echo "source /opt/ros/noetic/setup.bash 2>/dev/null" >> $AGROBOT_ENV_BIN/activate
echo "alias agrobot=exit" >> $AGROBOT_ENV_BIN/activate

AGROBOT_ALIAS="alias agrobot='source $AGROBOT_ENV_BIN/activate'"
AGROBOT_ALIAS_GREP="$(grep 'alias agrobot' $BASHRC)"

if [ "$AGROBOT_ALIAS_GREP" == "$AGROBOT_ALIAS" ]
then
    echo "agrobot alias already registered"
else
    echo "alias agrobot='source $AGROBOT_ENV_BIN/activate'" >> $BASHRC
    echo "alias agrobot='source $AGROBOT_ENV_BIN/activate'" >> $ZSHRC
fi

# ROS
## Catkin
cd $CATKIN && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
echo "source $CATKIN_DEVEL/setup.bash 2>/dev/null" >> $AGROBOT_ENV_BIN/activate
echo "source $CATKIN_DEVEL/setup.zsh 2>/dev/null" >> $AGROBOT_ENV_BIN/activate
cd $CATKIN_SRC && catkin_create_pkg agrobot std_msgs rospy roscpp message_generation message_runtime
cd $CATKIN && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

## Config
cd "$LOCAL_FOLDER/../config/" && cp -r ./* "$AGROBOT"

## Core scripts
cd "$LOCAL_FOLDER/../core/" && chmod +x ./* &&  cp -r ./* "$AGROBOT_SRC/"

## Services
VIRTUAL_ENV_SITE_PACKAGES=$AGROBOT_ENV/lib/$(ls $AGROBOT_ENV/lib/ | grep python*)/site-packages
AGROBOT_SERVICES=$VIRTUAL_ENV_SITE_PACKAGES/agrobot_services
mkdir $AGROBOT_SERVICES
cd "$LOCAL_FOLDER/../services/" && chmod +x ./* && cp -r ./* "$AGROBOT_SERVICES"

## Messages
mkdir $AGROBOT_MSG
cd "$LOCAL_FOLDER/../msg/" && cp -r ./* "$AGROBOT_MSG"

## Server for communication with app
cd "$LOCAL_FOLDER/../" && cp -r server "$AGROBOT"
cd "$AGROBOT/server" && yarn install

## Modules
cd "$AGROBOT/src/" && mkdir -p modules
cd "$LOCAL_FOLDER/../modules/"
g++ control_robot/control_motor/*.c*  -pthread -o control_robot/communication/controller.out
chmod -R +x ./*/*.py && cp -r ./* "$AGROBOT/src/modules/"

## Roslaunch
cd $AGROBOT
mkdir launch && cd launch
echo "<launch>" > run.launch
    echo "    <node pkg='agrobot' type='start_server.py' name='start_server' output='screen'/>" >> run.launch

## Core files in roslaunch
core_files=$(ls $LOCAL_FOLDER/../core)
for core_entry in $core_files
do
    core_name=$(echo "$core_entry" | cut -f 1 -d '.')
    echo "    <node pkg='agrobot' type='$core_entry' name='$core_name' output='screen'/>" >> run.launch
done

## Modules files in roslaunch
modules_files=$(basename -a $(ls $LOCAL_FOLDER/../modules/*/*.py))
for module_entry in $modules_files
do
    module_name=$(echo "$module_entry" | cut -f 1 -d '.')
    echo "    <node pkg='agrobot' type='$module_entry' name='$module_name' output='screen'/>" >> run.launch
done

echo "</launch>" >> run.launch

# Post install
## Catkin compilations
cd $CATKIN && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

## Install agrobot site packages from ROS into agrobot project
{
    ## Ubuntu Linux
    AGROBOT_SITE_PACKAGES=$CATKIN_DEVEL/lib/$(ls $CATKIN_DEVEL/lib/ | grep python*)/dist-packages/agrobot
    mkdir -p "$VIRTUAL_ENV_SITE_PACKAGES/agrobot/"
    cp -r "$AGROBOT_SITE_PACKAGES/"* "$VIRTUAL_ENV_SITE_PACKAGES/agrobot/"
} || {
    ## Arch Linux
    AGROBOT_SITE_PACKAGES=$CATKIN_DEVEL/lib/$(ls $CATKIN_DEVEL/lib/ | grep python*)/site-packages/agrobot
    mkdir -p "$VIRTUAL_ENV_SITE_PACKAGES/agrobot/"
    cp -r "$AGROBOT_SITE_PACKAGES/"* "$VIRTUAL_ENV_SITE_PACKAGES/agrobot/"
}


## Install service 
#sudo cp "$LOCAL_FOLDER/service/agrobot.service" /etc/systemd/system
#sudo cp "$LOCAL_FOLDER/service/start_agrobot.sh" /usr/bin
#sudo cp "$LOCAL_FOLDER/service/attach.sh $HOME" 

#clear
printf "${PURPLE}DONE${NC}\n    "