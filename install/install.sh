#!/bin/bash

# Color Constants
PURPLE='\033[1;35m'
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# Install Control
UBUNTU_DEPENDENCIES="0"
ARCH_DEPENDENCIES="0"
SITE_PACKAGES="0"

# Dependencies
{ ## Ubuntu
    sudo apt install python3-pip &&
    sudo apt install -y python3-venv &&
    curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash - &&
    sudo apt install -y nodejs &&
    sudo npm install -g yarn &&
    printf "${PURPLE}Instaled dependencies for Ubuntu Linux${NC}\n" &&
    UBUNTU_DEPENDENCIES="1"
} || {
    ## Arch
    sudo pacman -S --noconfirm python-pip &&
    sudo pacman -S --noconfirm python-virtualenv &&
    sudo pacman -S --noconfirm nodejs &&
    sudo pacman -S --noconfirm npm &&
    sudo npm install -g yarn &&
    printf "${PURPLE}Instaled dependencies for Arch Linux${NC}\n" &&
    ARCH_DEPENDENCIES="1"
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

{
    # Virtual Env
    python3 -m venv $AGROBOT_ENV/
    source "$AGROBOT_ENV_BIN/activate"
    packages=$(cat "$LOCAL_FOLDER/req")
    for package in $packages
    do
        pip3 install $package
    done
    deactivate
    echo "source /opt/ros/noetic/setup.zsh 2>/dev/null" >> $AGROBOT_ENV_BIN/activate &&
    echo "source /opt/ros/noetic/setup.bash 2>/dev/null" >> $AGROBOT_ENV_BIN/activate &&
    echo "alias agrobot=exit" >> $AGROBOT_ENV_BIN/activate &&
    VIRTUAL_ENV="1"
} || {
    VIRTUAL_ENV="0"
}

AGROBOT_ALIAS="alias agrobot='source $AGROBOT_ENV_BIN/activate'"
AGROBOT_ALIAS_GREP="$(grep 'alias agrobot' $BASHRC)"

if [ "$AGROBOT_ALIAS_GREP" == "$AGROBOT_ALIAS" ]
then
    echo "agrobot alias already registered"
else
    echo "alias agrobot='source $AGROBOT_ENV_BIN/activate'" >> $BASHRC
    echo "alias agrobot='source $AGROBOT_ENV_BIN/activate'" >> $ZSHRC
fi

# ROS
{
## Catkin
    cd $CATKIN && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 &&
    echo "source $CATKIN_DEVEL/setup.bash 2>/dev/null" >> $AGROBOT_ENV_BIN/activate &&
    echo "source $CATKIN_DEVEL/setup.zsh 2>/dev/null" >> $AGROBOT_ENV_BIN/activate &&
    cd $CATKIN_SRC && catkin_create_pkg agrobot std_msgs rospy roscpp message_generation message_runtime &&
    cd $CATKIN && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 &&
    CATKIN_INSTALL="1"
} || {
    CATKIN_INSTALL="0"
}

{
    ## Config
    cd "$LOCAL_FOLDER/../config/" && cp -r ./* "$AGROBOT" &&
    CONFIG="1"
} || {
    CONFIG="0"
}

{
    ## Core scripts
    cd "$LOCAL_FOLDER/../core/" && chmod +x ./* &&  cp -r ./* "$AGROBOT_SRC/" &&
    CORE="1"
} || {
    CORE="0"
}

{
    ## Services
    VIRTUAL_ENV_SITE_PACKAGES=$AGROBOT_ENV/lib/$(ls $AGROBOT_ENV/lib/ | grep python*)/site-packages
    AGROBOT_SERVICES=$VIRTUAL_ENV_SITE_PACKAGES/agrobot_services
    mkdir $AGROBOT_SERVICES &&
    cd "$LOCAL_FOLDER/../services/" && chmod +x ./* && cp -r ./* "$AGROBOT_SERVICES" &&
    SERVICES="1"
} || {
    SERVICES="0"
}

{
    ## Messages
    mkdir $AGROBOT_MSG &&
    cd "$LOCAL_FOLDER/../msg/" && cp -r ./* "$AGROBOT_MSG" &&
    MESSAGES="1"
} || {
    MESSAGES="0"
}

{
    ## Server for communication with app
    cd "$LOCAL_FOLDER/../" && cp -r server "$AGROBOT" &&
    cd "$AGROBOT/server" && yarn install &&
    SERVER="1"
} || {
    SERVER="0"
}

{
    ## Modules
    cd "$AGROBOT/src/" && mkdir -p modules &&
    cd "$LOCAL_FOLDER/../modules/" &&
    g++ control_robot/control_motor/*.c*  -pthread -o control_robot/communication/controller.out &&
    chmod -R +x ./*/*.py && cp -r ./* "$AGROBOT/src/modules/" &&
    MODULES="1"
} || {
    MODULES="0"
}

{
    ## Specials
    cd "$LOCAL_FOLDER/../special/" &&
    chmod -R +x ./*/*.py && cp -r ./* "$AGROBOT/src/modules/" &&
    SPECIALS="1"
} || {
    SPECIALS="0"
}

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

## Special nodes in roslauch
echo "    <node pkg='agrobot' type='control_direction.py' name='control_direction' args='16 19' output='screen' />" >> run.launch
echo "    <node pkg='agrobot' type='encoder.py' name='encoder_left' args='7 13' output='screen' />" >> run.launch
echo "    <node pkg='agrobot' type='encoder.py' name='encoder_right' args='29 31' output='screen' />" >> run.launch

echo "</launch>" >> run.launch

# Post install
## Catkin compilations
cd $CATKIN && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

## Install agrobot site packages from ROS into agrobot project
{
    ## Ubuntu Linux
    AGROBOT_SITE_PACKAGES=$CATKIN_DEVEL/lib/$(ls $CATKIN_DEVEL/lib/ | grep python*)/dist-packages/agrobot
    mkdir -p "$VIRTUAL_ENV_SITE_PACKAGES/agrobot/" &&
    cp -r "$AGROBOT_SITE_PACKAGES/"* "$VIRTUAL_ENV_SITE_PACKAGES/agrobot/" &&
    SITE_PACKAGES="1"
} || {
    ## Arch Linux
    AGROBOT_SITE_PACKAGES=$CATKIN_DEVEL/lib/$(ls $CATKIN_DEVEL/lib/ | grep python*)/site-packages/agrobot
    mkdir -p "$VIRTUAL_ENV_SITE_PACKAGES/agrobot/" &&
    cp -r "$AGROBOT_SITE_PACKAGES/"* "$VIRTUAL_ENV_SITE_PACKAGES/agrobot/" &&
    SITE_PACKAGES="1"
}

## setting ROS_IP and ROS_MASTER_URI
{
    ipv4=$(hostname -I | awk '{print $1}') &&
    echo "export ROS_MASTER_URI=http://$ipv4:11311" >> "$AGROBOT_ENV_BIN/activate" &&
    echo "export ROS_IP=$ipv4" >> "$AGROBOT_ENV_BIN/activate" &&
    ROS_IP_AND_MASTER_URI=1
} || {
    ROS_IP_AND_MASTER_URI=0
}


## Install service 
#sudo cp "$LOCAL_FOLDER/service/agrobot.service" /etc/systemd/system
#sudo cp "$LOCAL_FOLDER/service/start_agrobot.sh" /usr/bin
#sudo cp "$LOCAL_FOLDER/service/attach.sh $HOME" 

clear
printf "      Feature                 Situation\n\n"
if [ "$UBUNTU_DEPENDENCIES" == "1" ] 
    then
        printf "${BLUE}Dependencies${NC}                       ${GREEN}OK${NC}\n"
    elif [ "$ARCH_DEPENDENCIES" == "1" ]
    then
        printf "${BLUE}Dependencies${NC}                       ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Dependencies${NC}                       ${RED}NO${NC}\n"
fi
if [ "$VIRTUAL_ENV" == "1" ] 
    then
        printf "${BLUE}Virtual env${NC}                        ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Virtual env${NC}                        ${RED}NO${NC}\n"
fi
if [ "$CATKIN_INSTALL" == "1" ] 
    then
        printf "${BLUE}Catkin${NC}                             ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Catkin${NC}                             ${RED}NO${NC}\n"
fi
if [ "$CONFIG" == "1" ] 
    then
        printf "${BLUE}Configuration files${NC}                ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Configuration files${NC}                ${RED}NO${NC}\n"
fi
if [ "$CORE" == "1" ] 
    then
        printf "${BLUE}Core modules${NC}                       ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Core modules${NC}                       ${RED}NO${NC}\n"
fi
if [ "$MODULES" == "1" ] 
    then
        printf "${BLUE}Agrobot modules${NC}                    ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Agrobot modules${NC}                    ${RED}NO${NC}\n"
fi
if [ "$SPECIALS" == "1" ] 
    then
        printf "${BLUE}Agrobot special modules${NC}            ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Agrobot special modules${NC}            ${RED}NO${NC}\n"
fi
if [ "$SERVICES" == "1" ] 
    then
        printf "${BLUE}Agrobot services${NC}                   ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Agrobot services${NC}                   ${RED}NO${NC}\n"
fi
if [ "$MESSAGES" == "1" ] 
    then
        printf "${BLUE}Agrobot messages${NC}                   ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Agrobot mervices${NC}                   ${RED}NO${NC}\n"
fi
if [ "$SERVER" == "1" ] 
    then
        printf "${BLUE}App server${NC}                         ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}App server${NC}                         ${RED}NO${NC}\n"
fi
if [ "$SITE_PACKAGES" == "1" ] 
    then
        printf "${BLUE}Site packages${NC}                      ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Site packages${NC}                      ${RED}NO${NC}\n"
fi
if [ "$ROS_IP_AND_MASTER_URI" == "1" ] 
    then
        printf "${BLUE}Ros IP and Master Uri${NC}              ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Ros IP and Master Uri${NC}              ${RED}NO${NC}\n"
fi
printf "\n${GREEN}DONE${NC}\n"