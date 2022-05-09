#!/bin/bash

# Color Constants
PURPLE='\033[1;35m'
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# Install Control
UBUNTU_DEPENDENCIES="0"
SITE_PACKAGES="0"
MASTER_INSTALL="0"

echo "Master installation? (Y/n)"
read master_install

if [ ! -z "$master_install" ] && [ $master_install == "n" ] # Slave install
    then
        MASTER_INSTALL="0"
        echo "Installing Slave Version!!"
        sleep 3
    else # Master install
        MASTER_INSTALL="1"
        echo "Installing Master Version!!"
        sleep 3
fi

# Dependencies
if [ ! -z "$1" ] && [ $1 == "--no-dependency" ] 
    then
        echo "skipping dependency install."
        UBUNTU_DEPENDENCIES="1"
    else
        { ## Ubuntu
        sudo apt update &&
        sudo apt install -y python3-pip python3-venv tmux python3-defusedxml &&
        sudo apt install -y mosquitto mosquitto-clients libmosquitto-dev &&
        printf "${PURPLE}Instaled dependencies for Ubuntu Linux${NC}\n" &&
        UBUNTU_DEPENDENCIES="1"
    } || {
        UBUNTU_DEPENDENCIES="0"
    }
fi

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
    python3 -m pip install empy==3.3.4
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

if [ $MASTER_INSTALL == "1" ]
    then
    {
        ## Core scripts
        cd "$LOCAL_FOLDER/../core/" && chmod +x ./* &&  cp -r ./* "$AGROBOT_SRC/" &&
        CORE="1"
    } || {
        CORE="0"
    }
fi

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

if [ $MASTER_INSTALL == "1" ]
    then
    {
        ## Server for communication with app
        cd "$LOCAL_FOLDER/../" && cp -r server "$AGROBOT" &&
        SERVER="1"
    } || {
        SERVER="0"
    }
fi

{
    ## Modules
    cd "$AGROBOT/src/" && mkdir -p modules &&
    cd "$LOCAL_FOLDER/../modules/" &&
    g++ control_robot/control_motor/*.c* -lboost_system -pthread -lmosquitto -o control_robot/communication/controller.out &&
    chmod -R +x ./*/*.py && cp -r ./* "$AGROBOT/src/modules/" &&
    MODULES="1"
} || {
    MODULES="0"
}

if [ $MASTER_INSTALL == "0" ] # Install Slave modules
    then
        {
            cd "$LOCAL_FOLDER/../special/" &&
            chmod -R +x ./*/*.py && cp -r ./encoder ./relay ./control_direction "$AGROBOT/src/modules/" &&
            SPECIALS="1"
        } || {
            SPECIALS="0"
        }
    else
        {
            ## Specials
            cd "$LOCAL_FOLDER/../special/" &&
            chmod -R +x ./*/*.py && cp -r ./* "$AGROBOT/src/modules/" &&
            SPECIALS="1"
        } || {
            SPECIALS="0"
        }
fi

if [ $MASTER_INSTALL == "1" ]
    then
        {
            ## Simulations
            cd "$LOCAL_FOLDER/../simulation/" &&
            chmod -R +x ./*/*.py && cp -r ./* "$AGROBOT/src/modules/" &&
            SIMULATION="1"
        } || {
            SIMULATION="0"
        }
fi

## Roslaunch
cd $AGROBOT
mkdir launch && cd launch
echo "<launch>" > run.launch

if [ $MASTER_INSTALL == "1" ] # Master roslaunch
    then
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
        echo "    <node pkg='agrobot' type='control_direction.py' name='control_direction_master' args='0' output='screen' />" >> run.launch
        echo "    <node pkg='agrobot' type='encoder.py' name='encoder_1_master' args='7 13 encoder_1' output='screen' />" >> run.launch
        echo "    <node pkg='agrobot' type='encoder.py' name='encoder_2_master' args='29 31 encoder_2' output='screen' />" >> run.launch
        echo "    <node pkg='agrobot' type='relay.py' name='relay_master' args='40' output='screen' />" >> run.launch
    else # Slave roslaunch
        ## Modules files in roslaunch
        modules_files=$(basename -a $(ls $LOCAL_FOLDER/../modules/*/*.py))
        for module_entry in $modules_files
        do
            module_name=$(echo "$module_entry" | cut -f 1 -d '.')
            echo "    <node pkg='agrobot' type='$module_entry' name='$module_name' output='screen'/>" >> run.launch
        done

        echo "    <node pkg='agrobot' type='control_direction.py' name='control_direction_slave' args='0' output='screen' />" >> run.launch
        echo "    <node pkg='agrobot' type='encoder.py' name='encoder_3_slave' args='7 13 encoder_3' output='screen' />" >> run.launch
        echo "    <node pkg='agrobot' type='encoder.py' name='encoder_4_slave' args='29 31 encoder_4' output='screen' />" >> run.launch
        echo "    <node pkg='agrobot' type='relay.py' name='relay_slave' args='40' output='screen' />" >> run.launch
fi

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
    SITE_PACKAGES="0"
}

## setting ROS_IP and ROS_MASTER_URI
if [ $MASTER_INSTALL == "1" ] # Master
    then
        {
            ipv4=$(hostname -I | awk '{print $1}') &&
            echo "export ROS_MASTER_URI=http://$ipv4:11311" >> "$AGROBOT_ENV_BIN/activate" &&
            echo "export ROS_IP=$ipv4" >> "$AGROBOT_ENV_BIN/activate" &&
            ROS_IP_AND_MASTER_URI=1
        } || {
            ROS_IP_AND_MASTER_URI=0
        }
    else # Slave
        {
            ipv4=$(hostname -I | awk '{print $1}') &&
            echo "export ROS_MASTER_URI=http://192.168.1.2:11311" >> "$AGROBOT_ENV_BIN/activate" &&
            echo "export ROS_IP=$ipv4" >> "$AGROBOT_ENV_BIN/activate" &&
            ROS_IP_AND_MASTER_URI=1
        } || {
            ROS_IP_AND_MASTER_URI=0
        }
fi

## Install service 
sudo cp "$LOCAL_FOLDER/service/agrobot.service" /etc/systemd/system
sudo cp "$LOCAL_FOLDER/service/start_agrobot.sh" /usr/bin
sudo cp "$LOCAL_FOLDER/service/attach.sh" $HOME 

#clear
echo "------------------------------------------------------------------------"
if [ "$MASTER_INSTALL" == "1" ] 
    then
        printf "${BLUE}Master Install${NC}                     ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Slave Install${NC}                      ${GREEN}OK${NC}\n"
fi
if [ "$UBUNTU_DEPENDENCIES" == "1" ] 
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
if [ "$CORE" == "1" ] && [ "$MASTER_INSTALL" == "1" ]
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
if [ "$SERVER" == "1" ] && [ "$MASTER_INSTALL" == "1" ]
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
if [ "$SIMULATION" == "1" ] && [ "$MASTER_INSTALL" == "1" ]
    then
        printf "${BLUE}Simulations${NC}                        ${GREEN}OK${NC}\n"
    else
        printf "${BLUE}Simulations${NC}                        ${RED}NO${NC}\n"
fi
printf "\n${GREEN}DONE${NC}\n"