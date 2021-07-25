#!/bin/bash

# Dependencies
sudo apt install python3-pip
sudo apt install -y python3-venv
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt install -y nodejs
sudo npm install -g yarn

# Paths
LOCAL_FOLDER=$PWD
HOME=/home/$USER
BASHRC=$HOME/.bashrc
ZSHRC=$HOME/.zshrc
AGROBOT_FOLDER=$HOME/Agrobot
AGROBOT_ENV=$AGROBOT_FOLDER/agrobot_env
AGROBOT_ENV_BIN=$AGROBOT_ENV/bin
CATKIN=$AGROBOT_FOLDER/catkin_ws
CATKIN_SRC=$CATKIN/src
CATKIN_DEVEL=$CATKIN/devel
AGROBOT=$CATKIN_SRC/agrobot
AGROBOT_SRC=$AGROBOT/src

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
cd $CATKIN && catkin_make
echo "source $CATKIN_DEVEL/setup.bash 2>/dev/null" >> $AGROBOT_ENV_BIN/activate
echo "source $CATKIN_DEVEL/setup.zsh 2>/dev/null" >> $AGROBOT_ENV_BIN/activate
cd $CATKIN_SRC && catkin_create_pkg agrobot std_msgs rospy roscpp message_generation message_runtime
cd $CATKIN && catkin_make

## Scripts
cd "$LOCAL_FOLDER/../scripts/" && chmod +x ./* &&  cp -r ./* "$AGROBOT_SRC/"

## Roslaunch
cd $AGROBOT
mkdir launch && cd launch
echo "<launch>" > run.launch
files=$(ls $LOCAL_FOLDER/../scripts)
for entry in $files
do
    name=$(echo "$entry" | cut -f 1 -d '.')
    echo "    <node pkg='agrobot' type='$entry' name='$name' output='screen'/>" >> run.launch
done
echo "</launch>" >> run.launch

# Post install
#clear
echo DONE