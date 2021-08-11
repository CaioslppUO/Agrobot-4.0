#!/bin/bash
tmux new -s agrobot -d
tmux send-keys "agrobot && roslaunch agrobot run.launch" Enter