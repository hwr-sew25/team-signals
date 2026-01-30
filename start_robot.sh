#!/bin/bash

# --- LOGGING START ---
echo "[Autostart] Skript gestartet am $(date)"

# 1. ROS Noetic laden (Ubuntu 20.04)
source /opt/ros/noetic/setup.bash

# 2. Workspace laden
# Da dein User 'ubuntu' ist, liegt der Workspace hier:
source /home/ubuntu/team_signals_ws/devel/setup.bash

# 3. Umgebungsvariablen setzen (Wichtig für Display/Remote)
export ROS_HOME=/home/ubuntu/.ros
export TERM=xterm

# 4. Starten
# Paket: signal_project | Datei: state_machine.launch
echo "[Autostart] Starte roslaunch..."
roslaunch signal_project state_machine.launch
