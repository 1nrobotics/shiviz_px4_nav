# ~/.bashrc: executed by bash(1) for non-login shells.

# ROS environment setup
export ROS_DISTRO="noetic"
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# PX4 and MAVROS configuration
export PX4_SIM_HOST=${PX4_SIM_HOST:-localhost}
export PX4_SIM_MODEL=${PX4_SIM_MODEL:-iris}
export VEHICLE_ID=${VEHICLE_ID:-1}

# ROS networking (can be overridden by environment variables)
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
export ROS_IP=${ROS_IP:-localhost}

# Convenience aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias grep='grep --color=auto'

# ROS aliases
alias rosenv='env | grep ROS'
alias catkin_build_release='catkin build -DCMAKE_BUILD_TYPE=Release'
alias catkin_clean_all='catkin clean --all'

# Navigation specific aliases
alias nav_launch='roslaunch shiviz_px4_nav px4_offboard.launch'
alias nav_takeoff='rosservice call /waypoint_navigator/takeoff'
alias nav_land='rosservice call /waypoint_navigator/land'
alias nav_mission='rosservice call /waypoint_navigator/start_mission'
alias nav_stop='rosservice call /waypoint_navigator/stop_mission'

# PX4 specific aliases
alias px4_arm='rosservice call /mavros/cmd/arming "value: true"'
alias px4_disarm='rosservice call /mavros/cmd/arming "value: false"'
alias px4_mode_offboard='rosservice call /mavros/set_mode "base_mode: 0 custom_mode: OFFBOARD"'
alias px4_mode_manual='rosservice call /mavros/set_mode "base_mode: 0 custom_mode: MANUAL"'

# Monitoring aliases  
alias nav_status='rostopic echo /waypoint_navigator/status'
alias px4_state='rostopic echo /mavros/state'
alias px4_pose='rostopic echo /mavros/local_position/pose'
alias px4_velocity='rostopic echo /mavros/local_position/velocity_local'

# Development helpers
alias kill_ros='killall -9 rosmaster roscore; tmux kill-server 2>/dev/null || true'
alias restart_navigation='rosnode kill waypoint_navigator && sleep 2 && nav_launch'

# tmux convenience
alias tmux_nav='tmux new-session -d -s nav -c /catkin_ws'

# Color support
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
fi

# Enable programmable completion features
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

# Welcome message
echo "🚁 Shiviz PX4 Navigation Docker Environment"
echo "   ROS Master: $ROS_MASTER_URI"
echo "   PX4 Host: $PX4_SIM_HOST"
echo "   Vehicle ID: $VEHICLE_ID"
echo ""
echo "Quick commands:"
echo "  nav_launch    - Start navigation system"
echo "  nav_takeoff   - Takeoff drone"  
echo "  nav_mission   - Start waypoint mission"
echo "  px4_arm       - Arm the drone"
echo "  nav_status    - Monitor navigation status"
echo ""