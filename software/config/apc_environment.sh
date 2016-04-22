#!/bin/bash
# edit APCDATA_BASE=$HOME/apcdata to your apc data directory

thisFile=$_
if [ $BASH ] 
then
  # may be a relative or absolute path
  thisFile=${BASH_SOURCE[0]}
fi

set_apc_base()
{
  # use cd and pwd to get an absolute path
  configParentDir="$(cd "$(dirname "$thisFile")/.." && pwd)"

  # different cases for software/config or software/build/config
  case "$(basename $configParentDir)" in
    "software") export APC_BASE=$(dirname $configParentDir);;
    "build") export APC_BASE=$(dirname $(dirname $configParentDir));;
    *) echo "Warning: APC environment file is stored in unrecognized location: $thisFile";;
  esac
  export APCDATA_BASE=$APC_BASE/../apcdata
  export PATH=$PATH:$APC_BASE/software/build/bin
}

setup_apc()
{
  export PATH=$PATH:$APC_BASE/software/build/bin
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=$APC_BASE/software/build/lib:$APC_BASE/software/build/lib64:$LD_LIBRARY_PATH
  export CLASSPATH=$CLASSPATH:/usr/local/share/java/lcm.jar:$APC_BASE/software/build/share/java/lcmtypes_apc_lcmtypes.jar
  export CLASSPATH=$CLASSPATH:$APC_BASE/software/build/share/java/drake.jar:$APC_BASE/software/build/share/java/bot2-lcmgl.jar
  export PKG_CONFIG_PATH=$APC_BASE/software/build/lib/pkgconfig:$APC_BASE/software/build/lib64/pkgconfig:$PKG_CONFIG_PATH

  # python path
  export PYTHONPATH=$PYTHONPATH:$APC_BASE/software/build/lib/python2.7/site-packages:$APC_BASE/software/build/lib/python2.7/dist-packages
  # enable some warnings by default
  export CXXFLAGS="$CXXFLAGS -Wreturn-type -Wuninitialized"
  export CFLAGS="$CFLAGS -Wreturn-type -Wuninitialized"
  
  export PATH=$PATH:$HOME/software/ffmpeg-2.4.2-64bit-static # for ffmpeg software
  
  if [ "$HOSTNAME" == 'mcube-002' ] || [ "$HOSTNAME" == 'mcube-004' ] || [ "$HOSTNAME" == 'mcube-005' ]; then
    export ROS_MASTER_URI=http://mcube-002:11311
  fi  
  if [ "$HOSTNAME" == 'mcube-003' ] || [ "$HOSTNAME" == 'mcube-006' ]; then
    export ROS_MASTER_URI=http://mcube-003:11311
  fi
  if [ "$HOSTNAME" == 'mcube-003' ]; then
    export ROS_HOSTNAME=mcube-003
    export ROS_IP=192.168.37.13
  fi
  if [ "$HOSTNAME" == 'mcube-006' ]; then
    export ROS_HOSTNAME=mcube-006
    export ROS_IP=192.168.37.12
  fi
  if [ "$HOSTNAME" == 'mcube-005' ]; then
    export ROS_HOSTNAME=mcube-005
    export ROS_IP=192.168.37.10
  fi
  if [ "$HOSTNAME" == 'mcube-004' ]; then
    export ROS_HOSTNAME=mcube-004
    export ROS_IP=192.168.37.9
  fi
  if [ "$HOSTNAME" == 'mcube-002' ]; then
    export ROS_HOSTNAME=mcube-002
    export ROS_IP=192.168.37.7
  fi
  
  export ROSLAUNCH_SSH_UNKNOWN=1
}

set_ros()
{
  if [ -f $APC_BASE/catkin_ws/devel/setup.bash ]; then
    source $APC_BASE/catkin_ws/devel/setup.bash
  else
    source /opt/ros/indigo/setup.bash
  fi
  export ROS_PACKAGE_PATH=$HOME/apc/ros_ws/:$ROS_PACKAGE_PATH
}

gituser()
{
  if [ $# -eq 0 ]; then
    echo 'Current user is: '
    git config user.name
    git config user.email
    echo 'Usage: gituser <athena_id>'
    return 0
  fi

  case $1 in
  peterkty) email=peterkty@gmail.com ;;
  tayloro)  email=orion.thomas.taylor@gmail.com ;;
  nikhilcd) email=nikhilcd@mit.edu ;;
  nfazeli)  email=nfazeli@mit.edu ;;
  diazlank) email=diazlank@mit.edu ;;
  albertor) email=albertor@mit.edu ;;
  *) echo "$1 is not in the list, please enter your athena id"; return 1 ;;
  esac
  
  git config --global user.name $1
  git config --global user.email $email
}

# some useful commands
alias cdapc='cd $APC_BASE'
alias cdapcdata='cd $APCDATA_BASE'
alias matlabdrake='cd $APC_BASE/software; matlab -r "addpath_pods; addpath_drake"'
alias matlabapc='cd $APC_BASE/software; matlab -nodesktop -nodisplay -nosplash -r "tic; addpath_pods; addpath_drake; toc; cd ../software/planning/ik_server/; ikTrajServerSocket;"'

alias gitsub='git submodule update --init --recursive'
alias gitpull='git -C $APC_BASE pull'

alias rebash='source ~/.bashrc'
alias open='gnome-open'

alias yolo='rosservice call /robot1_SetSpeed 1600 180'
alias faster='rosservice call /robot1_SetSpeed 200 50'
alias fast='rosservice call /robot1_SetSpeed 100 30'
alias slow='rosservice call /robot1_SetSpeed 50 15'

alias gohome='rosservice call robot1_SetJoints "{j1: -0.32913211, j2: -39.41918751, j3: 33.58432661, j4: 5.55385908, j5: 6.08041137, j6: -5.98758923}"'
alias gohome2='rosservice call robot1_SetJoints "{j1: 0, j2: -40, j3: 33.58432661, j4: 5.55385908, j5: 6.08041137, j6: -5.98758923}"'

alias teleop='rosrun teleop teleop'
alias pythonapc='ipython -i -c "run $APC_BASE/catkin_ws/src/apc_config/python/pythonapc.py"'

alias pman='bot-procman-sheriff -l $APC_BASE/software/config/apc.pmd'

alias roslocal='export ROS_MASTER_URI=http://localhost:11311'

alias getjoint='rosservice call robot1_GetJoints'
alias getcart='rosservice call robot1_GetCartesian'

alias lcmlocal='sudo ifconfig lo multicast; sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo'

alias runapcvirtual='time rosrun apc_planning heuristic.py --jfilename multi_obj_3.json | tee /tmp/$(date +%Y%m%d_%H%M%S)'
alias runapc='time rosrun apc_planning heuristic.py --jfilename apc.json -s -v | tee /tmp/$(date +%Y%m%d_%H%M%S)'

ppms2mp4()
{
  bot-ppmsgz $1 mpeg4 10M 30 $1.mp4
}

function lowersuffix {
  cd "$1"
  find . -name '*.*' -exec sh -c '
  a=$(echo {} | sed -r "s/([^.]*)\$/\L\1/");
  [ "$a" != "{}" ] && mv "{}" "$a" ' \;
}

function ipmasq {
   if [ $# -eq 0 ]; then
     echo 'sharing wlan0 to eth0'
     sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE 
     sudo iptables -A FORWARD -i wlan0 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT 
     sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
   elif [ $# -eq 1 ]; then
     echo "sharing $1 to eth0"
     sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
     sudo iptables -A FORWARD -i $1 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i eth0 -o $1 -j ACCEPT
   elif [ $# -eq 2 ]; then
     echo "sharing $1 to $2"
     sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
     sudo iptables -A FORWARD -i $1 -o $2 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i $2 -o $1 -j ACCEPT
   fi
}

function set_bash {
   PROMPT_COMMAND='history -a'
   history -a

   # sorting in old style
   LC_COLLATE="C"
   export LC_COLLATE
   
   ulimit -c unlimited
   export HISTTIMEFORMAT="%d/%m/%y %T "
}

set_apc_base
setup_apc
set_ros
set_bash

exec "$@"
