#!/bin/bash
# Peter KT Yu, 2015


function ask {
    echo $1        # add this line
    read -n 1 -r
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
        return 1;
    else
        exit
        echo "Abort.."
    fi
}

if [ "$#" == 0 ] || [ "$1" == "APT" ]; then
    echo "Install useful packages from apt-get"
    sudo apt-get update
    sudo apt-get --yes install git gitk git-gui geany geany-plugins vim terminator meshlab recordmydesktop meld sagasu openssh-server retext filezilla vlc ipython mesa-utils bmon
    sudo apt-get --yes install hardinfo cpufrequtils   # for speedup cpu
        

    sudo apt-get --yes install build-essential cmake debhelper freeglut3-dev gtk-doc-tools libboost-filesystem-dev libboost-iostreams-dev libboost-program-options-dev libboost-random-dev libboost-regex-dev libboost-signals-dev libboost-system-dev libboost-thread-dev libcurl4-openssl-dev libfreeimage-dev libglew-dev libgtkmm-2.4-dev libltdl-dev libgsl0-dev libportmidi-dev libprotobuf-dev libprotoc-dev libqt4-dev libqwt-dev libtar-dev libtbb-dev libtinyxml-dev libxml2-dev ncurses-dev pkg-config protobuf-compiler python-matplotlib libvtk5.8 libvtk5-dev libvtk5-qt4-dev libqhull-dev python-pygame doxygen mercurial libglib2.0-dev openjdk-6-jdk python-dev gfortran f2c libf2c2-dev spacenavd libspnav-dev python-numpy python-scipy python-vtk python-pip libgmp3-dev libblas-dev liblapack-dev libv4l-dev subversion libxmu-dev libusb-1.0-0-dev python-pymodbus graphviz curl libwww-perl libterm-readkey-perl g++-4.4

    sudo apt-get --yes install ros-indigo-libpcan # for wsg hand
    sudo apt-get --yes install libopenal-dev # for drake converting wrl file
    sudo apt-get --yes install libgl1-mesa-dev  # for libbot libGL.so
    # sudo apt-get --yes install virtualbox
    sudo apt-get --yes install compizconfig-settings-manager
    #sudo pip install --upgrade scipy
    sudo pip install chan 
    sudo pip install openpyxl
    sudo easy_install pycollada
fi


if [ "$#" == 0 ] || [ "$1" == "ROS" ]; then
    echo "Install ROS"
    # no support for 14.04.02 
    #sudo apt-get --yes install xserver-xorg-dev-lts-utopic mesa-common-dev-lts-utopic libxatracker-dev-lts-utopic libopenvg1-mesa-dev-lts-utopic libgles2-mesa-dev-lts-utopic libgles1-mesa-dev-lts-utopic libgl1-mesa-dev-lts-utopic libgbm-dev-lts-utopic libegl1-mesa-dev-lts-utopic 
    # 
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
    wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get --yes install ros-indigo-desktop-full
    sudo rosdep init
    rosdep update
    sudo apt-get --yes install python-rosinstall
    source /opt/ros/indigo/setup.bash
    
    sudo apt-get --yes install ros-indigo-moveit-full
    sudo apt-get --yes install ros-indigo-pcl-ros
    sudo apt-get --yes install ros-indigo-joy
    sudo apt-get --yes install ros-indigo-perception  # for cv_bridge
    #sudo apt-get --yes install ros-indigo-moveit-full-pr2
    
fi

#if [ "$#" == 0 ] || [ "$1" == "ROSJAVA" ]; then
    #echo "Install ROS JAVA"
    #mkdir -p ~/rosjava
    #wstool init -j4 ~/rosjava/src https://raw.githubusercontent.com/rosjava/rosjava/indigo/rosjava.rosinstall
    #source /opt/ros/indigo/setup.bash
    #cd ~/rosjava
    ## Make sure we've got all rosdeps and msg packages.
    #rosdep update
    #rosdep install --from-paths src -i -y
    #catkin_make
#fi

if [ "$#" == 0 ] || [ "$1" == "SMACH" ]; then
    echo "Install ROS SMACH"
    sudo apt-get --yes install ros-indigo-smach
    sudo apt-get --yes install ros-indigo-smach-viewer
    sudo sed -i 's/return\ \int(self.read_code())/return\ \int(float(self.read_code()))/g' /opt/ros/indigo/lib/python2.7/dist-packages/xdot/xdot.py
fi

if [ "$#" == 0 ] || [ "$1" == "BIRD" ]; then
    echo "Download packages required for Berkeley dataset"
    sudo apt-get install python-h5py ipython hdfview
fi

if [ "$#" == 0 ] || [ "$1" == "CUDA" ]; then
    echo 'You have to be outside lightdm by pressing Ctrl+Alt+F1, and put cuda_6.5.14_linux_64.run inside ~/Downloads'
    echo 'In the installer please put Yes to every answers and install samples to /home/mcube/Documents/'
    ask "Ready? [y/N]" 
    #sudo apt-get install g++
    sudo service lightdm stop
    chmod +x ~/Downloads/cuda_6.5.14_linux_64.run
    ~/Downloads/cuda_6.5.14_linux_64.run
    
    ask "Install successful? If the installer tell you to reboot please press N and reboot? [y/N]" 
    sudo rm -rf /usr/local/cuda-6.5/include/CL  # this old version of CL inside it cause error for iai_kinect

    echo "Testing cuda by making and running cuda samples"
    export PATH=/usr/local/cuda-6.5/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda-6.5/lib64:$LD_LIBRARY_PATH
    sudo ldconfig 
    cd ~/Documents/NVIDIA_CUDA-6.5_Samples
    make -j
    1_Utilities/deviceQuery/deviceQuery 
    echo "Restarting lightdm, please reboot you computer about 3 times to make sure the lightdm works properly after cuda driver is installed."
    sudo service lightdm start
fi

if [ "$#" == 0 ] || [ "$1" == "CUDA7" ]; then
    #echo 'You have to be outside lightdm by pressing Ctrl+Alt+F1'
    #ask "Ready? [y/N]" 
    #sudo apt-get install g++
    sudo service lightdm stop
    wget -P ~/Downloads http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/cuda-repo-ubuntu1404_7.0-28_amd64.deb 
    sudo dpkg -i ~/Downloads/cuda-repo-ubuntu1404_7.0-28_amd64.deb
    sudo apt-get update
    sudo apt-get install cuda
    
    ask "Install successful? " 
    sudo rm -rf /usr/local/cuda-7.0/include/CL  # this old version of CL inside it cause error for iai_kinect

    echo "Testing cuda by making and running cuda samples"
    #export PATH=/usr/local/cuda-7.0/bin:$PATH
    #export LD_LIBRARY_PATH=/usr/local/cuda-7.0/lib64:$LD_LIBRARY_PATH
    sudo ldconfig 
    sudo ldconfig /usr/local/cuda/lib64
    cd /usr/local/cuda-7.0/samples
    sudo make -j
    1_Utilities/deviceQuery/deviceQuery 
    echo "Restarting lightdm, please reboot you computer about 3 times to make sure the lightdm works properly after cuda driver is installed."
    sudo service lightdm start
fi

if [ "$#" == 0 ] || [ "$1" == "MATLAB" ]; then
    echo "Please download matlab to ~/Downloads/matlab_R2015a_glnxa64.zip"
    echo "In the install wizard, install the matlab to your home folder e.g. /home/mcube/MATLAB/R2015a"
    ask "Ready? [y/N]" 
    cd ~/Downloads/
    unzip -o matlab_R2015a_glnxa64.zip -d matlab_R2015a_glnxa64
    cd matlab_R2015a_glnxa64
    ./install
    sudo ln -s  -f $HOME/MATLAB/R2015a/bin/matlab /usr/local/bin/matlab
    sudo ln -s  -f $HOME/MATLAB/R2015a/bin/mex /usr/local/bin/mex
    sudo ln -s  -f $HOME/MATLAB/R2015a/bin/mbuild /usr/local/bin/mbuild
    sudo ln -s  -f $HOME/MATLAB/R2015a/bin/mcc /usr/local/bin/mcc
fi

if [ "$#" == 0 ] || [ "$1" == "MATLABCOMPILER" ]; then  # for drake
    cd $HOME/MATLAB/R2015a/sys/os/glnxa64
    rm libgfortran.so.3
    ln -s /usr/lib/x86_64-linux-gnu/libgfortran.so.3.0.0 libgfortran.so.3
    rm libstdc++.so.6
    ln -s /usr/lib/gcc/x86_64-linux-gnu/4.4/libstdc++.so libstdc++.so.6
fi



if [ "$#" == 0 ] || [ "$1" == "SOFTWARE" ]; then
    echo "Make SOFTWARE"
    cd software/externals/drake
    git checkout master
    git pull
    git submodule update --init --recursive
    cd ../../
    make -j
fi

if [ "$#" == 0 ] || [ "$1" == "KINECT" ]; then
    sudo apt-get update
    sudo apt-get install -y build-essential libturbojpeg libtool autoconf libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev libopencv-dev libglewmx-dev libglew-dev gnuplot gnuplot-x11
    sudo ln -s /usr/lib/x86_64-linux-gnu/libturbojpeg.so.0 /usr/lib/x86_64-linux-gnu/libturbojpeg.so

    cd software/externals/libfreenect2/depends
    ./install_ubuntu.sh
    mkdir ../build
    cd ../build
    cmake ../examples/protonect/ -DENABLE_CXX11=ON
    make && sudo make install

    source /opt/ros/indigo/setup.bash
    cd $APC_BASE/catkin_ws
    catkin_make -DCMAKE_BUILD_TYPE="Release" --pkg iai_kinect2
    
    echo 'Adding permissions'
    sudo cp $APC_BASE/catkin_ws/src/apc_config/kinect_install/90-kinect2.rules /etc/udev/rules.d/
    
    echo "Please reconnect your kinect"
    echo "Test with: roscore"
    echo "           rosrun kinect2_bridge kinect2_bridge"
    echo "           rosrun registration_viewer viewer kinect2 sd cloud"
fi

#if [ "$#" == 0 ] || [ "$1" == "CUDA" ]; then
    #cd ~/Downloads
    #wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/cuda-repo-ubuntu1404_6.5-14_amd64.deb
    #sudo dpkg -i cuda-repo-ubuntu1404_6.5-14_amd64.deb
    #sudo apt-get update
    #sudo apt-get install cuda-repo-ubuntu1404
#fi

if [ "$#" == 0 ] || [ "$1" == "CATKIN" ]; then 
    echo "Make CATKIN"
    cd $APC_BASE/catkin_ws
    catkin_make
fi

if [ "$#" == 0 ] || [ "$1" == "MOPED" ]; then  # should remove this
    echo "Make MOPED"
    rosmake moped-modeling-py 
        
    # for camera calibration
    sudo apt-get --yes install mrpt-apps libmrpt-dev
    
    # for building models
    sudo apt-get --yes install imagemagick gfortran exuberant-ctags libatlas3gf-base minpack-dev
    cd ros_ws/perception/moped/pyublas/ && \
    make -C src && \
    mkdir -p lib && \
    cp src/build/lib.linux-x86_64-2.7/pyublas/*.so lib/
fi


if [ "$#" == 0 ] || [ "$1" == "ABB" ]; then
    echo "Make abb-ros"
    rosmake robot_comm robot_node
fi

if [ "$#" == 0 ] || [ "$1" == "FFMPEG" ]; then  ## needed to compile drake movies
    cd ~/Downloads/
    wget -q http://web.mit.edu/peterkty/www/shared/ffmpeg/ffmpeg-2.4.2-64bit-static.tar.gz  # should move to some labspace
    mkdir -p $HOME/software
    tar -zxvf ffmpeg-2.4.2-64bit-static.tar.gz -C $HOME/software
    echo 'FFMPEG install done'
fi

if [ "$#" == 0 ] || [ "$1" == "HAND" ]; then 
    echo 'Do catkin_make --pkg wsg_50_common wsg_50_driver wsg_50_simulation'
    cd $APC_BASE/catkin_ws
    catkin_make --pkg wsg_50_common wsg_50_driver wsg_50_simulation
fi

if [ "$#" == 0 ] || [ "$1" == "LABJACK" ]; then 
    cd $APC_BASE/software/labjack/exodriver-master
    sudo ./install.sh
    cd $APC_BASE/software/labjack/LabJackPython-4-24-2014
    sudo python setup.py install
fi

