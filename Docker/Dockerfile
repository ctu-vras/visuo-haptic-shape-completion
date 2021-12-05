FROM nvidia/cuda:11.2.1-cudnn8-devel-ubuntu18.04
# start with the nvidia container for latest CUDA and cudnn

LABEL maintainer "Lukas Rustler <rustlluk@fel.cvut.cz>"

#Non-interactive mode
ENV DEBIAN_FRONTEND='noninteractive'

#First update
RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install wget sudo -y 

#ROS repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#Chrome repository
RUN wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | apt-key add -
RUN echo 'deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main' | tee /etc/apt/sources.list.d/google-chrome.list

#Update and install all
RUN apt-get update
RUN apt-get install unzip apt-utils curl python3.6 python3-distutils python3-venv python3-opengl gedit nano libsm6 libxext6 libxrender-dev git python-pytest pkg-config python2.7 python-pip libpcl-dev ros-melodic-ros-base python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-python-orocos-kdl ros-melodic-tf-conversions cmake pkg-config yasm git gfortran libjpeg8-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine2-dev libv4l-dev qt5-default libgtk2.0-dev libtbb-dev libatlas-base-dev libfaac-dev libmp3lame-dev libtheora-dev libvorbis-dev libxvidcore-dev libopencore-amrnb-dev libopencore-amrwb-dev x264 v4l-utils libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev libgphoto2-dev libeigen3-dev libhdf5-dev doxygen ros-melodic-catkin python-catkin-tools python3-opencv ros-melodic-hardware-interface ros-melodic-controller-manager ros-melodic-cv-bridge ros-melodic-image-transport libglfw3-dev ros-melodic-pcl-ros ros-melodic-control-msgs ros-melodic-moveit* glew-utils libglew-dev mesa-utils libgl1-mesa-glx libgl1-mesa-dri ros-melodic-xacro xfce4 xfce4-goodies xserver-xorg-video-dummy xserver-xorg-legacy x11vnc firefox apt-utils software-properties-common apt-transport-https sudo psmisc tmux nano wget curl gedit gdb git gitk autoconf locales gdebi terminator meld dos2unix magic-wormhole ros-melodic-ompl* ros-melodic-joint-* ros-melodic-robot-state-publisher ros-melodic-industrial* x11-apps xvfb python-tk meshlab google-chrome-stable -y

#Remove unnecessary 
RUN apt remove -y xfce4-power-manager light-locker && sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config

#Set locale
RUN locale-gen en_US.UTF-8

# Python 3.6
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python3.6 get-pip.py
RUN pip3 install opencv-python plyfile scikit-image trimesh torch==1.0.0 pyyaml open3d scipy pyhocon plotly pymeshfix

# Python 2.7
RUN apt-get install  -y
RUN pip2 install --upgrade pip
RUN pip2 install torch numpy cython sklearn pyyaml future catkin_pkg trimesh open3d llvmlite==0.31.0
 
#Prepare git dump folder
WORKDIR /root
RUN mkdir git

#CURVOX
WORKDIR /root/git
RUN git clone https://github.com/numba/numba.git
WORKDIR numba
RUN git checkout release0.40
RUN python2 setup.py install
WORKDIR /root/git
RUN git clone https://github.com/jsll/curvox
WORKDIR curvox
RUN wget http://www.patrickmin.com/binvox/linux64/binvox
RUN chmod a+x binvox
RUN mkdir /root/.local
RUN mkdir /root/.local/bin
RUN mv binvox /root/.local/bin
RUN pip2 install -r requirements.txt
RUN python2 setup.py install
RUN pip2 install colorama==0.3.9
RUN pip2 install pymcubes==0.0.12
ADD binvox_rw.py /usr/lib/python2.7/
WORKDIR /root

#ROS init
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN rosdep init
RUN ln -s /root/.local/bin/binvox /usr/bin/binvox


#OpenCV
WORKDIR /root/git
RUN git clone https://github.com/opencv/opencv.git
RUN git clone https://github.com/opencv/opencv_contrib.git
WORKDIR opencv
RUN mkdir build
WORKDIR build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
RUN make -j4
RUN sudo make install
RUN sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
RUN sudo ldconfig

#XORG setup
COPY xorg.conf /etc/X11/xorg.conf
RUN dos2unix /etc/X11/xorg.conf

#Browser settings
ENV DONT_PROMPT_WSL_INSTALL=1
ENV DISPLAY=:1
RUN update-alternatives --install /usr/bin/x-www-browser x-www-browser /usr/bin/firefox 500
RUN update-alternatives --set x-www-browser /usr/bin/firefox

#noVNC
WORKDIR /root/git
RUN git clone https://github.com/novnc/noVNC.git /opt/novnc && \
    git clone https://github.com/novnc/websockify /opt/novnc/utils/websockify && \
    echo "<html><head><meta http-equiv=\"Refresh\" content=\"0; url=vnc.html?autoconnect=true&reconnect=true&reconnect_delay=1000&resize=scale&quality=9\"></head></html>" > /opt/novnc/index.html
COPY start-vnc-session.sh /usr/bin/start-vnc-session.sh
RUN chmod +x /usr/bin/start-vnc-session.sh && \
    dos2unix /usr/bin/start-vnc-session.sh
EXPOSE 5901 6080 10000/tcp 10000/udp

#MuJoCo
WORKDIR /root
RUN mkdir .mujoco
WORKDIR /root/.mujoco
RUN wget "https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz"
RUN tar -xzf mujoco210-linux-x86_64.tar.gz
RUN rm -r mujoco210-linux-x86_64.tar.gz

#Repository
WORKDIR /root
RUN mkdir act_vh_ws
WORKDIR /root/act_vh_ws
RUN git clone https://github.com/ctu-vras/visuo-haptic-shape-completion.git
RUN mv visuo-haptic-shape-completion src
RUN catkin init
RUn catkin config --extend /opt/ros/melodic
RUN catkin build
RUN sudo chmod +x /root/act_vh_ws/src/Fast-Robust-ICP/build/FRICP
RUN sudo chmod +x /root/act_vh_ws/src/v-hacd/build/linux2/test/testVHACD
RUN sudo chmod +x /root/act_vh_ws/src/shape_completion/scripts/run_simulation.sh
RUN ln -s /root/act_vh_ws/src/v-hacd/build/linux2/test/testVHACD /usr/bin/testVHACD
RUN ln -s /root/act_vh_ws/src/Fast-Robust-ICP/build/FRICP /usr/bin/FRICP
RUN ln -s /root/act_vh_ws/src/shape_completion/scripts/run_simulation.sh /usr/bin/run_simulation
RUN pip3 install gdown
WORKDIR /root/act_vh_ws/src/kinova_mujoco
RUN gdown "https://drive.google.com/uc?id=1KKg9DpzX_2IhxWZsIzGgwEh5kDoz6eCq"
RUN rm -r GT_meshes
RUN unzip GT_meshes.zip -d GT_meshes
RUN rm -r GT_meshes.zip
RUN echo 'source /root/act_vh_ws/devel/setup.bash' >> /root/.bashrc
WORKDIR /root/act_vh_ws
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
RUN echo "alias shape_rec_p3=''" >> /root/.bashrc
RUN echo "alias shape_rec_p2=''" >> /root/.bashrc

#Delete trash
RUN rm -Rf /var/lib/apt/lists/
CMD ["bash"]
