# https://www.generationrobots.com/en/content/83-carry-out-simulations-and-make-your-darwin-op-walk-with-gazebo-and-ros
# https://github.com/abatula/DanceSynth/tree/master
FROM ros:melodic-ros-base

ENV USERNAME darwin_op
ENV HOME /home/$USERNAME

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        # Replace 1000 with your user/group id
        usermod  --uid 1000 $USERNAME && \
  groupmod --gid 1000 $USERNAME

USER darwin_op
WORKDIR /home/${USERNAME}

RUN sudo apt-get update

  # Python and git:
RUN sudo apt-get install -y python python-pip python3-pip git
  # Python packages
RUN sudo pip install catkin-tools scipy

  # Libraries
RUN sudo apt-get install -y libyaml-cpp-dev libeigen3-dev libgoogle-glog-dev ccache tmux  net-tools iputils-ping nano wget usbutils htop gdb psmisc screen

RUN sudo apt-get update

  # Install gazebo:
RUN sudo apt-get install -y gazebo9 libgazebo9-dev 
RUN /bin/bash -c "sudo apt-get install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs"

  # Create a catkin workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"
RUN mkdir -p catkin_ws/src
RUN cd catkin_ws && catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

  # Clone catkin_simple
RUN cd catkin_ws/src && git clone https://github.com/catkin/catkin_simple.git
RUN cd catkin_ws/src && git clone https://github.com/HumaRobotics/darwin_description.git
RUN cd catkin_ws/src && git clone https://github.com/HumaRobotics/darwin_control.git
RUN cd catkin_ws/src && git clone https://github.com/HumaRobotics/darwin_gazebo.git

RUN /bin/bash -c "sudo apt-get install -y ros-${ROS_DISTRO}-controller-manager"
RUN /bin/bash -c "sudo apt-get install -y ros-${ROS_DISTRO}-robot-state-publisher"
RUN /bin/bash -c "sudo apt-get install -y ros-${ROS_DISTRO}-gazebo-ros-control"
# RUN /bin/bash -c "sudo apt-get install -y ros-${ROS_DISTRO}-control"
RUN /bin/bash -c "sudo apt-get install -y ros-${ROS_DISTRO}-ros-controllers"
 
RUN cd catkin_ws && catkin build

  # Give permissions to use tty to user
RUN sudo usermod -a -G tty $USERNAME
RUN sudo usermod -a -G dialout $USERNAME

CMD ["source /home/darwin_op/catkin_ws/devel/setup.bash"]

