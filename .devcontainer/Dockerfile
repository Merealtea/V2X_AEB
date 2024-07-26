FROM ros:melodic-perception-bionic
# Args for the non-root user
ARG USERNAME=platoon
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Change repository to SJTU mirror
RUN sudo sed -i 's/http:\/\/archive.ubuntu.com/http:\/\/mirror.sjtu.edu.cn/g' /etc/apt/sources.list
RUN sudo sed -i 's/http:\/\/packages.ros.org/http:\/\/mirror.sjtu.edu.cn/g' /etc/apt/sources.list.d/ros1-latest.list
# Update and install general packages
RUN apt-get update && apt-get upgrade -y\
  && apt-get install -q -y openssh-client vim bash-completion
# Project related packages
RUN apt-get -q -y install ros-melodic-mrpt2 libgoogle-glog-dev python-pip
RUN pip install pyserial modbus_tk
# Build and install osqp
WORKDIR /home/${USERNAME}/third_party
RUN git clone --recursive https://github.com/osqp/osqp
WORKDIR osqp/build
RUN cmake -G "Unix Makefiles" ..
RUN cmake --build .
RUN cmake --build . --target install
RUN ldconfig
USER $USERNAME
WORKDIR /home/$USERNAME
RUN mkdir cyberc3_platooning
RUN echo 'source /opt/ros/melodic/setup.bash' >> /home/$USERNAME/.bashrc
RUN echo 'source /home/platoon/cyberc3_platooning/devel/setup.bash' >> /home/$USERNAME/.bashrc
