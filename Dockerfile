FROM osrf/ros:noetic-desktop-full

#install ros package

RUN apt-get update \
 && apt-get install -y git \
                    python3-pip \
                    python3-tk \
                    nano \
                    bash-completion\
                    python3-argcomplete\
 ##&& mkdir -p /home/$USERNAME/catkin_ws/src \
 && rm -rf /var/lib/apt/lists/*
#RUN source /opt/ros/noetic/setup.bash
#RUN git clone 

#create a new user 
ARG USERNAME=khoi
ARG USER_UID=1000  
#1000 is home User ID
ARG USER_GID=$USER_UID
#Group ID = 1000


#create a non-root-user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

#set up sudo privilege/add ability to use sudo(alternative:docker exec to create a new terminal in container with root access )
RUN apt-get update \
 && apt-get install -y sudo \
 && echo $USERNAME ALL=\(root\) NOPASSWD: ALL > /etc/sudoers.d/$USERNAME\
 && chmod 0440 /etc/sudoers.d/$USERNAME \
 && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]
        #execute the script with
        # intepreter /bin/bash 

 #pass/übergeben commands als Argument zum Execute into entrypoint
 CMD ["bash"]       

 RUN echo "ALL Done"