FROM ros:noetic-ros-core-focal

#Create folder for Development and setup run scripts
COPY /shared/setup_ws.sh /home
ENV VISAVIS_WS=/home/visavis_ws/

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
	python3-pip \
    && rm -rf /var/lib/apt/lists/*
	
RUN pip install numpy
RUN pip install scipy
RUN pip install pandas
RUN pip install tk
	
# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
	git-all \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/visavis_ws/src && \
			cd /home/visavis_ws/src  && \
			git clone https://github.com/AndreaRoberti/vivavis_vision.git
			
# MAKE 
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash && cd /home/visavis_ws/ && catkin_make"
RUN echo '#!/bin/bash\ncd $VISAVIS_WS; source devel/setup.bash'
RUN chmod a+x /home/setup_ws.sh
