FROM osrf/ros:melodic-desktop-full

COPY requirements.txt /opt/app/requirements.txt
RUN apt-get update && apt-get install -y apt-utils build-essential psmisc vim-gtk
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN apt-get update && apt-get install -q -y python-catkin-tools
RUN apt-get update && apt-get install -q -y ros-melodic-hector-gazebo-plugins

RUN apt-get install -y python-pip
RUN pip install -r /opt/app/requirements.txt

##Initialize Catkin worskpace:
ENV MY_WS=/catkin_ws
RUN source /opt/ros/melodic/setup.bash && \
    mkdir -p $MY_WS/src && \
    cd $MY_WS/src && \
    catkin_init_workspace && \
    cd $MY_WS && \
    catkin_make && \
    source /$MY_WS/devel/setup.bash && \
    cd $MY_WS/src && \ 
    catkin_create_pkg navigation rospy

COPY catkin_ws/src/navigation /$MY_WS/src/navigation

RUN source /$MY_WS/devel/setup.bash
