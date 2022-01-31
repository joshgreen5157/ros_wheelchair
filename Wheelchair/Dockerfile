FROM osrf/ros:melodic-desktop-full

COPY requirements.txt /opt/app/requirements.txt
COPY catkin_ws/ /catkin_ws
RUN apt-get update && apt-get install -y apt-utils build-essential psmisc vim-gtk
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN apt-get update && apt-get install -q -y python-catkin-tools
RUN apt-get update && apt-get install -q -y ros-melodic-hector-gazebo-plugins

RUN apt-get install -y python-pip
# RUN source /catkin_ws/devel/setup.bash
RUN pip install -r /opt/app/requirements.txt


