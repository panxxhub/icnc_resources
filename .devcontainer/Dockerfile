ARG ROS_DISTRO=galactic
FROM moveit/moveit2:${ROS_DISTRO}-release

#https://mirrors.cloud.tencent.com/
RUN sed -i "s/http:\/\/archive.ubuntu.com/https:\/\/mirrors.cloud.tencent.com/g" /etc/apt/sources.list

RUN apt-get update -y && apt-get dist-upgrade -y 

RUN apt-get install -y git python3-pip

RUN pip install mypy autopep8
