ARG ROS_DISTRO=galactic
FROM moveit/moveit2:${ROS_DISTRO}-release

#https://mirrors.cloud.tencent.com/
ENV http_proxy 	http://proxy.cohort-study.com:43890
ENV https_proxy http://proxy.cohort-study.com:43890
ENV all_proxy 	socks5://proxy.cohort-study.com:43890

RUN sed -i "s/http:\/\/archive.ubuntu.com/https:\/\/mirrors.cloud.tencent.com/g" /etc/apt/sources.list

RUN apt-get update -y && apt-get dist-upgrade -y 

RUN apt-get install -y git python3-pip

RUN pip install mypy autopep8
