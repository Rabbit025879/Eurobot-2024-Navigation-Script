##############################################################################
# Base Image: ros-noetic ( Ubuntu 20.04 Focal Fossa )
# Source: https://hub.docker.com/layers/osrf/ros/noetic-desktop-full-focal/images/sha256-70037dab062e8edf988261a1ab937182676a984036219ebac4b8ec2ce6d1159e?context=explore 
##############################################################################

FROM osrf/ros:noetic-desktop-full-focal

##############################################################################
# Arguments
##############################################################################

ARG USERNAME=user
# [IMPORTANT 1/3] Change USERID to your user id
ARG USERID=1003
ARG USERGID=${USERID}
ARG SHELL=bash

##############################################################################
# Label
##############################################################################

LABEL org.opencontainers.image.authors="yuzhong"
LABEL shell="${SHELL}"

##############################################################################
# Set environment variables
##############################################################################

ENV USERNAME=${USERNAME}
ENV SHELL=/bin/${SHELL}
ENV TERM=xterm-256color

##############################################################################
# Install packages
##############################################################################

RUN apt-get update && \
    apt-get install -y \
    sudo \
    vim \
    curl \
    wget \
    tmux \
    htop \
    git \
    git-extras \
    gnupg2 \
    net-tools \
    locales

# Install dependencies for ROS
RUN apt-get update && \
    apt-get install -y \
    libsdl-image1.2-dev \
    libsdl-dev \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-move-base-msgs \
    ros-noetic-mbf-costmap-core \
    ros-noetic-mbf-msgs \
    libsuitesparse-dev \
    ros-noetic-libg2o \
    ros-noetic-rosserial-python \
    ros-noetic-rosserial-server

##############################################################################
# Set Language
##############################################################################

RUN echo "en_US.UTF-8 UTF-8" > /etc/local.gen && locale-gen en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8

##############################################################################
# Set timezone (Taipei)
##############################################################################

RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends tzdata
RUN TZ=Asia/Taipei && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata 

##############################################################################
# Add new user (Passwordless with sudo)
##############################################################################

RUN groupadd --gid ${USERGID} ${USERNAME} \
    && useradd --uid ${USERID} --gid ${USERGID} -m ${USERNAME} -s /bin/bash \
    # passwordless
    && apt-get update \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

##############################################################################
# Set Default User & Working Directory & Default .bashrc
##############################################################################

USER ${USERNAME}
WORKDIR /home/${USERNAME}
COPY ./settings/.bashrc /home/${USERNAME}/.bashrc 

##############################################################################
# Clean apt-cache
##############################################################################

RUN sudo apt autoremove -y && \
    sudo apt clean -y 

##############################################################################
# Change to root user
##############################################################################

# USER root/

##############################################################################
# Default command
##############################################################################

SHELL ["/bin/bash", "-c"]
CMD ["bash"]

