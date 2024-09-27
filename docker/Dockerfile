ARG BASE_IMAGE="ros:humble"

FROM ${BASE_IMAGE} AS base
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /root

ARG APT_DEPENDENCIES_FILE="apt-dependencies.txt"
ARG APT_DEPENDENCIES="/tmp/${APT_DEPENDENCIES_FILE}"
COPY ${APT_DEPENDENCIES_FILE} ${APT_DEPENDENCIES}

RUN apt-get update -y && \
    xargs apt-get install -y --no-install-recommends <${APT_DEPENDENCIES} && \
    rm ${APT_DEPENDENCIES} && \
    pip3 install --no-cache-dir --upgrade pip

FROM base AS user_base
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /root
    ## fzf
RUN git clone https://github.com/junegunn/fzf.git ~/.fzf --depth 1 && \
    ~/.fzf/install

    # Git prompt
RUN git clone https://github.com/magicmonty/bash-git-prompt.git ~/.bash-git-prompt --depth=1

    ## BTM
RUN mkdir -p /third_party_libs/btm && \
    BTM_VERSION=$(curl -sI https://github.com/ClementTsang/bottom/releases/latest | awk -F '/' '/^location/ {print  substr($NF, 1, length($NF)-1)}') && \
    curl -Ls "https://github.com/ClementTsang/bottom/releases/download/${BTM_VERSION}/bottom_${BTM_VERSION}-1_amd64.deb" --output /third_party_libs/btm/bottom_amd64.deb && \
    dpkg -i /third_party_libs/btm/bottom_amd64.deb

    ## vim plug
RUN curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim

RUN rm -rf /var/lib/apt/lists/*

COPY vimrc .vimrc
COPY bashrc .bashrc
COPY bashrc.d .bashrc.d
COPY ros_entrypoint.sh /ros_entrypoint.sh

WORKDIR /root/ros_ws
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
