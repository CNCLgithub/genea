#!/bin/bash

usage="Syntax: $(basename "$0") [-h|--help] [COMPONENTS...] -- will set up apptainer,
where:
    -h | --help     Print this help
    COMPONENTS...   Specify component to set up

Valid COMPONENTS:
    s1: run step 1 to download go
    s2: run step 2 -- follow the command pasted
    s3: run step 3 to download apptainer
    s4: run step 4 to download gocryptfs

NOTE: Run each of these separately, one after another!
"


echo_green () { echo -e "\033[1;38;5;047m$* \033[0m"; }
echo_blue () { echo -e "\033[1;38;5;051m$* \033[0m"; }
echo_orange () { echo -e "\033[1;38;5;202m$* \033[0m"; }
echo_red () { echo -e "\033[1;38;5;196m$* \033[0m"; }
echo_yellow () { echo -e "\033[1;38;5;226m$* \033[0m"; }
echo_purple () { echo -e "\033[1;38;5;141m$* \033[0m"; }
remove(){ if [ -d "$1" ]; then rm -rf "$1"; fi }
add(){ if [ ! -d "$1" ]; then mkdir -p "$1"; fi }


if [[ $# -eq 0 ]] || [[ "$*" =~ "--help" ]] || [[ "$*" =~ "-h" ]];then
    echo "$usage"
    exit 0
fi

## ==========================================================================
## ----------------------------- setup step 0 ---------------------------- ##
## ==========================================================================
if [[ "$*" =~ "s0" ]];then
    sudo rm -rf /usr/local/go
fi


## ==========================================================================
## ----------------------------- setup step 1 ---------------------------- ##
## ==========================================================================
if [[ "$*" =~ "s1" ]];then
  sudo apt-get update
  # Install debian packages for dependencies
  sudo apt-get install -y \
      build-essential \
      libseccomp-dev \
      pkg-config \
      uidmap \
      squashfs-tools \
      squashfuse \
      fuse2fs \
      fuse-overlayfs \
      fakeroot \
      cryptsetup \
      curl wget git

  export GOVERSION=1.21.4 OS=linux ARCH=amd64
  remove "/usr/local/go"
  wget -O /tmp/go${GOVERSION}.${OS}-${ARCH}.tar.gz https://dl.google.com/go/go${GOVERSION}.${OS}-${ARCH}.tar.gz
  sudo tar -C /usr/local -xzf /tmp/go${GOVERSION}.${OS}-${ARCH}.tar.gz

  echo "export PATH=$PATH:/usr/local/go/bin" >> ~/.bashrc
fi

## ==========================================================================
## ----------------------------- setup step 2 ---------------------------- ##
## ==========================================================================
if [[ "$*" =~ "s2" ]];then
  echo_blue RUN THE FOLLOWING COMMAND:
  echo source ~/.bashrc
fi


## ==========================================================================
## ----------------------------- setup step 3 ---------------------------- ##
## ==========================================================================
if [[ "$*" =~ "s3" ]];then
  git clone https://github.com/apptainer/apptainer.git
  # shellcheck disable=SC2164
  cd apptainer
  git checkout v1.2.4

  ./mconfig -p /usr/local
  # shellcheck disable=SC2164
  cd ./builddir
  make
  sudo make install

  apptainer --version
fi

## ==========================================================================
## ----------------------------- setup step 4 ---------------------------- ##
## ==========================================================================
if [[ "$*" =~ "s4" ]];then
  GOCRYPTFSVERSION=2.4.0
  curl -L -O https://github.com/rfjakob/gocryptfs/archive/v$GOCRYPTFSVERSION/gocryptfs-$GOCRYPTFSVERSION.tar.gz

  tar xzf gocryptfs-$GOCRYPTFSVERSION.tar.gz
  # shellcheck disable=SC2164
  cd gocryptfs-$GOCRYPTFSVERSION
  ./build-without-openssl.bash
  sudo cp gocryptfs /usr/local/libexec/apptainer/bin
fi
