#!/bin/bash


. load_config.sh


usage="Syntax: $(basename "$0") [-h|--help] [COMPONENTS...] -- will set up the project environment,
where:
    -h | --help     Print this help
    COMPONENTS...   Specify component to set up

Valid COMPONENTS:
    all: setup all components (container will be pulled, not built; python packages will be installed)
    cont_[pull|build]: pull the singularity container or build it
    python: add python venv and install necessary packages
    p_remove: remove python venv and installed packages
    data: pull data"


echo_blue () { echo -e "\033[0;36m $* \033[0m"; }
echo_orange () { echo -e "\033[1;38;5;202m$* \033[0m"; }
remove(){ if [ -d "$1" ]; then rm -rf "$1"; fi }
add(){ if [ ! -d "$1" ]; then mkdir -p "$1"; fi }


if [[ $# -eq 0 ]] || [[ "$*" =~ "--help" ]] || [[ "$*" =~ "-h" ]];then
    echo "$usage"
    exit 0
fi


## ==========================================================================
## ------------------------- singularity container setup ------------------- ##
## ==========================================================================
if [[ "$*" =~ "cont_pull" ]] || [[ "$*" =~ "all" ]];then
    echo_blue "Pulling singularity container..."
    wget --no-check-certificate "https://yaleedu-my.sharepoint.com/:u:/g/personal/aalap_shah_yale_edu/EVysC-tuizdIgs9wmTrXYwoBjwkTOXNlF9kTLsRMfbPj4w?e=Wav01N&download=1" -O "${ENV[cont_init]}"
    wget --no-check-certificate "https://yaleedu-my.sharepoint.com/:u:/g/personal/aalap_shah_yale_edu/EaFj9lg1b-5FseCdNFSJkcQB5VAYydkXVqjYYQu0LSeeJg?e=njvmDj&download=1" -O "${ENV[cont_main]}"
elif [[ "$*" =~ "cont_build" ]];then
    echo_blue "Building apptainer container..."
    remove "${ENV[cont_main]}"
    sudo -E apptainer build "${ENV[cont_main]}" "${ENV[cont_def]}"
else
    echo_orange "Not touching container"
fi


## ==========================================================================
## ------------------------- python setup ---------------------------------- ##
## ==========================================================================
if [[ "$*" =~ "python" ]] || [[ "$*" =~ "all" ]];then
    echo_blue "Setting up Python venv..."
    singularity exec "${ENV[cont_main]}" bash -c "python -m venv ${ENV[env]}"
    ./run.sh "python -m pip install --upgrade pip"
    ./run.sh "python -m pip install pillow==10.4.0"
    ./run.sh "python -m pip install numpy==1.24.4"
    ./run.sh "python -m pip install scipy==1.10.1"
    ./run.sh "python -m pip install scikit-learn==1.3.2"
    ./run.sh "python -m pip install matplotlib==3.7.5"
    ./run.sh "python -m pip install click==8.1.7"
    ./run.sh "python -m pip install click-help-colors==0.9.4"
fi


## ==========================================================================
## ------------------------- data setup ------------------------------------ ##
## ==========================================================================
if [[ "$*" =~ "data" ]] || [[ "$*" =~ "all" ]];then
    echo_blue "Pulling data..."
    wget --no-check-certificate "https://yaleedu-my.sharepoint.com/:u:/g/personal/aalap_shah_yale_edu/EU5kMTlcjEtJiT7VrRsJ85EBf-QTOi0D4bb1uDjYoZ2clg?e=lcyCuL&download=1" -O library.zip
    rm -rf mlr/share/projects/block_building/library
    chmod +777 library.zip
    unzip library.zip
    mv library mlr/share/projects/block_building
    rm -rf library.zip
else
    echo_orange "Not pulling any data"
fi
