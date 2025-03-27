#!/bin/bash


cd "$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )" || exit
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


echo_blue () { echo -e "\033[1;38;5;051m$* \033[0m"; }
echo_orange () { echo -e "\033[1;38;5;202m$* \033[0m"; }
echo_green () { echo -e "\033[1;38;5;047m$* \033[0m"; }
remove(){ if [ -d "$1" ]; then rm -rf "$1"; fi }
add(){ if [ ! -d "$1" ]; then mkdir -p "$1"; fi }

ROOT=$PWD


if [[ $# -eq 0 ]] || [[ "$*" =~ "--help" ]] || [[ "$*" =~ "-h" ]];then
    echo "$usage"
    exit 0
fi


## ==========================================================================
## ----------------------- singularity container setup ------------------- ##
## ==========================================================================
if [[ "$1" =~ "cont_pull" ]] || [[ "$1" =~ "all" ]];then
    echo_blue "Pulling singularity container..."
    wget --no-check-certificate "https://yaleedu-my.sharepoint.com/:u:/g/personal/aalap_shah_yale_edu/EVysC-tuizdIgs9wmTrXYwoBjwkTOXNlF9kTLsRMfbPj4w?e=M4ZrLd&download=1" -O "${ENV[cont_init]}"
    wget --no-check-certificate "https://yaleedu-my.sharepoint.com/:u:/g/personal/aalap_shah_yale_edu/EaFj9lg1b-5FseCdNFSJkcQB5VAYydkXVqjYYQu0LSeeJg?e=njvmDj&download=1" -O "${ENV[cont_main]}"
elif [[ "$1" =~ "cont_build" ]];then
    echo_blue "Building apptainer container..."
    remove "${ENV[cont_main]}"
    sudo -E apptainer build "${ENV[cont_main]}" "${ENV[cont_def]}"
else
    echo_green "Not touching container"
fi


## ==========================================================================
## ----------------------- python setup ---------------------------------- ##
## ==========================================================================
if [[ "$1" =~ "python" ]] || [[ "$1" =~ "all" ]];then
    echo_blue "Setting up Python venv..."
    singularity exec "${ENV[cont_main]}" bash -c "python -m venv ${ENV[env]}"
    ./run.sh s "python -m pip install --upgrade pip"
    ./run.sh s "python -m pip install pillow==10.4.0"
    ./run.sh s "python -m pip install numpy==1.24.4"
    ./run.sh s "python -m pip install scipy==1.10.1"
    ./run.sh s "python -m pip install scikit-learn==1.3.2"
    ./run.sh s "python -m pip install matplotlib==3.7.5"
    ./run.sh s "python -m pip install click==8.1.7"
    ./run.sh s "python -m pip install click-help-colors==0.9.4"
    ./run.sh s "python -m pip install shapely==2.0.6"
    ./run.sh s "python -m pip install seaborn==0.13.2"
    ./run.sh s "python -m pip install python-dotenv==1.0.1"
    ./run.sh s "python -m pip install openai==1.60.1"
    ./run.sh s "python -m pip install puremagic==1.28"
    ./run.sh s "python -m pip install xmltodict==0.14.2"

else
    echo_green "Not touching python"
fi


## ==========================================================================
## ------------------------- submodules setup  --------------------------- ##
## ==========================================================================
if [[ "$*" =~ "subs" ]] || [[ "$*" =~ "all" ]];then
    echo "Pulling submodules..."
else
    echo_green "Not touching submodules"
fi


## ==========================================================================
## ----------------------- data setup ------------------------------------ ##
## ==========================================================================
if [[ "$1" =~ "data" ]] || [[ "$1" =~ "all" ]];then
    echo_blue "Pulling data..."
    wget --no-check-certificate "https://yaleedu-my.sharepoint.com/:u:/g/personal/aalap_shah_yale_edu/EU5kMTlcjEtJiT7VrRsJ85EBq7wzKnjrUVCwGi2rt6Gn4g?e=Od3cZ8&download=1" -O library.zip
    rm -rf mlr/share/projects/block_building/library
    chmod +777 library.zip
    unzip library.zip
    mv library mlr/share/projects/block_building
    rm -rf library.zip
else
    echo_green "Not pulling any data"
fi
