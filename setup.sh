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


if [[ $# -eq 0 ]] || [[ "$*" =~ "--help" ]] || [[ "$*" =~ "-h" ]];then
    echo "$usage"
    exit 0
fi


APP_ENV=""
APP_DEF=""
APP_CONT="${ENV['cont_init']}"
if [[ "$1" =~ "bb" ]];then
    APP_ENV="${ENV[env_bb]}"
    APP_DEF="${ENV[def_bb]}"
    APP_CONT="${ENV[cont_bb]}"
elif [[ "$1" =~ "nav" ]];then
    APP_ENV="${ENV[env_nav]}"
    APP_DEF="${ENV[def_nav]}"
    APP_CONT="${ENV[cont_nav]}"
fi


## ==========================================================================
## ----------------------- singularity container setup ------------------- ##
## ==========================================================================
if [[ "$2" =~ "cont_pull" ]];then
    echo_blue "Pulling singularity container..."
    wget --no-check-certificate "https://yaleedu-my.sharepoint.com/:u:/g/personal/aalap_shah_yale_edu/EVysC-tuizdIgs9wmTrXYwoBjwkTOXNlF9kTLsRMfbPj4w?e=M4ZrLd&download=1" -O "${ENV[cont_init]}"
    wget --no-check-certificate "https://yaleedu-my.sharepoint.com/:u:/g/personal/aalap_shah_yale_edu/EaFj9lg1b-5FseCdNFSJkcQB5VAYydkXVqjYYQu0LSeeJg?e=njvmDj&download=1" -O "${ENV[cont_bb]}"
elif [[ "$2" =~ "cont_build" ]];then
    echo_blue "Building apptainer container..."
    remove "${APP_CONT}"
    sudo -E apptainer build "${APP_CONT}" "${APP_DEF}"
else
    echo_green "Not touching container"
fi


## ==========================================================================
## ----------------------- python setup ---------------------------------- ##
## ==========================================================================
if [[ "$1" =~ "bb" ]] && [[ "$2" =~ "python" ]];then
    echo_blue "Setting up Python ${APP_ENV}..."
    apptainer exec "${APP_CONT}" bash -c "python -m venv ${APP_ENV}"
    ./run.sh "$1" "python -m pip install --upgrade pip --no-cache-dir"
    ./run.sh "$1" "python -m pip install pillow==10.4.0"
    ./run.sh "$1" "python -m pip install numpy==1.24.4"
    ./run.sh "$1" "python -m pip install scipy==1.10.1"
    ./run.sh "$1" "python -m pip install scikit-learn==1.3.2"
    ./run.sh "$1" "python -m pip install matplotlib==3.7.5"
    ./run.sh "$1" "python -m pip install click==8.1.7"
    ./run.sh "$1" "python -m pip install click-help-colors==0.9.4"
    ./run.sh "$1" "python -m pip install shapely==2.0.6"
    ./run.sh "$1" "python -m pip install seaborn==0.13.2"
    ./run.sh "$1" "python -m pip install statsmodels==0.14.1"
elif [[ "$1" =~ "nav" ]] && [[ "$2" =~ "python" ]];then
    echo_blue "Setting up Python ${APP_ENV}..."
    apptainer exec "${APP_CONT}" bash -c "python -m venv ${APP_ENV}"
    ./run.sh "$1" "python -m pip install --upgrade pip --no-cache-dir"
    ./run.sh "$1" "python -m pip install pillow==12.2.0"
    ./run.sh "$1" "python -m pip install numpy==2.4.6"
    ./run.sh "$1" "python -m pip install scipy==1.17.1"
    ./run.sh "$1" "python -m pip install scikit-learn==1.8.0"
    ./run.sh "$1" "python -m pip install matplotlib==3.10.9"
    ./run.sh "$1" "python -m pip install click==8.4.1"
    ./run.sh "$1" "python -m pip install click-help-colors==0.9.4"
    ./run.sh "$1" "python -m pip install shapely==2.1.2"
    ./run.sh "$1" "python -m pip install seaborn==0.13.2"
    ./run.sh "$1" "python -m pip install python-dotenv==1.2.2"
    ./run.sh "$1" "python -m pip install openai==2.38.0"
    ./run.sh "$1" "python -m pip install puremagic==2.2.0"
    ./run.sh "$1" "python -m pip install crocoddyl==3.2.1"
    ./run.sh "$1" "python -m pip install trimesh==4.12.2"
    ./run.sh "$1" "python -m pip install xmltodict==1.0.4"
    ./run.sh "$1" "python -m pip install setuptools==82.0.1"
    ./run.sh "$1" "python -m pip install wheel==0.47.0"
    ./run.sh "$1" "python -m pip install mujoco==3.9.0"
    ./run.sh "$1" "python -m pip install meshcat==0.3.2"
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
## ------------------------- data setup ---------------------------------- ##
## ==========================================================================
if [[ "$1" =~ "data" ]] || [[ "$1" =~ "all" ]];then
    echo_blue "Pulling data..."
    wget --no-check-certificate "https://yaleedu-my.sharepoint.com/:u:/g/personal/aalap_shah_yale_edu/IQBOZDE5XIxLSYk-1a0bCfORAX_kEzotA-G29bg42KGdnJY?e=SOf1wL&download=1" -O library.zip
    rm -rf mlr/share/projects/block_building/library
    chmod +777 library.zip
    unzip library.zip
    mv library mlr/share/projects/block_building
    rm -rf library.zip
else
    echo_green "Not pulling any data"
fi


## ==========================================================================
## ------------------------- pdf to jpeg --------------------------------- ##
## ==========================================================================
if [[ "$1" =~ "pdftojpg" ]];then
    echo_blue "Converting PDFs to JPEGs..."
    for fn in "$2"/*.pdf; do
        basename="${fn%.pdf}"
        pdftoppm -jpeg "$fn" "$basename"
        if [ -f "${basename}-1.jpg" ]; then
            mv "${basename}-1.jpg" "${basename}.jpg"
        fi
    done
fi
