#!/bin/bash

echo_green () { echo -e "\033[1;38;5;047m$* \033[0m"; }
echo_blue () { echo -e "\033[1;38;5;051m$* \033[0m"; }
echo_orange () { echo -e "\033[1;38;5;202m$* \033[0m"; }
echo_red () { echo -e "\033[1;38;5;196m$* \033[0m"; }
echo_yellow () { echo -e "\033[1;38;5;226m$* \033[0m"; }
echo_purple () { echo -e "\033[1;38;5;141m$* \033[0m"; }
echo_blue_thin () { echo -e "\033[0;36m$* \033[0m"; }

cd "$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )" || exit
. load_config.sh

CONT="${ENV['cont_main']}"

if [ "$1" == "n" ]; then  # navigation
    . load_paths.sh
    PROJECT="navigation"
elif [ "$1" == "bb" ]; then  # block building
    . load_paths.sh
    PROJECT="block_building"
elif [ "$1" == "setup" ]; then  # executing setup
    PROJECT=""
elif [ "$1" == "x" ]; then
    . load_paths.sh
    PROJECT=""
else
    echo_orange "Invalid argument"
    exit 1
fi

COMMAND="${*:2}"

# Enter the container and run the command
SING="${ENV['exec']} exec --nv"

# Add the repo path to "/project"
BS="-B ${PWD}:/project"

echo_blue_thin "$(printf "=%.0s"  $(seq 1 79))"
echo_blue "Executing: ${COMMAND}"
echo_blue_thin "$(printf "=%.0s"  $(seq 1 79))"
echo


${SING} "${BS}" "${CONT}" bash -c "source ${ENV['env']}/bin/activate \
        && cd /project/mlr/share/projects/${PROJECT} \
        && exec ${COMMAND} \
        && cd /project \
        && deactivate"
