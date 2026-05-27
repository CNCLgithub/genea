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
. load_paths.sh


APP_ENV=""
APP_CONT="${ENV['cont_init']}"
if [[ "$1" == "bb" ]]; then  # block building
    PROJECT="block_building"
    APP_ENV="${ENV[env_bb]}"
    APP_CONT="${ENV[cont_bb]}"
elif [[ "$1" == "nav" ]]; then  # navigation
    PROJECT="navigation"
    APP_ENV="${ENV[env_nav]}"
    APP_CONT="${ENV[cont_nav]}"
else
    PROJECT=""
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


${SING} "${BS}" "${APP_CONT}" bash -c "source ${APP_ENV}/bin/activate \
        && cd /project/mlr/share/projects/${PROJECT} \
        && exec ${COMMAND} \
        && cd /project \
        && deactivate"
