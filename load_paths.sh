#!/bin/bash


APP_ENV=""
APP_PYTHON=""
if [[ "$1" == "bb" ]]; then  # block building
    APP_ENV="${ENV[env_bb]}"
    APP_PYTHON="8"
elif [[ "$1" == "nav" ]]; then  # navigation
    APP_ENV="${ENV[env_nav]}"
    APP_PYTHON="12"
fi


declare -gA PATHS
PATHS[PYTHON]="/project/${APP_ENV}/bin/python3.${APP_PYTHON}"
PATHS[PYTHONPATH]="/project:/opt/openrobots/lib/python3.${APP_PYTHON}/site-packages:/project/venv/lib/python3.${APP_PYTHON}/site-packages:$PYTHONPATH"
PATHS[LD_LIBRARY_PATH]="/opt/openrobots/lib:$LD_LIBRARY_PATH"
PATHS[CMAKE_PREFIX_PATH]="/opt/openrobots:$CMAKE_PREFIX_PATH"


# export the required path variables
for i in "${!PATHS[@]}"
do
    printf "%s \u2190 %s\n" "${i}" "${PATHS[$i]}"
    export "${i}=${PATHS[$i]}"
done