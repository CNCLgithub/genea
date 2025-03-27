#!/bin/bash


declare -gA PATHS
PATHS[PYTHON]="/project/${ENV[env]}/bin/python3.8"
PATHS[PYTHONPATH]="/project:/opt/openrobots/lib/python3.8/site-packages:/project/venv/lib/python3.8/site-packages:$PYTHONPATH"
PATHS[LD_LIBRARY_PATH]="/opt/openrobots/lib:$LD_LIBRARY_PATH"
PATHS[CMAKE_PREFIX_PATH]="/opt/openrobots:$CMAKE_PREFIX_PATH"


# export the required path variables
for i in "${!PATHS[@]}"
do
    printf "%s \u2190 %s\n" "${i}" "${PATHS[$i]}"
    export "${i}=${PATHS[$i]}"
done