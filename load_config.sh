#!/bin/bash

# Loads config constants
CFG_FILE="default.conf"
# shellcheck disable=SC1090
. "$CFG_FILE"

# export the required path variables
for i in "${!PATHS[@]}"
do
    printf "%s \u2190 %s\n" "${i}" "${PATHS[$i]}"
    export "${i}=${PATHS[$i]}"
done