#!/bin/bash

cd "$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )" || exit
. load_config.sh

CONT="${ENV['cont_main']}"

# Parse the incoming command
COMMAND="$*"

# Enter the container and run the command
SING="${ENV['exec']} exec --nv"

# Add the repo path to "/project"
BS="-B ${PWD}:/project"

printf "=%.0s"  $(seq 1 79)
printf "\nExecuting: %s\n" "${COMMAND}"
printf "=%.0s"  $(seq 1 79)
printf "\n"


${SING} ${BS} ${CONT} bash -c "source ${ENV['env']}/bin/activate \
        && cd ${PWD}/mlr/share/projects/block_building \
        && exec $COMMAND \
        && cd /project \
        && deactivate"
