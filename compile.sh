#!/bin/bash

cd "$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )" || exit

# Parse the incoming command
COMMAND="$*"

singularity exec cont_main.sif bash -c "cd mlr/share/projects/block_building/ && make --always-make --silent -f $COMMAND"
