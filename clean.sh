#!/bin/bash

cd "$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )" || exit

# Parse the incoming command
COMMAND="$*"

singularity exec cont_bb.sif bash -c "cd mlr/share && find . -name '*.o' -delete && find . -name '*.so' -delete"
