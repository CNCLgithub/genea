#!/bin/bash


declare -gA ENV
ENV[exec]=singularity
ENV[cont_init]=cont_init.simg
ENV[cont_main]=cont_main.sif
ENV[cont_def]=apptainer_build.def
ENV[env]=venv
