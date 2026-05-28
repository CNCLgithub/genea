#!/bin/bash


declare -gA ENV
ENV[exec]=singularity
ENV[cont_init]=cont_init.simg
ENV[cont_bb]=cont_bb.sif
ENV[cont_nav]=cont_nav.sif
ENV[def_bb]=apptainer_build_bb.def
ENV[def_nav]=apptainer_build_nav.def
ENV[env_bb]=venv_bb
ENV[env_nav]=venv_nav
