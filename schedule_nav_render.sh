#!/bin/bash
#SBATCH --job-name=genea_blender
#SBATCH --partition=gpu
#SBATCH --gpus=1
#SBATCH --ntasks-per-node=1
#SBATCH --mem=32G
#SBATCH --time=1-00:00:00
#SBATCH --mail-user=aalap.shah@yale.edu
#SBATCH --mail-type=ALL
#SBATCH --output=job_%A_%a.out
#SBATCH --array=0-41
pwd; hostname; date

blender/blender -b -P mlr/share/projects/navigation/utils/blender_utils.py $SLURM_ARRAY_TASK_ID

date
