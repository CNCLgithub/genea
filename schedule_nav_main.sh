#!/bin/bash
#SBATCH --job-name=genea_main
#SBATCH --partition=day
#SBATCH --cpus-per-task=1
#SBATCH --mem=32G
#SBATCH --mail-user=aalap.shah@yale.edu
#SBATCH --mail-type=ALL
#SBATCH --output=job_%A_%a.out
#SBATCH --time=1-00:00:00
#SBATCH --array=0-41


pwd; hostname; date

./run.sh nav python main.py -g -si $SLURM_ARRAY_TASK_ID

date
