#!/bin/bash
#SBATCH --job-name=genea_main
#SBATCH --partition=day
#SBATCH --cpus-per-task=42
#SBATCH --mem=64G
#SBATCH --mail-user=aalap.shah@yale.edu
#SBATCH --mail-type=ALL
#SBATCH --output=job_%A_%a.out
pwd; hostname; date

./run.sh nav python main.py

date
