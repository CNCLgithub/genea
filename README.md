# Understanding Embodied Plans
This repository contains the code that implements the computational model from the paper "A computational framework for understanding embodied plans".
In addition, it contains information about the experiment and analysis scripts that were used to generate the data for the paper.

## Getting Started

### Working with Apptainer (or Singularity)
We have set up the entire codebase so that it can conveniently be run via 
[Apptainer](https://apptainer.org/docs/user/main/introduction.html) (or Singularity). If your machine does not 
have either of these already installed, run the following commands:
```
./apptainer_download.sh s1
 echo source ~/.bashrc
./apptainer_download.sh s3
./apptainer_download.sh s4
```


### Setup
Download the Apptainer containers
```bash
./setup.sh cont_pull  
```

Download and install all the necessary python packages
```bash
./setup.sh python
```

Download data for running the experiments and to perform analysis
```bash
./setup.sh data
```

### Compile the codebase
When running the code for the first time, you will need to compile CPP files in the codebase. 
This is done by running the following command:
```
singularity exec cont_init.simg bash -c "cd mlr/share && make"
```
NOTE: A lot of targets under `mlr/share` will fail to compile with messages such as `***** FAILED`. This is completely fine and expected!

Continue with the following commands:
```bash
./compile.sh makefile_make_g_file
./compile.sh makefile_run_physx
./compile.sh makefile_run_risk_test
./compile.sh makefile_run_stability
./compile.sh makefile_run_viewer
./compile.sh makefile_run_ppm_generator
```

## Define block configurations
Each trial of a study is described via a pair of `.g` files. 

The files under the `library/init_files` directory define initial block configurations 
whereas those under `library/fin_files` directory specify final block configurations.

To view any block configuration, run the following command:
```bash
./run.sh python run_viewer.py -p <absolute path of the .g file>
```

You can also generate images of any block configuration by running the following command:
```bash
./run.sh python run_image_generator.py -p <absolute path of the .g file> -i
```


## Run the model
All key model variables and parameters are defined in the `utils/core_utils.py` file, 
including `ConfigUtils.IS_PHYSICS_ON` which when set to `False` runs the standard TAMP ablation model.

Use the `library/exp_data/trials_list.csv` file to specify the trials you want to run the model on. 
- The first column in this file states the study name (e.g., "action" for the action inference study), 
- The second column is the number associated with the trial. 
- The last column specifies the first actions that the model should consider (e.g., `['red']` means picking the red block; `['blue','yellow']` implies that the first action will involve picking both the blue and the yellow blocks). ""

To run the model on the studies specified in the `trials_list.csv` file, run the following command:
```bash
./run.sh python run_experiment.py -a   # run action inference trials
./run.sh python run_experiment.py -g   # run goal inference trials
./run.sh python run_experiment.py -e   # run embodiment strategies inference (how many hands) trials
./run.sh python run_experiment.py -d   # run difficulty trials
```

The output of the model is stored in the `library/out_robot_data` directory. 
Data from the action, goal and embodiment strategies inference studies will be stored in the `overall_out.csv` file.
Data from the difficulty study will be stored in the `diff_overall_out.csv` file.
All intermediate files generated during the model run will be stored in the `library/intermediate_files` directory.

## Analyze the results
```bash
./run.sh python perform_analysis.py -ag   # analyze action-goal inference data
./run.sh python perform_analysis.py -e   # analyze embodiment strategies inference data
./run.sh python perform_analysis.py -d   # analyze difficulty data
```
