# DWA Planner
This repository contains the implementation of the Dynamic Window Approach (DWA) local planner.

DWA is a velocity based planner which considers the robot dynamics to find a feasible path. It's core is really simple, calculate a valid a velocity search space and select an optimal velocity based on some calculations, the following image (Provided by [The Dynamic Window Approach to Collision Avoidance](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)) provide a great overview of its algrotihm:


<div align="center"><img src="https://user-images.githubusercontent.com/49252525/122049874-85ec7c80-cda8-11eb-92c7-5c9134a8b5c9.png" width="35%" height="35%"/></div>


## Setting up the virtual environment
Anaconda or Minicoda is required for this project as you must create a conda environment with the required dependencies.

### Create the environment
For creating the environment use the following bash command.
```bash
conda env create -f environment.yml
```

Check if the environment was successfully created (dwa_planner should be listed when running the command below)
```bash
conda env list
```

### Activate the environment
For activating the environment use the following bash command (Use the Ananconda Prompt if not recognized)
```bash
conda activate dwa_planner
```

## Usage
Before usage you can modify the `dwa.py` to set a new starting pose and a goal location. For running the script you only need to run (Inside the folder if running from a Terminal - Anaconda Prompt):
```bash
python dwa.py
```


