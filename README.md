# brownbotics_dev
Development repository for the robot cafe 

New update for the readme file to test github behavior


# Install IsaacLab 

In order to install IsaacLab, some modification were needed it in the submodule. The version compatible with the isaac-sim container is the v1.4.1

There were some problems when trying to install the rsl_rl packages: 

## Change in the isaaclab.sh 

This script needs to be modified in the following lines: 

```sh
# install the rl-frameworks specified
${python_exe} -m pip install --upgrade pip
${python_exe} -m pip install git+https://github.com/leggedrobotics/rsl_rl.git
${python_exe} -m pip install -e ${ISAACLAB_PATH}/source/extensions/omni.isaac.lab_tasks["${framework_name}"]

```

## Change in the setup.py of the lab_tasks

The setup.py is located in: IsaacLab/source/extensions/omni.isaac.lab_tasks 

and the rsl-rl line needs to be modifying as follows:  

```python

# Extra dependencies for RL agents
EXTRAS_REQUIRE = {
    "sb3": ["stable-baselines3>=2.1"],
    "skrl": ["skrl>=1.3.0"],
    "rl-games": ["rl-games==1.6.1", "gym"],  # rl-games still needs gym :(
    #"rsl-rl": ["rsl-rl@git+https://github.com/leggedrobotics/rsl_rl.git"],
    "rsl-rl": ["rsl-rl-lib"],
    "robomimic": [],
}

```

Once this is modified the container can be built successfully