# Project Robot Autonomy


## Setup the workspace
source setup.sh
## Run the simulation to generate map
ros2 launch my_turtlebot turtlebot_simulation.launch.py slam:=True
## To run the python script by itself:
python3 project_submodules/test.py
