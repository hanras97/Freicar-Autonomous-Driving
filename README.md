# Freicar-Autonomous-Driving 
## How to run ?
### Install Dependencies
`./setup_env.sh`
### Run start bash script
`./start.sh`
## Debug each module seperately!
### Starting the simulator
`roslaunch freicar_launch_ sim_base.launch`.

### Starting the localizer:
`rosrun freicar_localization_1 freicar_localization_1_node `
`rosrun freicar_sign_detect freicar_sign_detect_node `

### Starting the controller:
`roslaunch freicar_control_ start_controller.launch `

### Publishing the path:
`roscd freicar_control_/scripts/`
`anaconda`
`python save_path.py `
