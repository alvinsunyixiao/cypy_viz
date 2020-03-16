# Cyphyhouse Real-Time Visualization

## Dependancies
* cmake
* [lcm-1.4.0](https://github.com/lcm-proj/lcm/releases/tag/v1.4.0)
    (follow [this instruction](http://lcm-proj.github.io/build_instructions.html) 
    for installation)
* Gazebo 9
* rospy

## Compilation
```sh
mkdir build && cd build
cmake ..
make -j
```

## Environment Setup
Include `source <path_to_cypy_vis>/env.sh` in `~/.bashrc` or `~/.zshrc`

## Launch Visualization
Launch the following commands (in order) in different terminal sessions.

1. `gazebo custom.world` (launch gazebo server / client)
2. `roslaunch vrpn_client_ros sample.launch server:=192.168.1.2` (launch vrpn client)
3. `python3 visualize.py -c configs/<xxx>.json` (launch visualization relay script)
