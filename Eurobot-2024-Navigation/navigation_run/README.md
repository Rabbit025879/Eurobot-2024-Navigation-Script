# Navigation Run

### Launch Procedure

```
source devel/setup.bash
catkin_make
```

1 robot simulation:

```bash
roslaunch navigation_run sim.launch
```

2 robot simulation:

```bash
roslaunch navigation_run sim_two.launch
```

## Launch Files

1. sim.launch & sim_two.launch

    Launch "basic.launch" and Rviz.

2. basic.launch

    Launch the "move_base.launch", "map_server", "odometry_sim", and tf of "map -> odometry".

3. move_base.launch

    Launch "move_base", and load the parameter "move_base_params.yaml"

### Config Files

1. config/move_base_params.yaml

    Parameters for move_base, global planner, local planner, global/local costmap.

2. map/basic_map.yaml

    Specify the static map to use in Rviz.

3. rviz/rviz_sim.rviz

    Cofig file for Rviz.
