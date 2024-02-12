# f1tenth_launch
<!-- Required -->
<!-- Package description -->
F1TENTH launch files.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->
Check external links for dependencies.

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to f1tenth_launch
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->
The package requires map directory path to be provided with following structure:
- `map_name_directory/`
    - `map.pbstream`
    - `trajectory.csv`
    - `map.pgm`
    - `map.yaml`

The `map.pgm` and `map.yaml` files are not required, but are useful for trajectory generation purposes with external tools.
You can generate these files with nav2_map_server while running the simulator or real vehicle:
```bash
ros2 run nav2_map_server map_saver_cli -f map
```
Then, move the files to the `map_name_directory/` directory.

### Simulator

1. Run F1TENTH AWSIM simulator.
2. Run launch file:
```bash
ros2 launch f1tenth_launch e2e_simulator.launch.py map_path:=<path_to_map_directory>
```

### Real vehicle

1.  Run launch file:
```bash
ros2 launch f1tenth_launch car.launch.py map_path:=<path_to_map_directory> vehicle_model:=<vehicle_model> sensor_model:=<sensor_model>
```

### Mapping
Use `mapping:=True` argument to enable mapping mode. You can also turn off reference trajectory loader if the trajectory is not generated yet with `use_trajectory_loader:=False` argument.

## References / External links
<!-- Optional -->
* [Meta package](https://github.com/amadeuszsz/autoware/blob/f1tenth/)
* [F1TENTH AWSIM](https://github.com/amadeuszsz/AWSIM/releases)