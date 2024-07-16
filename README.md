# AMR Docking Fiducial

## Compile

```
colcon build --symlink-install --packages-select docking_with_fiducial
```

## Launch

```
ros2 launch docking_with_fiducial docking_with_markers.launch.py namespace:=robot1
```
