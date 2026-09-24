# vrx_gazebo

## Description
This package provides gazebo models, plugins, and examples for simulating the [RobotX challenge](https://www.robotx.org/index.php/2014-01-05-21-55-32/2016-rules-requirements) within Gazebo using ROS.

## Usage
To launch gazebo with an example course layout and WAM-V platform, run:

```roslaunch vrx_gazebo sandisland.launch```

### Dynamic channel integration test

The existing `example_course.world` can be used as the base map for a longer
navigation lane with deterministic moving traffic.  The channel extension is
defined by the following approximate world-frame corners:

```text
entrance: (-545, 203), (-553, 265)
exit:     (-273, 285), (-276, 324)
```

It is approximately 283.4 m long, with a width tapering from 62.5 m at the
entrance to 39.1 m at the exit:

The two boundaries now use approximately 6 m spacing (49 buoys per side,
including the entrance, middle and exit markers). Because each marker uses a
buoyancy plugin, reduce the density or disable unused actors if the physics
update rate becomes too low.

The channel also contains a reproducible random static-obstacle layer made
from blocks of `robotx_light_buoy_static` cells. It contains 10 blocks and 60
fixed buoy cells, generated with seed `20260901`; the layer is controlled by
`spawn_static_obstacles` in `dynamic_channel.launch`.

```bash
roslaunch vrx_gazebo dynamic_channel.launch
```

`dynamic_channel.launch` keeps the original world intact, adds boundary buoys
at the entrance, middle and exit, and spawns two crossing ships, one opposing
ship and one slower same-direction ship. Disable individual actors while
debugging with, for example:

```bash
roslaunch vrx_gazebo dynamic_channel.launch \
  cross_ship_2_enabled:=false head_on_ship_enabled:=false
```

The ships are generated from `models/obstacle_ship/model.sdf.xacro`, so each
actor has independent world-frame waypoints, `start_delay` and `max_speed`.
The current speed limits are 1.40 m/s for crossing ships, 1.20 m/s for the
head-on ships and 0.90 m/s for the slower same-direction ships.
Each obstacle ship keeps four buoyancy spheres (radius 0.25 m); their local
vertical offset is now `z=-0.15 m` instead of `z=0.10 m` so the hull floats
higher with a shallower draft.
为了避免保存世界中 03、04、05 号船相互靠近，三条往复航线调整为：
`03: (-504.492,262.225) <-> (-490.900,245.000)`、
`04: (-420.908,247.384) <-> (-490.000,226.000)`、
`05: (-531.115,249.903) <-> (-524.000,218.000)`。
Their ground-truth poses remain available through Gazebo's
`/gazebo/model_states`; use that only for evaluation, not as the planner's
obstacle input. The WAM-V point cloud is published on
`/wamv/sensors/lidars/lidar_wamv/points` when the VRX sensors are enabled.

If the world was saved after the dynamic channel had already been spawned, the
saved file contains the WAM-V, channel extension and moving ships. Reuse it
without spawning duplicate entities:

```bash
roslaunch vrx_gazebo dynamic_channel.launch \
  world:=/absolute/path/dynamic_channel_saved.world \
  spawn_wamv:=false spawn_dynamic_actors:=false \
  spawn_static_obstacles:=false
```

在当前保存的世界中，9 艘障碍船已按 Gazebo 模型名编号，编号与原始船型的对应关系为：

```text
obstacle_ship_01 = head_on_ship
obstacle_ship_02 = cross_ship_1
obstacle_ship_03 = cross_ship_2
obstacle_ship_04 = slow_ship
obstacle_ship_05 = cross_ship_2_clone
obstacle_ship_06 = head_on_ship_clone
obstacle_ship_07 = slow_ship_clone
obstacle_ship_08 = head_on_ship_clone_0
obstacle_ship_09 = slow_ship_clone_clone
```

编号会显示在 Gazebo 的 World/模型树以及 `/gazebo/model_states` 的
`name` 数组中；船的当前位姿和往复航点保持不变。


## Course models
The following models are used in the RobotX challenge and will be included in this package.

| Task Element                 | Product                  | Status | Model  |
|:-----------------------------|:-------------------------|:-------|:-------|
| Light Buoy                   | Custom                   | ADDED  | robotx_light_buoy |
| Obstacle - Small             | PolyForm A-3 black       | ADDED  | polyform_a3 |
| Obstacle - Medium            | PolyForm A-5 black       | ADDED  | polyform_a5 |
| Obstacle - Large             | PolyForm A-7 black       | ADDED  | polyform_a7 |
| Red Can buoy                 | Sur-Mark Can Buoy 950410 | ADDED  | surmark950410 |
| Green Can buoy               | Sur-Mark Can Buoy 950400 | ADDED  | surmark950400 |
| White Can buoy               | Sur-Mark Can Buoy 46104  | ADDED  | surmark46104 |
| Green Totem                  | 46104 w/ Green Cover     | ADDED  | green_totem |
| Yellow Totem                 | 46104 w/ Yellow Cover    | ADDED  | yellow_totem |
| Blue Totem                   | 46104 w/ Blue Cover      | ADDED  | blue_totem |
| Red Totem                    | 46104 w/ Red Cover       | ADDED  | red_totem |
| Black Totem                  | 46104 w/ Black Cover     | ADDED  | black_totem |
| Dock Material                | JetDock C000000008       | ADDED  | dock_block |
| 2016 Dock                    | Custom Assembly          | ADDED  | robotx_dock_2016 |
| 2018 Dock                    | Custom Assembly          | ADDED  | robotx_dock_2018 |
| Blue Circle Symbol           | Custom                   | ADDED  | symbol_circle |
| Blue Cruciform Symbol        | Custom                   | ADDED  | symbol_cross |
| Blue Triangle Symbol         | Custom                   | ADDED  | symbol_triangle |
| Green Circle Symbol          | Custom                   | ADDED  | symbol_circle |
| Green Cruciform Symbol       | Custom                   | ADDED  | symbol_cross |
| Green Triangle Symbol        | Custom                   | ADDED  | symbol_triangle |
| Red Circle Symbol            | Custom                   | ADDED  | symbol_circle |
| Red Cruciform Symbol         | Custom                   | ADDED  | symbol_cross |
| Red Triangle Symbol          | Custom                   | ADDED  | symbol_triangle |
| 2016 Pinger Transit (quals)  | Custom Group of objects  | ADDED  | robotx_2016_qualifying_pinger_transit |
| 2016 Pinger Transit (finals) | Custom Group of objects  | ADDED  | robotx_2016_finals_pinger_transit |
| 2018 Entrance/Exit Gate      | Custom Group of objects  | ADDED  | robotx_2018_entrance_gate |
| White placard                | Custom                   | ADDED  | placard |

*= More detailed model needed
