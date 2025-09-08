# pumas_nav2

Service robots are intended to help humans in non-industrial environments such as houses or offices. To accomplish their goal, service robots must have several skills such as object recognition and manipulation, face detection and recognition, speech recognition and synthesis, task planning and, one of the most important, navigation in dynamic environments. This repository describes a fully implemented motion-planning system that comprehends from motion and path planning algorithms to spatial representation and behavior-based active navigation.

This paper can be consulted online for free at this [link](https://bit.ly/40YEcZR). The following video shows this system working at RoboCup where we have won the **Smoothest, Safest Navigation Award** in 2022, 2023, 2024, and 2025.

[![Watch the video](https://img.youtube.com/vi/s2g95Y9Me3c/hqdefault.jpg)](https://www.youtube.com/embed/s2g95Y9Me3c)

Please, if you use this material, don't forget to add the following reference:

```
@article{negrete:2018,
author 		= {Marco Negrete and Jesus Savage and Luis Contreras},
title 		= {{A Motion-Planning System for a Domestic Service Robot}},
journal		= {{SPIIRAS Proceedings}},
volume		= {60},
number		= {5},
pages		= {5--38},
year		= {2018}
}
```

These ROS2 version details are described in:

```
@article{contreras:2025,
author 		= {Luis Contreras and Marco Negrete and Tomoaki Yoshiaki and Yuichiro Hirose and Hiroyuki Okada},
title 		= {{A Motion Planning Framework for Multi-Robot Navigation in ROS 2}},
journal		= {{RSJ 2025}},
year		= {2025}
}
```

# Setup

1. Create an env of your choice.

e.g.

First, create a workspace:

```bash
mkdir -p ~/pumas_nav2_ws/src
```

Then, clone this repository into the src folder:

```bash
cd ~/pumas_nav2_ws/src
git clone https://github.com/ARTenshi/robot_navigation.git
```


2. Install missing packages:

e.g.

```bash
sudo apt update
sudo apt install ros-humble-control-msgs
```

3. Build the project:

e.g 

```bash
cd ~/pumas_nav2_ws
colcon build
```

# Robot Navigation

## Structure

**Topics**

TODO

**Services**

TODO

## Initialization

### Prerequisites

To use SLAM and navigation, you need to start a mapping system. 

e.g.
[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

Also, a localization system is required for the correct TF2 frame transformations.

e.g.
[Nav2 AMCL](https://github.com/ros-navigation/navigation2)

or
[EMCL](https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2)

### Notes

Relevant parameters can be found in the navigation launch files in ```~/pumas_nav2_ws/src/pumas_nav2/navigation_start/launch```. In specific: 

**Update your map names**

```xml
  <arg name="static_map_file"  default="/path/to/your/map/maps/map.yaml/map.yaml"/>
  <arg name="prohibition_map_file"  default="/path/to/your/map/prohibition_maps/map.yaml"/>
```

For robot localisation, we use the original slam map from the ```map_server map_saver``` in ```maps/maps/```. Additionally, for path planning, we edit the previous map to add prohibited and or closed areas (e.g. considering a laser-scan-based mapping, we add the unmapped table area, or, to travel between two main doors, to avoid surrounding it from the outside, we add closed edges; we do not do this with the localisation map to avoid misslocalisation errors between the real sensor reading and an edited map.)

**Enable/Disable potential fields**

```xml
<arg name="use_pot_fields" default="True"/>
```
For dynamic obstacle avoidance, we use rejective potential fields that can be enabled or disabled with this parameter. 

Additionally, in the package:

```xml
<node name="potential_fields"  
      pkg="potential_fields"  
      exec="potential_fields_node"
      namespace="$(var ns_prefix)"
      output="screen">
  <param name="use_namespace" value="$(var use_namespace)"/>
```

```xml
  <param name="laser_pot_fields_d0" value="0.50"/>
  <param name="laser_pot_fields_k_rej" value="0.40"/>

  <param name="cloud_pot_fields_d0" value="0.80"/>
  <param name="cloud_pot_fields_k_rej" value="0.20"/>
```
these parameters indicate the obstacle's distance and rejective force during navigation (too low means that the robot won't avoid obstacles, and too high means that the robot will avoid obstacles too far from it -- these two extreme cases might cause undesired behaviours).

# Authors

* **Luis Contreras** - [TID Professional University](https://www.tid.ac.jp/contents/special-interview/2007/)
* **Marco Negrete** - [BioRobotics UNAM](https://biorobotics.fi-p.unam.mx/)
* **Ryohei Kobayashi** - [Kyushu Institute of Technology](https://www.brain.kyutech.ac.jp/~tamukoh/en/)
