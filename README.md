
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

# krabi_description

<img width="868" height="637" alt="image" src="https://github.com/user-attachments/assets/607d036c-9bfc-47ba-a3d6-04611b0c6f1e" />

This project contains the urdf files describing the krabi robot, a robot participating to [Eurobot 2026](https://www.eurobot.org/eurobot/eurobot-2026) (Winter is Coming) robotics competition.

### Krabi Robot
A mobile base with two wheels, two lidars and a camera

## Prerequisites
This project uses [ROS 2](https://www.ros.org/) and [Gazebo](http://gazebosim.org/). It was tested with ROS 2 Jazzy and Gazebo Harmonic, on Ubuntu 24.04. 
* [How to install ROS 2 (ideally: Ubuntu (deb packages))](https://docs.ros.org/en/jazzy/Installation.html) 
* [How to install Gazebo](https://gazebosim.org/docs/harmonic/install/)


## Installation

Clone the project into your ROS workspace, then run ```colcon build --symlink-install```

## To view the urdf in Rviz2

```shell
ros2 launch krabby_description display.launch.xml
```

## Usage
To start the simulation with default parameter

```bash
ros2 launch krabby_description spawn_world.py
```
For the simulation with game elements (caisses)

```bash
ros2 launch krabby_description spawn_world.py world:=table2026Full.world
```

## Roadmap
See the open issues for a list of proposed features (and known issues).

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## What to do for a new year/rules
- Model the table and the elements, and add them to the "models" folder.
  - The .sdf file describes an object: its visual appearance, how it collides (mecanical shape), etc. [More info here](https://medium.com/@14sohaibbk97/get-started-with-robotic-simulations-using-ros2-and-gazebo-part-1-59e4d5d04b88#:~:text=of%20defining%20an-,SDF%20Model%20in%20Gazebo,-%2C%20usually%20it%20comprises). Note: I usually copy-paste an existing model, and modify it (I don't really know what I'm doing :p)
  - If the geometry is really simple, you can directly describe it in the .sdf file. Ex: [2025's Planks are just rectangles](https://github.com/VictorDubois/krabby_description/blob/main/models/Plank/model.sdf#L46)
  - If it is more complex, you can model them in Blender, and export them as .dae. Ex: [All the tables](https://github.com/VictorDubois/krabby_description/blob/main/models/Table2026/model.sdf#L38). The 3D model is used both for visuals AND for the collision. Warning: the option to export .dae has been removed from new versions of Blender! I keep an old version for this purpose. Other software are probably as good.
- You can create the .world for the new year. Usually I make two:
  - One with the table and all the game elements (in 2026 it was needed to test the color detection algorithm to detect the caisses)
  - One with just the table, no game elements => the simulation is far less CPU-intensive, and most tests do not require the game elements
- If the shape of the robot changes, you can update [its properties](https://github.com/VictorDubois/krabby_description/blob/main/urdf/properties.xacro) (but not everything is in this file).

## License
Distributed under the [MIT](https://choosealicense.com/licenses/mit/) License. See `LICENSE` for more information.

## Contact
[Yanis Mazouz](ymazouz.com)

[contributors-shield]: https://img.shields.io/github/contributors/scout22/krabby_description?style=flat-square
[contributors-url]: https://github.com/scout22/krabby_description/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/scout22/krabby_description?style=social
[forks-url]: https://github.com/scout22/krabby_description/network/members
[stars-shield]: https://img.shields.io/github/stars/scout22/krabby_description?style=flat-square
[stars-url]: https://github.com/scout22/krabby_description/stargazers
[issues-shield]: https://img.shields.io/github/issues/scout22/krabby_description?style=flat-square
[issues-url]: https://github.com/scout22/krabby_description/issues
[license-shield]: https://img.shields.io/github/license/scout22/krabby_description?style=flat-square
[license-url]: https://github.com/scout22/krabby_description/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/yanis-mazouz
[product-screenshot]: images/screenshot.png
