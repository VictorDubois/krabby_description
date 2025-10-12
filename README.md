
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

# krabi_description
![Eurobot2020 simulation running][product-screenshot]
This project contains the urdf files describing the krabi robot, a robot participating to [Eurobot 2026](https://www.eurobot.org/eurobot/eurobot-2026) (Sail the world) robotics competition.

### Krabi Robot
A mobile base with two wheels, two lidars and a camera

## Prerequisites
This project uses [ROS 2](https://www.ros.org/) and [Gazebo](http://gazebosim.org/). It was tested with ROS 2 Jazzy and Gazebo Harmonic, on Ubuntu 24.04. 
* [How to install ROS 2](http://wiki.ros.org/melodic/Installation](https://docs.ros.org/en/jazzy/Installation.html)) 
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
